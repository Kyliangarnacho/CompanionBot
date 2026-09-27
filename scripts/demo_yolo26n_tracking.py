"""Continuous YOLO person detection + temporary ByteTrack tracklets.

Capture runs on the Stage 7.2 producer thread and writes to its capacity-one
latest-frame slot. Only detector results that actually exist are sent to the
tracker; no boxes or intermediate frames are synthesized.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Sequence

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.detection_stream import LatestFrameSlot, process_person_frame
from perception.person_detector import PersonDetector, PersonDetectorConfig
from perception.person_tracker import PersonTracker, PersonTrackingFrame
from perception.ultralytics_bytetrack import UltralyticsByteTrackBackend
from scripts import demo_yolo26n_stream as source_stream


FRAME_CSV_FIELDS = (
    "source_id", "source_sequence_id", "source_time_s", "frame_width", "frame_height",
    "sequence_gap", "detection_count", "active_track_count", "created_track_ids",
    "newly_lost_track_ids", "newly_removed_track_ids", "tracker_update_index",
    "detector_wall_ms", "tracker_wall_ms", "host_receive_to_tracking_result_age_ms",
    "single_person_track_id", "single_person_id_changed", "track_boxes_json",
)
LIFETIME_CSV_FIELDS = (
    "track_id", "first_sequence_id", "last_sequence_id", "first_source_time_s",
    "last_source_time_s", "observed_frame_count", "observed_lifetime_s",
    "lost_event_count", "removed", "final_status",
)


@dataclass
class _TrackLifetime:
    track_id: int
    first_sequence_id: int
    last_sequence_id: int
    first_source_time_s: float
    last_source_time_s: float
    observed_frame_count: int = 0
    lost_event_count: int = 0
    removed: bool = False
    final_status: str = "active"

    def observe(self, sequence_id: int, source_time_s: float) -> None:
        self.last_sequence_id = sequence_id
        self.last_source_time_s = source_time_s
        self.observed_frame_count += 1
        self.final_status = "active"


def _device(value: str) -> int | str:
    value = value.strip()
    return int(value) if value.isdecimal() else value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--video", type=Path, help="local video file")
    sources.add_argument("--camera-device", type=_device,
                         help="OpenCV camera index or source string")
    parser.add_argument("--realtime-playback", action="store_true",
                        help="pace a video by its reported FPS")
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--opencv-backend", type=int, default=None,
                        help="optional OpenCV backend ID")
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    # Let upstream ByteTrack receive its configured low-score band (>0.10).
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--device", default=None)
    parser.add_argument("--warmup-frames", type=int, default=3)
    parser.add_argument("--max-source-frames", type=int, default=None)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "models" / "minisegway" / "stage7" / "stage7_3" / "results")
    parser.add_argument("--save-video", type=Path, default=None)
    parser.add_argument("--preview", action="store_true",
                        help="show a local window; press q to stop")
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.imgsz < 1 or not 0.0 <= args.conf <= 1.0:
        parser.error("--imgsz must be positive and --conf must be in [0, 1]")
    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be nonnegative")
    if args.max_source_frames is not None and args.max_source_frames < 1:
        parser.error("--max-source-frames must be positive")
    if (args.width is None) != (args.height is None):
        parser.error("--width and --height must be supplied together")
    if args.width is not None and (args.width < 1 or args.height < 1):
        parser.error("--width and --height must be positive")
    if args.fps is not None and (not np.isfinite(args.fps) or args.fps <= 0):
        parser.error("--fps must be finite and positive")
    if isinstance(args.camera_device, str) and not args.camera_device.strip():
        parser.error("--camera-device must not be empty")
    if isinstance(args.camera_device, int) and args.camera_device < 0:
        parser.error("--camera-device index must be nonnegative")
    if args.width is not None and (args.width < 1 or args.height < 1):
        parser.error("--width and --height must be positive")
    if args.fps is not None and (not np.isfinite(args.fps) or args.fps <= 0):
        parser.error("--fps must be finite and positive")
    if isinstance(args.camera_device, str) and not args.camera_device.strip():
        parser.error("--camera-device must not be empty")
    if isinstance(args.camera_device, int) and args.camera_device < 0:
        parser.error("--camera-device index must be nonnegative")
    if args.video is not None and not args.video.is_file():
        parser.error(f"video file does not exist: {args.video}")
    if args.realtime_playback and args.video is None:
        parser.error("--realtime-playback applies to --video only")


def _source_args(args: argparse.Namespace) -> argparse.Namespace:
    # Reuse the already-validated Stage 7.2 camera/video source lifecycle.
    return argparse.Namespace(
        video=args.video,
        realtime_playback=args.realtime_playback,
        max_source_frames=args.max_source_frames,
        camera_device=args.camera_device,
        width=args.width,
        height=args.height,
        fps=args.fps,
        opencv_backend=args.opencv_backend,
    )


def _fps(count: int, first_time_s: float | None, last_time_s: float | None) -> float | None:
    if count < 2 or first_time_s is None or last_time_s is None:
        return None
    elapsed = last_time_s - first_time_s
    return (count - 1) / elapsed if elapsed > 0.0 else None


def _percentile(values: deque[float], quantile: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values), quantile))


def _ids_text(track_ids: tuple[int, ...]) -> str:
    return ";".join(str(value) for value in track_ids)


def _write_frame_row(writer: csv.DictWriter, result: PersonTrackingFrame,
                     detector_wall_s: float, *, single_person_track_id: int | None,
                     single_person_id_changed: bool) -> None:
    diagnostics = result.diagnostics
    tracks = [
        {
            "track_id": item.track_id,
            "bbox_xyxy_px": item.bbox_xyxy_px,
            "confidence": item.confidence,
            "class_id": item.class_id,
            "class_name": item.class_name,
        }
        for item in result.tracks
    ]
    writer.writerow({
        "source_id": result.source_id,
        "source_sequence_id": result.source_sequence_id,
        "source_time_s": f"{result.source_time_s:.9f}",
        "frame_width": result.frame_width,
        "frame_height": result.frame_height,
        "sequence_gap": diagnostics.source_sequence_gap,
        "detection_count": result.detection_count,
        "active_track_count": diagnostics.active_track_count,
        "created_track_ids": _ids_text(diagnostics.created_track_ids),
        "newly_lost_track_ids": _ids_text(diagnostics.newly_lost_track_ids),
        "newly_removed_track_ids": _ids_text(diagnostics.newly_removed_track_ids),
        "tracker_update_index": diagnostics.update_index,
        "detector_wall_ms": f"{detector_wall_s * 1000.0:.3f}",
        "tracker_wall_ms": f"{diagnostics.tracker_update_wall_time_s * 1000.0:.3f}",
        "host_receive_to_tracking_result_age_ms":
            f"{result.host_receive_to_result_age_s * 1000.0:.3f}",
        "single_person_track_id": ("" if single_person_track_id is None
                                   else single_person_track_id),
        "single_person_id_changed": int(single_person_id_changed),
        "track_boxes_json": json.dumps(tracks, separators=(",", ":")),
    })


def _annotate(frame, result: PersonTrackingFrame, processed_fps: float | None,
              skipped_count: int) -> np.ndarray:
    image = frame.bgr.copy()
    for track in result.tracks:
        x1, y1, x2, y2 = track.bbox_xyxy_px
        left = max(0, min(frame.width - 1, int(round(x1))))
        top = max(0, min(frame.height - 1, int(round(y1))))
        right = max(0, min(frame.width - 1, int(round(x2))))
        bottom = max(0, min(frame.height - 1, int(round(y2))))
        color = (0, 220, 40)
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        cv2.putText(image, f"ID {track.track_id} | person {track.confidence:.2f}",
                    (left, max(18, top - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2)
    fps_text = "N/A" if processed_fps is None else f"{processed_fps:.2f}"
    overlay = (
        f"seq={result.source_sequence_id} processed={fps_text} fps "
        f"age={result.host_receive_to_result_age_s * 1000.0:.0f}ms "
        f"skipped={skipped_count} active={len(result.tracks)}"
    )
    cv2.putText(image, overlay, (12, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 220, 255), 2)
    return image


def _record_lifetimes(result: PersonTrackingFrame,
                      lifetimes: dict[int, _TrackLifetime]) -> None:
    diagnostics = result.diagnostics
    for track_id in diagnostics.newly_lost_track_ids:
        lifetime = lifetimes.get(track_id)
        if lifetime is not None:
            lifetime.lost_event_count += 1
            lifetime.final_status = "lost"
    for track_id in diagnostics.newly_removed_track_ids:
        lifetime = lifetimes.get(track_id)
        if lifetime is not None:
            lifetime.removed = True
            lifetime.final_status = "removed"
    for track in result.tracks:
        lifetime = lifetimes.get(track.track_id)
        if lifetime is None:
            lifetime = _TrackLifetime(
                track.track_id,
                result.source_sequence_id,
                result.source_sequence_id,
                result.source_time_s,
                result.source_time_s,
            )
            lifetimes[track.track_id] = lifetime
        lifetime.observe(result.source_sequence_id, result.source_time_s)


def _write_lifetime_rows(path: Path, lifetimes: dict[int, _TrackLifetime]) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=LIFETIME_CSV_FIELDS)
        writer.writeheader()
        for item in sorted(lifetimes.values(), key=lambda row: row.track_id):
            writer.writerow({
                "track_id": item.track_id,
                "first_sequence_id": item.first_sequence_id,
                "last_sequence_id": item.last_sequence_id,
                "first_source_time_s": f"{item.first_source_time_s:.9f}",
                "last_source_time_s": f"{item.last_source_time_s:.9f}",
                "observed_frame_count": item.observed_frame_count,
                "observed_lifetime_s": f"{item.last_source_time_s-item.first_source_time_s:.6f}",
                "lost_event_count": item.lost_event_count,
                "removed": int(item.removed),
                "final_status": item.final_status,
            })


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)

    # Keep Ultralytics' settings file inside the project's selected environment
    # unless the operator explicitly configured another location.
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".venv"))
    detector = PersonDetector(PersonDetectorConfig(
        model_path=args.model,
        imgsz=args.imgsz,
        confidence_threshold=args.conf,
        device=args.device,
    ))
    backend = UltralyticsByteTrackBackend()
    tracker = PersonTracker(backend)
    print(f"detector: {args.model}; imgsz={args.imgsz}; conf={args.conf}; "
          f"device={args.device if args.device is not None else 'Ultralytics auto'}")
    print(f"tracker: {tracker.backend_name}; Ultralytics {backend.upstream_version}")
    print(f"upstream tracker config: {backend.config_path}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    frame_csv = args.output_dir / "person_tracking_frames.csv"
    lifetime_csv = args.output_dir / "person_track_lifetimes.csv"
    config_path = args.output_dir / "stage7_3_run_config.json"
    metrics_path = args.output_dir / "stage7_3_tracking_metrics.json"
    report_path = args.output_dir / "STAGE7_3_TRACKING_REPORT.md"
    config_bytes = backend.config_path.read_bytes()
    config_path.write_text(json.dumps({
        "detector": {
            "model": str(args.model), "imgsz": args.imgsz,
            "confidence_threshold": args.conf,
            "device": args.device if args.device is not None else "automatic",
        },
        "tracker_backend": tracker.backend_name,
        "ultralytics_version": backend.upstream_version,
        "upstream_config_source": str(backend.config_path),
        "upstream_config_sha256": hashlib.sha256(config_bytes).hexdigest(),
        "effective_upstream_config": backend.effective_config,
        "source_time_semantics": "ColorFrame.host_receive_time_s (host read-complete)",
        "frame_schedule": "latest-frame wins; no synthetic updates for skipped frames",
    }, indent=2), encoding="utf-8")

    slot = LatestFrameSlot()
    runtime = source_stream._SourceRuntime.create(slot)
    producer = source_stream._source_thread(_source_args(args), runtime)
    producer.start()
    if not runtime.ready.wait(timeout=15.0):
        runtime.stop.set()
        raise RuntimeError("timed out waiting for source initialization")
    with runtime.lock:
        print(f"source: {runtime.description}")
        if runtime.camera_profile is not None:
            profile = runtime.camera_profile
            print(f"OpenCV backend: {profile.backend_name}")
            print(f"requested profile: {profile.requested_width}x{profile.requested_height}"
                  f"@{profile.requested_fps} fps")
            print(f"OpenCV-reported profile: {profile.reported_width}x"
                  f"{profile.reported_height}@{profile.reported_fps:g} fps")

    process_ms: deque[float] = deque(maxlen=10_000)
    tracker_ms: deque[float] = deque(maxlen=10_000)
    age_ms: deque[float] = deque(maxlen=10_000)
    gaps_sample: deque[int] = deque(maxlen=10_000)
    lifetimes: dict[int, _TrackLifetime] = {}
    processed_count = 0
    steady_count = 0
    total_detection_count = 0
    total_gap = 0
    max_gap = 0
    lost_event_count = 0
    removed_event_count = 0
    active_track_count_at_end = 0
    last_processed_sequence_id: int | None = None
    previous_single_person_track_id: int | None = None
    single_person_observation_frames = 0
    single_person_id_changes = 0
    first_result_s: float | None = None
    last_result_s: float | None = None
    video_writer = None
    video_fps: float | None = None
    stopped_early = False
    upstream_frame_count = 0

    try:
        with frame_csv.open("w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=FRAME_CSV_FIELDS)
            writer.writeheader()
            while True:
                frame = slot.get(timeout_s=0.2)
                if frame is None:
                    if runtime.done.is_set():
                        break
                    continue
                detection_frame = process_person_frame(frame, detector)
                result = tracker.update(detection_frame)
                diagnostics = result.diagnostics
                processed_count += 1
                active_track_count_at_end = diagnostics.active_track_count
                last_processed_sequence_id = result.source_sequence_id
                total_detection_count += result.detection_count
                gap = diagnostics.source_sequence_gap
                total_gap += gap
                max_gap = max(max_gap, gap)
                gaps_sample.append(gap)
                _record_lifetimes(result, lifetimes)
                lost_event_count += len(diagnostics.newly_lost_track_ids)
                removed_event_count += len(diagnostics.newly_removed_track_ids)
                single_person_track_id = None
                single_person_id_changed = False
                if result.detection_count > 1:
                    # A multi-detection frame has no unambiguous single-person timeline.
                    previous_single_person_track_id = None
                elif result.detection_count == 1 and len(result.tracks) == 1:
                    single_person_track_id = result.tracks[0].track_id
                    single_person_observation_frames += 1
                    single_person_id_changed = (
                        previous_single_person_track_id is not None
                        and previous_single_person_track_id != single_person_track_id
                    )
                    single_person_id_changes += int(single_person_id_changed)
                    previous_single_person_track_id = single_person_track_id
                steady = processed_count > args.warmup_frames
                if steady:
                    steady_count += 1
                    process_ms.append(detection_frame.detector_call_wall_time_s * 1000.0)
                    tracker_ms.append(diagnostics.tracker_update_wall_time_s * 1000.0)
                    age_ms.append(result.host_receive_to_result_age_s * 1000.0)
                    if first_result_s is None:
                        first_result_s = result.result_ready_time_s
                    last_result_s = result.result_ready_time_s
                _write_frame_row(
                    writer,
                    result,
                    detection_frame.detector_call_wall_time_s,
                    single_person_track_id=single_person_track_id,
                    single_person_id_changed=single_person_id_changed,
                )
                output.flush()

                processed_fps = _fps(steady_count, first_result_s, last_result_s)
                if args.preview or args.save_video is not None:
                    annotated = _annotate(frame, result, processed_fps, total_gap)
                    if args.save_video is not None:
                        if video_writer is None:
                            args.save_video.parent.mkdir(parents=True, exist_ok=True)
                            video_fps = runtime.reported_fps
                            if video_fps is None or video_fps <= 0.0:
                                video_fps = 20.0
                            video_writer = cv2.VideoWriter(
                                str(args.save_video),
                                cv2.VideoWriter_fourcc(*"mp4v"),
                                video_fps,
                                (frame.width, frame.height),
                            )
                            if not video_writer.isOpened():
                                raise RuntimeError(f"could not create video: {args.save_video}")
                        video_writer.write(annotated)
                    if args.preview:
                        cv2.imshow("CompanionBot Stage 7.3 Tracking", annotated)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            runtime.stop.set()
                            stopped_early = True
                            break
                print(
                    f"seq={result.source_sequence_id} detections={result.detection_count} "
                    f"tracks={diagnostics.active_track_count} "
                    f"created={_ids_text(diagnostics.created_track_ids) or '-'} "
                    f"gap={gap} tracker={diagnostics.tracker_update_wall_time_s*1000:.2f}ms "
                    f"age={result.host_receive_to_result_age_s*1000:.1f}ms"
                )
    finally:
        runtime.stop.set()
        producer.join(timeout=10.0)
        upstream_frame_count = backend.upstream_update_count
        tracker.close()
        if video_writer is not None:
            video_writer.release()
        if args.preview:
            cv2.destroyAllWindows()
        if producer.is_alive():
            raise RuntimeError("camera/video source thread did not stop within 10 seconds")

    if runtime.error_value() is not None:
        raise runtime.error_value()

    _write_lifetime_rows(lifetime_csv, lifetimes)
    source = runtime.stats.snapshot()
    trailing_gap = max(
        0,
        int(source["produced_count"])
        - (last_processed_sequence_id + 1 if last_processed_sequence_id is not None else 0),
    )
    total_skipped = total_gap + trailing_gap
    source_fps = _fps(int(source["produced_count"]), source["first_time_s"], source["last_time_s"])
    processed_fps = _fps(steady_count, first_result_s, last_result_s)
    track_lifetimes = [item.last_source_time_s - item.first_source_time_s
                       for item in lifetimes.values()]
    metrics = {
        "source": runtime.description,
        "source_frames": source["produced_count"],
        "processed_tracking_frames": processed_count,
        "steady_state_tracking_frames": steady_count,
        "source_fps_measured": source_fps,
        "processed_fps_steady_state": processed_fps,
        "processed_detection_count": total_detection_count,
        "sequence_gap_skipped_frames_between_updates": total_gap,
        "trailing_unprocessed_source_frames": trailing_gap,
        "total_unprocessed_source_frames": total_skipped,
        "max_sequence_gap": max_gap,
        "mean_sequence_gap": (total_gap / (processed_count - 1)
                              if processed_count > 1 else None),
        "slot_overwritten_frames": slot.overwritten_count,
        "terminal_pending_frames": slot.pending_count,
        "created_track_count": len(lifetimes),
        "active_track_count_at_end": active_track_count_at_end,
        "single_person_observation_frames": single_person_observation_frames,
        "single_person_track_id_changes": single_person_id_changes,
        "lost_track_events": lost_event_count,
        "unique_lost_track_count": sum(
            1 for item in lifetimes.values() if item.lost_event_count > 0
        ),
        "currently_lost_track_count_at_end": sum(
            1 for item in lifetimes.values() if item.final_status == "lost"
        ),
        "removed_track_events": removed_event_count,
        "unique_removed_track_count": sum(1 for item in lifetimes.values() if item.removed),
        "observed_track_lifetime_s_mean": statistics.fmean(track_lifetimes) if track_lifetimes else None,
        "observed_track_lifetime_s_max": max(track_lifetimes) if track_lifetimes else None,
        "detector_wall_ms_mean": statistics.fmean(process_ms) if process_ms else None,
        "detector_wall_ms_median": statistics.median(process_ms) if process_ms else None,
        "detector_wall_ms_p95": _percentile(process_ms, 95.0),
        "tracker_update_wall_ms_mean": statistics.fmean(tracker_ms) if tracker_ms else None,
        "tracker_update_wall_ms_median": statistics.median(tracker_ms) if tracker_ms else None,
        "tracker_update_wall_ms_p95": _percentile(tracker_ms, 95.0),
        "host_receive_to_tracking_result_age_ms_mean": statistics.fmean(age_ms) if age_ms else None,
        "host_receive_to_tracking_result_age_ms_median": statistics.median(age_ms) if age_ms else None,
        "host_receive_to_tracking_result_age_ms_p95": _percentile(age_ms, 95.0),
        "latency_sample_window": len(age_ms),
        "tracker_upstream_frame_count": upstream_frame_count,
        "preview_stopped_early": stopped_early,
        "frame_csv": str(frame_csv.resolve()),
        "lifetime_csv": str(lifetime_csv.resolve()),
        "config_json": str(config_path.resolve()),
    }
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    source_rate_text = "N/A" if source_fps is None else f"{source_fps:.2f} FPS"
    mean_gap_text = ("N/A" if processed_count <= 1 else
                     f"{total_gap / (processed_count - 1):.3f}")
    report_path.write_text(
        "# Stage 7.3 Person Tracking Baseline\n\n"
        f"- Source: `{runtime.description}`; frames {source['produced_count']} at "
        f"{source_rate_text}\n"
        f"- Tracker: Ultralytics {backend.upstream_version} ByteTrack; config "
        f"SHA-256 `{hashlib.sha256(config_bytes).hexdigest()}`\n"
        f"- Processed / steady-state frames: {processed_count} / {steady_count}; "
        f"processed rate {processed_fps if processed_fps is not None else 'N/A'} FPS\n"
        f"- Detection boxes: {total_detection_count}; tracklets first observed: {len(lifetimes)}; "
        f"active at end: {active_track_count_at_end}; unique lost tracklets: "
        f"{sum(1 for item in lifetimes.values() if item.lost_event_count > 0)}; "
        f"lost events: {lost_event_count}; removed tracklets/events: "
        f"{sum(1 for item in lifetimes.values() if item.removed)}/{removed_event_count}\n"
        f"- Unambiguous single-detection/single-track observations: {single_person_observation_frames}; "
        f"track ID changes across those observations: {single_person_id_changes}\n"
        f"- Sequence gaps between tracker updates: total {total_gap}, max {max_gap}, "
        f"mean {mean_gap_text}; trailing skipped frames {trailing_gap}; "
        f"slot overwrites {slot.overwritten_count}\n"
        f"- Tracker update wall mean / median / p95: "
        f"{statistics.fmean(tracker_ms) if tracker_ms else 0.0:.3f} / "
        f"{statistics.median(tracker_ms) if tracker_ms else 0.0:.3f} / "
        f"{_percentile(tracker_ms, 95.0) if tracker_ms else 0.0:.3f} ms\n"
        f"- Read-complete-to-tracking-result age mean / median / p95: "
        f"{statistics.fmean(age_ms) if age_ms else 0.0:.3f} / "
        f"{statistics.median(age_ms) if age_ms else 0.0:.3f} / "
        f"{_percentile(age_ms, 95.0) if age_ms else 0.0:.3f} ms\n"
        "- Timing input to ByteTrack: none. Its Kalman transition uses fixed dt=1 and its "
        "track_buffer is measured in actual tracker update calls, not source sequence or seconds.\n"
        "- No FOV/lost-person decision or camera-motion compensation is present. Leaving and re-entering "
        "the view may create a new tracklet; rapid yaw, pitch, or gimbal motion may degrade association.\n"
        "- Track IDs are temporary tracklets. This run does not perform person identity, Master Lock, or ReID.\n"
        "- Human scenes A-F (stationary, walking, fast lateral motion, occlusion, leaving/re-entering FOV, "
        "optional multiple people) require operator preview and are not claimed as passed by this run.\n"
        f"- Frame log: `{frame_csv.resolve()}`\n"
        f"- Tracklet lifetime summary: `{lifetime_csv.resolve()}`\n"
        f"- Run config / metrics JSON: `{config_path.resolve()}` / `{metrics_path.resolve()}`\n",
        encoding="utf-8",
    )
    print(f"frames={processed_count}; tracks first observed={len(lifetimes)}; "
          f"sequence gaps={total_gap} (+{trailing_gap} trailing); "
          f"tracker update mean={statistics.fmean(tracker_ms) if tracker_ms else 0.0:.3f}ms")
    print(f"report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
