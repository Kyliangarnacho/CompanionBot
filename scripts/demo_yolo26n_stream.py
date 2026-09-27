"""Continuous person-detection demo with a latest-frame-wins input slot.

Use either a local video file or an OpenCV camera source. Video-file timestamps
are host read-complete times; ``--realtime-playback`` simulates playback cadence
from the file's reported FPS and does not turn them into exposure timestamps.
"""

from __future__ import annotations

import argparse
import csv
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time
from typing import Sequence

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.camera import CameraConfig, CameraProfile, CameraStream, ColorFrame
from perception.detection_stream import (
    LatestFrameSlot,
    PersonDetectionFrame,
    process_person_frame,
)
from perception.person_detector import PersonDetector, PersonDetectorConfig


@dataclass
class _SourceStats:
    lock: threading.Lock
    produced_count: int = 0
    first_time_s: float | None = None
    last_time_s: float | None = None
    width: int | None = None
    height: int | None = None

    @classmethod
    def create(cls) -> "_SourceStats":
        return cls(lock=threading.Lock())

    def record(self, frame: ColorFrame) -> None:
        with self.lock:
            if self.produced_count == 0:
                self.first_time_s = frame.host_receive_time_s
            self.produced_count += 1
            self.last_time_s = frame.host_receive_time_s
            if self.width is None:
                self.width, self.height = frame.width, frame.height
            elif (self.width, self.height) != (frame.width, frame.height):
                raise RuntimeError("source frame resolution changed during the stream")

    def snapshot(self) -> dict[str, int | float | None]:
        with self.lock:
            return {
                "produced_count": self.produced_count,
                "first_time_s": self.first_time_s,
                "last_time_s": self.last_time_s,
                "width": self.width,
                "height": self.height,
            }


@dataclass
class _SourceRuntime:
    slot: LatestFrameSlot
    stats: _SourceStats
    ready: threading.Event
    done: threading.Event
    stop: threading.Event
    lock: threading.Lock
    description: str = ""
    reported_fps: float | None = None
    camera_profile: CameraProfile | None = None
    error: BaseException | None = None

    @classmethod
    def create(cls, slot: LatestFrameSlot) -> "_SourceRuntime":
        return cls(slot, _SourceStats.create(), threading.Event(), threading.Event(),
                   threading.Event(), threading.Lock())

    def set_ready(
        self,
        description: str,
        *,
        reported_fps: float | None = None,
        camera_profile: CameraProfile | None = None,
    ) -> None:
        with self.lock:
            self.description = description
            self.reported_fps = reported_fps
            self.camera_profile = camera_profile
        self.ready.set()

    def set_error(self, error: BaseException) -> None:
        with self.lock:
            self.error = error
        self.ready.set()

    def error_value(self) -> BaseException | None:
        with self.lock:
            return self.error


CSV_FIELDS = (
    "source_id", "source_sequence_id", "frame_width", "frame_height", "source_time_s",
    "inference_start_time_s", "inference_end_time_s", "result_ready_time_s",
    "detector_call_wall_time_ms", "processing_wall_time_ms",
    "host_receive_to_result_age_ms", "sequence_gap", "overwritten_frames",
    "steady_state", "person_count", "detection_index", "class_id", "class_name",
    "confidence", "bbox_x1_px", "bbox_y1_px", "bbox_x2_px", "bbox_y2_px",
    "center_x_px", "center_y_px", "width_px", "height_px",
)


def _parse_device(value: str) -> int | str:
    stripped = value.strip()
    if stripped.isdecimal():
        return int(stripped)
    return stripped


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--video", type=Path, help="local video file source")
    source.add_argument("--camera-device", type=_parse_device,
                        help="OpenCV device index or backend-specific source string")
    parser.add_argument("--realtime-playback", action="store_true",
                        help="pace local-file decoding at its reported FPS (simulation)")
    parser.add_argument("--width", type=int, default=None,
                        help="optional requested camera width")
    parser.add_argument("--height", type=int, default=None,
                        help="optional requested camera height")
    parser.add_argument("--fps", type=float, default=None,
                        help="optional requested camera FPS")
    parser.add_argument("--opencv-backend", type=int, default=None,
                        help="optional OpenCV backend ID; unset uses OpenCV selection")
    parser.add_argument("--model", default="yolo26n.pt", help="model path/name")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default=None,
                        help="optional Ultralytics device; unset lets it select")
    parser.add_argument("--warmup-frames", type=int, default=3,
                        help="processed results excluded from steady-state metrics")
    parser.add_argument("--max-source-frames", type=int, default=None,
                        help="optional source-frame limit, useful for finite camera runs")
    parser.add_argument(
        "--output-csv", type=Path,
        default=ROOT / "models" / "minisegway" / "stage7" / "stage7_2" / "results"
                / "person_detection_stream.csv",
    )
    parser.add_argument("--save-video", type=Path, default=None,
                        help="optional annotated video output")
    parser.add_argument("--preview", action="store_true",
                        help="show a local OpenCV window; press q to stop")
    return parser


def _validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.imgsz < 1:
        parser.error("--imgsz must be positive")
    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be in [0, 1]")
    if args.warmup_frames < 0:
        parser.error("--warmup-frames must be nonnegative")
    if args.max_source_frames is not None and args.max_source_frames < 1:
        parser.error("--max-source-frames must be positive")
    if (args.width is None) != (args.height is None):
        parser.error("--width and --height must be supplied together")
    if args.video is not None and not args.video.is_file():
        parser.error(f"video file does not exist: {args.video}")
    if args.realtime_playback and args.video is None:
        parser.error("--realtime-playback applies to --video only")
    if args.preview and not (sys.platform.startswith("win")
                             or sys.platform.startswith("linux")
                             or sys.platform == "darwin"):
        parser.error("OpenCV preview is unavailable on this platform")


def _video_source_thread(
    runtime: _SourceRuntime,
    path: Path,
    realtime_playback: bool,
    max_frames: int | None,
) -> None:
    capture = None
    try:
        capture = cv2.VideoCapture(str(path))
        if not capture.isOpened():
            raise RuntimeError(f"could not open video file: {path}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not math.isfinite(fps) or fps <= 0.0:
            fps = None
        if realtime_playback and fps is None:
            raise RuntimeError("video file does not report a usable FPS for paced playback")
        runtime.set_ready(f"video:{path.resolve()}", reported_fps=fps)

        playback_start_s = time.perf_counter()
        sequence_id = 0
        last_host_receive_ns: int | None = None
        while not runtime.stop.is_set():
            if realtime_playback:
                target_s = playback_start_s + sequence_id / fps
                delay_s = target_s - time.perf_counter()
                if delay_s > 0 and runtime.stop.wait(delay_s):
                    break
            ok, image = capture.read()
            host_receive_ns = time.perf_counter_ns()
            if not ok or image is None:
                break
            if (last_host_receive_ns is not None
                    and host_receive_ns <= last_host_receive_ns):
                host_receive_ns = last_host_receive_ns + 1
            last_host_receive_ns = host_receive_ns
            if (not isinstance(image, np.ndarray) or image.ndim != 3
                    or image.shape[2] != 3 or image.dtype != np.uint8):
                raise RuntimeError("video source did not return H×W×3 uint8 BGR")
            frame = ColorFrame(
                bgr=image,
                sequence_id=sequence_id,
                source_id=f"video:{path.resolve()}",
                width=int(image.shape[1]),
                height=int(image.shape[0]),
                host_receive_time_s=host_receive_ns * 1e-9,
            )
            runtime.stats.record(frame)
            runtime.slot.put(frame)
            sequence_id += 1
            if max_frames is not None and sequence_id >= max_frames:
                break
    except BaseException as error:
        runtime.set_error(error)
    finally:
        if capture is not None:
            capture.release()
        runtime.slot.close()
        runtime.done.set()
        runtime.ready.set()


def _camera_source_thread(
    runtime: _SourceRuntime,
    device: int | str,
    requested_width: int | None,
    requested_height: int | None,
    requested_fps: float | None,
    backend: int | None,
    max_frames: int | None,
) -> None:
    camera = CameraStream(CameraConfig(
        device=device,
        requested_width=requested_width,
        requested_height=requested_height,
        requested_fps=requested_fps,
        backend=backend,
    ))
    try:
        profile = camera.open()
        runtime.set_ready(f"camera:{profile.device_identifier}",
                          reported_fps=profile.reported_fps,
                          camera_profile=profile)
        produced_count = 0
        while not runtime.stop.is_set():
            frame = camera.read()
            runtime.stats.record(frame)
            runtime.slot.put(frame)
            produced_count += 1
            if max_frames is not None and produced_count >= max_frames:
                break
    except BaseException as error:
        runtime.set_error(error)
    finally:
        camera.release()
        runtime.slot.close()
        runtime.done.set()
        runtime.ready.set()


def _source_thread(args: argparse.Namespace, runtime: _SourceRuntime) -> threading.Thread:
    if args.video is not None:
        target = _video_source_thread
        kwargs = (runtime, args.video, args.realtime_playback, args.max_source_frames)
    else:
        target = _camera_source_thread
        kwargs = (runtime, args.camera_device, args.width, args.height, args.fps,
                  args.opencv_backend, args.max_source_frames)
    return threading.Thread(target=target, args=kwargs, name="color-frame-source")


def _annotate(frame: ColorFrame, result: PersonDetectionFrame,
              fps_text: str, skipped: int) -> np.ndarray:
    annotated = frame.bgr.copy()
    for index, detection in enumerate(result.detections):
        x1, y1, x2, y2 = detection.bbox_xyxy_px
        left = max(0, min(frame.width - 1, int(round(x1))))
        top = max(0, min(frame.height - 1, int(round(y1))))
        right = max(0, min(frame.width - 1, int(round(x2))))
        bottom = max(0, min(frame.height - 1, int(round(y2))))
        cv2.rectangle(annotated, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(annotated, f"person {detection.confidence:.2f}",
                    (left, max(18, top - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0), 2)
    overlay = (
        f"seq={result.source_sequence_id} persons={len(result.detections)} "
        f"processed={fps_text} proc={result.processing_wall_time_s*1000:.0f}ms "
        f"age={result.host_receive_to_result_age_s*1000:.0f}ms "
        f"skipped={skipped}"
    )
    cv2.putText(annotated, overlay, (12, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 220, 255), 2)
    return annotated


def _write_frame_rows(
    writer: csv.DictWriter,
    result: PersonDetectionFrame,
    *,
    sequence_gap: int,
    overwritten_count: int,
    steady_state: bool,
) -> None:
    common = {
        "source_id": result.source_id,
        "source_sequence_id": result.source_sequence_id,
        "frame_width": result.frame_width,
        "frame_height": result.frame_height,
        "source_time_s": f"{result.source_time_s:.9f}",
        "inference_start_time_s": f"{result.inference_start_time_s:.9f}",
        "inference_end_time_s": f"{result.inference_end_time_s:.9f}",
        "result_ready_time_s": f"{result.result_ready_time_s:.9f}",
        "detector_call_wall_time_ms": f"{result.detector_call_wall_time_s*1000:.3f}",
        "processing_wall_time_ms": f"{result.processing_wall_time_s*1000:.3f}",
        "host_receive_to_result_age_ms": f"{result.host_receive_to_result_age_s*1000:.3f}",
        "sequence_gap": sequence_gap,
        "overwritten_frames": overwritten_count,
        "steady_state": int(steady_state),
        "person_count": len(result.detections),
    }
    if not result.detections:
        writer.writerow({**common, "detection_index": "", "class_id": "",
                         "class_name": "",
                         "confidence": "", "bbox_x1_px": "", "bbox_y1_px": "",
                         "bbox_x2_px": "", "bbox_y2_px": "", "center_x_px": "",
                         "center_y_px": "", "width_px": "", "height_px": ""})
        return
    for index, detection in enumerate(result.detections):
        x1, y1, x2, y2 = detection.bbox_xyxy_px
        writer.writerow({
            **common,
            "detection_index": index,
            "class_id": detection.class_id,
            "class_name": detection.class_name,
            "confidence": f"{detection.confidence:.6f}",
            "bbox_x1_px": f"{x1:.3f}",
            "bbox_y1_px": f"{y1:.3f}",
            "bbox_x2_px": f"{x2:.3f}",
            "bbox_y2_px": f"{y2:.3f}",
            "center_x_px": f"{(x1+x2)/2:.3f}",
            "center_y_px": f"{(y1+y2)/2:.3f}",
            "width_px": f"{x2-x1:.3f}",
            "height_px": f"{y2-y1:.3f}",
        })


def _fps(count: int, first_time_s: float | None,
         last_time_s: float | None) -> float | None:
    if count < 2 or first_time_s is None or last_time_s is None:
        return None
    duration = last_time_s - first_time_s
    return (count - 1) / duration if duration > 0 else None


def _format_rate(value: float | None) -> str:
    return "N/A" if value is None else f"{value:.2f} fps"


def _percentile(values: deque[float], percentile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)

    detector = PersonDetector(PersonDetectorConfig(
        model_path=args.model,
        imgsz=args.imgsz,
        confidence_threshold=args.conf,
        device=args.device,
    ))
    print(f"detector loaded once: {args.model}; imgsz={args.imgsz}; conf={args.conf}; "
          f"device={args.device if args.device is not None else 'Ultralytics auto'}")

    slot = LatestFrameSlot()
    runtime = _SourceRuntime.create(slot)
    source_thread = _source_thread(args, runtime)
    source_thread.start()
    if not runtime.ready.wait(timeout=15.0):
        runtime.stop.set()
        raise RuntimeError("timed out waiting for source initialization")
    if runtime.error_value() is not None:
        source_thread.join()
        raise runtime.error_value()

    with runtime.lock:
        print(f"source: {runtime.description}")
        if runtime.camera_profile is not None:
            profile = runtime.camera_profile
            print(f"OpenCV backend: {profile.backend_name}")
            print(f"requested profile: {profile.requested_width}x{profile.requested_height}"
                  f"@{profile.requested_fps} fps")
            print(f"OpenCV-reported profile: {profile.reported_width}x"
                  f"{profile.reported_height}@{profile.reported_fps:g} fps")
        elif runtime.reported_fps is not None:
            mode = "paced playback simulation" if args.realtime_playback else "decode speed"
            print(f"video-reported FPS: {runtime.reported_fps:g}; source mode: {mode}")

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    # Keep live-stream memory bounded; CSV preserves the complete per-frame log.
    # Finite clips shorter than this window use every steady-state sample.
    process_ms: deque[float] = deque(maxlen=10_000)
    age_ms: deque[float] = deque(maxlen=10_000)
    result_count = 0
    steady_count = 0
    empty_result_count = 0
    empty_steady_result_count = 0
    person_detection_count = 0
    previous_sequence_id: int | None = None
    total_sequence_gaps = 0
    first_ready_s: float | None = None
    last_ready_s: float | None = None
    video_writer = None
    writer_fps: float | None = None
    output_video_path = args.save_video

    try:
        with args.output_csv.open("w", newline="", encoding="utf-8") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            csv_writer.writeheader()
            while True:
                frame = slot.get(timeout_s=0.2)
                if frame is None:
                    if runtime.done.is_set():
                        break
                    continue

                result = process_person_frame(frame, detector)
                gap = (result.source_sequence_id if previous_sequence_id is None
                       else max(0, result.source_sequence_id - previous_sequence_id - 1))
                previous_sequence_id = result.source_sequence_id
                total_sequence_gaps += gap
                result_count += 1
                steady_state = result_count > args.warmup_frames
                if steady_state:
                    steady_count += 1
                    process_ms.append(result.processing_wall_time_s * 1000.0)
                    age_ms.append(result.host_receive_to_result_age_s * 1000.0)
                    person_detection_count += len(result.detections)
                    if not result.detections:
                        empty_steady_result_count += 1
                    if first_ready_s is None:
                        first_ready_s = result.result_ready_time_s
                    last_ready_s = result.result_ready_time_s
                if not result.detections:
                    empty_result_count += 1
                _write_frame_rows(
                    csv_writer,
                    result,
                    sequence_gap=gap,
                    overwritten_count=slot.overwritten_count,
                    steady_state=steady_state,
                )
                csv_file.flush()

                if args.preview or output_video_path is not None:
                    fps_text = _format_rate(_fps(steady_count, first_ready_s, last_ready_s))
                    annotated = _annotate(frame, result, fps_text, total_sequence_gaps)
                    if output_video_path is not None:
                        if video_writer is None:
                            output_video_path.parent.mkdir(parents=True, exist_ok=True)
                            writer_fps = runtime.reported_fps
                            if writer_fps is None or writer_fps <= 0:
                                writer_fps = 20.0
                            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                            video_writer = cv2.VideoWriter(
                                str(output_video_path), fourcc, writer_fps,
                                (frame.width, frame.height),
                            )
                            if not video_writer.isOpened():
                                raise RuntimeError(
                                    f"could not create output video: {output_video_path}"
                                )
                        video_writer.write(annotated)
                    if args.preview:
                        cv2.imshow("CompanionBot Stage 7.2", annotated)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            runtime.stop.set()
                            break

                print(
                    f"seq={result.source_sequence_id} persons={len(result.detections)} "
                    f"process={result.processing_wall_time_s*1000:.1f}ms "
                    f"age={result.host_receive_to_result_age_s*1000:.1f}ms "
                    f"gap={gap} overwritten={slot.overwritten_count}"
                )
    finally:
        runtime.stop.set()
        source_thread.join(timeout=10.0)
        if video_writer is not None:
            video_writer.release()
        if args.preview:
            cv2.destroyAllWindows()
        if source_thread.is_alive():
            raise RuntimeError("source thread did not stop within 10 seconds")

    if runtime.error_value() is not None:
        raise runtime.error_value()

    source = runtime.stats.snapshot()
    trailing_sequence_gaps = max(
        0,
        int(source["produced_count"])
        - (previous_sequence_id + 1 if previous_sequence_id is not None else 0),
    )
    final_sequence_gaps = total_sequence_gaps + trailing_sequence_gaps
    terminal_unprocessed_frames = slot.pending_count
    source_fps = _fps(
        int(source["produced_count"]), source["first_time_s"], source["last_time_s"]
    )
    processed_fps = _fps(steady_count, first_ready_s, last_ready_s)
    throughput_ratio = (
        processed_fps / source_fps
        if processed_fps is not None and source_fps is not None and source_fps > 0
        else None
    )
    source_frame_count = int(source["produced_count"])
    processed_source_frame_ratio = (
        result_count / source_frame_count if source_frame_count else None
    )
    steady_source_frame_ratio = (
        steady_count / source_frame_count if source_frame_count else None
    )
    skipped_source_frame_ratio = (
        final_sequence_gaps / source_frame_count if source_frame_count else None
    )
    dropout_rate = (empty_steady_result_count / steady_count
                    if steady_count else None)
    metrics = {
        "detector_model": str(args.model),
        "imgsz": args.imgsz,
        "confidence_threshold": args.conf,
        "requested_device": args.device if args.device is not None else "automatic",
        "warmup_results_excluded": args.warmup_frames,
        "processed_results_including_warmup": result_count,
        "steady_state_processed_results": steady_count,
        "source_frames": source["produced_count"],
        "source_fps_measured": source_fps,
        "processed_fps_steady_state": processed_fps,
        "steady_processed_source_fps_ratio": throughput_ratio,
        "processed_source_frame_ratio": processed_source_frame_ratio,
        "steady_processed_source_frame_ratio": steady_source_frame_ratio,
        "sequence_gap_skipped_frames": total_sequence_gaps,
        "trailing_sequence_gap_skipped_frames": trailing_sequence_gaps,
        "total_sequence_gap_skipped_frames": final_sequence_gaps,
        "skipped_source_frame_ratio": skipped_source_frame_ratio,
        "slot_overwritten_frames": slot.overwritten_count,
        "terminal_unprocessed_frames": terminal_unprocessed_frames,
        "empty_detection_results": empty_result_count,
        "empty_steady_state_results": empty_steady_result_count,
        "empty_detection_rate": dropout_rate,
        "steady_state_person_detection_count": person_detection_count,
        "processing_wall_ms_mean": statistics.fmean(process_ms) if process_ms else None,
        "processing_wall_ms_median": statistics.median(process_ms) if process_ms else None,
        "processing_wall_ms_p95": _percentile(process_ms, 95.0),
        "host_receive_to_result_age_ms_mean": statistics.fmean(age_ms) if age_ms else None,
        "host_receive_to_result_age_ms_median": statistics.median(age_ms) if age_ms else None,
        "host_receive_to_result_age_ms_p95": _percentile(age_ms, 95.0),
        "latency_summary_sample_count": len(process_ms),
        "latency_summary_window_max": process_ms.maxlen,
        "reported_source_fps": runtime.reported_fps,
        "output_video": (None if output_video_path is None
                         else str(output_video_path.resolve())),
        "output_video_fps": writer_fps,
    }
    report_path = args.output_csv.with_name("STAGE7_2_STREAM_REPORT.md")
    args.output_csv.with_name("stage7_2_stream_metrics.json").write_text(
        json.dumps(metrics, indent=2), encoding="utf-8"
    )
    process_summary = (
        "N/A" if not process_ms else
        f"{statistics.fmean(process_ms):.2f} / "
        f"{statistics.median(process_ms):.2f} / "
        f"{_percentile(process_ms, 95.0):.2f} ms"
    )
    age_summary = (
        "N/A" if not age_ms else
        f"{statistics.fmean(age_ms):.2f} / "
        f"{statistics.median(age_ms):.2f} / "
        f"{_percentile(age_ms, 95.0):.2f} ms"
    )
    rate_ratio_summary = "N/A" if throughput_ratio is None else f"{throughput_ratio:.3f}"
    result_ratio_summary = (
        "N/A" if processed_source_frame_ratio is None
        else f"{processed_source_frame_ratio:.1%}"
    )
    skipped_ratio_summary = (
        "N/A" if skipped_source_frame_ratio is None
        else f"{skipped_source_frame_ratio:.1%}"
    )
    dropout_summary = "N/A" if dropout_rate is None else f"{dropout_rate:.1%}"
    report_path.write_text(
        "# Stage 7.2 Stream Benchmark\n\n"
        f"- Source: `{runtime.description}`\n"
        f"- Frame profile: {source['width']}×{source['height']} BGR uint8\n"
        f"- Detector: `{args.model}`; imgsz={args.imgsz}; conf={args.conf}; "
        f"device={args.device if args.device is not None else 'automatic'}\n"
        f"- Source frames / measured source rate: {source['produced_count']} / "
        f"{_format_rate(source_fps)}\n"
        f"- Processed results: {result_count} total, {steady_count} steady-state "
        f"after {args.warmup_frames} warmup results\n"
        f"- Steady-state processed FPS: {_format_rate(processed_fps)}\n"
        f"- Processed results/source frames: {result_count}/{source_frame_count} "
        f"({result_ratio_summary})\n"
        f"- Steady-state processed/source FPS ratio: {rate_ratio_summary}\n"
        f"- Sequence gaps / overwritten / final pending: {final_sequence_gaps} / "
        f"{slot.overwritten_count} / {terminal_unprocessed_frames} "
        f"(skipped/source {skipped_ratio_summary})\n"
        f"- Steady-state person detections: {person_detection_count}\n"
        f"- Empty steady-state results: {empty_steady_result_count}/{steady_count} "
        f"({dropout_summary})\n"
        f"- Processing wall mean / median / p95: {process_summary}\n"
        f"- Host receive-to-result age mean / median / p95: {age_summary}\n"
        f"- Latency summary sample window: last {len(process_ms)} steady-state "
        f"results (maximum {process_ms.maxlen})\n"
        "- Frame times use the host monotonic read-complete clock; video playback "
        "pacing is a simulation and does not provide camera exposure timestamps.\n"
        "- Preprocess/inference/postprocess split is unavailable through the "
        "backend-neutral Stage 7.1 detector API; full detector-call and frame-result "
        "wall times are recorded in the CSV.\n"
        f"- Per-frame/detection CSV: `{args.output_csv.resolve()}`\n"
        f"- Metrics JSON: `{args.output_csv.with_name('stage7_2_stream_metrics.json').resolve()}`\n",
        encoding="utf-8",
    )
    print(f"source frames: {source['produced_count']}; measured source rate: "
          f"{_format_rate(source_fps)}")
    print(f"steady-state processed rate: {_format_rate(processed_fps)}; "
          f"steady FPS/source FPS ratio: {rate_ratio_summary}; "
          f"processed results/source frames: {result_ratio_summary}")
    print(f"sequence gaps={final_sequence_gaps} (trailing={trailing_sequence_gaps}); "
          f"slot overwritten={slot.overwritten_count}; "
          f"terminal pending={terminal_unprocessed_frames}; "
          f"empty steady-state results={empty_steady_result_count}/{steady_count}")
    print(f"CSV: {args.output_csv.resolve()}")
    print(f"report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
