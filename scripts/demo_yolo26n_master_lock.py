"""Manual Master selection with Stage 7.6 ReID reacquisition when enabled."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import queue
import sys
from typing import Sequence

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.detection_stream import LatestFrameSlot, process_person_frame
from perception.master_selection import (
    MasterEvent,
    MasterManager,
    MasterSelection,
    MasterSelector,
    MasterState,
)
from perception.person_detector import PersonDetector, PersonDetectorConfig
from perception.person_tracker import PersonTracker, PersonTrackingFrame, TrackedPerson
from perception.person_reid import (
    CandidateScoringFrame,
    MasterReIDEvidence,
    PersonReIdentifier,
    ReIDCandidateScore,
    ReIDInputError,
    TorchReIDOSNetBackend,
    default_osnet_weights_path,
    download_default_osnet_weights,
)
from perception.ultralytics_bytetrack import UltralyticsByteTrackBackend
from scripts import demo_yolo26n_stream as source_stream


REID_ACCEPT_THRESHOLD = 0.68
REID_RETRY_THRESHOLD = 0.50
REID_RETRY_INTERVAL_S = 0.20
REID_REACQUIRE_STABLE_UPDATES = 3


def _device(value: str) -> int | str:
    value = value.strip()
    return int(value) if value.isdecimal() else value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--video", type=Path, help="local video file source")
    sources.add_argument("--camera-device", type=_device,
                         help="OpenCV camera index or source string")
    parser.add_argument("--realtime-playback", action="store_true",
                        help="pace local video playback at its reported FPS")
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--fps", type=float, default=None)
    parser.add_argument("--opencv-backend", type=int, default=None,
                        help="optional OpenCV backend ID")
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.1)
    parser.add_argument("--device", default=None)
    parser.add_argument("--warmup-frames", type=int, default=3)
    parser.add_argument("--max-source-frames", type=int, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--reid", action="store_true",
                        help="enable Stage 7.6 OSNet reference confirmation and auto reacquire")
    parser.add_argument("--reid-weights", type=Path, default=None,
                        help="local TorchReID OSNet x0.25 MSMT17 checkpoint")
    parser.add_argument("--download-reid-weights", action="store_true",
                        help="download the pinned OSNet checkpoint into this .venv cache")
    parser.add_argument("--preview", action="store_true",
                        help="show original-size frames; click a tracked box to select Master")
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
    if args.fps is not None and (not math.isfinite(args.fps) or args.fps <= 0):
        parser.error("--fps must be finite and positive")
    if args.camera_device is not None and isinstance(args.camera_device, int) \
            and args.camera_device < 0:
        parser.error("--camera-device index must be nonnegative")
    if isinstance(args.camera_device, str) and not args.camera_device.strip():
        parser.error("--camera-device must not be empty")
    if args.video is not None and not args.video.is_file():
        parser.error(f"video file does not exist: {args.video}")
    if args.realtime_playback and args.video is None:
        parser.error("--realtime-playback applies to --video only")
    if args.download_reid_weights and not args.reid:
        parser.error("--download-reid-weights requires --reid")
    if args.reid_weights is not None and args.download_reid_weights:
        parser.error("use either --reid-weights or --download-reid-weights")


def _source_args(args: argparse.Namespace) -> argparse.Namespace:
    """Use the same Stage 7.2 source thread and CameraStream owner as Stage 7.3."""
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


def _mouse_callback(event: int, x: int, y: int, _flags: int, clicks: queue.SimpleQueue) -> None:
    """The GUI callback forwards only a click coordinate; it owns no Master state."""
    if event == cv2.EVENT_LBUTTONDOWN:
        clicks.put((x, y))


def _record_event(output, event: MasterEvent) -> None:
    record = asdict(event)
    record["event"] = event.event.value
    if event.similarity is None:
        record.pop("similarity")
    output.write(json.dumps(record, sort_keys=True) + "\n")
    output.flush()
    print(
        f"{event.event.value} seq={event.source_sequence_id} "
        f"old={event.old_track_id} new={event.new_track_id} "
        f"candidates={list(event.candidate_track_ids)}"
    )


def _draw_preview(
    frame,
    tracking_frame: PersonTrackingFrame,
    master_frame,
    processed_fps: float | None,
    click_feedback: str,
    candidate_scores: dict[int, ReIDCandidateScore] | None = None,
    latest_scoring: CandidateScoringFrame | None = None,
    reference_track_id: int | None = None,
    pending_sample_count: int = 0,
):
    candidate_scores = candidate_scores or {}
    image = frame.bgr.copy()
    master_id = master_frame.bound_track_id
    for track in tracking_frame.tracks:
        x1, y1, x2, y2 = track.bbox_xyxy_px
        left = max(0, min(frame.width - 1, int(round(x1))))
        top = max(0, min(frame.height - 1, int(round(y1))))
        right = max(0, min(frame.width - 1, int(round(x2))))
        bottom = max(0, min(frame.height - 1, int(round(y2))))
        is_master = master_frame.state is MasterState.LOCKED and track.track_id == master_id
        color = (255, 0, 255) if is_master else (0, 220, 40)
        thickness = 3 if is_master else 2
        cv2.rectangle(image, (left, top), (right, bottom), color, thickness)
        label = f"MASTER | ID {track.track_id}" if is_master else f"ID {track.track_id}"
        if not is_master and track.track_id in candidate_scores:
            score = candidate_scores[track.track_id]
            label += f" | ReID {score.similarity:.2f} s{score.source_sequence_id}"
        cv2.putText(image, label, (left, max(18, top - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    fps_text = "N/A" if processed_fps is None else f"{processed_fps:.2f}"
    bound_text = "none" if master_id is None else str(master_id)
    age_ms = tracking_frame.host_receive_to_result_age_s * 1000.0
    if master_frame.state is MasterState.PENDING_LOCK:
        state_text = (
            f"MASTER PENDING_LOCK ID {master_id} | reference samples {pending_sample_count}/3"
        )
    elif master_frame.state is MasterState.LOST and reference_track_id is not None:
        state_text = f"MASTER LOST | reference from ID {reference_track_id}"
    elif master_frame.state is MasterState.LOCKED and reference_track_id is not None:
        state_text = f"MASTER LOCKED ID {master_id}"
    else:
        state_text = f"Master={master_frame.state.value} bound={bound_text}"
    cv2.putText(
        image,
        f"{state_text} | seq={tracking_frame.source_sequence_id} fps={fps_text} "
        f"age={age_ms:.0f}ms tracks={len(tracking_frame.tracks)}",
        (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2,
    )
    if latest_scoring is not None and latest_scoring.scores:
        first = latest_scoring.top1
        margin = latest_scoring.top1_top2_margin
        cv2.putText(
            image,
            f"Top candidate: ID {first.candidate_track_id} "
            f"{first.similarity:.2f} | Margin: "
            f"{'N/A' if margin is None else f'{margin:.2f}'} "
            f"(scored seq {latest_scoring.source_sequence_id})",
            (12, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2,
        )
    if click_feedback:
        cv2.putText(image, click_feedback, (12, 76 if latest_scoring else 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
    return image


def _process_click(
    click: tuple[int, int],
    tracking_frame: PersonTrackingFrame,
    selector: MasterSelector,
    manager: MasterManager,
    *,
    pending_lock: bool = False,
) -> tuple[MasterSelection, MasterEvent | None]:
    x_px, y_px = click
    selection = selector.select_at_pixel(tracking_frame, x_px, y_px)
    if selection.selected_track_id is None:
        return selection, None
    lock_method = manager.begin_pending_lock if pending_lock else manager.lock
    event = lock_method(
        tracking_frame,
        selection.selected_track_id,
        candidate_track_ids=selection.candidate_track_ids,
        click_x_px=x_px,
        click_y_px=y_px,
    )
    return selection, event


def _record_reference(output, reference) -> None:
    record = {
        "event": "MASTER_REFERENCE_CREATED",
        "source_id": reference.source_id,
        "source_sequence_id": reference.source_sequence_id,
        "source_time_s": reference.source_time_s,
        "bound_track_at_selection": reference.bound_track_at_selection,
        "backend": reference.embedding.backend_name,
        "model": reference.embedding.model_name,
        "embedding_dimension": reference.embedding.dimension,
        "reference_sample_count": len(reference.sample_sequence_ids),
        "reference_sample_sequence_ids": reference.sample_sequence_ids,
        "preprocess_wall_time_s": reference.embedding.preprocess_wall_time_s,
        "inference_wall_time_s": reference.embedding.inference_wall_time_s,
        "total_embedding_wall_time_s": reference.embedding.total_wall_time_s,
    }
    output.write(json.dumps(record, sort_keys=True) + "\n")
    output.flush()
    print(
        f"MASTER_REFERENCE_CREATED seq={reference.source_sequence_id} "
        f"track={reference.bound_track_at_selection} "
        f"model={reference.embedding.model_name} "
        f"dim={reference.embedding.dimension}"
    )


def _record_candidate_scores(output, result: CandidateScoringFrame) -> None:
    if not result.scores and not result.rejected:
        return
    record = {
        "event": "REID_CANDIDATE_SCORING",
        "source_id": result.source_id,
        "source_sequence_id": result.source_sequence_id,
        "source_time_s": result.source_time_s,
        "reference_track_id": result.reference_track_id,
        "master_state": result.master_state.value,
        "candidate_count": len(result.scores) + len(result.rejected),
        "scored_candidate_count": len(result.scores),
        "rejected_candidate_count": len(result.rejected),
        "top1_similarity": None if result.top1 is None else result.top1.similarity,
        "top2_similarity": None if result.top2 is None else result.top2.similarity,
        "top1_top2_margin": result.top1_top2_margin,
        "scores": [
            {
                "candidate_track_id": item.candidate_track_id,
                "bbox_xyxy_px": item.bbox_xyxy_px,
                "similarity": item.similarity,
                "cosine_distance": item.cosine_distance,
                "ranking": item.rank,
                "retry_count": item.retry_count,
                "preprocess_wall_time_s": item.preprocess_wall_time_s,
                "inference_wall_time_s": item.inference_wall_time_s,
                "total_embedding_wall_time_s": item.total_embedding_wall_time_s,
                "master_state": item.master_state.value,
            }
            for item in result.scores
        ],
        "rejected_candidates": [
            {
                "candidate_track_id": item.candidate_track_id,
                "bbox_xyxy_px": item.bbox_xyxy_px,
                "reason": item.reason,
            }
            for item in result.rejected
        ],
        "automatic_master_rebind": True,
    }
    output.write(json.dumps(record, sort_keys=True) + "\n")
    output.flush()
    for item in result.scores:
        print(
            f"ReID rank={item.rank} ID={item.candidate_track_id} "
            f"similarity={item.similarity:.4f} "
            f"distance={item.cosine_distance:.4f}"
        )
    if result.top1 is not None:
        margin = "N/A" if result.top1_top2_margin is None else f"{result.top1_top2_margin:.4f}"
        print(f"ReID top1={result.top1.candidate_track_id} margin={margin}")


def _format_reid_summary(enabled: bool, evidence, reid_path: Path) -> str:
    if evidence is None:
        return ""
    return (
        "- ReID evidence enabled: `True`\n"
        f"- ReID model/backend: `{evidence.model_name}` / `"
        f"{evidence.backend_name}`\n"
        f"- Automatic Master rebind: `{enabled}`; accept threshold: "
        f"`{REID_ACCEPT_THRESHOLD}`; retry threshold: `{REID_RETRY_THRESHOLD}` "
        "(both provisional engineering thresholds; no different-person negative calibration)\n"
        f"- Reference/candidate log: `{reid_path.resolve()}`\n"
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)

    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".venv"))
    detector = PersonDetector(PersonDetectorConfig(
        model_path=args.model,
        imgsz=args.imgsz,
        confidence_threshold=args.conf,
        device=args.device,
    ))
    backend = UltralyticsByteTrackBackend()
    tracker = PersonTracker(backend)
    selector = MasterSelector()
    manager = MasterManager()
    reid_evidence: MasterReIDEvidence | None = None
    weights_path = args.reid_weights or default_osnet_weights_path()
    if args.reid:
        if args.download_reid_weights:
            weights_path = download_default_osnet_weights(weights_path)
        reid_evidence = MasterReIDEvidence(PersonReIdentifier(
            TorchReIDOSNetBackend(weights_path)
        ))
    print(f"detector: {args.model}; imgsz={args.imgsz}; conf={args.conf}; "
          f"device={args.device if args.device is not None else 'Ultralytics auto'}")
    print(f"tracker: {tracker.backend_name}; Ultralytics {backend.upstream_version}")
    if reid_evidence is not None:
        print(f"ReID enabled; backend={reid_evidence.backend_name}; "
              f"model={reid_evidence.model_name}; "
              f"weights={weights_path}")

    output_dir = args.output_dir or (
        ROOT / "models" / "minisegway" / "stage7" / ("stage7_6" if args.reid else "stage7_4")
        / "results"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    event_path = output_dir / "master_lifecycle_events.jsonl"
    reid_path = output_dir / "master_reid_evidence.jsonl"
    config_path = output_dir / ("stage7_6_run_config.json" if args.reid
                                else "stage7_4_run_config.json")
    report_path = output_dir / ("STAGE7_6_MASTER_REID_RUN_REPORT.md" if args.reid
                                else "STAGE7_4_MASTER_LOCK_RUN_REPORT.md")
    config_path.write_text(json.dumps({
        "detector": {
            "model": str(args.model),
            "imgsz": args.imgsz,
            "confidence_threshold": args.conf,
            "device": args.device if args.device is not None else "automatic",
        },
        "tracker_backend": tracker.backend_name,
        "ultralytics_version": backend.upstream_version,
        "source_request": {
            "video": None if args.video is None else str(args.video),
            "camera_device": args.camera_device,
            "requested_width": args.width,
            "requested_height": args.height,
            "requested_fps": args.fps,
            "opencv_backend_id": args.opencv_backend,
        },
        "master_selection": (
            "manual click on current tracked bbox; PENDING_LOCK until three exact-frame embeddings span 0.3 seconds"
            if args.reid else "manual click on current tracked bbox"
        ),
        "preview_geometry": "original source frame; no resize, letterbox, mirror, or crop",
        "click_coordinates": "integer original-frame pixels; x right, y down",
        "source_time_semantics": "ColorFrame.host_receive_time_s (host read-complete)",
        "automatic_identity_reassociation": args.reid,
        "stage7_6_reid": {
            "enabled": args.reid,
            "backend": (None if reid_evidence is None
                        else reid_evidence._reidentifier.backend_name),
            "model": (None if reid_evidence is None
                      else reid_evidence._reidentifier.model_name),
            "weights": str(weights_path) if args.reid else None,
            "similarity": "cosine similarity of L2-normalized embeddings",
            "reference": {
                "pending_lock_min_duration_s": 0.30,
                "valid_exact_frame_embeddings": 3,
                "sampling_offsets_s": [0.0, 0.15, 0.30],
                "aggregation": "arithmetic mean followed by L2 normalization",
                "embedding_dimension": 512,
            },
            "auto_reacquire": {
                "enabled": args.reid,
                "reid_accept_threshold": REID_ACCEPT_THRESHOLD,
                "reid_retry_threshold": REID_RETRY_THRESHOLD,
                "threshold_status": "both are provisional engineering thresholds; no different-person negative calibration has been performed",
                "minimum_stable_tracker_updates": REID_REACQUIRE_STABLE_UPDATES,
                "retry_interval_s": REID_RETRY_INTERVAL_S,
                "candidate_score_policy": "first score after three stable tracker updates; retry current-frame score only while prior score is in [retry, accept)",
                "rebind_authority": "MasterManager after threshold and update-count confirmation",
            },
            "candidate_policy": "one score per active temporary track ID per LOST episode",
            "embedding_vectors_logged": False,
        },
    }, indent=2), encoding="utf-8")

    slot = LatestFrameSlot()
    runtime = source_stream._SourceRuntime.create(slot)
    producer = source_stream._source_thread(_source_args(args), runtime)
    producer.start()
    if not runtime.ready.wait(timeout=15.0):
        runtime.stop.set()
        raise RuntimeError("timed out waiting for source initialization")
    if runtime.error_value() is not None:
        producer.join()
        raise runtime.error_value()
    print(f"source: {runtime.description}")
    if runtime.camera_profile is not None:
        profile = runtime.camera_profile
        print(f"OpenCV backend: {profile.backend_name}; requested="
              f"{profile.requested_width}x{profile.requested_height}@"
              f"{profile.requested_fps} fps; reported="
              f"{profile.reported_width}x{profile.reported_height}@"
              f"{profile.reported_fps:g} fps")

    clicks: queue.SimpleQueue = queue.SimpleQueue()
    window_name = ("CompanionBot Stage 7.6 ReID Auto Reacquire" if args.reid
                   else "CompanionBot Stage 7.4 Master Lock")
    if args.preview:
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, _mouse_callback, clicks)

    event_counts: Counter[str] = Counter()
    processed = 0
    first_ready_s: float | None = None
    last_ready_s: float | None = None
    stopped_early = False
    click_feedback = ""
    candidate_similarity_by_track: dict[int, ReIDCandidateScore] = {}
    latest_scoring: CandidateScoringFrame | None = None

    try:
        with ExitStack() as outputs:
            events_output = outputs.enter_context(event_path.open("w", encoding="utf-8"))
            reid_output = (outputs.enter_context(reid_path.open("w", encoding="utf-8"))
                          if reid_evidence is not None else None)
            while True:
                frame = slot.get(timeout_s=0.2)
                if frame is None:
                    if runtime.done.is_set():
                        break
                    continue
                detection_frame = process_person_frame(frame, detector)
                tracking_frame = tracker.update(detection_frame)
                for event in manager.update(tracking_frame):
                    _record_event(events_output, event)
                    event_counts[event.event.value] += 1
                    if (reid_evidence is not None
                            and event.event.value == "MASTER_PENDING_LOCK_CANCELLED"):
                        reid_evidence.cancel_pending_reference()
                        click_feedback = "pending selection cancelled; Master unselected"

                processed += 1
                if processed > args.warmup_frames:
                    if first_ready_s is None:
                        first_ready_s = tracking_frame.result_ready_time_s
                    last_ready_s = tracking_frame.result_ready_time_s
                duration = (None if first_ready_s is None or last_ready_s is None
                            else last_ready_s - first_ready_s)
                processed_fps = (
                    None if duration is None or duration <= 0.0
                    else (processed - args.warmup_frames - 1) / duration
                )

                if (reid_evidence is not None
                        and manager.state is MasterState.PENDING_LOCK):
                    try:
                        reference = reid_evidence.collect_pending_reference_sample(
                            frame, tracking_frame
                        )
                    except ReIDInputError as error:
                        rejected = {
                            "event": "MASTER_REFERENCE_SAMPLE_REJECTED",
                            "source_id": frame.source_id,
                            "source_sequence_id": frame.sequence_id,
                            "selected_track_id": manager.bound_track_id,
                            "valid_samples_collected": reid_evidence.pending_sample_count,
                            "reason": str(error),
                        }
                        reid_output.write(json.dumps(rejected, sort_keys=True) + "\n")
                        reid_output.flush()
                    else:
                        if reference is not None:
                            event = manager.confirm_pending_lock(tracking_frame)
                            _record_event(events_output, event)
                            event_counts[event.event.value] += 1
                            _record_reference(reid_output, reference)
                            click_feedback = (
                                f"selected ID {reference.bound_track_at_selection}; LOCKED"
                            )

                master_frame = manager.result(tracking_frame)
                if reid_evidence is not None:
                    if master_frame.state is not MasterState.LOST:
                        reid_evidence.score_candidates(
                            frame,
                            tracking_frame,
                            master_frame,
                            stable_candidate_track_ids=(),
                        )
                        candidate_similarity_by_track.clear()
                        latest_scoring = None
                    else:
                        stable_candidate_ids = manager.stable_reid_candidate_ids(
                            tracking_frame,
                            stable_updates=REID_REACQUIRE_STABLE_UPDATES,
                        )
                        new_scoring = reid_evidence.score_candidates(
                            frame,
                            tracking_frame,
                            master_frame,
                            stable_candidate_track_ids=stable_candidate_ids,
                            accept_threshold=REID_ACCEPT_THRESHOLD,
                            retry_threshold=REID_RETRY_THRESHOLD,
                            retry_interval_s=REID_RETRY_INTERVAL_S,
                        )
                        if new_scoring is not None:
                            _record_candidate_scores(reid_output, new_scoring)
                            candidate_similarity_by_track.update({
                                item.candidate_track_id: item
                                for item in new_scoring.scores
                            })
                            if new_scoring.scores:
                                latest_scoring = new_scoring
                        reacquired = manager.consider_reid_candidate_scores(
                            tracking_frame,
                            {
                                item.candidate_track_id: item.similarity
                                for item in reid_evidence.lost_episode_candidate_scores
                            },
                            threshold=REID_ACCEPT_THRESHOLD,
                            stable_updates=REID_REACQUIRE_STABLE_UPDATES,
                        )
                        if reacquired is not None:
                            _record_event(events_output, reacquired)
                            event_counts[reacquired.event.value] += 1
                            click_feedback = (
                                f"ReID reacquired as ID {reacquired.new_track_id} "
                                f"({reacquired.similarity:.2f})"
                            )
                            candidate_similarity_by_track.clear()
                            latest_scoring = None
                        master_frame = manager.result(tracking_frame)
                        active_track_ids = {track.track_id for track in tracking_frame.tracks}
                        candidate_similarity_by_track = {
                            track_id: score
                            for track_id, score in candidate_similarity_by_track.items()
                            if track_id in active_track_ids
                        }
                if args.preview:
                    if (frame.width, frame.height) != (
                        tracking_frame.frame_width, tracking_frame.frame_height
                    ):
                        raise RuntimeError("preview and tracking source-frame dimensions differ")
                    image = _draw_preview(
                        frame,
                        tracking_frame,
                        master_frame,
                        processed_fps,
                        click_feedback,
                        candidate_scores=candidate_similarity_by_track,
                        latest_scoring=latest_scoring,
                        reference_track_id=(None if reid_evidence is None
                                            or reid_evidence.reference is None
                                            else reid_evidence.reference.bound_track_at_selection),
                        pending_sample_count=(0 if reid_evidence is None
                                              else reid_evidence.pending_sample_count),
                    )
                    cv2.imshow(window_name, image)
                    key = cv2.waitKey(1) & 0xFF
                    click_feedback = ""
                    while True:
                        try:
                            click = clicks.get_nowait()
                        except queue.Empty:
                            break
                        selection, event = _process_click(
                            click, tracking_frame, selector, manager,
                            pending_lock=reid_evidence is not None,
                        )
                        if selection.selected_track_id is None:
                            click_feedback = "No tracked box at click"
                            print(f"selection miss seq={tracking_frame.source_sequence_id} "
                                  f"pixel=({click[0]},{click[1]}) candidates=[]")
                        elif event is not None:
                            _record_event(events_output, event)
                            event_counts[event.event.value] += 1
                        if selection.selected_track_id is not None:
                            candidate_similarity_by_track.clear()
                            latest_scoring = None
                            if reid_evidence is not None:
                                try:
                                    reid_evidence.begin_pending_reference(
                                        frame, tracking_frame, selection.selected_track_id
                                    )
                                    reference = reid_evidence.collect_pending_reference_sample(
                                        frame, tracking_frame
                                    )
                                except ReIDInputError as error:
                                    failed = {
                                        "event": "MASTER_REFERENCE_SAMPLE_REJECTED",
                                        "source_id": frame.source_id,
                                        "source_sequence_id": frame.sequence_id,
                                        "selected_track_id": selection.selected_track_id,
                                        "valid_samples_collected": reid_evidence.pending_sample_count,
                                        "reason": str(error),
                                    }
                                    reid_output.write(json.dumps(failed, sort_keys=True) + "\n")
                                    reid_output.flush()
                                    click_feedback = (
                                        f"selected ID {selection.selected_track_id}; PENDING_LOCK"
                                    )
                                else:
                                    if reference is not None:
                                        event = manager.confirm_pending_lock(tracking_frame)
                                        _record_event(events_output, event)
                                        event_counts[event.event.value] += 1
                                        _record_reference(reid_output, reference)
                                        click_feedback = (
                                            f"selected ID {selection.selected_track_id}; LOCKED"
                                        )
                                    else:
                                        click_feedback = (
                                            f"selected ID {selection.selected_track_id}; "
                                            f"PENDING_LOCK samples="
                                            f"{reid_evidence.pending_sample_count}/3"
                                        )
                            else:
                                click_feedback = (
                                    f"selected ID {selection.selected_track_id}; "
                                    f"candidates={list(selection.candidate_track_ids)}"
                                )
                    if key in (ord("c"), ord("r")):
                        event = manager.clear(tracking_frame)
                        if event is not None:
                            _record_event(events_output, event)
                            event_counts[event.event.value] += 1
                        if reid_evidence is not None:
                            reid_evidence.clear_reference()
                            candidate_similarity_by_track.clear()
                            latest_scoring = None
                        click_feedback = "Master cleared"
                    elif key == ord("q"):
                        runtime.stop.set()
                        stopped_early = True
                        break

                print(
                    f"seq={tracking_frame.source_sequence_id} "
                    f"master={manager.state.value}:{manager.bound_track_id} "
                    f"tracks={len(tracking_frame.tracks)} "
                    f"age={tracking_frame.host_receive_to_result_age_s * 1000.0:.1f}ms"
                )
    finally:
        runtime.stop.set()
        producer.join(timeout=10.0)
        tracker.close()
        if args.preview:
            cv2.destroyAllWindows()
        if producer.is_alive():
            raise RuntimeError("camera/video source thread did not stop within 10 seconds")

    if runtime.error_value() is not None:
        raise runtime.error_value()
    stage_title = (
        "Stage 7.6 ReID Auto Reacquire Run" if args.reid
        else "Stage 7.4 Manual Master Lock Run"
    )
    reid_summary = _format_reid_summary(args.reid, reid_evidence, reid_path)
    report_path.write_text(
        f"# {stage_title}\n\n"
        f"- Source: `{runtime.description}`\n"
        f"- Tracking frames processed: {processed}\n"
        f"- Final Master state: `{manager.state.value}`; bound track ID: "
        f"`{manager.bound_track_id}`\n"
        f"- Preview stopped early: `{stopped_early}`\n"
        f"- Lifecycle events: `{dict(event_counts)}`\n"
        "- Manual human acceptance is not inferred from this run report; the operator must perform the scenes.\n"
        f"{reid_summary}"
        f"- Event log: `{event_path.resolve()}`\n"
        f"- Run config: `{config_path.resolve()}`\n",
        encoding="utf-8",
    )
    print(f"report: {report_path.resolve()}")
    print(f"events: {event_path.resolve()}")
    if args.reid:
        print(f"ReID evidence: {reid_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
