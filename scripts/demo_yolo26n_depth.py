"""Stage 7 perception RGB demo and independent depth/detector benchmarks.

One Stage 7.2 source owns VideoCapture. Its frames fan out to independent
capacity-one slots, so neither inference channel can queue old camera frames.
``--preview`` opens one user-operated RGB window; headless runs record data.
"""

from __future__ import annotations

import argparse
from collections import OrderedDict, deque
import csv
from dataclasses import asdict, dataclass, field
import json
import os
from pathlib import Path
import re
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

from perception.camera import ColorFrame
from perception.camera_calibration import load_camera_calibration, validate_calibration_resolution
from perception.depth import DepthFrame, YoloDepthAdapter, YoloDepthConfig
from perception.detection_stream import LatestFrameSlot, process_person_frame
from perception.master_depth import (
    MasterDepthSample, back_project_camera_point, representative_master_depth,
    torso_roi_xyxy,
)
from perception.master_selection import (
    MasterManager, MasterSelector, MasterState, MasterTrackingFrame,
)
from perception.master_reid_session import MasterPerceptionResult, MasterReIDSession
from perception.person_reid import (
    MasterReIDEvidence, PersonReIdentifier, TorchReIDOSNetBackend, default_osnet_weights_path,
)
from perception.robot_geometry import (
    CameraExtrinsic, ConstantPitchProvider, camera_to_robot_observation,
)
from perception.person_detector import PersonDetector, PersonDetectorConfig
from perception.person_tracker import PersonTracker, PersonTrackingFrame
from perception.ultralytics_bytetrack import UltralyticsByteTrackBackend
from scripts import demo_yolo26n_stream as source_stream


DEFAULT_DEPTH_MODEL = ROOT / ".venv" / "models" / "yolo26n-depth.pt"
DEFAULT_OUTPUT = ROOT / "models" / "minisegway" / "stage7" / "stage7_7" / "results"
FINAL_CONFIG = ROOT / "models" / "minisegway" / "stage7" / "config" / "final_demo.json"


class _FrameFanout:
    """Offer the same source frame to independent latest-frame slots."""

    def __init__(self, slots: tuple[LatestFrameSlot, ...], calibration=None):
        self.slots = slots
        self.calibration = calibration

    def put(self, frame: ColorFrame) -> None:
        if self.calibration is not None:
            validate_calibration_resolution(self.calibration, frame.width, frame.height)
        for slot in self.slots:
            slot.put(frame)

    def close(self) -> None:
        for slot in self.slots:
            slot.close()


@dataclass
class _Channel:
    slot: LatestFrameSlot
    done: threading.Event = field(default_factory=threading.Event)
    lock: threading.Lock = field(default_factory=threading.Lock)
    rows: list[dict] = field(default_factory=list)
    latest: tuple[ColorFrame, object] | None = None
    recent: deque = field(default_factory=lambda: deque(maxlen=16))
    error: BaseException | None = None

    def publish(self, frame: ColorFrame, result: object, row: dict) -> None:
        with self.lock:
            self.rows.append(row)
            self.latest = (frame, result)
            self.recent.append((frame, result, row))

    def snapshot(self) -> tuple[tuple[ColorFrame, object] | None, list[dict]]:
        with self.lock:
            return self.latest, list(self.rows)

    def completed_after(self, sequence_id: int) -> list[tuple[ColorFrame, object, dict]]:
        """Bounded completed-result view; worker input remains latest-frame-wins."""
        with self.lock:
            return [item for item in self.recent if item[0].sequence_id > sequence_id]


@dataclass(frozen=True)
class _PairedFrame:
    color: ColorFrame
    depth: DepthFrame
    depth_row: dict
    tracking: PersonTrackingFrame
    master: MasterTrackingFrame


class _SameFramePairer:
    """Pair completed slow-channel results, retaining only a few recent frames."""

    def __init__(self, capacity: int = 16) -> None:
        self.capacity = capacity
        self._depth: OrderedDict[tuple, tuple[ColorFrame, DepthFrame, dict]] = OrderedDict()
        self._tracking: OrderedDict[tuple, tuple[ColorFrame, PersonTrackingFrame, MasterTrackingFrame]] = OrderedDict()

    @staticmethod
    def _key(frame: ColorFrame) -> tuple:
        return (frame.source_id, frame.sequence_id, frame.width, frame.height,
                frame.host_receive_time_s)

    def add_depth(self, frame: ColorFrame, depth: DepthFrame, row: dict) -> _PairedFrame | None:
        key = self._key(frame)
        if key != (depth.source_id, depth.source_sequence_id, depth.width,
                   depth.height, depth.source_time_s):
            raise ValueError("depth result source metadata differs from ColorFrame")
        self._depth[key] = (frame, depth, row)
        self._trim(self._depth)
        return self._match(key)

    def add_tracking(self, frame: ColorFrame, tracking: PersonTrackingFrame,
                     master: MasterTrackingFrame) -> _PairedFrame | None:
        key = self._key(frame)
        for result in (tracking, master):
            if key != (result.source_id, result.source_sequence_id,
                       result.frame_width, result.frame_height, result.source_time_s):
                raise ValueError("tracking/Master source metadata differs from ColorFrame")
        self._tracking[key] = (frame, tracking, master)
        self._trim(self._tracking)
        return self._match(key)

    def _match(self, key: tuple) -> _PairedFrame | None:
        if key not in self._depth or key not in self._tracking:
            return None
        color, depth, row = self._depth.pop(key)
        _, tracking, master = self._tracking.pop(key)
        return _PairedFrame(color, depth, row, tracking, master)

    def _trim(self, pending: OrderedDict) -> None:
        while len(pending) > self.capacity:
            pending.popitem(last=False)


def _device(value: str) -> int | str:
    return int(value) if value.isdecimal() else value


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--camera-device", type=_device)
    sources.add_argument("--video", type=Path)
    parser.add_argument("--mode", choices=("depth", "detector", "concurrent", "full"), default="depth")
    parser.add_argument("--config", type=Path, default=FINAL_CONFIG,
                        help="full-mode geometry config; asset paths are relative to this file")
    parser.add_argument("--reid-weights", type=Path, default=None)
    parser.add_argument("--select-track-id", type=int, default=None,
                        help="explicit one-time Master selection by ID, also usable headlessly")
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--opencv-backend", type=int)
    parser.add_argument("--realtime-playback", action="store_true")
    parser.add_argument("--max-source-frames", type=int)
    parser.add_argument("--calibration", type=Path,
                        help="optional exact-resolution NPZ/OpenCV YAML K/D")
    parser.add_argument("--depth-model", type=Path, default=DEFAULT_DEPTH_MODEL)
    parser.add_argument("--depth-imgsz", type=int, default=768)
    parser.add_argument("--detector-model", default="yolo26n.pt")
    parser.add_argument("--detector-imgsz", type=int, default=640)
    parser.add_argument("--detector-conf", type=float, default=0.1)
    parser.add_argument("--device", default=None, help="optional Ultralytics device")
    parser.add_argument("--warmup-results", type=int, default=2)
    parser.add_argument("--run-name", default=None)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--compare-detector-baseline", type=Path,
                        help="detector-only metrics JSON for same-profile FPS/latency delta")
    parser.add_argument("--preview", action="store_true")
    return parser


def _source_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        video=args.video, realtime_playback=args.realtime_playback,
        max_source_frames=args.max_source_frames,
        camera_device=args.camera_device, width=args.width, height=args.height,
        fps=args.fps, opencv_backend=args.opencv_backend,
    )


def _consume(channel: _Channel, runtime, operation) -> None:
    try:
        while not runtime.stop.is_set():
            frame = channel.slot.get(timeout_s=0.1)
            if frame is None:
                if runtime.done.is_set():
                    break
                continue
            result, row = operation(frame)
            channel.publish(frame, result, row)
    except BaseException as error:
        channel.error = error
        runtime.stop.set()
    finally:
        channel.done.set()


def _depth_operation(adapter: YoloDepthAdapter, frame: ColorFrame):
    start_s = time.perf_counter_ns() * 1e-9
    result = adapter.process(frame)
    ready_s = time.perf_counter_ns() * 1e-9
    if (result.source_id, result.source_sequence_id, result.source_time_s,
            result.width, result.height) != (
            frame.source_id, frame.sequence_id, frame.host_receive_time_s,
            frame.width, frame.height):
        raise RuntimeError("depth result does not match its inference ColorFrame")
    return result, {
        "source_sequence_id": frame.sequence_id,
        "source_time_s": frame.host_receive_time_s,
        "width": frame.width, "height": frame.height,
        "ready_time_s": ready_s,
        "inference_adapter_wall_ms": (ready_s - start_s) * 1000,
        "result_age_ms": (ready_s - frame.host_receive_time_s) * 1000,
        "valid_fraction": float(np.isfinite(result.depth_m).mean()),
    }


def _detector_operation(detector: PersonDetector, tracker: PersonTracker,
                        frame: ColorFrame, session: MasterReIDSession | None = None):
    detections = process_person_frame(frame, detector)
    tracked = tracker.update(detections)
    if (tracked.source_id, tracked.source_sequence_id, tracked.source_time_s,
            tracked.frame_width, tracked.frame_height) != (
            frame.source_id, frame.sequence_id, frame.host_receive_time_s,
            frame.width, frame.height):
        raise RuntimeError("tracking result does not match its inference ColorFrame")
    result = tracked if session is None else session.process(frame, tracked)
    ready_s = time.perf_counter_ns() * 1e-9
    return result, {
        "source_sequence_id": frame.sequence_id,
        "source_time_s": frame.host_receive_time_s,
        "width": frame.width, "height": frame.height,
        "ready_time_s": ready_s,
        "detector_wall_ms": detections.detector_call_wall_time_s * 1000,
        "tracker_wall_ms": tracked.diagnostics.tracker_update_wall_time_s * 1000,
        "result_age_ms": (ready_s - frame.host_receive_time_s) * 1000,
        "person_count": len(detections.detections),
        "track_count": len(tracked.tracks),
    }


def _rate(rows: list[dict]) -> float | None:
    if len(rows) < 2:
        return None
    duration = rows[-1]["ready_time_s"] - rows[0]["ready_time_s"]
    return (len(rows) - 1) / duration if duration > 0 else None


def _summarize(rows: list[dict], warmup: int) -> dict:
    steady = rows[warmup:]
    summary = {"total_results": len(rows), "steady_results": len(steady),
               "warmup_excluded": min(warmup, len(rows)), "effective_hz": _rate(steady)}
    for field_name in ("inference_adapter_wall_ms", "detector_wall_ms",
                       "tracker_wall_ms", "result_age_ms"):
        values = [row[field_name] for row in steady if field_name in row]
        if values:
            summary[field_name + "_median"] = statistics.median(values)
            summary[field_name + "_p95"] = float(np.percentile(values, 95))
    if rows:
        summary["first_source_sequence_id"] = rows[0]["source_sequence_id"]
        summary["last_source_sequence_id"] = rows[-1]["source_sequence_id"]
    return summary


def _overlay(image: np.ndarray, lines: list[str]) -> np.ndarray:
    shown = image.copy()
    for index, line in enumerate(lines):
        y = 25 + index * 27
        cv2.putText(shown, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 4)
        cv2.putText(shown, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2)
    return shown


def _depth_preview_lines(depth: DepthFrame, row: dict, rate_hz: float | None) -> list[str]:
    """Display fixed result-ready latency, not growing display staleness."""
    return [
        f"Depth seq={depth.source_sequence_id} "
        f"{depth.width}x{depth.height} meters",
        f"{rate_hz or 0:.2f} Hz  "
        f"infer={row['inference_adapter_wall_ms']:.0f}ms  "
        f"age={row['result_age_ms']:.0f}ms",
    ]


def _select_preview_track(
    click: tuple[int, int], displayed: PersonTrackingFrame,
    current: PersonTrackingFrame, selector: MasterSelector, manager: MasterManager,
) -> int | None:
    """Hit-test displayed source pixels; bind only if that ID is still visible."""
    x, y = click
    if not (0 <= x < displayed.frame_width and 0 <= y < displayed.frame_height):
        return None
    selected = selector.select_at_pixel(displayed, x, y).selected_track_id
    if selected is None or displayed.source_id != current.source_id:
        return None
    if not any(track.track_id == selected for track in current.tracks):
        return None
    manager.lock(current, selected, click_x_px=x, click_y_px=y)
    return selected


def _master_diagnostic(
    paired: _PairedFrame, calibration, depth_convention="axial_z",
) -> tuple[MasterDepthSample | None, np.ndarray | None]:
    sample = representative_master_depth(paired.master, paired.depth)
    if sample is None or calibration is None:
        return sample, None
    validate_calibration_resolution(calibration, paired.depth.width, paired.depth.height)
    # The selected depth convention remains an explicit provisional assumption.
    point = back_project_camera_point(
        sample.anchor_uv_px, sample.depth_m, calibration,
        depth_convention=depth_convention,
    )
    return sample, point


def _draw_tracks(frame: ColorFrame, tracking: PersonTrackingFrame,
                 master: MasterTrackingFrame) -> np.ndarray:
    rgb = frame.bgr.copy()
    for track in tracking.tracks:
        box = track.bbox_xyxy_px
        is_master = master.master_track is not None and track.track_id == master.bound_track_id
        color = (0, 255, 0) if is_master else (0, 220, 255)
        cv2.rectangle(rgb, (round(box[0]), round(box[1])),
                      (round(box[2]), round(box[3])), color, 2)
        cv2.putText(rgb, f"ID {track.track_id}",
                    (round(box[0]), max(20, round(box[1]) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
    master_track = master.master_track
    if master_track is not None:
        roi = torso_roi_xyxy(master_track.bbox_xyxy_px,
                             frame.width, frame.height)
        cv2.rectangle(rgb, roi[:2], roi[2:], (255, 255, 0), 2)
        x1, y1, x2, y2 = master_track.bbox_xyxy_px
        anchor = (round((x1 + x2) / 2), round((y1 + y2) / 2))
        cv2.drawMarker(rgb, anchor, (255, 0, 255), cv2.MARKER_CROSS, 22, 2)
    return rgb


def _paired_preview(
    paired: _PairedFrame, calibration, depth_rate_hz: float | None,
    detector_rate_hz: float | None, *, robot_observation=None, depth_convention="axial_z",
) -> tuple[np.ndarray, MasterDepthSample | None, np.ndarray | None]:
    sample, point = _master_diagnostic(paired, calibration, depth_convention)
    rgb = _draw_tracks(paired.color, paired.tracking, paired.master)
    lines = [f"Master {paired.master.bound_track_id or '-'} | {paired.master.state.value}",
             f"Depth {sample.depth_m:.2f} m" if sample else "Depth unavailable"]
    if point is not None:
        lines.append(f"Camera XYZ ({point[0]:+.2f}, {point[1]:+.2f}, {point[2]:+.2f}) m")
    if robot_observation is not None:
        lines.append(f"Robot x={robot_observation.x_forward_m:+.2f} "
                     f"y={robot_observation.y_left_m:+.2f} m [provisional]")
    lines.append(f"Detect {detector_rate_hz or 0:.1f} Hz | Depth {depth_rate_hz or 0:.1f} Hz "
                 f"infer={paired.depth_row['inference_adapter_wall_ms']:.0f}ms "
                 f"age={paired.depth_row['result_age_ms']:.0f}ms")
    lines.append(f"seq={paired.color.sequence_id} | {depth_convention} assumed | click / c clear / q quit")
    return _overlay(rgb, lines), sample, point


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv: Sequence[str] | None = None, *, pitch_provider_override=None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.video is not None and not args.video.is_file():
        parser.error(f"video does not exist: {args.video}")
    if args.video is None and args.max_source_frames is None and not args.preview:
        parser.error("headless camera run needs --max-source-frames")
    if (args.width is None) != (args.height is None):
        parser.error("--width and --height must be supplied together")
    if args.width is not None and (args.width < 1 or args.height < 1):
        parser.error("capture dimensions must be positive")
    if args.max_source_frames is not None and args.max_source_frames < 1:
        parser.error("--max-source-frames must be positive")
    if args.warmup_results < 0:
        parser.error("--warmup-results must be nonnegative")
    if args.realtime_playback and args.video is None:
        parser.error("--realtime-playback applies to --video only")
    if args.select_track_id is not None and (args.select_track_id < 1 or args.mode != "full"):
        parser.error("--select-track-id needs a positive ID and --mode full")
    run_name = args.run_name or args.mode
    if re.fullmatch(r"[A-Za-z0-9_-]+", run_name) is None:
        parser.error("--run-name must contain only letters, digits, _ or -")

    final_config = None
    extrinsic = pitch_provider = reid_policy = None
    depth_convention = "axial_z"
    if args.mode == "full":
        final_config = json.loads(args.config.read_text(encoding="utf-8"))
        config_directory = args.config.resolve().parent
        if args.calibration is None:
            args.calibration = config_directory / final_config["calibration_asset"]
        reid_policy = json.loads((config_directory / final_config["reid_policy_asset"]).read_text(
            encoding="utf-8"))["auto_reacquire"]
        mount = final_config["camera_extrinsic"]
        extrinsic = CameraExtrinsic(mount["rotation_body_camera"],
                                    mount["translation_body_camera_m"], mount["provisional"])
        pitch_config = final_config["pitch"]
        if pitch_provider_override is not None:
            pitch_provider = pitch_provider_override
        else:
            if (pitch_config["provider"] != "constant_simulated"
                    or pitch_config["time_semantics"] != "host_read_complete"
                    or pitch_config["clock_domain"] != "host_perf_counter"
                    or pitch_config["provisional"] is not True):
                raise ValueError("A measured pitch config requires an injected PitchProvider")
            pitch_provider = ConstantPitchProvider(np.deg2rad(pitch_config["theta_deg"]))
        depth_convention = final_config["depth_convention"]
        if depth_convention not in ("axial_z", "euclidean_range"):
            raise ValueError("depth convention must be explicit")
        if args.output_dir == DEFAULT_OUTPUT:
            args.output_dir = ROOT / "models" / "minisegway" / "stage7" / "results"
        if args.video is None and args.fps is None:
            args.fps = 30.0
    calibration = load_camera_calibration(args.calibration) if args.calibration else None
    if calibration is not None and args.video is None:
        if args.width is None:
            args.width, args.height = calibration.frame_width, calibration.frame_height
        validate_calibration_resolution(calibration, args.width, args.height)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".venv"))

    depth_adapter = None
    if args.mode in ("depth", "concurrent", "full"):
        if args.depth_model == DEFAULT_DEPTH_MODEL:
            DEFAULT_DEPTH_MODEL.parent.mkdir(parents=True, exist_ok=True)
        depth_adapter = YoloDepthAdapter(YoloDepthConfig(
            model_path=args.depth_model, imgsz=args.depth_imgsz, device=args.device))
    detector = tracker = None
    if args.mode in ("detector", "concurrent", "full"):
        detector = PersonDetector(PersonDetectorConfig(
            model_path=args.detector_model, imgsz=args.detector_imgsz,
            confidence_threshold=args.detector_conf, device=args.device))
        tracker = PersonTracker(UltralyticsByteTrackBackend())
    reid_session = None
    if args.mode == "full":
        reid_session = MasterReIDSession(
            MasterReIDEvidence(PersonReIdentifier(TorchReIDOSNetBackend(
                args.reid_weights or default_osnet_weights_path()))),
            initial_track_id=args.select_track_id,
            accept_threshold=reid_policy["reid_accept_threshold"],
            retry_threshold=reid_policy["reid_retry_threshold"],
            retry_interval_s=reid_policy["retry_interval_s"],
            stable_updates=reid_policy["minimum_stable_tracker_updates"],
        )

    depth_channel = _Channel(LatestFrameSlot()) if depth_adapter is not None else None
    detector_channel = _Channel(LatestFrameSlot()) if detector is not None else None
    preview_slot = LatestFrameSlot()
    active_channels = [channel for channel in (depth_channel, detector_channel)
                       if channel is not None]
    fanout = _FrameFanout(tuple([preview_slot] + [c.slot for c in active_channels]),
                          calibration)
    runtime = source_stream._SourceRuntime.create(fanout)
    workers = []
    if depth_channel is not None:
        workers.append(threading.Thread(
            target=_consume,
            args=(depth_channel, runtime,
                  lambda frame: _depth_operation(depth_adapter, frame)),
            name="depth-worker"))
    if detector_channel is not None:
        workers.append(threading.Thread(
            target=_consume,
            args=(detector_channel, runtime,
                  lambda frame: _detector_operation(detector, tracker, frame, reid_session)),
            name="detector-tracker-worker"))
    producer = source_stream._source_thread(_source_args(args), runtime)
    for worker in workers:
        worker.start()
    producer.start()
    last_depth_seq = -1
    last_depth_seq_for_pairing = -1
    last_detector_seq = -1
    latest_rgb = None
    last_preview_key = None
    last_preview_image = None
    pairer = _SameFramePairer()
    master_manager = MasterManager() if detector_channel is not None and reid_session is None else None
    master_selector = MasterSelector() if detector_channel is not None else None
    current_tracking: PersonTrackingFrame | None = None
    current_master: MasterTrackingFrame | None = None
    current_tracking_color: ColorFrame | None = None
    displayed_tracking: PersonTrackingFrame | None = None
    latest_paired: _PairedFrame | None = None
    last_accepted_pair_seq = -1
    paired_count = 0
    master_rows: list[dict] = []
    robot_rows: list[dict] = []
    latest_robot_observation = None
    click_requests: deque[tuple[tuple[int, int], PersonTrackingFrame]] = deque()

    def _on_click(event: int, x: int, y: int, flags: int, param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and displayed_tracking is not None:
            click_requests.append(((x, y), displayed_tracking))

    def _accept_pair(paired: _PairedFrame | None) -> None:
        nonlocal latest_paired, last_accepted_pair_seq, paired_count, latest_robot_observation
        if paired is None or paired.color.sequence_id <= last_accepted_pair_seq:
            return
        latest_paired = paired
        last_accepted_pair_seq = paired.color.sequence_id
        paired_count += 1
        latest_robot_observation = None
        if paired.master.master_track is not None:
            sample, point = _master_diagnostic(paired, calibration, depth_convention)
            if point is not None and extrinsic is not None:
                latest_robot_observation = camera_to_robot_observation(
                    point, paired.color, paired.master.bound_track_id,
                    extrinsic, pitch_provider, depth_convention=depth_convention)
                if latest_robot_observation is not None:
                    robot_rows.append(asdict(latest_robot_observation))
            master_rows.append({
                "source_id": paired.color.source_id,
                "source_sequence_id": paired.color.sequence_id,
                "source_time_s": paired.color.host_receive_time_s,
                "width": paired.color.width, "height": paired.color.height,
                "master_track_id": paired.master.bound_track_id,
                "valid_roi_pixels": sample.valid_pixel_count if sample else 0,
                "master_depth_m": sample.depth_m if sample else None,
                "camera_x_right_m": float(point[0]) if point is not None else None,
                "camera_y_down_m": float(point[1]) if point is not None else None,
                "camera_z_forward_m": float(point[2]) if point is not None else None,
            })
    try:
        if not runtime.ready.wait(timeout=90.0):
            raise RuntimeError("timed out waiting for Stage 7.2 source initialization")
        if runtime.error_value() is not None:
            raise runtime.error_value()
        print(f"source: {runtime.description}")
        profile = runtime.camera_profile
        if profile is not None:
            print(f"backend: {profile.backend_name}; requested: "
                  f"{profile.requested_width}x{profile.requested_height}@"
                  f"{profile.requested_fps}; reported: "
                  f"{profile.reported_width}x{profile.reported_height}@"
                  f"{profile.reported_fps:.3f}")
        print("calibration: " + ("NOT_PROVIDED" if calibration is None else
              f"LOADED {calibration.frame_width}x{calibration.frame_height} "
              f"from {calibration.source_path}"))
        if reid_session is not None:
            print("Full perception: OSNet pending reference + automatic ReID reacquire enabled")
            print(f"Geometry: {depth_convention} assumed; mount provisional={extrinsic.provisional}; "
                  f"pitch provider={type(pitch_provider).__name__}; "
                  "robot axes +X forward / +Y left / +Z up")
        if args.preview:
            cv2.namedWindow("CompanionBot Stage 7 RGB", cv2.WINDOW_NORMAL)
            if detector_channel is not None:
                cv2.setMouseCallback("CompanionBot Stage 7 RGB", _on_click)
        while True:
            frame = preview_slot.get(timeout_s=0.05)
            if frame is not None:
                latest_rgb = frame
            depth_pair, depth_rows = (depth_channel.snapshot()
                                      if depth_channel else (None, []))
            detector_pair, detector_rows = (detector_channel.snapshot()
                                            if detector_channel else (None, []))
            if detector_channel is not None:
                for tracked_color, tracked_result, _ in detector_channel.completed_after(
                        last_detector_seq):
                    last_detector_seq = tracked_color.sequence_id
                    current_tracking_color = tracked_color
                    if isinstance(tracked_result, MasterPerceptionResult):
                        current_tracking, current_master = tracked_result.tracking, tracked_result.master
                    else:
                        current_tracking = tracked_result
                        master_manager.update(tracked_result)
                        current_master = master_manager.result(tracked_result)
                    _accept_pair(pairer.add_tracking(
                        tracked_color, current_tracking, current_master))
            while click_requests:
                click, displayed = click_requests.popleft()
                if current_tracking is not None:
                    if reid_session is not None:
                        selected = master_selector.select_at_pixel(displayed, *click).selected_track_id
                        if selected is not None:
                            reid_session.select(displayed.source_id, selected)
                    else:
                        selected = _select_preview_track(
                            click, displayed, current_tracking, master_selector, master_manager)
                        current_master = master_manager.result(current_tracking)
                    if selected is not None:
                        latest_paired = None  # Await a new exact-frame pair after selection.
                        print(f"Master selected: track_id={selected} "
                              f"current_seq={current_tracking.source_sequence_id}")
            if depth_pair is not None:
                depth_frame = depth_pair[1]
                if depth_frame.source_sequence_id != last_depth_seq:
                    last_depth_seq = depth_frame.source_sequence_id
                    if len(depth_rows) == 1 or len(depth_rows) % 10 == 0:
                        row = depth_rows[-1]
                        print(f"depth seq={last_depth_seq} "
                              f"infer={row['inference_adapter_wall_ms']:.1f}ms "
                              f"age={row['result_age_ms']:.1f}ms "
                              f"rate={_rate(depth_rows) or 0:.2f}Hz")
            if depth_channel is not None and detector_channel is not None:
                for depth_color, depth_result, depth_row in depth_channel.completed_after(
                        last_depth_seq_for_pairing):
                    last_depth_seq_for_pairing = depth_color.sequence_id
                    _accept_pair(pairer.add_depth(depth_color, depth_result, depth_row))
            if (latest_paired is not None
                    and (time.perf_counter() - latest_paired.depth_row["ready_time_s"] > 1.0
                         or (current_master is not None and
                             (current_master.state, current_master.bound_track_id) !=
                             (latest_paired.master.state, latest_paired.master.bound_track_id)))):
                latest_paired = None  # Do not display an old pair as the current Master.
            preview_rgb = (latest_paired.color if latest_paired is not None else
                           current_tracking_color if current_tracking_color is not None else
                           depth_pair[0] if depth_pair is not None else latest_rgb)
            if args.preview and preview_rgb is not None:
                if latest_paired is not None:
                    displayed_tracking = latest_paired.tracking
                    preview_key = ("paired", latest_paired.color.sequence_id)
                    if preview_key != last_preview_key:
                        last_preview_image, _, _ = _paired_preview(
                            latest_paired, calibration, _rate(depth_rows),
                            _rate(detector_rows),
                            robot_observation=latest_robot_observation,
                            depth_convention=depth_convention)
                else:
                    displayed_tracking = current_tracking
                    preview_key = ("source", preview_rgb.sequence_id)
                    if preview_key != last_preview_key:
                        if current_tracking is not None:
                            rgb_image = _draw_tracks(preview_rgb, current_tracking, current_master)
                            lines = [f"Master {current_master.bound_track_id or '-'} | {current_master.state.value}",
                                     "Depth / XYZ unavailable: waiting for same-frame result",
                                     f"Detect {_rate(detector_rows) or 0:.1f} Hz | Depth {_rate(depth_rows) or 0:.1f} Hz",
                                     "click person / c clear / q quit"]
                        else:
                            rgb_image = preview_rgb.bgr
                            lines = [f"RGB {preview_rgb.width}x{preview_rgb.height} seq={preview_rgb.sequence_id}"]
                            if depth_pair is not None:
                                lines += _depth_preview_lines(depth_pair[1], depth_rows[-1], _rate(depth_rows))
                        last_preview_image = _overlay(rgb_image, lines)
                last_preview_key = preview_key
                cv2.imshow("CompanionBot Stage 7 RGB", last_preview_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("c") and current_tracking is not None:
                    if reid_session is not None:
                        reid_session.clear(current_tracking.source_id)
                    else:
                        master_manager.clear(current_tracking)
                        current_master = master_manager.result(current_tracking)
                    latest_paired = None
                    last_preview_key = None
                if key in (ord("q"), 27):
                    runtime.stop.set()
                    break
            if runtime.error_value() is not None or any(c.error for c in active_channels):
                break
            if runtime.done.is_set() and all(c.done.is_set() for c in active_channels):
                break
    finally:
        runtime.stop.set()
        producer.join(timeout=15)
        fanout.close()
        for worker in workers:
            worker.join(timeout=30)
        if tracker is not None:
            tracker.close()
        if args.preview:
            cv2.destroyAllWindows()
    if producer.is_alive() or any(worker.is_alive() for worker in workers):
        raise RuntimeError("source or inference worker did not stop")
    if runtime.error_value() is not None:
        raise runtime.error_value()
    for channel in active_channels:
        if channel.error is not None:
            raise channel.error

    source = runtime.stats.snapshot()
    if source["produced_count"] < 1:
        raise RuntimeError("source produced no ColorFrame")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    depth_rows = depth_channel.snapshot()[1] if depth_channel else []
    detector_rows = detector_channel.snapshot()[1] if detector_channel else []
    _write_csv(args.output_dir / f"{run_name}_depth_frames.csv", depth_rows)
    _write_csv(args.output_dir / f"{run_name}_detector_frames.csv", detector_rows)
    _write_csv(args.output_dir / f"{run_name}_master_camera_points.csv", master_rows)
    _write_csv(args.output_dir / f"{run_name}_robot_observations.csv", robot_rows)
    if reid_session is not None:
        events_path = args.output_dir / f"{run_name}_master_reid_events.jsonl"
        events_path.write_text("".join(json.dumps(row) + "\n" for row in reid_session.events),
                               encoding="utf-8")
        (args.output_dir / f"{run_name}_resolved_config.json").write_text(
            json.dumps({"geometry": final_config, "reid_policy": reid_policy,
                        "pitch_provider_override": pitch_provider_override is not None}, indent=2),
            encoding="utf-8")
    source_duration = (source["last_time_s"] - source["first_time_s"])
    source_hz = ((source["produced_count"] - 1) / source_duration
                 if source["produced_count"] > 1 and source_duration > 0 else None)
    profile = runtime.camera_profile
    metrics = {
        "mode": args.mode,
        "source": {**source, "description": runtime.description,
                   "effective_hz": source_hz,
                   "backend": profile.backend_name if profile else "video",
                   "requested_width": args.width, "requested_height": args.height,
                   "requested_fps": args.fps,
                   "reported_width": profile.reported_width if profile else None,
                   "reported_height": profile.reported_height if profile else None,
                   "reported_fps": profile.reported_fps if profile else runtime.reported_fps},
        "calibration": None if calibration is None else {
            "path": calibration.source_path,
            "resolution": [calibration.frame_width, calibration.frame_height],
            "status": "loaded_and_exact_source_resolution_validated"},
        "models": {"depth": str(args.depth_model) if depth_adapter else None,
                   "depth_imgsz": args.depth_imgsz if depth_adapter else None,
                   "detector": args.detector_model if detector else None,
                   "detector_imgsz": args.detector_imgsz if detector else None},
        "depth": _summarize(depth_rows, args.warmup_results) if depth_adapter else None,
        "detector_tracker": (_summarize(detector_rows, args.warmup_results)
                             if detector else None),
        "source_time_semantics": "host read-complete perf_counter time, not exposure time",
        "schedule": "one Stage 7.2 source; independent latest-frame-wins slots",
        "reid": None if reid_session is None else {
            "model": reid_session.evidence.model_name,
            "event_count": len(reid_session.events),
            "reference_count": sum(row["event"] == "MASTER_REFERENCE_CREATED" for row in reid_session.events),
            "reacquire_count": sum(row["event"] == "MASTER_REID_REACQUIRED" for row in reid_session.events),
            "accept_threshold": reid_session.accept_threshold,
            "retry_threshold": reid_session.retry_threshold,
            "threshold_status": reid_policy["threshold_status"],
        },
        "robot_geometry": {
            "observation_count": len(robot_rows),
            "configuration": final_config,
            "pitch_provider_type": type(pitch_provider).__name__ if pitch_provider is not None else None,
            "pitch_provider_override": pitch_provider_override is not None,
            "stage6_connected": False,
        },
        "master_camera_geometry": {
            "same_frame_pairs": paired_count,
            "matched_visible_master_frames": len(master_rows),
            "valid_depth_frames": sum(row["master_depth_m"] is not None for row in master_rows),
            "depth_convention": depth_convention + " provisional; YOLO26 checkpoint convention unconfirmed",
            "camera_axes": "+X right, +Y down, +Z forward",
        },
    }
    if args.compare_detector_baseline is not None:
        baseline = json.loads(args.compare_detector_baseline.read_text(encoding="utf-8"))
        if args.mode not in ("concurrent", "full") or baseline["mode"] != "detector":
            raise ValueError("comparison requires concurrent run and detector-only baseline")
        if (baseline["source"]["width"], baseline["source"]["height"],
                baseline["models"]["detector_imgsz"]) != (
                source["width"], source["height"], args.detector_imgsz):
            raise ValueError("detector baseline source/model resolution differs")
        base_hz = baseline["detector_tracker"]["effective_hz"]
        current_hz = metrics["detector_tracker"]["effective_hz"]
        metrics["detector_comparison"] = {
            "baseline_path": str(args.compare_detector_baseline.resolve()),
            "processed_hz_change_percent": (100 * (current_hz / base_hz - 1)
                                            if base_hz and current_hz else None),
            "median_call_wall_ms_change": (
                metrics["detector_tracker"]["detector_wall_ms_median"]
                - baseline["detector_tracker"]["detector_wall_ms_median"]),
            "median_result_age_ms_change": (
                metrics["detector_tracker"]["result_age_ms_median"]
                - baseline["detector_tracker"]["result_age_ms_median"]),
        }
    metrics_path = args.output_dir / f"{run_name}_metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    report_path = args.output_dir / f"{run_name}_report.md"
    report_path.write_text(
        f"# Stage 7.7 {run_name}\n\n"
        f"- Mode: {args.mode}; source: {runtime.description}\n"
        f"- Actual source: {source['width']}x{source['height']} BGR uint8; "
        f"{source_hz or 0:.2f} Hz; {source['produced_count']} frames\n"
        f"- Calibration: {metrics['calibration']}\n"
        f"- Depth: {metrics['depth']}\n"
        f"- Detector/tracker: {metrics['detector_tracker']}\n"
        f"- ReID: {metrics['reid']}\n"
        f"- Robot observations: {len(robot_rows)}; see per-observation provisional flags; Stage 6 disconnected.\n"
        f"- Detector comparison: {metrics.get('detector_comparison')}\n"
        "- Source time is host read-complete, not exposure time. Depth values are "
        "single-image model estimates in metres, not range-sensor ground truth.\n"
        "- Master torso median uses only exact same-source-frame bbox/depth pairs; "
        f"camera XYZ assumes {depth_convention} provisionally (checkpoint convention unconfirmed).\n"
        f"- Metrics: {metrics_path.resolve()}\n",
        encoding="utf-8",
    )
    print(f"metrics: {metrics_path.resolve()}")
    print(f"report: {report_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
