"""Probe an explicitly selected OpenCV RGB source and optionally preview raw BGR."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import statistics
import sys
from typing import Sequence

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from perception.camera import CameraConfig, CameraStream, ColorFrame
from perception.camera_calibration import (
    load_camera_calibration,
    validate_calibration_resolution,
)


def _device_value(value: str) -> int | str:
    try:
        return int(value)
    except ValueError:
        return value


def _backend_value(value: str) -> int | None:
    if value.lower() == "auto":
        return None
    constant_name = value.upper()
    if not constant_name.startswith("CAP_"):
        constant_name = "CAP_" + constant_name
    backend = getattr(cv2, constant_name, None)
    if not isinstance(backend, int):
        raise argparse.ArgumentTypeError(
            f"OpenCV backend {value!r} is unavailable in this installation"
        )
    return backend


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device", required=True,
        help="Explicit OpenCV device index or source path/URI; indices may change",
    )
    parser.add_argument("--backend", type=_backend_value, default=None,
                        help="Optional OpenCV backend (for example auto, DSHOW, MSMF, V4L2)")
    parser.add_argument("--width", type=int, help="Requested width; no default")
    parser.add_argument("--height", type=int, help="Requested height; no default")
    parser.add_argument("--fps", type=float, help="Requested FPS; no default")
    parser.add_argument("--samples", type=int, default=120,
                        help="Frames used for timing summary (default: 120)")
    parser.add_argument("--calibration", type=Path,
                        help="Optional NPZ/OpenCV YAML camera intrinsic asset")
    parser.add_argument("--preview", action="store_true",
                        help="Show the unmodified BGR image until Q or Escape")
    return parser


def _print_initial_profile(stream: CameraStream) -> None:
    profile = stream.profile
    requested = ("unspecified" if profile.requested_width is None else
                 f"{profile.requested_width}x{profile.requested_height}"
                 f"@{profile.requested_fps or 'unspecified'} fps")
    print(f"device/source identifier: {profile.device_identifier}")
    print(f"selected OpenCV backend: {profile.backend_name}")
    print(f"requested profile: {requested}")
    print("OpenCV-reported profile: "
          f"{profile.reported_width}x{profile.reported_height}"
          f"@{profile.reported_fps:.3f} fps")


def _print_calibration_status(
    path: Path | None,
    frame: ColorFrame,
) -> bool:
    if path is None:
        print("calibration: NOT_PROVIDED (no intrinsic asset available to match)")
        return True
    try:
        calibration = load_camera_calibration(path)
        validate_calibration_resolution(calibration, frame.width, frame.height)
    except (OSError, ValueError) as error:
        print(f"calibration: NO_MATCH ({error})")
        return False
    print("calibration: MATCH "
          f"{calibration.frame_width}x{calibration.frame_height} "
          f"from {calibration.source_path} ({calibration.source_format})")
    return True


def _print_frame_contract(frame: ColorFrame) -> None:
    print("actual frame: "
          f"{frame.width}x{frame.height}, dtype={frame.bgr.dtype}, "
          f"shape={frame.bgr.shape}, color=BGR")


def _print_timing(frames: list[ColorFrame]) -> None:
    timestamps = [frame.host_receive_time_s for frame in frames]
    intervals = [right-left for left, right in zip(timestamps, timestamps[1:])]
    continuous = all(
        right.sequence_id == left.sequence_id + 1
        for left, right in zip(frames, frames[1:])
    )
    monotonic = all(right > left for left, right in zip(timestamps, timestamps[1:]))
    elapsed = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0.0
    measured_fps = (len(frames)-1) / elapsed if elapsed > 0.0 else 0.0
    print(f"sequence continuity: {continuous} "
          f"({frames[0].sequence_id}..{frames[-1].sequence_id}, {len(frames)} frames)")
    print(f"host receive timestamp monotonicity: {monotonic}")
    if intervals:
        print("host read-complete interval: "
              f"median={statistics.median(intervals)*1000.0:.2f} ms, "
              f"mean={statistics.mean(intervals)*1000.0:.2f} ms, "
              f"min={min(intervals)*1000.0:.2f} ms, "
              f"max={max(intervals)*1000.0:.2f} ms")
        print(f"measured read-complete rate: {measured_fps:.2f} fps")
    else:
        print("host read-complete interval/FPS: unavailable (one frame)")


def main(argv: Sequence[str] | None = None) -> int:
    parser = _argument_parser()
    arguments = parser.parse_args(argv)
    if arguments.samples < 2:
        parser.error("--samples must be at least 2")
    try:
        config = CameraConfig(
            device=_device_value(arguments.device),
            requested_width=arguments.width,
            requested_height=arguments.height,
            requested_fps=arguments.fps,
            backend=arguments.backend,
        )
    except ValueError as error:
        parser.error(str(error))
    stream = CameraStream(config)
    frames: list[ColorFrame] = []
    calibration_ok = True
    summary_printed = False
    try:
        stream.open()
        _print_initial_profile(stream)
        while True:
            frame = stream.read()
            if len(frames) < arguments.samples:
                frames.append(frame)
                if len(frames) == 1:
                    _print_frame_contract(frame)
                    calibration_ok = _print_calibration_status(
                        arguments.calibration, frame
                    )
                if len(frames) == arguments.samples:
                    _print_timing(frames)
                    summary_printed = True
            if arguments.preview:
                cv2.imshow("CompanionBot raw RGB camera (BGR)", frame.bgr)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
            elif len(frames) >= arguments.samples:
                break
    except (RuntimeError, ValueError, cv2.error) as error:
        print(f"camera probe failed: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("camera probe interrupted")
    finally:
        stream.release()
        if arguments.preview:
            cv2.destroyAllWindows()

    if not summary_printed and frames:
        _print_timing(frames)
    return 0 if calibration_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
