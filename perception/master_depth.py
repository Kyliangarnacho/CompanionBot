"""Visible-Master torso depth and calibrated OpenCV camera-frame geometry.

The depth convention is supplied explicitly. YOLO26 depth's global axial/ray
convention is not conclusively documented; callers must label that assumption.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from perception.camera_calibration import (
    CameraCalibration, undistort_image_points,
)
from perception.depth import DepthFrame
from perception.master_selection import MasterTrackingFrame


@dataclass(frozen=True)
class MasterDepthSample:
    track_id: int
    source_sequence_id: int
    depth_m: float
    roi_xyxy_px: tuple[int, int, int, int]
    anchor_uv_px: tuple[float, float]
    valid_pixel_count: int


def torso_roi_xyxy(
    bbox_xyxy_px: tuple[float, float, float, float], width: int, height: int,
) -> tuple[int, int, int, int]:
    """Central 40% of a source-pixel bbox, clipped to the source image."""
    x1, y1, x2, y2 = bbox_xyxy_px
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    half_w, half_h = (x2 - x1) * 0.2, (y2 - y1) * 0.2
    return (max(0, min(width, math.floor(cx - half_w))),
            max(0, min(height, math.floor(cy - half_h))),
            max(0, min(width, math.ceil(cx + half_w))),
            max(0, min(height, math.ceil(cy + half_h))))


def representative_master_depth(
    master: MasterTrackingFrame, depth: DepthFrame, *, min_valid_pixels: int = 10,
) -> MasterDepthSample | None:
    """Median valid torso depth for a visible Master in the *same* source frame."""
    if not isinstance(master, MasterTrackingFrame) or not isinstance(depth, DepthFrame):
        raise TypeError("master and depth must be MasterTrackingFrame and DepthFrame")
    if (master.source_id, master.source_sequence_id, master.source_time_s,
            master.frame_width, master.frame_height) != (
            depth.source_id, depth.source_sequence_id, depth.source_time_s,
            depth.width, depth.height):
        raise ValueError("Master bbox and depth must belong to the same source frame")
    if isinstance(min_valid_pixels, bool) or not isinstance(min_valid_pixels, int) or min_valid_pixels < 1:
        raise ValueError("min_valid_pixels must be a positive integer")
    track = master.master_track
    if track is None:
        return None
    roi = torso_roi_xyxy(track.bbox_xyxy_px, depth.width, depth.height)
    left, top, right, bottom = roi
    if left >= right or top >= bottom:
        return None
    values = depth.depth_m[top:bottom, left:right]
    valid = values[np.isfinite(values) & (values > 0)]
    if valid.size < min_valid_pixels:
        return None
    x1, y1, x2, y2 = track.bbox_xyxy_px
    return MasterDepthSample(
        track_id=track.track_id,
        source_sequence_id=master.source_sequence_id,
        depth_m=float(np.median(valid)),
        roi_xyxy_px=roi,
        anchor_uv_px=((x1 + x2) / 2, (y1 + y2) / 2),
        valid_pixel_count=int(valid.size),
    )


def back_project_camera_point(
    anchor_uv_px: tuple[float, float], depth_m: float,
    calibration: CameraCalibration, *, depth_convention: str,
) -> np.ndarray:
    """Return [X right, Y down, Z forward] in metres after pixel undistortion."""
    if depth_convention not in ("axial_z", "euclidean_range"):
        raise ValueError("depth_convention must be axial_z or euclidean_range")
    if not math.isfinite(float(depth_m)) or depth_m <= 0:
        raise ValueError("depth_m must be finite and positive")
    if not isinstance(calibration, CameraCalibration):
        raise TypeError("calibration must be a CameraCalibration")
    u, v = (float(value) for value in anchor_uv_px)
    if not (0 <= u < calibration.frame_width and 0 <= v < calibration.frame_height):
        raise ValueError("anchor must be inside the calibrated source frame")
    undistorted = undistort_image_points([[u, v]], calibration)[0]
    ray = np.linalg.solve(calibration.K, [undistorted[0], undistorted[1], 1.0])
    divisor = ray[2] if depth_convention == "axial_z" else np.linalg.norm(ray)
    if not math.isfinite(float(divisor)) or divisor <= 0:
        raise ValueError("calibrated camera ray is invalid")
    return np.asarray(ray * (depth_m / divisor), dtype=np.float64)
