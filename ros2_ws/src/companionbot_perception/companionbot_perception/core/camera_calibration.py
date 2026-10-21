"""Camera intrinsics loading and sparse pixel-point undistortion.

Adapted from MonoTeach's ``stage2/camera_calibration.py``. This module covers
only camera K/D; it intentionally contains no workspace homography or metric
target localization. Input pixels must be in the original calibrated frame
resolution, even if a detector internally resizes or letterboxes an image.
Before applying K/D, a future detector adapter must map output pixels back
through the inverse resize/letterbox transform into original-frame pixels.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Union

import cv2
import numpy as np


ArrayLike = Union[np.ndarray, list, tuple]
_REQUIRED_FIELDS = {"camera_matrix", "dist_coeffs", "image_width", "image_height"}
_VALID_DISTORTION_LENGTHS = {4, 5, 8, 12, 14}


def _readonly_array(value: ArrayLike, name: str) -> np.ndarray:
    try:
        result = np.array(value, dtype=np.float64, copy=True, order="C")
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be numeric") from error
    result.setflags(write=False)
    return result


def _positive_integer(value: object, name: str) -> int:
    if isinstance(value, np.ndarray):
        if value.shape != ():
            raise ValueError(f"{name} must be a scalar positive integer")
        value = value.item()
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"{name} must be a positive integer")
    result = int(value)
    if result <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return result


def _load_matrix(K: ArrayLike) -> np.ndarray:
    matrix = _readonly_array(K, "K")
    if matrix.shape != (3, 3):
        raise ValueError(f"K must have shape (3, 3); got {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise ValueError("K must contain only finite values")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("K focal lengths fx and fy must be positive")
    return matrix


def _load_distortion(D: ArrayLike) -> np.ndarray:
    coefficients = _readonly_array(D, "D")
    if coefficients.ndim == 1:
        flattened = coefficients
    elif coefficients.ndim == 2 and 1 in coefficients.shape:
        flattened = coefficients.reshape(-1)
    else:
        raise ValueError("D must have shape (N,), (1, N), or (N, 1)")
    if flattened.size not in _VALID_DISTORTION_LENGTHS:
        raise ValueError(
            "D must contain 4, 5, 8, 12, or 14 OpenCV coefficients; "
            f"got {flattened.size}"
        )
    if not np.isfinite(flattened).all():
        raise ValueError("D must contain only finite values")
    result = np.array(flattened.reshape(1, -1), dtype=np.float64, copy=True)
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class CameraCalibration:
    """Validated OpenCV intrinsics for one exact image resolution."""

    K: ArrayLike
    D: ArrayLike
    frame_width: int
    frame_height: int
    source_path: str
    source_format: str

    def __post_init__(self) -> None:
        if not isinstance(self.source_path, str) or not self.source_path:
            raise ValueError("source_path must be a non-empty string")
        if not isinstance(self.source_format, str) or not self.source_format:
            raise ValueError("source_format must be a non-empty string")
        object.__setattr__(self, "K", _load_matrix(self.K))
        object.__setattr__(self, "D", _load_distortion(self.D))
        object.__setattr__(self, "frame_width",
                           _positive_integer(self.frame_width, "frame_width"))
        object.__setattr__(self, "frame_height",
                           _positive_integer(self.frame_height, "frame_height"))


def _load_npz(path: Path) -> CameraCalibration:
    try:
        loaded = np.load(path, allow_pickle=False)
    except Exception as error:
        raise ValueError(f"Cannot read NumPy calibration file {path}: {error}") from error
    if not isinstance(loaded, np.lib.npyio.NpzFile):
        raise ValueError(f"Calibration file must be an NPZ archive: {path}")
    try:
        missing = sorted(_REQUIRED_FIELDS.difference(loaded.files))
        if missing:
            raise ValueError(f"NPZ calibration is missing required fields: {missing}")
        return CameraCalibration(
            K=loaded["camera_matrix"],
            D=loaded["dist_coeffs"],
            frame_width=_positive_integer(loaded["image_width"], "image_width"),
            frame_height=_positive_integer(loaded["image_height"], "image_height"),
            source_path=str(path),
            source_format="npz",
        )
    finally:
        loaded.close()


def _load_opencv_yaml(path: Path) -> CameraCalibration:
    try:
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    except (cv2.error, SystemError) as error:
        raise ValueError(f"Cannot parse OpenCV YAML calibration: {path}") from error
    if not storage.isOpened():
        storage.release()
        raise ValueError(f"Cannot open OpenCV YAML calibration: {path}")
    try:
        missing = sorted(name for name in _REQUIRED_FIELDS
                         if storage.getNode(name).empty())
        if missing:
            raise ValueError(f"OpenCV YAML calibration is missing fields: {missing}")
        K = storage.getNode("camera_matrix").mat()
        D = storage.getNode("dist_coeffs").mat()
        width_value = storage.getNode("image_width").real()
        height_value = storage.getNode("image_height").real()
        if K is None or D is None:
            raise ValueError("OpenCV YAML K/D fields must be matrices")
        if (not np.isfinite(width_value) or not float(width_value).is_integer()
                or not np.isfinite(height_value) or not float(height_value).is_integer()):
            raise ValueError("OpenCV YAML image dimensions must be integer scalars")
        return CameraCalibration(
            K=K,
            D=D,
            frame_width=_positive_integer(int(width_value), "image_width"),
            frame_height=_positive_integer(int(height_value), "image_height"),
            source_path=str(path),
            source_format="opencv_yaml",
        )
    except (cv2.error, SystemError) as error:
        raise ValueError(f"Malformed OpenCV YAML calibration: {path}") from error
    finally:
        storage.release()


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    """Load K/D and calibration resolution from NPZ or OpenCV YAML."""
    calibration_path = Path(path).expanduser().resolve()
    if not calibration_path.is_file():
        raise ValueError(f"Camera calibration file does not exist: {calibration_path}")
    suffix = calibration_path.suffix.lower()
    if suffix == ".npz":
        return _load_npz(calibration_path)
    if suffix in {".yaml", ".yml"}:
        return _load_opencv_yaml(calibration_path)
    raise ValueError("Expected a camera calibration file ending in .npz, .yaml, or .yml")


def validate_calibration_resolution(
    calibration: CameraCalibration,
    frame_width: int,
    frame_height: int,
) -> None:
    """Require exact dimensions; K is never silently scaled."""
    if not isinstance(calibration, CameraCalibration):
        raise TypeError("calibration must be a CameraCalibration")
    width = _positive_integer(frame_width, "frame_width")
    height = _positive_integer(frame_height, "frame_height")
    if (width, height) != (calibration.frame_width, calibration.frame_height):
        raise ValueError(
            "Frame resolution does not match camera calibration: "
            f"frame={width}x{height}, "
            f"calibration={calibration.frame_width}x{calibration.frame_height}; "
            "automatic K scaling is not supported"
        )


def undistort_image_points(
    image_points: ArrayLike,
    calibration: CameraCalibration,
) -> np.ndarray:
    """Undistort sparse pixel points while preserving pixel units and frame size."""
    if not isinstance(calibration, CameraCalibration):
        raise TypeError("calibration must be a CameraCalibration")
    try:
        points = np.asarray(image_points, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError("image_points must be numeric with shape (N, 2)") from error
    if points.ndim != 2 or points.shape[1] != 2 or points.shape[0] < 1:
        raise ValueError(f"image_points must have shape (N, 2); got {points.shape}")
    if not np.isfinite(points).all():
        raise ValueError("image_points must contain only finite values")
    try:
        result = cv2.undistortPoints(
            np.ascontiguousarray(points.reshape(-1, 1, 2)),
            cameraMatrix=calibration.K,
            distCoeffs=calibration.D,
            P=calibration.K,
        ).reshape(-1, 2)
    except cv2.error as error:
        raise ValueError(f"OpenCV could not undistort image_points: {error}") from error
    if not np.isfinite(result).all():
        raise ValueError("Undistortion produced non-finite image points")
    return np.array(result, dtype=np.float64, copy=True)
