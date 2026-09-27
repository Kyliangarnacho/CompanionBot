"""Minimal OpenCV RGB camera capture with an explicit BGR frame contract.

Adapted from MonoTeach's ``stage2/camera_stream.py``. ``host_receive_time_s``
is sampled with the host's high-resolution monotonic performance clock after
``VideoCapture.read()`` returns; it is not a sensor exposure timestamp and is
not mapped to another clock. Equal clock readings are advanced by one
nanosecond to keep the public timestamp strictly ordered.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Union

import cv2
import numpy as np


DeviceIdentifier = Union[int, str]


def _positive_optional_integer(value: object, name: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{name} must be a positive integer or None")
    return value


@dataclass(frozen=True)
class CameraConfig:
    """Requested OpenCV source and capture profile.

    ``device`` must be supplied by the caller. Integer indices are OpenCV's
    current enumeration only, not stable physical camera identifiers. Leaving
    ``backend`` unset asks OpenCV to select an available backend (CAP_ANY).
    """

    device: DeviceIdentifier
    requested_width: int | None = None
    requested_height: int | None = None
    requested_fps: float | None = None
    backend: int | None = None

    def __post_init__(self) -> None:
        if isinstance(self.device, bool) or not isinstance(self.device, (int, str)):
            raise ValueError("device must be an OpenCV integer index or source string")
        if isinstance(self.device, int) and self.device < 0:
            raise ValueError("integer device index must be nonnegative")
        if isinstance(self.device, str) and not self.device.strip():
            raise ValueError("string device identifier must not be empty")
        width = _positive_optional_integer(self.requested_width, "requested_width")
        height = _positive_optional_integer(self.requested_height, "requested_height")
        if (width is None) != (height is None):
            raise ValueError("requested_width and requested_height must be set together")
        if self.requested_fps is not None:
            try:
                requested_fps = float(self.requested_fps)
            except (TypeError, ValueError) as error:
                raise ValueError("requested_fps must be finite and positive") from error
            if not math.isfinite(requested_fps) or requested_fps <= 0:
                raise ValueError("requested_fps must be finite and positive")
            object.__setattr__(self, "requested_fps", requested_fps)
        if self.backend is not None and (
            isinstance(self.backend, bool)
            or not isinstance(self.backend, int)
            or self.backend < 0
        ):
            raise ValueError("backend must be a nonnegative OpenCV backend ID or None")


@dataclass(frozen=True)
class CameraProfile:
    """OpenCV-reported values after open; frame dimensions are authoritative."""

    device_identifier: str
    backend_name: str
    requested_width: int | None
    requested_height: int | None
    requested_fps: float | None
    reported_width: int
    reported_height: int
    reported_fps: float


@dataclass(frozen=True)
class ColorFrame:
    """One raw OpenCV BGR frame and its host-side receipt metadata.

    The array contract is H×W×3 ``uint8`` BGR. Width and height describe the
    array itself. ``source_id`` is the configured OpenCV source identifier; an
    integer index is not guaranteed to identify the same hardware after
    re-enumeration.
    """

    bgr: np.ndarray
    sequence_id: int
    source_id: str
    width: int
    height: int
    host_receive_time_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.bgr, np.ndarray):
            raise ValueError("frame must be a NumPy array")
        if self.bgr.ndim != 3 or self.bgr.shape[2] != 3:
            raise ValueError("frame must have shape H×W×3")
        if self.bgr.dtype != np.uint8:
            raise ValueError("frame must use uint8 pixels")
        if self.width != self.bgr.shape[1] or self.height != self.bgr.shape[0]:
            raise ValueError("frame width/height must match the image array")
        if (isinstance(self.sequence_id, bool)
                or not isinstance(self.sequence_id, int)
                or isinstance(self.width, bool) or not isinstance(self.width, int)
                or isinstance(self.height, bool) or not isinstance(self.height, int)
                or self.width < 1 or self.height < 1 or self.sequence_id < 0):
            raise ValueError("frame dimensions and sequence_id must be valid")
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must not be empty")
        if not math.isfinite(float(self.host_receive_time_s)):
            raise ValueError("host_receive_time_s must be finite")


class CameraStream:
    """Own one OpenCV capture handle and return validated BGR frames."""

    def __init__(self, config: CameraConfig) -> None:
        self.config = config
        self._capture = None
        self._profile: CameraProfile | None = None
        self._next_sequence_id = 0
        self._last_host_receive_time_ns: int | None = None

    @property
    def is_open(self) -> bool:
        return self._capture is not None and self._capture.isOpened()

    @property
    def profile(self) -> CameraProfile:
        """Return OpenCV's reported profile after a successful open."""
        if self._profile is None:
            raise RuntimeError("Camera profile is unavailable before open().")
        return self._profile

    def open(self) -> CameraProfile:
        """Open the configured source, optionally requesting profile values."""
        if self.is_open:
            return self.profile
        self.release()

        if self.config.backend is None:
            capture = cv2.VideoCapture(self.config.device)
        else:
            capture = cv2.VideoCapture(self.config.device, self.config.backend)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                "Could not open OpenCV source "
                f"{self.config.device!r} (backend={self.config.backend!r})."
            )

        try:
            if self.config.requested_width is not None:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.requested_width)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.requested_height)
            if self.config.requested_fps is not None:
                capture.set(cv2.CAP_PROP_FPS, float(self.config.requested_fps))
            try:
                backend_name = str(capture.getBackendName())
            except (AttributeError, cv2.error):
                backend_name = "UNKNOWN"
            self._profile = CameraProfile(
                device_identifier=str(self.config.device),
                backend_name=backend_name,
                requested_width=self.config.requested_width,
                requested_height=self.config.requested_height,
                requested_fps=(None if self.config.requested_fps is None
                               else float(self.config.requested_fps)),
                reported_width=int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH))),
                reported_height=int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))),
                reported_fps=float(capture.get(cv2.CAP_PROP_FPS)),
            )
        except Exception:
            capture.release()
            self._capture = None
            self._profile = None
            raise

        self._capture = capture
        self._next_sequence_id = 0
        self._last_host_receive_time_ns = None
        return self._profile

    def read(self) -> ColorFrame:
        """Return one BGR frame; timestamp read completion on the host clock."""
        if not self.is_open or self._capture is None:
            raise RuntimeError("Cannot read frame: camera is not open.")

        ok, image = self._capture.read()
        host_receive_time_ns = time.perf_counter_ns()
        if not ok or image is None:
            raise RuntimeError("Camera frame read failed.")
        if (not isinstance(image, np.ndarray) or image.ndim != 3
                or image.shape[2] != 3 or image.dtype != np.uint8):
            raise RuntimeError("OpenCV source did not return an H×W×3 uint8 frame.")

        # Some clocks can return the same tick for adjacent reads. Preserve the
        # host-clock semantics while making the API's ordering guarantee true.
        if (self._last_host_receive_time_ns is not None
                and host_receive_time_ns <= self._last_host_receive_time_ns):
            host_receive_time_ns = self._last_host_receive_time_ns + 1
        self._last_host_receive_time_ns = host_receive_time_ns

        frame = ColorFrame(
            bgr=image,
            sequence_id=self._next_sequence_id,
            source_id=str(self.config.device),
            width=int(image.shape[1]),
            height=int(image.shape[0]),
            host_receive_time_s=host_receive_time_ns * 1e-9,
        )
        self._next_sequence_id += 1
        return frame

    def release(self) -> None:
        """Release the capture; repeated calls are safe."""
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        self._profile = None
        self._last_host_receive_time_ns = None

    def __enter__(self) -> "CameraStream":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.release()
