"""Frame-level person detection results and a bounded latest-frame handoff.

The source timestamp is ``ColorFrame.host_receive_time_s``. It is a host-side
read-complete timestamp, not a camera exposure or device-clock timestamp.
Inference/result timestamps use the same host ``perf_counter`` clock domain.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Protocol

from perception.camera import ColorFrame
from perception.person_detector import PersonDetection


class PersonDetectionProcessor(Protocol):
    def process(self, frame: ColorFrame) -> tuple[PersonDetection, ...]: ...


@dataclass(frozen=True)
class PersonDetectionFrame:
    """All person detections and host timing metadata for one processed frame.

    Boxes remain in original-frame pixel coordinates. There are deliberately
    no cross-frame identities or metric target coordinates in this type.
    Empty ``detections`` is a valid completed result for a frame.
    """

    source_id: str
    source_sequence_id: int
    source_time_s: float
    frame_width: int
    frame_height: int
    detections: tuple[PersonDetection, ...]
    inference_start_time_s: float
    inference_end_time_s: float
    result_ready_time_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must not be empty")
        if (isinstance(self.source_sequence_id, bool)
                or not isinstance(self.source_sequence_id, int)
                or self.source_sequence_id < 0):
            raise ValueError("source_sequence_id must be a nonnegative integer")
        for name in ("frame_width", "frame_height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")

        for name in (
            "source_time_s",
            "inference_start_time_s",
            "inference_end_time_s",
            "result_ready_time_s",
        ):
            try:
                value = float(getattr(self, name))
            except (TypeError, ValueError) as error:
                raise ValueError(f"{name} must be finite") from error
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)

        if not (
            self.source_time_s
            <= self.inference_start_time_s
            <= self.inference_end_time_s
            <= self.result_ready_time_s
        ):
            raise ValueError("frame timestamps must be ordered in one host clock domain")

        if not isinstance(self.detections, tuple):
            object.__setattr__(self, "detections", tuple(self.detections))
        for detection in self.detections:
            if not isinstance(detection, PersonDetection):
                raise ValueError("detections must contain PersonDetection values")
            if (detection.source_sequence_id != self.source_sequence_id
                    or detection.source_time_s != self.source_time_s):
                raise ValueError("detection metadata must match its source frame")
            x1, y1, x2, y2 = detection.bbox_xyxy_px
            if x2 > self.frame_width or y2 > self.frame_height:
                raise ValueError("detection bbox must fit inside the source frame")

    @property
    def detector_call_wall_time_s(self) -> float:
        """Time spent inside ``PersonDetector.process`` (includes predict)."""
        return self.inference_end_time_s - self.inference_start_time_s

    @property
    def processing_wall_time_s(self) -> float:
        """Start-to-result-ready latency for the backend-neutral frame result."""
        return self.result_ready_time_s - self.inference_start_time_s

    @property
    def host_receive_to_result_age_s(self) -> float:
        """Read-complete-to-result age, not exposure-to-result latency."""
        return self.result_ready_time_s - self.source_time_s


def process_person_frame(
    frame: ColorFrame,
    detector: PersonDetectionProcessor,
) -> PersonDetectionFrame:
    """Process one canonical color frame using an already-loaded detector."""
    if not isinstance(frame, ColorFrame):
        raise TypeError("frame must be a ColorFrame")

    start_s = time.perf_counter_ns() * 1e-9
    detections = tuple(detector.process(frame))
    end_s = time.perf_counter_ns() * 1e-9
    ready_s = time.perf_counter_ns() * 1e-9
    return PersonDetectionFrame(
        source_id=frame.source_id,
        source_sequence_id=frame.sequence_id,
        source_time_s=frame.host_receive_time_s,
        frame_width=frame.width,
        frame_height=frame.height,
        detections=detections,
        inference_start_time_s=start_s,
        inference_end_time_s=end_s,
        result_ready_time_s=ready_s,
    )


class LatestFrameSlot:
    """Thread-safe capacity-one mailbox; putting a new frame replaces old work.

    ``put`` never waits for the consumer or for inference. ``close`` wakes a
    blocked consumer and preserves the last pending frame so it can be drained.
    """

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._pending: ColorFrame | None = None
        self._closed = False
        self._submitted_count = 0
        self._overwritten_count = 0
        self._taken_count = 0

    def put(self, frame: ColorFrame) -> None:
        if not isinstance(frame, ColorFrame):
            raise TypeError("LatestFrameSlot accepts ColorFrame values only")
        with self._condition:
            if self._closed:
                raise RuntimeError("cannot put a frame into a closed slot")
            self._submitted_count += 1
            if self._pending is not None:
                self._overwritten_count += 1
            self._pending = frame
            self._condition.notify()

    def get(self, timeout_s: float | None = None) -> ColorFrame | None:
        """Take the newest pending frame, or return None on timeout/closed-empty."""
        if timeout_s is not None:
            timeout_s = float(timeout_s)
            if not math.isfinite(timeout_s) or timeout_s < 0:
                raise ValueError("timeout_s must be finite and nonnegative or None")
        with self._condition:
            self._condition.wait_for(
                lambda: self._pending is not None or self._closed,
                timeout=timeout_s,
            )
            frame = self._pending
            if frame is not None:
                self._pending = None
                self._taken_count += 1
            return frame

    def close(self) -> None:
        """Stop new submissions and wake waiters; retain pending work to drain."""
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def submitted_count(self) -> int:
        with self._condition:
            return self._submitted_count

    @property
    def overwritten_count(self) -> int:
        with self._condition:
            return self._overwritten_count

    @property
    def taken_count(self) -> int:
        with self._condition:
            return self._taken_count

    @property
    def pending_count(self) -> int:
        with self._condition:
            return int(self._pending is not None)

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed
