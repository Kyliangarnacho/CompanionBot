"""On-demand vision over Stage 7 ColorFrame, without owning camera/tracking."""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Protocol

import cv2
from pydantic_ai import BinaryContent

from perception.camera import ColorFrame


@dataclass(frozen=True)
class FrameSnapshot:
    frame: ColorFrame
    valid: bool = True
    reason: str = "valid_rgb"
    time_semantics: str = "host_read_complete"
    clock_domain: str = "host_perf_counter"


@dataclass(frozen=True)
class FrameROI:
    """Source-pixel ROI from exactly the same frame; never a stale tracking box."""
    source_id: str
    sequence_id: int
    source_time_s: float
    xyxy: tuple[int, int, int, int]


class FrameProvider(Protocol):
    def latest(self) -> FrameSnapshot | None: ...


class Stage7FrameBuffer:
    """Non-consuming single-slot tap. Publish from the existing capture owner.

    Do not share Stage 7's consuming LatestFrameSlot with the agent: it would steal
    detector/depth inputs. Source restart requires explicit clear() at the owner.
    """
    def __init__(self) -> None:
        self._snapshot: FrameSnapshot | None = None
        self._lock = threading.Lock()

    def publish(self, frame: ColorFrame, *, valid: bool = True, reason: str = "valid_rgb") -> None:
        with self._lock:
            previous = self._snapshot
            if previous and (previous.frame.source_id != frame.source_id
                             or frame.sequence_id <= previous.frame.sequence_id
                             or frame.host_receive_time_s <= previous.frame.host_receive_time_s):
                raise ValueError("source restart/change or out-of-order frame: clear the buffer first")
            # Stage 7 may reuse its array; retain an independent immutable snapshot.
            pixels = frame.bgr.copy()
            pixels.flags.writeable = False
            copy = ColorFrame(pixels, frame.sequence_id, frame.source_id,
                              frame.width, frame.height, frame.host_receive_time_s)
            self._snapshot = FrameSnapshot(copy, valid, reason)

    def latest(self) -> FrameSnapshot | None:
        with self._lock:
            return self._snapshot

    def clear(self) -> None:
        with self._lock:
            self._snapshot = None


def encode_keyframe(snapshot: FrameSnapshot | None, *, now_s: float,
                    max_age_s: float, roi: FrameROI | None = None,
                    roi_xyxy: tuple[int, int, int, int] | None = None) -> tuple[BinaryContent, dict]:
    if snapshot is None:
        raise ValueError("frame_unavailable")
    if not isinstance(snapshot, FrameSnapshot) or not isinstance(snapshot.frame, ColorFrame):
        raise ValueError("frame_contract_mismatch")
    frame = snapshot.frame
    age = now_s - frame.host_receive_time_s
    if (snapshot.valid is not True or snapshot.time_semantics != "host_read_complete"
            or snapshot.clock_domain != "host_perf_counter"):
        raise ValueError("frame_invalid_or_clock_unsupported")
    if not math.isfinite(age) or not 0 <= age <= max_age_s:
        raise ValueError("frame_stale_or_future")
    pixels = frame.bgr
    if roi_xyxy is not None:
        if roi is not None:
            raise ValueError("ambiguous_roi")
        roi = FrameROI(frame.source_id, frame.sequence_id, frame.host_receive_time_s, roi_xyxy)
    if roi:
        if (roi.source_id, roi.sequence_id, roi.source_time_s) != (
                frame.source_id, frame.sequence_id, frame.host_receive_time_s):
            raise ValueError("roi_source_mismatch")
        if any(type(v) is not int for v in roi.xyxy) or len(roi.xyxy) != 4:
            raise ValueError("invalid_roi")
        x1, y1, x2, y2 = roi.xyxy
        if not (0 <= x1 < x2 <= frame.width and 0 <= y1 < y2 <= frame.height):
            raise ValueError("invalid_roi")
        pixels = pixels[y1:y2, x1:x2]
    ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        raise ValueError("frame_encode_failed")
    return BinaryContent(data=encoded.tobytes(), media_type="image/jpeg"), {
        "source_id": frame.source_id, "source_sequence_id": frame.sequence_id,
        "source_time_s": frame.host_receive_time_s, "time_semantics": snapshot.time_semantics,
        "width": frame.width, "height": frame.height, "selected_age_s": age,
        "roi_xyxy": list(roi.xyxy) if roi else None,
        "frame_contract_version": 1, "pixel_format": "bgr8", "image_encoding": "jpeg",
        "clock_domain": snapshot.clock_domain, "roi_space": "source_pixels",
        "encoded_width": int(pixels.shape[1]), "encoded_height": int(pixels.shape[0]),
        "valid": True, "validity_reason": snapshot.reason, "validity_scope": "raw_rgb_contract",
    }
