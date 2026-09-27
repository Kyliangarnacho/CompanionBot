"""Backend-neutral online tracking contract for person detection frames.

Track IDs are temporary tracklet labels scoped to one tracker session. They do
not identify a person and must not be used as identity or Master-selection IDs.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Protocol, runtime_checkable

from perception.detection_stream import PersonDetectionFrame
from perception.person_detector import PersonDetection


@dataclass(frozen=True)
class BackendTrack:
    """Backend-neutral active track in original source-frame pixel coordinates."""

    track_id: int
    bbox_xyxy_px: tuple[float, float, float, float]
    confidence: float
    class_id: int


@dataclass(frozen=True)
class TrackerBackendUpdate:
    """Portable results and lifecycle events returned by a tracker backend."""

    tracks: tuple[BackendTrack, ...]
    newly_lost_track_ids: tuple[int, ...] = ()
    newly_removed_track_ids: tuple[int, ...] = ()


@runtime_checkable
class TrackingBackend(Protocol):
    """Minimal adapter surface; camera and detector objects are not accepted."""

    @property
    def name(self) -> str: ...

    def update(
        self,
        detections: tuple[PersonDetection, ...],
        frame_width: int,
        frame_height: int,
    ) -> TrackerBackendUpdate: ...

    def reset(self) -> None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class TrackedPerson:
    """One active tracklet, with a temporary backend-assigned ID.

    The box uses ``(x1, y1, x2, y2)`` pixels in the original source frame,
    where x increases right and y increases down. Track IDs are ephemeral and
    have no person-identity semantics.
    """

    track_id: int
    bbox_xyxy_px: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str

    def __post_init__(self) -> None:
        if (isinstance(self.track_id, bool) or not isinstance(self.track_id, int)
                or self.track_id <= 0):
            raise ValueError("track_id must be a positive temporary tracklet ID")
        try:
            bbox = tuple(float(value) for value in self.bbox_xyxy_px)
        except (TypeError, ValueError) as error:
            raise ValueError("bbox_xyxy_px must contain four finite pixel values") from error
        if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
            raise ValueError("bbox_xyxy_px must contain four finite pixel values")
        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            raise ValueError("bbox_xyxy_px must have positive area")
        object.__setattr__(self, "bbox_xyxy_px", bbox)
        try:
            confidence = float(self.confidence)
        except (TypeError, ValueError) as error:
            raise ValueError("confidence must be finite in [0, 1]") from error
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be finite in [0, 1]")
        object.__setattr__(self, "confidence", confidence)
        if (isinstance(self.class_id, bool) or not isinstance(self.class_id, int)
                or self.class_id < 0):
            raise ValueError("class_id must be a nonnegative integer")
        if not isinstance(self.class_name, str) or self.class_name.strip().casefold() != "person":
            raise ValueError("TrackedPerson class_name must be 'person'")


@dataclass(frozen=True)
class PersonTrackingDiagnostics:
    """Per-update lifecycle and timing measurements, not person identity data."""

    update_index: int
    active_track_count: int
    created_track_ids: tuple[int, ...]
    newly_lost_track_ids: tuple[int, ...]
    newly_removed_track_ids: tuple[int, ...]
    source_sequence_gap: int
    tracker_update_wall_time_s: float

    def __post_init__(self) -> None:
        if isinstance(self.update_index, bool) or not isinstance(self.update_index, int) \
                or self.update_index < 1:
            raise ValueError("update_index must be a positive integer")
        if isinstance(self.active_track_count, bool) or not isinstance(self.active_track_count, int) \
                or self.active_track_count < 0:
            raise ValueError("active_track_count must be a nonnegative integer")
        if (isinstance(self.source_sequence_gap, bool)
                or not isinstance(self.source_sequence_gap, int)
                or self.source_sequence_gap < 0):
            raise ValueError("source_sequence_gap must be a nonnegative integer")
        for name in ("created_track_ids", "newly_lost_track_ids", "newly_removed_track_ids"):
            ids = tuple(getattr(self, name))
            if any(isinstance(value, bool) or not isinstance(value, int) or value <= 0
                   for value in ids):
                raise ValueError(f"{name} must contain positive track IDs")
            if len(ids) != len(set(ids)):
                raise ValueError(f"{name} must not contain duplicate IDs")
            object.__setattr__(self, name, ids)
        elapsed = float(self.tracker_update_wall_time_s)
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError("tracker_update_wall_time_s must be finite and nonnegative")
        object.__setattr__(self, "tracker_update_wall_time_s", elapsed)


@dataclass(frozen=True)
class PersonTrackingFrame:
    """Tracking output for exactly one input detection frame, including empty output."""

    source_id: str
    source_sequence_id: int
    source_time_s: float
    frame_width: int
    frame_height: int
    detection_count: int
    tracks: tuple[TrackedPerson, ...]
    diagnostics: PersonTrackingDiagnostics
    tracking_start_time_s: float
    tracking_end_time_s: float
    result_ready_time_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must not be empty")
        for name in ("source_sequence_id", "frame_width", "frame_height", "detection_count"):
            value = getattr(self, name)
            minimum = 0 if name in {"source_sequence_id", "detection_count"} else 1
            if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
                raise ValueError(f"{name} is outside its valid range")
        if not isinstance(self.tracks, tuple):
            object.__setattr__(self, "tracks", tuple(self.tracks))
        if any(not isinstance(track, TrackedPerson) for track in self.tracks):
            raise ValueError("tracks must contain TrackedPerson values")
        if not isinstance(self.diagnostics, PersonTrackingDiagnostics):
            raise ValueError("diagnostics must be PersonTrackingDiagnostics")
        ids = [track.track_id for track in self.tracks]
        if len(ids) != len(set(ids)):
            raise ValueError("track IDs must be unique within a frame")
        for name in ("source_time_s", "tracking_start_time_s",
                     "tracking_end_time_s", "result_ready_time_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value)
        if not (self.source_time_s <= self.tracking_start_time_s
                <= self.tracking_end_time_s <= self.result_ready_time_s):
            raise ValueError("tracking timestamps must be ordered in one host clock domain")
        if self.diagnostics.active_track_count != len(self.tracks):
            raise ValueError("active_track_count must match tracks")

    @property
    def host_receive_to_result_age_s(self) -> float:
        """Host read-complete-to-tracking-result age, not exposure latency."""
        return self.result_ready_time_s - self.source_time_s


class PersonTracker:
    """Apply one backend to ordered detection frames from a single source stream."""

    def __init__(self, backend: TrackingBackend) -> None:
        if not isinstance(backend, TrackingBackend):
            raise TypeError("backend must implement the TrackingBackend protocol")
        self._backend = backend
        self._source_id: str | None = None
        self._last_sequence_id: int | None = None
        self._last_source_time_s: float | None = None
        self._update_index = 0
        self._seen_track_ids: set[int] = set()
        self._class_names: dict[int, str] = {}
        self._closed = False

    @property
    def backend_name(self) -> str:
        return self._backend.name

    def update(self, detection_frame: PersonDetectionFrame) -> PersonTrackingFrame:
        """Track one real detector result; sequence gaps are recorded, never filled."""
        if self._closed:
            raise RuntimeError("tracker is closed")
        if not isinstance(detection_frame, PersonDetectionFrame):
            raise TypeError("detection_frame must be a PersonDetectionFrame")
        if any(item.class_name.casefold() != "person" for item in detection_frame.detections):
            raise ValueError("PersonTracker accepts person detections only")
        if self._source_id is not None and detection_frame.source_id != self._source_id:
            raise ValueError("source_id changed; reset the tracker before switching sources")
        if (self._last_sequence_id is not None
                and detection_frame.source_sequence_id <= self._last_sequence_id):
            raise ValueError("source_sequence_id must increase; out-of-order frames are rejected")
        if (self._last_source_time_s is not None
                and detection_frame.source_time_s < self._last_source_time_s):
            raise ValueError("source_time_s moved backwards; reset before clock/source changes")

        start_s = time.perf_counter()
        update = self._backend.update(
            detection_frame.detections,
            detection_frame.frame_width,
            detection_frame.frame_height,
        )
        end_s = time.perf_counter()
        if not isinstance(update, TrackerBackendUpdate):
            raise TypeError("tracking backend must return TrackerBackendUpdate")

        for detection in detection_frame.detections:
            self._class_names[detection.class_id] = detection.class_name
        tracks = tuple(
            TrackedPerson(
                track_id=item.track_id,
                bbox_xyxy_px=item.bbox_xyxy_px,
                confidence=item.confidence,
                class_id=item.class_id,
                class_name=self._class_names.get(item.class_id, "person"),
            )
            for item in update.tracks
        )
        current_ids = {item.track_id for item in tracks}
        created_ids = tuple(sorted(current_ids - self._seen_track_ids))
        gap = (0 if self._last_sequence_id is None else
               detection_frame.source_sequence_id - self._last_sequence_id - 1)
        self._update_index += 1
        self._source_id = detection_frame.source_id
        self._last_sequence_id = detection_frame.source_sequence_id
        self._last_source_time_s = detection_frame.source_time_s
        self._seen_track_ids.update(current_ids)
        ready_s = time.perf_counter()
        diagnostics = PersonTrackingDiagnostics(
            update_index=self._update_index,
            active_track_count=len(tracks),
            created_track_ids=created_ids,
            newly_lost_track_ids=tuple(sorted(set(update.newly_lost_track_ids))),
            newly_removed_track_ids=tuple(sorted(set(update.newly_removed_track_ids))),
            source_sequence_gap=gap,
            tracker_update_wall_time_s=end_s - start_s,
        )
        return PersonTrackingFrame(
            source_id=detection_frame.source_id,
            source_sequence_id=detection_frame.source_sequence_id,
            source_time_s=detection_frame.source_time_s,
            frame_width=detection_frame.frame_width,
            frame_height=detection_frame.frame_height,
            detection_count=len(detection_frame.detections),
            tracks=tracks,
            diagnostics=diagnostics,
            tracking_start_time_s=max(start_s, detection_frame.result_ready_time_s),
            tracking_end_time_s=max(end_s, detection_frame.result_ready_time_s),
            result_ready_time_s=max(ready_s, detection_frame.result_ready_time_s),
        )

    def reset(self) -> None:
        """Start a fresh tracklet session and clear sequence/lifecycle state."""
        if self._closed:
            raise RuntimeError("tracker is closed")
        self._backend.reset()
        self._source_id = None
        self._last_sequence_id = None
        self._last_source_time_s = None
        self._update_index = 0
        self._seen_track_ids.clear()
        self._class_names.clear()

    def close(self) -> None:
        """Release backend state; repeated close calls are harmless."""
        if not self._closed:
            self._backend.close()
            self._closed = True
