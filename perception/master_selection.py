"""Manual Master binding and narrowly gated OSNet identity reacquisition.

Track IDs are temporary tracker labels. A new ID is accepted only after an
explicit pending user selection or the configured ReID candidate gate.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
import math

from perception.person_tracker import PersonTrackingFrame, TrackedPerson


class MasterState(str, Enum):
    UNSELECTED = "UNSELECTED"
    PENDING_LOCK = "PENDING_LOCK"
    LOCKED = "LOCKED"
    TEMPORARILY_LOST = "TEMPORARILY_LOST"
    LOST = "LOST"


class MasterEventType(str, Enum):
    MASTER_PENDING_LOCK_STARTED = "MASTER_PENDING_LOCK_STARTED"
    MASTER_PENDING_LOCK_CANCELLED = "MASTER_PENDING_LOCK_CANCELLED"
    MASTER_SELECTED = "MASTER_SELECTED"
    MASTER_SWITCHED = "MASTER_SWITCHED"
    MASTER_TEMP_LOST = "MASTER_TEMP_LOST"
    MASTER_REACQUIRED_SAME_TRACK = "MASTER_REACQUIRED_SAME_TRACK"
    MASTER_LOST = "MASTER_LOST"
    MASTER_CLEARED = "MASTER_CLEARED"
    MASTER_REID_REACQUIRED = "MASTER_REID_REACQUIRED"


@dataclass(frozen=True)
class MasterSelection:
    """The result of hit-testing one click in original source-frame pixels."""

    source_sequence_id: int
    click_x_px: int
    click_y_px: int
    candidate_track_ids: tuple[int, ...]
    selected_track_id: int | None


@dataclass(frozen=True)
class MasterEvent:
    """One user selection or tracker lifecycle transition; never a per-frame row."""

    source_id: str
    source_sequence_id: int
    source_time_s: float
    old_track_id: int | None
    new_track_id: int | None
    event: MasterEventType
    candidate_track_ids: tuple[int, ...] = ()
    click_x_px: int | None = None
    click_y_px: int | None = None
    similarity: float | None = None


@dataclass(frozen=True)
class MasterTrackingFrame:
    """Tracking metadata plus the current business-level Master binding."""

    source_id: str
    source_sequence_id: int
    source_time_s: float
    frame_width: int
    frame_height: int
    state: MasterState
    bound_track_id: int | None
    master_track: TrackedPerson | None

    def __post_init__(self) -> None:
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must not be empty")
        if (isinstance(self.source_sequence_id, bool)
                or not isinstance(self.source_sequence_id, int)
                or self.source_sequence_id < 0):
            raise ValueError("source_sequence_id must be a nonnegative integer")
        if (isinstance(self.frame_width, bool) or not isinstance(self.frame_width, int)
                or self.frame_width < 1 or isinstance(self.frame_height, bool)
                or not isinstance(self.frame_height, int) or self.frame_height < 1):
            raise ValueError("frame dimensions must be positive integers")
        source_time_s = float(self.source_time_s)
        if not math.isfinite(source_time_s):
            raise ValueError("source_time_s must be finite")
        object.__setattr__(self, "source_time_s", source_time_s)
        if not isinstance(self.state, MasterState):
            raise ValueError("state must be a MasterState")
        if self.bound_track_id is not None and (
            isinstance(self.bound_track_id, bool)
            or not isinstance(self.bound_track_id, int)
            or self.bound_track_id <= 0
        ):
            raise ValueError("bound_track_id must be a positive track ID or None")
        if self.master_track is not None and not isinstance(self.master_track, TrackedPerson):
            raise ValueError("master_track must be a TrackedPerson or None")
        if self.state is MasterState.UNSELECTED:
            if self.bound_track_id is not None or self.master_track is not None:
                raise ValueError("UNSELECTED cannot have a bound or visible track")
        elif self.bound_track_id is None:
            raise ValueError(f"{self.state.value} requires a bound_track_id")
        if self.state is MasterState.LOCKED:
            if self.master_track is None or self.master_track.track_id != self.bound_track_id:
                raise ValueError("LOCKED requires the visible track matching bound_track_id")
        elif self.master_track is not None:
            raise ValueError("master_track is only populated while LOCKED")


class MasterSelector:
    """Select the tracked box hit by a click in original frame pixel space."""

    def select_at_pixel(
        self,
        tracking_frame: PersonTrackingFrame,
        x_px: int,
        y_px: int,
    ) -> MasterSelection:
        if not isinstance(tracking_frame, PersonTrackingFrame):
            raise TypeError("tracking_frame must be a PersonTrackingFrame")
        if (isinstance(x_px, bool) or not isinstance(x_px, int)
                or isinstance(y_px, bool) or not isinstance(y_px, int)):
            raise TypeError("click coordinates must be integer source-frame pixels")

        candidates: list[TrackedPerson] = []
        if 0 <= x_px < tracking_frame.frame_width and 0 <= y_px < tracking_frame.frame_height:
            for track in tracking_frame.tracks:
                x1, y1, x2, y2 = track.bbox_xyxy_px
                if x1 <= x_px <= x2 and y1 <= y_px <= y2:
                    candidates.append(track)
        candidates.sort(key=lambda track: track.track_id)

        selected_track_id: int | None = None
        if candidates:
            selected_track_id = min(
                candidates,
                key=lambda track: (
                    (0.5 * (track.bbox_xyxy_px[0] + track.bbox_xyxy_px[2]) - x_px) ** 2
                    + (0.5 * (track.bbox_xyxy_px[1] + track.bbox_xyxy_px[3]) - y_px) ** 2,
                    track.track_id,
                ),
            ).track_id
        return MasterSelection(
            source_sequence_id=tracking_frame.source_sequence_id,
            click_x_px=x_px,
            click_y_px=y_px,
            candidate_track_ids=tuple(track.track_id for track in candidates),
            selected_track_id=selected_track_id,
        )


class MasterManager:
    """Track one selected Master with explicit confirmation for ID changes."""

    def __init__(self) -> None:
        self._state = MasterState.UNSELECTED
        self._bound_track_id: int | None = None
        self._last_frame: PersonTrackingFrame | None = None
        self._pending_previous_track_id: int | None = None
        self._reid_candidate_streaks: dict[int, tuple[int, int]] = {}

    @property
    def state(self) -> MasterState:
        return self._state

    @property
    def bound_track_id(self) -> int | None:
        return self._bound_track_id

    def update(self, tracking_frame: PersonTrackingFrame) -> tuple[MasterEvent, ...]:
        """Project this frame's active/lost/removed diagnostics into Master state."""
        if not isinstance(tracking_frame, PersonTrackingFrame):
            raise TypeError("tracking_frame must be a PersonTrackingFrame")
        previous = self._last_frame
        if previous is not None:
            if tracking_frame.source_id != previous.source_id:
                raise ValueError("source_id changed; clear/reset Master before switching sources")
            if tracking_frame.source_sequence_id <= previous.source_sequence_id:
                raise ValueError("source_sequence_id must increase")
        self._last_frame = tracking_frame

        track_ids = {track.track_id for track in tracking_frame.tracks}
        removed_ids = set(tracking_frame.diagnostics.newly_removed_track_ids)
        if self._state is MasterState.PENDING_LOCK:
            pending_track_id = self._bound_track_id
            if pending_track_id in track_ids:
                return ()
            self._state = MasterState.UNSELECTED
            self._bound_track_id = None
            self._pending_previous_track_id = None
            self._reid_candidate_streaks.clear()
            return (self._event(
                tracking_frame, pending_track_id, None,
                MasterEventType.MASTER_PENDING_LOCK_CANCELLED,
            ),)

        if self._state is not MasterState.LOST:
            self._reid_candidate_streaks.clear()
        if self._bound_track_id is None or self._state is MasterState.UNSELECTED:
            return ()

        old_track_id = self._bound_track_id
        if self._state is MasterState.LOST:
            self._advance_reid_candidate_streaks(tracking_frame)
            return ()
        if old_track_id in removed_ids:
            self._state = MasterState.LOST
            self._advance_reid_candidate_streaks(tracking_frame)
            return (self._event(tracking_frame, old_track_id, None,
                                MasterEventType.MASTER_LOST),)
        if old_track_id in track_ids:
            if self._state is MasterState.TEMPORARILY_LOST:
                self._state = MasterState.LOCKED
                return (self._event(
                    tracking_frame, old_track_id, old_track_id,
                    MasterEventType.MASTER_REACQUIRED_SAME_TRACK,
                ),)
            self._state = MasterState.LOCKED
            return ()
        if self._state is MasterState.LOCKED:
            self._state = MasterState.TEMPORARILY_LOST
            return (self._event(
                tracking_frame, old_track_id, None,
                MasterEventType.MASTER_TEMP_LOST,
            ),)
        return ()

    def lock(
        self,
        tracking_frame: PersonTrackingFrame,
        track_id: int,
        *,
        candidate_track_ids: tuple[int, ...] = (),
        click_x_px: int | None = None,
        click_y_px: int | None = None,
    ) -> MasterEvent | None:
        """Bind to a currently visible track after an explicit user selection."""
        self._require_current_frame(tracking_frame)
        if isinstance(track_id, bool) or not isinstance(track_id, int) or track_id <= 0:
            raise ValueError("track_id must be a positive integer")
        if not any(track.track_id == track_id for track in tracking_frame.tracks):
            raise ValueError("Master can only be locked to a track in the current frame")
        if (click_x_px is None) != (click_y_px is None):
            raise ValueError("click_x_px and click_y_px must be supplied together")
        if click_x_px is not None and (
            isinstance(click_x_px, bool) or not isinstance(click_x_px, int)
            or isinstance(click_y_px, bool) or not isinstance(click_y_px, int)
        ):
            raise ValueError("click coordinates must be integer source-frame pixels")
        candidates = tuple(sorted(set(candidate_track_ids or (track_id,))))
        if track_id not in candidates:
            raise ValueError("candidate_track_ids must include the selected track_id")

        old_track_id = self._bound_track_id
        old_state = self._state
        if old_track_id == track_id and old_state is MasterState.LOCKED:
            return None
        self._bound_track_id = track_id
        self._state = MasterState.LOCKED
        self._pending_previous_track_id = None
        self._reid_candidate_streaks.clear()
        event_type = (
            MasterEventType.MASTER_SWITCHED
            if old_track_id is not None and old_track_id != track_id
            else MasterEventType.MASTER_SELECTED
        )
        return self._event(
            tracking_frame,
            old_track_id,
            track_id,
            event_type,
            candidate_track_ids=candidates,
            click_x_px=click_x_px,
            click_y_px=click_y_px,
        )

    def begin_pending_lock(
        self,
        tracking_frame: PersonTrackingFrame,
        track_id: int,
        *,
        candidate_track_ids: tuple[int, ...] = (),
        click_x_px: int | None = None,
        click_y_px: int | None = None,
    ) -> MasterEvent:
        """Enter PENDING_LOCK for a currently visible manually selected track."""
        self._require_current_frame(tracking_frame)
        self._validate_selection(
            tracking_frame, track_id, candidate_track_ids, click_x_px, click_y_px
        )
        old_track_id = (
            self._pending_previous_track_id
            if self._state is MasterState.PENDING_LOCK
            else self._bound_track_id
        )
        self._pending_previous_track_id = old_track_id
        self._bound_track_id = track_id
        self._state = MasterState.PENDING_LOCK
        self._reid_candidate_streaks.clear()
        return self._event(
            tracking_frame,
            old_track_id,
            track_id,
            MasterEventType.MASTER_PENDING_LOCK_STARTED,
            candidate_track_ids=tuple(sorted(set(candidate_track_ids or (track_id,)))),
            click_x_px=click_x_px,
            click_y_px=click_y_px,
        )

    def confirm_pending_lock(
        self, tracking_frame: PersonTrackingFrame
    ) -> MasterEvent:
        """Confirm a pending user selection after its appearance reference is ready."""
        self._require_current_frame(tracking_frame)
        if self._state is not MasterState.PENDING_LOCK or self._bound_track_id is None:
            raise RuntimeError("Master is not in PENDING_LOCK")
        track_id = self._bound_track_id
        if not any(track.track_id == track_id for track in tracking_frame.tracks):
            raise ValueError("pending track is not active in the current frame")
        previous_track_id = self._pending_previous_track_id
        self._state = MasterState.LOCKED
        self._pending_previous_track_id = None
        event_type = (
            MasterEventType.MASTER_SWITCHED
            if previous_track_id is not None and previous_track_id != track_id
            else MasterEventType.MASTER_SELECTED
        )
        return self._event(
            tracking_frame, previous_track_id, track_id, event_type
        )

    def consider_reid_candidate_scores(
        self,
        tracking_frame: PersonTrackingFrame,
        similarities_by_track_id: Mapping[int, float],
        *,
        threshold: float = 0.68,
        stable_updates: int = 3,
    ) -> MasterEvent | None:
        """Rebind from LOST after a scored candidate persists across updates."""
        self._require_current_frame(tracking_frame)
        threshold = float(threshold)
        if not math.isfinite(threshold) or not -1.0 <= threshold <= 1.0:
            raise ValueError("threshold must be finite and in [-1, 1]")
        if (isinstance(stable_updates, bool) or not isinstance(stable_updates, int)
                or stable_updates < 1):
            raise ValueError("stable_updates must be a positive integer")
        for track_id, similarity in similarities_by_track_id.items():
            if isinstance(track_id, bool) or not isinstance(track_id, int) or track_id <= 0:
                raise ValueError("similarity keys must be positive track IDs")
            similarity = float(similarity)
            if not math.isfinite(similarity) or not -1.0 <= similarity <= 1.0:
                raise ValueError("similarities must be finite and in [-1, 1]")

        if self._state is not MasterState.LOST or self._bound_track_id is None:
            self._reid_candidate_streaks.clear()
            return None

        active_candidate_ids = sorted(
            track.track_id for track in tracking_frame.tracks
            if track.track_id != self._bound_track_id
        )
        qualified: list[tuple[int, float]] = []
        for track_id in active_candidate_ids:
            similarity = similarities_by_track_id.get(track_id)
            streak = self._reid_candidate_streaks.get(track_id)
            if (similarity is None or float(similarity) < threshold or streak is None
                    or streak[0] < stable_updates
                    or streak[1] != tracking_frame.diagnostics.update_index):
                continue
            qualified.append((track_id, float(similarity)))
        if not qualified:
            return None

        new_track_id, similarity = min(qualified, key=lambda item: item[0])
        old_track_id = self._bound_track_id
        self._bound_track_id = new_track_id
        self._state = MasterState.LOCKED
        self._reid_candidate_streaks.clear()
        return self._event(
            tracking_frame,
            old_track_id,
            new_track_id,
            MasterEventType.MASTER_REID_REACQUIRED,
            similarity=similarity,
        )

    def stable_reid_candidate_ids(
        self,
        tracking_frame: PersonTrackingFrame,
        *,
        stable_updates: int = 3,
    ) -> tuple[int, ...]:
        """Return currently active LOST candidates stable for the requested updates."""
        self._require_current_frame(tracking_frame)
        if (isinstance(stable_updates, bool) or not isinstance(stable_updates, int)
                or stable_updates < 1):
            raise ValueError("stable_updates must be a positive integer")
        if self._state is not MasterState.LOST or self._bound_track_id is None:
            return ()
        update_index = tracking_frame.diagnostics.update_index
        return tuple(sorted(
            track_id for track_id, (count, last_update_index)
            in self._reid_candidate_streaks.items()
            if count >= stable_updates and last_update_index == update_index
            and track_id != self._bound_track_id
        ))

    def _advance_reid_candidate_streaks(
        self, tracking_frame: PersonTrackingFrame
    ) -> None:
        """Count consecutive tracker updates for active non-Master IDs in LOST."""
        update_index = tracking_frame.diagnostics.update_index
        active_ids = {
            track.track_id for track in tracking_frame.tracks
            if track.track_id != self._bound_track_id
        }
        next_streaks: dict[int, tuple[int, int]] = {}
        for track_id in active_ids:
            previous = self._reid_candidate_streaks.get(track_id)
            count = (
                previous[0] + 1
                if previous is not None and previous[1] == update_index - 1
                else 1
            )
            next_streaks[track_id] = (count, update_index)
        self._reid_candidate_streaks = next_streaks

    def clear(self, tracking_frame: PersonTrackingFrame) -> MasterEvent | None:
        """Clear an existing binding on explicit user request."""
        self._require_current_frame(tracking_frame)
        old_track_id = self._bound_track_id
        if old_track_id is None:
            self._state = MasterState.UNSELECTED
            self._pending_previous_track_id = None
            self._reid_candidate_streaks.clear()
            return None
        self._bound_track_id = None
        self._state = MasterState.UNSELECTED
        self._pending_previous_track_id = None
        self._reid_candidate_streaks.clear()
        return self._event(
            tracking_frame, old_track_id, None, MasterEventType.MASTER_CLEARED
        )

    def result(self, tracking_frame: PersonTrackingFrame) -> MasterTrackingFrame:
        """Build a metadata-preserving Master view without copying TrackedPerson."""
        self._require_current_frame(tracking_frame)
        visible_track = None
        if self._state is MasterState.LOCKED:
            visible_track = next(
                track for track in tracking_frame.tracks
                if track.track_id == self._bound_track_id
            )
        return MasterTrackingFrame(
            source_id=tracking_frame.source_id,
            source_sequence_id=tracking_frame.source_sequence_id,
            source_time_s=tracking_frame.source_time_s,
            frame_width=tracking_frame.frame_width,
            frame_height=tracking_frame.frame_height,
            state=self._state,
            bound_track_id=self._bound_track_id,
            master_track=visible_track,
        )

    def _require_current_frame(self, tracking_frame: PersonTrackingFrame) -> None:
        if not isinstance(tracking_frame, PersonTrackingFrame):
            raise TypeError("tracking_frame must be a PersonTrackingFrame")
        if self._last_frame is None:
            raise RuntimeError("update() must be called before selecting, clearing, or reading")
        if (tracking_frame.source_id != self._last_frame.source_id
                or tracking_frame.source_sequence_id != self._last_frame.source_sequence_id):
            raise ValueError("operation must use the most recently observed tracking frame")

    @staticmethod
    def _validate_selection(
        tracking_frame: PersonTrackingFrame,
        track_id: int,
        candidate_track_ids: tuple[int, ...],
        click_x_px: int | None,
        click_y_px: int | None,
    ) -> None:
        if isinstance(track_id, bool) or not isinstance(track_id, int) or track_id <= 0:
            raise ValueError("track_id must be a positive integer")
        if not any(track.track_id == track_id for track in tracking_frame.tracks):
            raise ValueError("Master can only be locked to a track in the current frame")
        if (click_x_px is None) != (click_y_px is None):
            raise ValueError("click_x_px and click_y_px must be supplied together")
        if click_x_px is not None and (
            isinstance(click_x_px, bool) or not isinstance(click_x_px, int)
            or isinstance(click_y_px, bool) or not isinstance(click_y_px, int)
        ):
            raise ValueError("click coordinates must be integer source-frame pixels")
        candidates = tuple(sorted(set(candidate_track_ids or (track_id,))))
        if track_id not in candidates:
            raise ValueError("candidate_track_ids must include the selected track_id")

    @staticmethod
    def _event(
        tracking_frame: PersonTrackingFrame,
        old_track_id: int | None,
        new_track_id: int | None,
        event: MasterEventType,
        *,
        candidate_track_ids: tuple[int, ...] = (),
        click_x_px: int | None = None,
        click_y_px: int | None = None,
        similarity: float | None = None,
    ) -> MasterEvent:
        return MasterEvent(
            source_id=tracking_frame.source_id,
            source_sequence_id=tracking_frame.source_sequence_id,
            source_time_s=tracking_frame.source_time_s,
            old_track_id=old_track_id,
            new_track_id=new_track_id,
            event=event,
            candidate_track_ids=candidate_track_ids,
            click_x_px=click_x_px,
            click_y_px=click_y_px,
            similarity=similarity,
        )
