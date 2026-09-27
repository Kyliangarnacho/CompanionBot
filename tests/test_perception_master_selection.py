"""Hardware-free tests for manual Master selection and lifecycle binding."""

from __future__ import annotations

import pytest

from perception.master_selection import (
    MasterEventType,
    MasterManager,
    MasterSelector,
    MasterState,
)
from perception.person_tracker import (
    PersonTrackingDiagnostics,
    PersonTrackingFrame,
    TrackedPerson,
)


def _track(track_id: int, bbox: tuple[float, float, float, float],
           confidence: float = 0.8) -> TrackedPerson:
    return TrackedPerson(
        track_id=track_id,
        bbox_xyxy_px=bbox,
        confidence=confidence,
        class_id=0,
        class_name="person",
    )


def _frame(
    sequence_id: int,
    tracks: tuple[TrackedPerson, ...],
    *,
    lost: tuple[int, ...] = (),
    removed: tuple[int, ...] = (),
) -> PersonTrackingFrame:
    source_time_s = 10.0 + sequence_id * 0.05
    return PersonTrackingFrame(
        source_id="synthetic:master-test",
        source_sequence_id=sequence_id,
        source_time_s=source_time_s,
        frame_width=100,
        frame_height=100,
        detection_count=len(tracks),
        tracks=tracks,
        diagnostics=PersonTrackingDiagnostics(
            update_index=sequence_id + 1,
            active_track_count=len(tracks),
            created_track_ids=(),
            newly_lost_track_ids=lost,
            newly_removed_track_ids=removed,
            source_sequence_gap=0,
            tracker_update_wall_time_s=0.001,
        ),
        tracking_start_time_s=source_time_s,
        tracking_end_time_s=source_time_s,
        result_ready_time_s=source_time_s,
    )


def _select(manager: MasterManager, selector: MasterSelector,
            frame: PersonTrackingFrame, x_px: int, y_px: int):
    manager.update(frame)
    selection = selector.select_at_pixel(frame, x_px, y_px)
    if selection.selected_track_id is not None:
        event = manager.lock(
            frame,
            selection.selected_track_id,
            candidate_track_ids=selection.candidate_track_ids,
            click_x_px=x_px,
            click_y_px=y_px,
        )
    else:
        event = None
    return selection, event


def test_click_inside_track_locks_master_and_preserves_metadata_and_reference():
    track = _track(3, (10, 10, 40, 50))
    frame = _frame(4, (track,))
    manager = MasterManager()
    selector = MasterSelector()

    selection, event = _select(manager, selector, frame, 25, 30)
    result = manager.result(frame)

    assert selection.candidate_track_ids == (3,)
    assert selection.selected_track_id == 3
    assert event is not None and event.event is MasterEventType.MASTER_SELECTED
    assert (event.source_id, event.source_sequence_id, event.source_time_s) == (
        frame.source_id, frame.source_sequence_id, frame.source_time_s
    )
    assert event.old_track_id is None and event.new_track_id == 3
    assert event.candidate_track_ids == (3,)
    assert (event.click_x_px, event.click_y_px) == (25, 30)
    assert result.state is MasterState.LOCKED
    assert result.bound_track_id == 3
    assert result.master_track is track
    assert (result.source_id, result.source_sequence_id, result.source_time_s) == (
        frame.source_id, frame.source_sequence_id, frame.source_time_s
    )
    assert (result.frame_width, result.frame_height) == (frame.frame_width, frame.frame_height)


def test_click_outside_every_box_leaves_master_state_unchanged():
    frame = _frame(0, (_track(3, (5, 5, 25, 25)),))
    manager = MasterManager()
    manager.update(frame)
    manager.lock(frame, 3)
    selection = MasterSelector().select_at_pixel(frame, 80, 80)

    assert selection.candidate_track_ids == ()
    assert selection.selected_track_id is None
    assert manager.result(frame).state is MasterState.LOCKED
    assert manager.result(frame).bound_track_id == 3


def test_click_among_multiple_boxes_selects_the_box_containing_pixel():
    frame = _frame(0, (
        _track(3, (0, 0, 30, 40)),
        _track(7, (60, 20, 95, 80)),
    ))

    selection = MasterSelector().select_at_pixel(frame, 70, 30)

    assert selection.candidate_track_ids == (7,)
    assert selection.selected_track_id == 7


def test_overlapping_boxes_use_nearest_center_then_track_id_tie_break():
    frame = _frame(0, (
        _track(7, (20, 0, 80, 100), confidence=0.99),
        _track(3, (0, 0, 60, 100), confidence=0.1),
    ))
    selector = MasterSelector()

    tied = selector.select_at_pixel(frame, 40, 50)
    nearest_low_confidence = selector.select_at_pixel(frame, 31, 50)
    nearest_right_center = selector.select_at_pixel(frame, 49, 50)

    assert tied.candidate_track_ids == (3, 7)
    assert tied.selected_track_id == 3
    assert nearest_low_confidence.selected_track_id == 3
    assert nearest_right_center.selected_track_id == 7


def test_visible_bound_track_remains_locked():
    manager = MasterManager()
    selector = MasterSelector()
    first = _frame(0, (_track(3, (10, 10, 40, 50)),))
    _select(manager, selector, first, 20, 20)

    next_frame = _frame(1, (_track(3, (11, 10, 41, 50)),))
    assert manager.update(next_frame) == ()
    assert manager.result(next_frame).state is MasterState.LOCKED
    assert manager.result(next_frame).bound_track_id == 3


def test_missing_bound_track_becomes_temporarily_lost_once():
    manager = MasterManager()
    first = _frame(0, (_track(3, (10, 10, 40, 50)),))
    manager.update(first)
    manager.lock(first, 3)

    missing = _frame(1, (), lost=(3,))
    events = manager.update(missing)
    assert len(events) == 1 and events[0].event is MasterEventType.MASTER_TEMP_LOST
    assert events[0].old_track_id == 3 and events[0].new_track_id is None
    assert manager.result(missing).state is MasterState.TEMPORARILY_LOST
    assert manager.result(missing).bound_track_id == 3
    assert manager.result(missing).master_track is None

    still_missing = _frame(2, ())
    assert manager.update(still_missing) == ()
    assert manager.result(still_missing).state is MasterState.TEMPORARILY_LOST


def test_same_track_return_reacquires_without_identity_inference():
    manager = MasterManager()
    first = _frame(0, (_track(3, (10, 10, 40, 50)),))
    manager.update(first)
    manager.lock(first, 3)
    manager.update(_frame(1, (), lost=(3,)))

    returned_track = _track(3, (12, 11, 42, 51))
    returned = _frame(2, (returned_track,))
    events = manager.update(returned)

    assert len(events) == 1
    assert events[0].event is MasterEventType.MASTER_REACQUIRED_SAME_TRACK
    assert events[0].old_track_id == events[0].new_track_id == 3
    assert manager.result(returned).state is MasterState.LOCKED
    assert manager.result(returned).master_track is returned_track


def test_removed_bound_track_becomes_lost():
    manager = MasterManager()
    first = _frame(0, (_track(3, (10, 10, 40, 50)),))
    manager.update(first)
    manager.lock(first, 3)

    removed = _frame(1, (), removed=(3,))
    events = manager.update(removed)

    assert len(events) == 1 and events[0].event is MasterEventType.MASTER_LOST
    assert events[0].old_track_id == 3 and events[0].new_track_id is None
    assert manager.result(removed).state is MasterState.LOST
    assert manager.result(removed).bound_track_id == 3
    assert manager.result(removed).master_track is None


def test_lost_master_never_auto_binds_other_tracks_or_even_reactivates_after_removed():
    manager = MasterManager()
    first = _frame(0, (_track(3, (10, 10, 40, 50)),))
    manager.update(first)
    manager.lock(first, 3)
    manager.update(_frame(1, (), removed=(3,)))

    other_tracks = _frame(2, (
        _track(4, (10, 10, 40, 50)),
        _track(5, (55, 10, 90, 60)),
    ))
    assert manager.update(other_tracks) == ()
    lost_result = manager.result(other_tracks)
    assert lost_result.state is MasterState.LOST
    assert lost_result.bound_track_id == 3
    assert lost_result.master_track is None

    same_numeric_id_after_removal = _frame(3, (_track(3, (10, 10, 40, 50)),))
    assert manager.update(same_numeric_id_after_removal) == ()
    assert manager.result(same_numeric_id_after_removal).state is MasterState.LOST
    assert manager.result(same_numeric_id_after_removal).master_track is None


def test_user_can_reselect_after_lost_and_switch_master_explicitly():
    manager = MasterManager()
    selector = MasterSelector()
    first = _frame(0, (_track(3, (5, 5, 35, 55)),))
    _select(manager, selector, first, 15, 15)
    removed = _frame(1, (), removed=(3,))
    manager.update(removed)

    reselection_frame = _frame(2, (_track(4, (0, 0, 40, 50)), _track(5, (50, 0, 95, 50))))
    selection, event = _select(manager, selector, reselection_frame, 70, 20)
    result = manager.result(reselection_frame)

    assert selection.selected_track_id == 5
    assert event is not None and event.event is MasterEventType.MASTER_SWITCHED
    assert event.old_track_id == 3 and event.new_track_id == 5
    assert result.state is MasterState.LOCKED and result.bound_track_id == 5


def test_user_clicking_another_visible_track_emits_switch_event():
    manager = MasterManager()
    selector = MasterSelector()
    frame = _frame(0, (_track(3, (0, 0, 40, 50)), _track(5, (50, 0, 95, 50))))
    manager.update(frame)
    manager.lock(frame, 3)

    selection = selector.select_at_pixel(frame, 70, 20)
    event = manager.lock(
        frame,
        selection.selected_track_id,
        candidate_track_ids=selection.candidate_track_ids,
        click_x_px=70,
        click_y_px=20,
    )

    assert event is not None and event.event is MasterEventType.MASTER_SWITCHED
    assert event.old_track_id == 3 and event.new_track_id == 5
    assert manager.result(frame).bound_track_id == 5


def test_clear_returns_to_unselected_and_logs_old_binding():
    manager = MasterManager()
    frame = _frame(0, (_track(3, (10, 10, 40, 50)),))
    manager.update(frame)
    manager.lock(frame, 3)

    event = manager.clear(frame)

    assert event is not None and event.event is MasterEventType.MASTER_CLEARED
    assert event.old_track_id == 3 and event.new_track_id is None
    assert manager.result(frame).state is MasterState.UNSELECTED
    assert manager.result(frame).bound_track_id is None
    assert manager.clear(frame) is None


def test_duplicate_track_does_not_change_master_or_rescue_removed_master():
    manager = MasterManager()
    first = _frame(0, (
        _track(3, (5, 5, 45, 60)),
        _track(9, (6, 6, 44, 59)),
    ))
    manager.update(first)
    manager.lock(first, 3)

    both = _frame(1, (
        _track(3, (6, 5, 46, 60)),
        _track(9, (7, 6, 45, 59)),
    ))
    assert manager.update(both) == ()
    assert manager.result(both).bound_track_id == 3

    duplicate_only = _frame(2, (_track(9, (8, 6, 46, 59)),), lost=(3,))
    events = manager.update(duplicate_only)
    assert len(events) == 1 and events[0].event is MasterEventType.MASTER_TEMP_LOST
    assert manager.result(duplicate_only).bound_track_id == 3

    removed = _frame(3, (_track(9, (9, 6, 47, 59)),), removed=(3,))
    events = manager.update(removed)
    assert len(events) == 1 and events[0].event is MasterEventType.MASTER_LOST
    assert manager.result(removed).state is MasterState.LOST
    assert manager.result(removed).bound_track_id == 3


def test_selection_rejects_non_integer_and_manager_rejects_stale_frame():
    frame = _frame(0, (_track(3, (10, 10, 40, 50)),))
    with pytest.raises(TypeError, match="integer source-frame pixels"):
        MasterSelector().select_at_pixel(frame, 12.0, 15)

    manager = MasterManager()
    manager.update(frame)
    later = _frame(1, (_track(3, (10, 10, 40, 50)),))
    manager.update(later)
    with pytest.raises(ValueError, match="most recently observed"):
        manager.lock(frame, 3)
    with pytest.raises(ValueError, match="must increase"):
        manager.update(frame)


def test_pending_lock_confirms_only_after_explicit_confirmation():
    selected = _track(3, (10, 10, 40, 50))
    first = _frame(0, (selected,))
    manager = MasterManager()
    manager.update(first)

    started = manager.begin_pending_lock(first, 3, candidate_track_ids=(3,))
    assert started.event is MasterEventType.MASTER_PENDING_LOCK_STARTED
    assert manager.result(first).state is MasterState.PENDING_LOCK
    assert manager.result(first).bound_track_id == 3
    assert manager.result(first).master_track is None

    stable = _frame(1, (selected,))
    assert manager.update(stable) == ()
    assert manager.result(stable).state is MasterState.PENDING_LOCK
    confirmed = manager.confirm_pending_lock(stable)
    assert confirmed.event is MasterEventType.MASTER_SELECTED
    assert manager.result(stable).state is MasterState.LOCKED
    assert manager.result(stable).master_track is selected


def test_pending_lock_disappearance_cancels_to_unselected():
    first = _frame(0, (_track(3, (10, 10, 40, 50)),))
    manager = MasterManager()
    manager.update(first)
    manager.begin_pending_lock(first, 3)

    missing = _frame(1, (), lost=(3,))
    events = manager.update(missing)

    assert len(events) == 1
    assert events[0].event is MasterEventType.MASTER_PENDING_LOCK_CANCELLED
    assert events[0].old_track_id == 3 and events[0].new_track_id is None
    result = manager.result(missing)
    assert result.state is MasterState.UNSELECTED
    assert result.bound_track_id is None
    assert result.master_track is None


def _start_master_lost() -> MasterManager:
    manager = MasterManager()
    first = _frame(0, (_track(1, (5, 5, 25, 45)),))
    manager.update(first)
    manager.lock(first, 1)
    manager.update(_frame(1, (), removed=(1,)))
    assert manager.state is MasterState.LOST
    return manager


def test_lost_reid_score_below_threshold_never_binds():
    manager = _start_master_lost()
    candidate = _track(2, (30, 5, 50, 45))
    for sequence_id in (2, 3, 4, 5):
        current = _frame(sequence_id, (candidate,))
        manager.update(current)
        assert manager.consider_reid_candidate_scores(
            current, {2: 0.679}, threshold=0.68, stable_updates=3
        ) is None

    assert manager.state is MasterState.LOST
    assert manager.bound_track_id == 1


def test_lost_reid_requires_three_updates_then_binds_new_id_and_emits_event():
    manager = _start_master_lost()
    candidate = _track(2, (30, 5, 50, 45))
    for sequence_id in (2, 3):
        current = _frame(sequence_id, (candidate,))
        manager.update(current)
        assert manager.consider_reid_candidate_scores(
            current, {2: 0.68}, threshold=0.68, stable_updates=3
        ) is None
        assert manager.result(current).state is MasterState.LOST
        assert manager.bound_track_id == 1

    third = _frame(4, (candidate,))
    manager.update(third)
    event = manager.consider_reid_candidate_scores(
        third, {2: 0.68}, threshold=0.68, stable_updates=3
    )

    assert event is not None
    assert event.event is MasterEventType.MASTER_REID_REACQUIRED
    assert event.old_track_id == 1 and event.new_track_id == 2
    assert event.similarity == pytest.approx(0.68)
    assert event.source_sequence_id == third.source_sequence_id
    result = manager.result(third)
    assert result.state is MasterState.LOCKED
    assert result.bound_track_id == 2
    assert result.master_track is candidate
