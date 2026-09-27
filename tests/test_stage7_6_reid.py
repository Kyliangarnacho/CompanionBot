"""Hardware-free contracts for Stage 7.6 Master reference and reacquisition."""

from __future__ import annotations

import numpy as np
import pytest

from perception.camera import ColorFrame
from perception.master_selection import (
    MasterEventType,
    MasterManager,
    MasterState,
)
from perception.person_reid import (
    BackendEmbedding,
    MasterReIDEvidence,
    PersonReIdentifier,
)
from perception.person_tracker import (
    PersonTrackingDiagnostics,
    PersonTrackingFrame,
    TrackedPerson,
)


class FakeEmbeddingBackend:
    backend_name = "tests.fake"
    model_name = "tests.512d"

    def __init__(self, vectors: dict[int, np.ndarray] | None = None) -> None:
        self.vectors = vectors or {}
        self.calls = 0

    def embed_crop_bgr(self, crop_bgr: np.ndarray) -> BackendEmbedding:
        self.calls += 1
        marker = int(crop_bgr[0, 0, 0])
        return BackendEmbedding(
            self.vectors.get(marker, np.eye(1, 512, 0, dtype=np.float32).reshape(-1)),
            0.001,
            0.002,
        )


def _track(track_id: int, marker_box: tuple[float, float, float, float] | None = None):
    box = marker_box or ((4.0, 4.0, 20.0, 24.0) if track_id == 1
                         else (24.0, 4.0, 40.0, 24.0))
    return TrackedPerson(
        track_id=track_id,
        bbox_xyxy_px=box,
        confidence=0.9,
        class_id=0,
        class_name="person",
    )


def _pair(
    sequence_id: int,
    tracks: tuple[TrackedPerson, ...],
    *,
    time_s: float,
    markers: dict[int, int] | None = None,
    removed: tuple[int, ...] = (),
    lost: tuple[int, ...] = (),
) -> tuple[ColorFrame, PersonTrackingFrame]:
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    for track in tracks:
        marker = (markers or {}).get(track.track_id, 0)
        x1, y1, x2, y2 = (int(value) for value in track.bbox_xyxy_px)
        image[y1:y2, x1:x2, 0] = marker
    frame = ColorFrame(
        bgr=image,
        sequence_id=sequence_id,
        source_id="test-camera",
        width=64,
        height=64,
        host_receive_time_s=time_s,
    )
    tracking = PersonTrackingFrame(
        source_id="test-camera",
        source_sequence_id=sequence_id,
        source_time_s=time_s,
        frame_width=64,
        frame_height=64,
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
        tracking_start_time_s=time_s,
        tracking_end_time_s=time_s,
        result_ready_time_s=time_s,
    )
    return frame, tracking


def _vector(*components: tuple[int, float]) -> np.ndarray:
    result = np.zeros(512, dtype=np.float32)
    for index, value in components:
        result[index] = value
    return result


def test_pending_reference_needs_three_samples_over_03_seconds_before_locked():
    backend = FakeEmbeddingBackend({
        10: _vector((0, 1.0)),
        20: _vector((0, 1.0)),
        30: _vector((1, 1.0)),
    })
    evidence = MasterReIDEvidence(PersonReIdentifier(backend))
    manager = MasterManager()
    selected = _track(1)

    frame0, tracking0 = _pair(0, (selected,), time_s=10.0, markers={1: 10})
    manager.update(tracking0)
    manager.begin_pending_lock(tracking0, 1)
    evidence.begin_pending_reference(frame0, tracking0, 1)
    assert evidence.collect_pending_reference_sample(frame0, tracking0) is None
    assert evidence.pending_sample_count == 1
    assert manager.state is MasterState.PENDING_LOCK

    frame1, tracking1 = _pair(1, (selected,), time_s=10.10, markers={1: 20})
    manager.update(tracking1)
    assert evidence.collect_pending_reference_sample(frame1, tracking1) is None
    assert evidence.pending_sample_count == 1
    assert manager.state is MasterState.PENDING_LOCK

    frame2, tracking2 = _pair(2, (selected,), time_s=10.15, markers={1: 20})
    manager.update(tracking2)
    assert evidence.collect_pending_reference_sample(frame2, tracking2) is None
    assert evidence.pending_sample_count == 2
    assert manager.state is MasterState.PENDING_LOCK

    frame3, tracking3 = _pair(3, (selected,), time_s=10.30, markers={1: 30})
    manager.update(tracking3)
    reference = evidence.collect_pending_reference_sample(frame3, tracking3)
    assert reference is not None
    assert reference.sample_sequence_ids == (0, 2, 3)
    assert reference.embedding.dimension == 512
    assert np.linalg.norm(reference.embedding.vector) == pytest.approx(1.0)
    assert reference.embedding.vector[0] == pytest.approx(2.0 / np.sqrt(5.0))
    assert reference.embedding.vector[1] == pytest.approx(1.0 / np.sqrt(5.0))
    assert backend.calls == 3

    selected_event = manager.confirm_pending_lock(tracking3)
    assert selected_event.event is MasterEventType.MASTER_SELECTED
    assert manager.result(tracking3).state is MasterState.LOCKED
    assert manager.result(tracking3).bound_track_id == 1


def test_pending_selection_disappearance_cancels_to_unselected():
    manager = MasterManager()
    selected = _track(1)
    _, first = _pair(0, (selected,), time_s=1.0)
    manager.update(first)
    manager.begin_pending_lock(first, 1)

    _, missing = _pair(1, (), time_s=1.1, lost=(1,))
    events = manager.update(missing)

    assert len(events) == 1
    assert events[0].event is MasterEventType.MASTER_PENDING_LOCK_CANCELLED
    assert manager.result(missing).state is MasterState.UNSELECTED
    assert manager.result(missing).bound_track_id is None


def _lost_manager() -> MasterManager:
    manager = MasterManager()
    old_track = _track(1)
    _, first = _pair(0, (old_track,), time_s=1.0)
    manager.update(first)
    manager.lock(first, 1)
    _, removed = _pair(1, (), time_s=1.1, removed=(1,))
    manager.update(removed)
    assert manager.state is MasterState.LOST
    return manager


def test_lost_candidate_below_068_remains_lost():
    manager = _lost_manager()
    candidate = _track(2)
    for sequence_id in (2, 3, 4):
        _, current = _pair(sequence_id, (candidate,), time_s=1.0 + sequence_id / 10)
        manager.update(current)
        event = manager.consider_reid_candidate_scores(
            current, {2: 0.679}, threshold=0.68, stable_updates=3
        )
        assert event is None

    assert manager.state is MasterState.LOST
    assert manager.bound_track_id == 1


def _reid_episode(vectors: dict[int, np.ndarray]):
    backend = FakeEmbeddingBackend(vectors)
    evidence = MasterReIDEvidence(PersonReIdentifier(backend))
    manager = MasterManager()
    original = _track(1)
    frame0, tracking0 = _pair(0, (original,), time_s=10.0, markers={1: 10})
    manager.update(tracking0)
    manager.lock(tracking0, 1)
    evidence.create_reference(frame0, tracking0, 1)
    return backend, evidence, manager


def _lost_candidate_frame(
    sequence_id: int,
    time_s: float,
    candidate: TrackedPerson | None,
    markers: dict[int, int] | None,
    manager: MasterManager,
    evidence: MasterReIDEvidence,
    *,
    removed: tuple[int, ...] = (),
):
    tracks = () if candidate is None else (candidate,)
    frame, tracking = _pair(
        sequence_id, tracks, time_s=time_s, markers=markers or {}, removed=removed
    )
    manager.update(tracking)
    master_frame = manager.result(tracking)
    stable_ids = manager.stable_reid_candidate_ids(tracking, stable_updates=3)
    scores = evidence.score_candidates(
        frame,
        tracking,
        master_frame,
        stable_candidate_track_ids=stable_ids,
        accept_threshold=0.68,
        retry_threshold=0.50,
        retry_interval_s=0.20,
    )
    event = manager.consider_reid_candidate_scores(
        tracking,
        {item.candidate_track_id: item.similarity
         for item in evidence.lost_episode_candidate_scores},
        threshold=0.68,
        stable_updates=3,
    )
    return tracking, scores, event


def _unit_pair_score(score: float) -> np.ndarray:
    return np.asarray([score, np.sqrt(1.0 - score ** 2)], dtype=np.float32)


def test_first_reid_score_waits_for_three_updates_then_070_reacquires():
    backend, evidence, manager = _reid_episode({
        10: np.asarray([1.0, 0.0], dtype=np.float32),
        70: _unit_pair_score(0.70),
    })
    candidate = _track(2)
    for sequence_id in (1, 2):
        tracking, scores, event = _lost_candidate_frame(
            sequence_id, 10.0 + sequence_id / 10, candidate, {2: 70},
            manager, evidence, removed=(1,) if sequence_id == 1 else (),
        )
        assert scores is not None and scores.scores == ()
        assert event is None
        assert manager.state is MasterState.LOST
        assert backend.calls == 1  # reference only; candidate not stable yet

    tracking, scores, event = _lost_candidate_frame(
        3, 10.3, candidate, {2: 70}, manager, evidence
    )
    assert scores is not None and len(scores.scores) == 1
    assert scores.scores[0].retry_count == 0
    assert scores.scores[0].similarity == pytest.approx(0.70, abs=1e-6)
    assert event is not None
    assert event.event is MasterEventType.MASTER_REID_REACQUIRED
    assert event.old_track_id == 1 and event.new_track_id == 2
    assert event.similarity == pytest.approx(0.70, abs=1e-6)
    assert event.source_sequence_id == tracking.source_sequence_id
    assert manager.result(tracking).state is MasterState.LOCKED
    assert manager.result(tracking).bound_track_id == 2
    assert backend.calls == 2


def test_moderate_score_retries_after_02s_and_071_reacquires():
    backend, evidence, manager = _reid_episode({
        10: np.asarray([1.0, 0.0], dtype=np.float32),
        60: _unit_pair_score(0.60),
        71: _unit_pair_score(0.71),
    })
    candidate = _track(2)
    for sequence_id, time_s, marker in (
        (1, 10.1, 60), (2, 10.2, 60), (3, 10.3, 60),
    ):
        tracking, scores, event = _lost_candidate_frame(
            sequence_id, time_s, candidate, {2: marker}, manager, evidence,
            removed=(1,) if sequence_id == 1 else (),
        )
    assert scores is not None and scores.scores[0].similarity == pytest.approx(0.60)
    assert scores.scores[0].retry_count == 0
    assert event is None and manager.state is MasterState.LOST
    assert backend.calls == 2

    tracking, scores, event = _lost_candidate_frame(
        4, 10.41, candidate, {2: 71}, manager, evidence
    )
    assert scores is not None and scores.scores == ()
    assert event is None and manager.state is MasterState.LOST
    assert backend.calls == 2  # 0.11 seconds has not elapsed

    tracking, scores, event = _lost_candidate_frame(
        5, 10.52, candidate, {2: 71}, manager, evidence
    )
    assert scores is not None and len(scores.scores) == 1
    assert scores.scores[0].retry_count == 1
    assert scores.scores[0].similarity == pytest.approx(0.71, abs=1e-6)
    assert event is not None and event.event is MasterEventType.MASTER_REID_REACQUIRED
    assert event.old_track_id == 1 and event.new_track_id == 2
    assert manager.result(tracking).state is MasterState.LOCKED
    assert manager.result(tracking).bound_track_id == 2
    assert backend.calls == 3


def test_score_below_050_stops_retry_and_candidate_removal_clears_retry_state():
    backend, evidence, manager = _reid_episode({
        10: np.asarray([1.0, 0.0], dtype=np.float32),
        40: _unit_pair_score(0.40),
        60: _unit_pair_score(0.60),
    })
    candidate = _track(2)
    for sequence_id, time_s in ((1, 10.1), (2, 10.2), (3, 10.3)):
        tracking, scores, event = _lost_candidate_frame(
            sequence_id, time_s, candidate, {2: 40}, manager, evidence,
            removed=(1,) if sequence_id == 1 else (),
        )
    assert scores is not None and scores.scores[0].similarity == pytest.approx(0.40)
    assert event is None and manager.state is MasterState.LOST

    for sequence_id, time_s in ((4, 10.5), (5, 10.8)):
        _, scores, event = _lost_candidate_frame(
            sequence_id, time_s, candidate, {2: 60}, manager, evidence
        )
        assert scores is not None and scores.scores == ()
        assert event is None and manager.state is MasterState.LOST
    assert backend.calls == 2
    assert evidence.lost_episode_candidate_scores[0].similarity == pytest.approx(0.40)

    _, scores, _ = _lost_candidate_frame(
        6, 10.9, None, None, manager, evidence, removed=(2,)
    )
    assert scores is not None and scores.scores == ()
    assert evidence.lost_episode_candidate_scores == ()

    # The ID is scored as a fresh active candidate after its retry state was pruned.
    for sequence_id, time_s in ((7, 11.0), (8, 11.1), (9, 11.2)):
        tracking, scores, event = _lost_candidate_frame(
            sequence_id, time_s, candidate, {2: 60}, manager, evidence
        )
    assert scores is not None and len(scores.scores) == 1
    assert scores.scores[0].retry_count == 0
    assert scores.scores[0].similarity == pytest.approx(0.60, abs=1e-6)
    assert event is None and manager.state is MasterState.LOST
    assert backend.calls == 3
