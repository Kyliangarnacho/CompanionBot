from __future__ import annotations

from dataclasses import replace
import numpy as np
import pytest

from perception.camera import ColorFrame
from perception.master_selection import MasterManager, MasterState
from perception.person_reid import (
    AppearanceEmbedding,
    BackendEmbedding,
    MasterReIDEvidence,
    PersonReIdentifier,
    ReIDInputError,
    crop_person_bgr,
)
from perception.person_tracker import (
    PersonTrackingDiagnostics,
    PersonTrackingFrame,
    TrackedPerson,
)


class FakeEmbeddingBackend:
    backend_name = "tests.fake"
    model_name = "tests.mapping"

    def __init__(self, vectors: dict[int, np.ndarray] | None = None) -> None:
        self.vectors = vectors or {}
        self.crop_shapes: list[tuple[int, int, int]] = []
        self.calls = 0

    def embed_crop_bgr(self, crop_bgr: np.ndarray) -> BackendEmbedding:
        self.calls += 1
        self.crop_shapes.append(crop_bgr.shape)
        marker = int(crop_bgr[0, 0, 0])
        vector = self.vectors.get(marker, np.asarray([1.0, 0.0], dtype=np.float32))
        return BackendEmbedding(vector, 0.001, 0.002)


def _track(track_id: int, box=(4.0, 4.0, 20.0, 24.0)) -> TrackedPerson:
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
    removed: tuple[int, ...] = (),
    created: tuple[int, ...] = (),
    image: np.ndarray | None = None,
) -> tuple[ColorFrame, PersonTrackingFrame]:
    if image is None:
        image = np.zeros((64, 64, 3), dtype=np.uint8)
    timestamp = float(sequence_id + 1)
    frame = ColorFrame(
        bgr=image,
        sequence_id=sequence_id,
        source_id="test-camera",
        width=image.shape[1],
        height=image.shape[0],
        host_receive_time_s=timestamp,
    )
    tracking = PersonTrackingFrame(
        source_id="test-camera",
        source_sequence_id=sequence_id,
        source_time_s=timestamp,
        frame_width=image.shape[1],
        frame_height=image.shape[0],
        detection_count=len(tracks),
        tracks=tracks,
        diagnostics=PersonTrackingDiagnostics(
            update_index=sequence_id + 1,
            active_track_count=len(tracks),
            created_track_ids=created,
            newly_lost_track_ids=(),
            newly_removed_track_ids=removed,
            source_sequence_gap=0,
            tracker_update_wall_time_s=0.001,
        ),
        tracking_start_time_s=timestamp + 0.01,
        tracking_end_time_s=timestamp + 0.02,
        result_ready_time_s=timestamp + 0.03,
    )
    return frame, tracking


def _marker_image(markers: dict[int, int]) -> np.ndarray:
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    for track_id, marker in markers.items():
        left, top = ((58, 30) if track_id == 4
                     else (4 + (track_id - 1) * 18, 4))
        image[top:top + 20, left:left + 16, 0] = marker
    return image


def _start_lost_episode(
    evidence: MasterReIDEvidence,
    manager: MasterManager,
    backend: FakeEmbeddingBackend,
) -> tuple[ColorFrame, PersonTrackingFrame]:
    frame0, tracking0 = _pair(
        0, (_track(1),), image=_marker_image({1: 10})
    )
    manager.update(tracking0)
    manager.lock(tracking0, 1)
    reference = evidence.create_reference(frame0, tracking0, 1)
    assert reference.bound_track_at_selection == 1

    frame1, tracking1 = _pair(
        1,
        (_track(2, (22.0, 4.0, 38.0, 24.0)),),
        removed=(1,),
        created=(2,),
        image=_marker_image({2: 20}),
    )
    manager.update(tracking1)
    assert manager.state is MasterState.LOST
    return frame1, tracking1


def test_crop_clips_to_original_frame_without_expansion() -> None:
    image = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    crop = crop_person_bgr(image, (-2.0, 1.2, 4.1, 9.0))
    assert crop.shape == (5, 5, 3)
    assert np.array_equal(crop, image[1:6, 0:5])


@pytest.mark.parametrize(
    "box",
    [(-20.0, 0.0, -1.0, 5.0), (2.0, 2.0, 2.0, 2.0), (3.0, 4.0, 2.0, 8.0)],
)
def test_invalid_crop_is_rejected_before_backend(box) -> None:
    backend = FakeEmbeddingBackend()
    reidentifier = PersonReIdentifier(backend)
    with pytest.raises(ReIDInputError):
        reidentifier.embed(
            np.zeros((8, 8, 3), dtype=np.uint8), box,
            source_id="camera", source_sequence_id=1, track_id=1,
        )
    assert backend.calls == 0


def test_exact_color_and_tracking_sequence_are_required() -> None:
    backend = FakeEmbeddingBackend()
    reidentifier = PersonReIdentifier(backend)
    frame, tracking = _pair(4, (_track(1),))
    wrong_frame = ColorFrame(
        bgr=frame.bgr,
        sequence_id=5,
        source_id=frame.source_id,
        width=frame.width,
        height=frame.height,
        host_receive_time_s=frame.host_receive_time_s,
    )
    with pytest.raises(ReIDInputError, match="sequence_id"):
        reidentifier.embed_track(wrong_frame, tracking, tracking.tracks[0])
    assert backend.calls == 0

    wrong_time_tracking = replace(
        tracking, source_time_s=tracking.source_time_s + 0.005
    )
    with pytest.raises(ReIDInputError, match="receive time"):
        reidentifier.embed_track(frame, wrong_time_tracking, tracking.tracks[0])
    assert backend.calls == 0


def test_track_must_belong_to_the_paired_tracking_frame() -> None:
    reidentifier = PersonReIdentifier(FakeEmbeddingBackend())
    frame, tracking = _pair(0, (_track(1),))
    with pytest.raises(ReIDInputError, match="exact active track"):
        reidentifier.embed_track(frame, tracking, _track(2))


@pytest.mark.parametrize(
    "vector",
    [
        np.asarray([], dtype=np.float32),
        np.asarray([[1.0, 0.0]], dtype=np.float32),
        np.asarray([1.0, np.nan], dtype=np.float32),
        np.asarray([0.0, 0.0], dtype=np.float32),
    ],
)
def test_embedding_shape_and_finite_validation(vector) -> None:
    with pytest.raises(ValueError):
        AppearanceEmbedding(
            source_id="camera",
            source_sequence_id=0,
            track_id=1,
            vector=vector,
            backend_name="fake",
            model_name="fake",
            preprocess_wall_time_s=0.0,
            inference_wall_time_s=0.0,
            total_wall_time_s=0.0,
        )


def test_embedding_is_numpy_normalized_and_backend_objects_do_not_escape() -> None:
    backend = FakeEmbeddingBackend({10: np.asarray([3.0, 4.0], dtype=np.float32)})
    reidentifier = PersonReIdentifier(backend)
    frame, tracking = _pair(0, (_track(1),), image=_marker_image({1: 10}))
    embedding = reidentifier.embed_track(frame, tracking, tracking.tracks[0])
    assert type(embedding.vector) is np.ndarray
    assert embedding.vector.dtype == np.float32
    assert np.allclose(embedding.vector, [0.6, 0.8])
    assert not embedding.vector.flags.writeable
    assert backend.crop_shapes == [(20, 16, 3)]


def test_reference_creation_replacement_and_clear() -> None:
    backend = FakeEmbeddingBackend()
    evidence = MasterReIDEvidence(PersonReIdentifier(backend))
    frame0, tracking0 = _pair(0, (_track(1),))
    first = evidence.create_reference(frame0, tracking0, 1)
    frame1, tracking1 = _pair(1, (_track(2),))
    second = evidence.create_reference(frame1, tracking1, 2)
    assert first.bound_track_at_selection == 1
    assert evidence.reference is second
    assert second.source_sequence_id == 1
    evidence.clear_reference()
    assert evidence.reference is None
    evidence.reset()
    assert evidence.reference is None


def test_lost_candidates_rank_by_cosine_margin_without_changing_master() -> None:
    vectors = {
        10: np.asarray([1.0, 0.0], dtype=np.float32),
        20: np.asarray([0.8, 0.6], dtype=np.float32),
        30: np.asarray([0.0, 1.0], dtype=np.float32),
        40: np.asarray([1.0, 0.0], dtype=np.float32),
    }
    backend = FakeEmbeddingBackend(vectors)
    evidence = MasterReIDEvidence(PersonReIdentifier(backend))
    manager = MasterManager()
    frame0, tracking0 = _pair(0, (_track(1),), image=_marker_image({1: 10}))
    manager.update(tracking0)
    manager.lock(tracking0, 1)
    evidence.create_reference(frame0, tracking0, 1)

    tracks = (
        _track(2, (22.0, 4.0, 38.0, 24.0)),
        _track(3, (40.0, 4.0, 56.0, 24.0)),
        _track(4, (58.0, 30.0, 64.0, 50.0)),
    )
    image = _marker_image({2: 20, 3: 30, 4: 40})
    frame1, tracking1 = _pair(1, tracks, removed=(1,), created=(2, 3, 4), image=image)
    manager.update(tracking1)
    master_frame = manager.result(tracking1)
    original_binding = manager.bound_track_id
    result = evidence.score_candidates(
        frame1, tracking1, master_frame,
        stable_candidate_track_ids=(2, 3, 4),
    )

    assert result is not None
    assert [item.candidate_track_id for item in result.scores] == [4, 2, 3]
    assert [item.rank for item in result.scores] == [1, 2, 3]
    assert result.top1.similarity == pytest.approx(1.0)
    assert result.top2.similarity == pytest.approx(0.8)
    assert result.top1_top2_margin == pytest.approx(0.2)
    assert all(item.master_state is MasterState.LOST for item in result.scores)
    assert manager.state is MasterState.LOST
    assert manager.bound_track_id == original_binding == 1


def test_candidate_score_state_clears_when_track_disappears() -> None:
    backend = FakeEmbeddingBackend()
    evidence = MasterReIDEvidence(PersonReIdentifier(backend))
    manager = MasterManager()
    frame1, tracking1 = _start_lost_episode(evidence, manager, backend)
    first = evidence.score_candidates(
        frame1, tracking1, manager.result(tracking1), stable_candidate_track_ids=(2,)
    )
    assert first is not None and len(first.scores) == 1
    assert first.top2 is None and first.top1_top2_margin is None

    frame2, tracking2 = _pair(2, ())
    manager.update(tracking2)
    empty = evidence.score_candidates(
        frame2, tracking2, manager.result(tracking2), stable_candidate_track_ids=()
    )
    assert empty is not None and empty.scores == ()

    frame3, tracking3 = _pair(3, (_track(2, (22.0, 4.0, 38.0, 24.0)),))
    manager.update(tracking3)
    again = evidence.score_candidates(
        frame3, tracking3, manager.result(tracking3), stable_candidate_track_ids=(2,)
    )
    assert again is not None and len(again.scores) == 1
    assert again.scores[0].retry_count == 0
    assert backend.calls == 3  # reference crop plus one score per active appearance


def test_reselection_replaces_reference_and_clear_prevents_future_scoring() -> None:
    backend = FakeEmbeddingBackend()
    evidence = MasterReIDEvidence(PersonReIdentifier(backend))
    manager = MasterManager()
    frame0, tracking0 = _pair(0, (_track(1),))
    manager.update(tracking0)
    manager.lock(tracking0, 1)
    old_reference = evidence.create_reference(frame0, tracking0, 1)

    frame1, tracking1 = _pair(1, (_track(2),), removed=(1,), created=(2,))
    manager.update(tracking1)
    manager.lock(tracking1, 2)
    evidence.create_reference(frame1, tracking1, 2)
    assert evidence.reference.bound_track_at_selection == 2
    assert evidence.reference is not old_reference
    assert manager.state is MasterState.LOCKED
    assert manager.bound_track_id == 2

    evidence.clear_reference()
    frame2, tracking2 = _pair(2, (_track(3),), removed=(2,), created=(3,))
    manager.update(tracking2)
    assert manager.state is MasterState.LOST
    assert evidence.score_candidates(
        frame2, tracking2, manager.result(tracking2), stable_candidate_track_ids=()
    ) is None
