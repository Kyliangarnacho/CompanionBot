"""Hardware-free contract tests plus a real upstream ByteTrack smoke test."""

from __future__ import annotations

from dataclasses import fields
import time

import numpy as np
import pytest

from perception.detection_stream import PersonDetectionFrame
from perception.person_detector import PersonDetection
from perception.person_tracker import (
    BackendTrack,
    PersonTracker,
    TrackerBackendUpdate,
    TrackedPerson,
)


def detection(sequence_id: int, box=(10.0, 8.0, 30.0, 44.0), confidence=0.91):
    source_time_s = time.perf_counter() - 0.02
    return PersonDetection(sequence_id, source_time_s, box, confidence, 0, "person")


def detection_frame(sequence_id: int, detections=(), *, source_id="camera-A", source_time_s=None):
    now = time.perf_counter()
    source_time_s = now - 0.02 if source_time_s is None else source_time_s
    normalized = tuple(
        PersonDetection(sequence_id, source_time_s, item.bbox_xyxy_px,
                        item.confidence, item.class_id, item.class_name)
        for item in detections
    )
    return PersonDetectionFrame(
        source_id, sequence_id, source_time_s, 128, 96, normalized,
        max(source_time_s, now - 0.003), max(source_time_s, now - 0.002),
        max(source_time_s, now - 0.001),
    )


class FakeBackend:
    name = "test.fake"

    def __init__(self):
        self.calls = []
        self.reset_count = 0
        self.close_count = 0

    def update(self, detections, frame_width, frame_height):
        self.calls.append((tuple(detections), frame_width, frame_height))
        tracks = tuple(
            BackendTrack(index + 4, item.bbox_xyxy_px, item.confidence, item.class_id)
            for index, item in enumerate(detections)
        )
        return TrackerBackendUpdate(tracks)

    def reset(self):
        self.reset_count += 1

    def close(self):
        self.close_count += 1


def test_empty_detection_frame_produces_valid_empty_tracking_frame_with_metadata():
    backend = FakeBackend()
    tracker = PersonTracker(backend)
    source = detection_frame(12)

    result = tracker.update(source)

    assert result.source_id == source.source_id
    assert result.source_sequence_id == source.source_sequence_id
    assert result.source_time_s == source.source_time_s
    assert (result.frame_width, result.frame_height) == (128, 96)
    assert result.detection_count == 0
    assert result.tracks == ()
    assert result.diagnostics.active_track_count == 0
    assert result.diagnostics.update_index == 1
    assert len(backend.calls) == 1


def test_person_tracks_keep_original_pixel_box_confidence_and_public_types():
    backend = FakeBackend()
    tracker = PersonTracker(backend)
    source = detection_frame(3, [detection(3, (22.5, 11.25, 96.0, 90.0), 0.84)])

    result = tracker.update(source)

    assert result.tracks == (TrackedPerson(4, (22.5, 11.25, 96.0, 90.0), 0.84, 0, "person"),)
    assert result.diagnostics.created_track_ids == (4,)
    assert all(field.type not in {object, np.ndarray} for field in fields(TrackedPerson))
    assert type(result.tracks[0]) is TrackedPerson
    assert type(result.tracks[0].bbox_xyxy_px) is tuple


def test_multiple_people_and_empty_frame_are_each_one_real_tracker_update():
    backend = FakeBackend()
    tracker = PersonTracker(backend)
    two = detection_frame(10, [
        detection(10, (2, 3, 20, 45)), detection(10, (70, 2, 120, 80), 0.79),
    ])
    empty = detection_frame(11)

    first = tracker.update(two)
    second = tracker.update(empty)

    assert [track.track_id for track in first.tracks] == [4, 5]
    assert first.diagnostics.active_track_count == 2
    assert second.source_sequence_id == 11 and second.tracks == ()
    assert len(backend.calls) == 2


def test_sequence_gaps_are_recorded_without_synthetic_updates():
    backend = FakeBackend()
    tracker = PersonTracker(backend)

    first = tracker.update(detection_frame(100, [detection(100)]))
    second = tracker.update(detection_frame(103, [detection(103)]))

    assert first.diagnostics.source_sequence_gap == 0
    assert second.diagnostics.source_sequence_gap == 2
    assert second.diagnostics.update_index == 2
    assert len(backend.calls) == 2
    assert [call[0][0].source_sequence_id for call in backend.calls] == [100, 103]


def test_out_of_order_or_changed_source_frames_are_rejected_before_backend_update():
    backend = FakeBackend()
    tracker = PersonTracker(backend)
    tracker.update(detection_frame(8))
    call_count = len(backend.calls)

    with pytest.raises(ValueError, match="must increase"):
        tracker.update(detection_frame(8))
    with pytest.raises(ValueError, match="source_id changed"):
        tracker.update(detection_frame(9, source_id="camera-B"))
    with pytest.raises(ValueError, match="moved backwards"):
        tracker.update(detection_frame(9, source_time_s=time.perf_counter() - 1))
    assert len(backend.calls) == call_count


def test_reset_clears_sequence_and_seen_ids_and_close_is_clean():
    backend = FakeBackend()
    tracker = PersonTracker(backend)
    first = tracker.update(detection_frame(40, [detection(40)]))
    assert first.diagnostics.created_track_ids == (4,)

    tracker.reset()
    after_reset = tracker.update(detection_frame(2, [detection(2)]))
    assert after_reset.diagnostics.update_index == 1
    assert after_reset.diagnostics.source_sequence_gap == 0
    assert after_reset.diagnostics.created_track_ids == (4,)
    assert backend.reset_count == 1

    tracker.close()
    tracker.close()
    assert backend.close_count == 1
    with pytest.raises(RuntimeError, match="closed"):
        tracker.update(detection_frame(3))


def test_real_ultralytics_bytetrack_smoke_tracks_synthetic_person_boxes(monkeypatch, tmp_path):
    monkeypatch.setenv("YOLO_CONFIG_DIR", str(tmp_path))
    from perception.ultralytics_bytetrack import UltralyticsByteTrackBackend

    backend = UltralyticsByteTrackBackend()
    tracker = PersonTracker(backend)
    first = detection_frame(200, [
        detection(200, (10, 10, 50, 90), 0.91),
        detection(200, (75, 8, 115, 88), 0.88),
    ])
    second = detection_frame(202, [
        detection(202, (12, 10, 52, 90), 0.90),
        detection(202, (73, 8, 113, 88), 0.87),
    ])

    result1 = tracker.update(first)
    result2 = tracker.update(second)

    assert backend.upstream_version
    assert backend.effective_config["track_buffer"] == 30
    assert len(result1.tracks) == 2
    assert {item.track_id for item in result2.tracks} == {
        item.track_id for item in result1.tracks
    }
    assert result2.diagnostics.source_sequence_gap == 1
    assert backend.upstream_update_count == 2
    tracker.close()
