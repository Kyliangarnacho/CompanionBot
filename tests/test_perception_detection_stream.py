"""Hardware-free tests for Stage 7.2 frame results and latest-frame scheduling."""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from perception.camera import ColorFrame
from perception.camera import CameraProfile
from perception.detection_stream import (
    LatestFrameSlot,
    PersonDetectionFrame,
    process_person_frame,
)
from perception.person_detector import PersonDetection


def frame(sequence_id: int, source_time_s: float | None = None) -> ColorFrame:
    return ColorFrame(
        bgr=np.zeros((48, 64, 3), dtype=np.uint8),
        sequence_id=sequence_id,
        source_id="test-camera",
        width=64,
        height=48,
        host_receive_time_s=(time.perf_counter() if source_time_s is None
                             else source_time_s),
    )


class FakeDetector:
    def __init__(self, detections=(), delay_s=0.0):
        self.detections = tuple(detections)
        self.delay_s = delay_s
        self.calls = []

    def process(self, color_frame):
        self.calls.append(color_frame.sequence_id)
        if self.delay_s:
            time.sleep(self.delay_s)
        return self.detections


def test_frame_result_preserves_source_metadata_dimensions_and_empty_detection():
    source = frame(18)
    result = process_person_frame(source, FakeDetector())

    assert result.source_id == "test-camera"
    assert result.source_sequence_id == 18
    assert result.source_time_s == source.host_receive_time_s
    assert (result.frame_width, result.frame_height) == (64, 48)
    assert result.detections == ()
    assert (result.source_time_s <= result.inference_start_time_s
            <= result.inference_end_time_s <= result.result_ready_time_s)
    assert result.detector_call_wall_time_s >= 0.0
    assert result.processing_wall_time_s >= result.detector_call_wall_time_s
    assert result.host_receive_to_result_age_s >= result.processing_wall_time_s


def test_frame_result_preserves_all_detection_metadata_for_multiple_people():
    source = frame(7)
    detections = (
        PersonDetection(7, source.host_receive_time_s, (2, 3, 18, 40), 0.91, 0, "person"),
        PersonDetection(7, source.host_receive_time_s, (32, 4, 60, 45), 0.83, 0, "person"),
    )
    detector = FakeDetector(detections)

    result = process_person_frame(source, detector)

    assert result.detections == detections
    assert len(detector.calls) == 1
    assert [item.source_sequence_id for item in result.detections] == [7, 7]


def test_result_rejects_out_of_order_times_and_detection_metadata_mismatch():
    with pytest.raises(ValueError, match="timestamps must be ordered"):
        PersonDetectionFrame("source", 1, 1.0, 64, 48, (), 0.9, 1.1, 1.2)

    wrong_frame_detection = PersonDetection(3, 4.0, (1, 1, 5, 5), 0.8, 0, "person")
    with pytest.raises(ValueError, match="metadata must match"):
        PersonDetectionFrame("source", 2, 4.0, 64, 48,
                             (wrong_frame_detection,), 4.1, 4.2, 4.3)

    outside_frame_detection = PersonDetection(2, 4.0, (1, 1, 65, 20), 0.8, 0, "person")
    with pytest.raises(ValueError, match="fit inside"):
        PersonDetectionFrame("source", 2, 4.0, 64, 48,
                             (outside_frame_detection,), 4.1, 4.2, 4.3)


def test_latest_frame_slot_overwrites_old_work_and_keeps_no_backlog():
    slot = LatestFrameSlot()
    slot.put(frame(0))
    first = slot.get(timeout_s=0.1)
    assert first is not None and first.sequence_id == 0

    # Keep the inference consumer busy outside the slot while capture submits.
    inference_started = threading.Event()
    release_inference = threading.Event()
    producer_finished = threading.Event()

    def slow_inference():
        inference_started.set()
        assert release_inference.wait(timeout=2.0)

    def produce_burst():
        for sequence_id in range(1, 101):
            slot.put(frame(sequence_id))
        producer_finished.set()

    consumer = threading.Thread(target=slow_inference)
    consumer.start()
    assert inference_started.wait(timeout=0.5)
    producer = threading.Thread(target=produce_burst)
    producer.start()
    assert producer_finished.wait(timeout=0.5)

    assert slot.pending_count == 1
    assert slot.overwritten_count == 99
    release_inference.set()
    producer.join(timeout=0.5)
    consumer.join(timeout=0.5)
    assert not producer.is_alive()
    assert not consumer.is_alive()

    latest = slot.get(timeout_s=0.1)
    assert latest is not None and latest.sequence_id == 100
    assert latest.sequence_id - first.sequence_id - 1 == 99
    assert slot.pending_count == 0
    assert slot.get(timeout_s=0.01) is None
    assert slot.submitted_count == 101
    assert slot.taken_count == 2


def test_close_wakes_waiter_and_drains_last_frame_before_clean_stop():
    slot = LatestFrameSlot()
    slot.put(frame(5))
    slot.close()

    final_frame = slot.get(timeout_s=0.1)
    assert final_frame is not None and final_frame.sequence_id == 5
    assert slot.get(timeout_s=0.1) is None
    assert slot.closed
    with pytest.raises(RuntimeError, match="closed slot"):
        slot.put(frame(6))

    waiting_slot = LatestFrameSlot()
    returned = []
    waiter = threading.Thread(target=lambda: returned.append(waiting_slot.get()))
    waiter.start()
    waiting_slot.close()
    waiter.join(timeout=1.0)
    assert not waiter.is_alive()
    assert returned == [None]


def test_camera_source_releases_and_closes_slot_after_frame_limit(monkeypatch):
    from scripts import demo_yolo26n_stream as stream_demo

    source_frames = [frame(0), frame(1)]
    fake_profile = CameraProfile("fake-device", "FAKE", None, None, None, 64, 48, 30.0)

    class FakeCamera:
        released = False

        def __init__(self, _config):
            self._frames = list(source_frames)

        def open(self):
            return fake_profile

        def read(self):
            return self._frames.pop(0)

        def release(self):
            self.released = True

    fake_camera = FakeCamera(None)
    monkeypatch.setattr(stream_demo, "CameraStream", lambda _config: fake_camera)
    slot = LatestFrameSlot()
    runtime = stream_demo._SourceRuntime.create(slot)

    stream_demo._camera_source_thread(
        runtime, 4, None, None, None, None, max_frames=2
    )

    assert fake_camera.released
    assert runtime.ready.is_set() and runtime.done.is_set()
    assert runtime.error_value() is None
    assert slot.closed
    assert runtime.stats.snapshot()["produced_count"] == 2
    assert slot.overwritten_count == 1
    latest = slot.get(timeout_s=0.1)
    assert latest is not None and latest.sequence_id == 1
    assert slot.get(timeout_s=0.1) is None


def test_slot_rejects_non_color_frames_and_invalid_timeout():
    slot = LatestFrameSlot()
    with pytest.raises(TypeError, match="ColorFrame"):
        slot.put(np.zeros((10, 10, 3), dtype=np.uint8))
    with pytest.raises(ValueError, match="timeout_s"):
        slot.get(timeout_s=-1)
