"""Hardware-free tests for the backend-neutral person-detection contract."""

from __future__ import annotations

import numpy as np
import pytest

from perception.camera import ColorFrame
from perception.person_detector import (
    PersonDetection,
    PersonDetector,
    PersonDetectorConfig,
)


class FakeBoxes:
    def __init__(self, xyxy, confidence, classes):
        self.xyxy = np.asarray(xyxy, dtype=np.float32).reshape(-1, 4)
        self.conf = np.asarray(confidence, dtype=np.float32)
        self.cls = np.asarray(classes, dtype=np.float32)


class FakeResult:
    def __init__(self, boxes, original_shape):
        self.boxes = boxes
        self.orig_shape = original_shape


class FakeModel:
    names = {2: "bicycle", 7: "person"}

    def __init__(self, boxes=None, original_shape=(80, 120)):
        self.result = FakeResult(boxes, original_shape)
        self.predict_args = None

    def predict(self, **kwargs):
        self.predict_args = kwargs
        return [self.result]


def test_person_detection_validates_bbox_confidence_and_class_contract():
    detection = PersonDetection(
        source_sequence_id=12,
        source_time_s=4.25,
        bbox_xyxy_px=(10, 11, 90.5, 70),
        confidence=0.93,
        class_id=7,
        class_name="person",
    )
    assert detection.bbox_xyxy_px == (10.0, 11.0, 90.5, 70.0)
    assert detection.confidence == 0.93

    for bbox in ((1, 2, 3), (1, 2, np.inf, 4), (8, 2, 3, 4), (-1, 2, 3, 4)):
        with pytest.raises(ValueError, match="bbox_xyxy_px"):
            PersonDetection(None, None, bbox, 0.5, 7, "person")
    for confidence in (-0.01, 1.01, np.nan):
        with pytest.raises(ValueError, match="confidence"):
            PersonDetection(None, None, (1, 2, 3, 4), confidence, 7, "person")
    with pytest.raises(ValueError, match="class_name"):
        PersonDetection(None, None, (1, 2, 3, 4), 0.5, 2, "bicycle")


def test_detector_filters_non_person_and_preserves_color_frame_metadata():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    frame = ColorFrame(image, 41, "camera-source", 120, 80, 123.456)
    model = FakeModel(FakeBoxes(
        [[2, 3, 20, 25], [10.25, 5.5, 100.75, 72]],
        [0.88, 0.91],
        [2, 7],
    ))
    detector = PersonDetector(PersonDetectorConfig(
        model_path="test-model", imgsz=320, confidence_threshold=0.3,
        device="cpu",
    ), model=model)

    detections = detector.process(frame)

    assert len(detections) == 1
    assert detections[0].class_id == 7
    assert detections[0].class_name == "person"
    assert detections[0].bbox_xyxy_px == (10.25, 5.5, 100.75, 72.0)
    assert detections[0].source_sequence_id == 41
    assert detections[0].source_time_s == 123.456
    assert model.predict_args["source"] is image
    assert model.predict_args["imgsz"] == 320
    assert model.predict_args["conf"] == 0.3
    assert model.predict_args["device"] == "cpu"
    assert model.predict_args["stream"] is False


def test_detector_accepts_static_bgr_array_without_camera_or_fake_metadata():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    model = FakeModel(FakeBoxes([[1, 2, 30, 40]], [0.75], [7]))

    detections = PersonDetector(model=model).process(image)

    assert len(detections) == 1
    assert detections[0].source_sequence_id is None
    assert detections[0].source_time_s is None


def test_empty_detection_returns_empty_tuple():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    model = FakeModel(FakeBoxes([], [], []))

    assert PersonDetector(model=model).process(image) == ()


@pytest.mark.parametrize(
    "frame",
    [np.zeros((80, 120), dtype=np.uint8),
     np.zeros((80, 120, 4), dtype=np.uint8),
     np.zeros((80, 120, 3), dtype=np.float32)],
)
def test_malformed_frame_is_rejected_before_inference(frame):
    model = FakeModel(FakeBoxes([], [], []))
    with pytest.raises(ValueError, match="H×W×3 uint8 BGR"):
        PersonDetector(model=model).process(frame)
    assert model.predict_args is None


def test_model_without_person_class_fails_at_initialization():
    class NoPersonModel:
        names = {0: "bicycle", 1: "car"}

    with pytest.raises(ValueError, match="no 'person' class"):
        PersonDetector(model=NoPersonModel())


def test_result_shape_mismatch_is_rejected_to_protect_pixel_semantics():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    model = FakeModel(FakeBoxes([[1, 2, 30, 40]], [0.75], [7]), (640, 640))

    with pytest.raises(RuntimeError, match="original shape"):
        PersonDetector(model=model).process(image)


def test_person_bbox_outside_source_frame_is_rejected():
    image = np.zeros((80, 120, 3), dtype=np.uint8)
    model = FakeModel(FakeBoxes([[1, 2, 130, 40]], [0.75], [7]))

    with pytest.raises(RuntimeError, match="outside original input pixel"):
        PersonDetector(model=model).process(image)


def test_detector_config_validates_simple_predict_settings():
    assert PersonDetectorConfig(imgsz=(384, 640)).imgsz == (384, 640)
    with pytest.raises(ValueError, match="imgsz"):
        PersonDetectorConfig(imgsz=(640, 0))
    with pytest.raises(ValueError, match="confidence_threshold"):
        PersonDetectorConfig(confidence_threshold=1.1)
