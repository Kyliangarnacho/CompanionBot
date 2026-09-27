"""Person-only object-detection boundary backed by Ultralytics predict().

Ultralytics, PyTorch, and Results objects stay inside this adapter. Public
detections use image-pixel coordinates and preserve optional source metadata.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Tuple, Union

import numpy as np

from perception.camera import ColorFrame


ImageInput = Union[ColorFrame, np.ndarray]
ModelPath = Union[str, Path]
ImageSize = Union[int, Tuple[int, int]]


@dataclass(frozen=True)
class PersonDetectorConfig:
    """Small set of Ultralytics predict settings used by this detector."""

    model_path: ModelPath = "yolo26n.pt"
    imgsz: ImageSize = 640
    confidence_threshold: float = 0.25
    device: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.model_path, (str, Path)) or not str(self.model_path):
            raise ValueError("model_path must be a non-empty path or model name")
        if isinstance(self.imgsz, bool):
            raise ValueError("imgsz must be a positive integer or (height, width)")
        if isinstance(self.imgsz, int):
            valid_size = self.imgsz > 0
        elif isinstance(self.imgsz, tuple) and len(self.imgsz) == 2:
            valid_size = all(
                isinstance(value, int) and not isinstance(value, bool) and value > 0
                for value in self.imgsz
            )
        else:
            valid_size = False
        if not valid_size:
            raise ValueError("imgsz must be a positive integer or (height, width)")
        try:
            threshold = float(self.confidence_threshold)
        except (TypeError, ValueError) as error:
            raise ValueError("confidence_threshold must be finite in [0, 1]") from error
        if not math.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
            raise ValueError("confidence_threshold must be finite in [0, 1]")
        object.__setattr__(self, "confidence_threshold", threshold)
        if self.device is not None and (
            not isinstance(self.device, str) or not self.device.strip()
        ):
            raise ValueError("device must be a non-empty string or None")


@dataclass(frozen=True)
class PersonDetection:
    """One person box in the original input frame's pixel coordinate system.

    ``bbox_xyxy_px`` is ``(x1, y1, x2, y2)`` with x right, y down, and units
    in pixels. Optional sequence/time metadata is copied from a ``ColorFrame``;
    for a static ndarray both values are ``None``. ``source_time_s`` is the
    source frame's host receive timestamp, not detector completion time.
    """

    source_sequence_id: int | None
    source_time_s: float | None
    bbox_xyxy_px: tuple[float, float, float, float]
    confidence: float
    class_id: int
    class_name: str

    def __post_init__(self) -> None:
        if self.source_sequence_id is not None and (
            isinstance(self.source_sequence_id, bool)
            or not isinstance(self.source_sequence_id, int)
            or self.source_sequence_id < 0
        ):
            raise ValueError("source_sequence_id must be a nonnegative integer or None")
        if self.source_time_s is not None:
            try:
                source_time = float(self.source_time_s)
            except (TypeError, ValueError) as error:
                raise ValueError("source_time_s must be finite or None") from error
            if not math.isfinite(source_time):
                raise ValueError("source_time_s must be finite or None")
            object.__setattr__(self, "source_time_s", source_time)

        try:
            bbox = tuple(float(value) for value in self.bbox_xyxy_px)
        except (TypeError, ValueError) as error:
            raise ValueError("bbox_xyxy_px must contain four finite pixel values") from error
        if len(bbox) != 4 or not all(math.isfinite(value) for value in bbox):
            raise ValueError("bbox_xyxy_px must contain four finite pixel values")
        x1, y1, x2, y2 = bbox
        if x1 < 0.0 or y1 < 0.0 or x2 <= x1 or y2 <= y1:
            raise ValueError("bbox_xyxy_px must be a positive-area xyxy pixel box")
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
        if not isinstance(self.class_name, str) or not self.class_name.strip():
            raise ValueError("class_name must not be empty")
        if self.class_name.strip().casefold() != "person":
            raise ValueError("PersonDetection class_name must be 'person'")


def _class_name_items(names: object) -> list[tuple[int, str]]:
    if isinstance(names, Mapping):
        items = names.items()
    elif isinstance(names, (list, tuple)):
        items = enumerate(names)
    else:
        raise ValueError("Loaded model metadata does not expose class names")

    parsed: list[tuple[int, str]] = []
    for key, value in items:
        try:
            class_id = int(key)
        except (TypeError, ValueError) as error:
            raise ValueError(f"Model class ID {key!r} is not an integer") from error
        if class_id < 0:
            raise ValueError(f"Model class ID must be nonnegative: {class_id}")
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"Model class name for ID {class_id} is empty")
        parsed.append((class_id, value.strip()))
    return parsed


def _resolve_person_class(names: object) -> tuple[int, str]:
    matches = [
        (class_id, name)
        for class_id, name in _class_name_items(names)
        if name.casefold() == "person"
    ]
    if not matches:
        raise ValueError("Loaded detector model has no 'person' class")
    if len(matches) != 1:
        raise ValueError("Loaded detector model has ambiguous 'person' classes")
    return matches[0]


def _frame_input(frame: ImageInput) -> tuple[np.ndarray, int | None, float | None]:
    if isinstance(frame, ColorFrame):
        image = frame.bgr
        sequence_id = frame.sequence_id
        source_time_s = frame.host_receive_time_s
        if (frame.width != image.shape[1] or frame.height != image.shape[0]):
            raise ValueError("ColorFrame dimensions do not match its BGR array")
    elif isinstance(frame, np.ndarray):
        image = frame
        sequence_id = None
        source_time_s = None
    else:
        raise TypeError("frame must be a ColorFrame or BGR NumPy ndarray")

    if (not isinstance(image, np.ndarray) or image.ndim != 3
            or image.shape[2] != 3 or image.dtype != np.uint8
            or image.shape[0] < 1 or image.shape[1] < 1):
        raise ValueError("frame must be a non-empty H×W×3 uint8 BGR image")
    return image, sequence_id, source_time_s


def _to_numpy(values: object, name: str) -> np.ndarray:
    # Ultralytics normally returns torch tensors. Keep that conversion confined
    # to this module while also accepting NumPy arrays in hardware-free tests.
    converted = values
    if hasattr(converted, "detach"):
        converted = converted.detach()
    if hasattr(converted, "cpu"):
        converted = converted.cpu()
    if hasattr(converted, "numpy"):
        converted = converted.numpy()
    try:
        return np.asarray(converted)
    except (TypeError, ValueError) as error:
        raise RuntimeError(f"Ultralytics {name} values are not array-like") from error


class PersonDetector:
    """Run person detection with Ultralytics and return backend-neutral boxes."""

    def __init__(
        self,
        config: PersonDetectorConfig | None = None,
        *,
        model: object | None = None,
    ) -> None:
        self.config = config or PersonDetectorConfig()
        if model is None:
            try:
                from ultralytics import YOLO
            except ImportError as error:
                raise RuntimeError(
                    "Ultralytics is required; install requirements-yolo.txt"
                ) from error
            model = YOLO(str(self.config.model_path))
        self._model = model
        self._person_class_id, self._person_class_name = _resolve_person_class(
            getattr(model, "names", None)
        )

    def process(self, frame: ImageInput) -> tuple[PersonDetection, ...]:
        """Detect people in a ColorFrame or raw BGR ndarray without a camera."""
        image, sequence_id, source_time_s = _frame_input(frame)
        predict_args = {
            "source": image,
            "imgsz": self.config.imgsz,
            "conf": self.config.confidence_threshold,
            "verbose": False,
            "stream": False,
        }
        if self.config.device is not None:
            predict_args["device"] = self.config.device
        results = self._model.predict(**predict_args)
        if len(results) != 1:
            raise RuntimeError(
                f"Expected one Ultralytics Results object for one frame; got {len(results)}"
            )

        result = results[0]
        original_shape = getattr(result, "orig_shape", None)
        if tuple(original_shape or ()) != image.shape[:2]:
            raise RuntimeError(
                "Ultralytics result original shape does not match the input frame"
            )
        boxes = getattr(result, "boxes", None)
        if boxes is None:
            return ()

        xyxy = _to_numpy(boxes.xyxy, "boxes.xyxy").astype(np.float64, copy=False)
        confidences = _to_numpy(boxes.conf, "boxes.conf").astype(np.float64, copy=False)
        class_ids = _to_numpy(boxes.cls, "boxes.cls").astype(np.float64, copy=False)
        if xyxy.ndim != 2 or xyxy.shape[1] != 4:
            raise RuntimeError("Ultralytics boxes.xyxy must have shape (N, 4)")
        if (confidences.shape != (len(xyxy),)
                or class_ids.shape != (len(xyxy),)):
            raise RuntimeError("Ultralytics box, confidence, and class counts differ")

        height, width = image.shape[:2]
        detections = []
        for bbox, confidence, class_value in zip(xyxy, confidences, class_ids):
            if not math.isfinite(float(class_value)) or not float(class_value).is_integer():
                raise RuntimeError("Ultralytics class IDs must be finite integers")
            class_id = int(class_value)
            if class_id != self._person_class_id:
                continue
            if not np.isfinite(bbox).all() or not math.isfinite(float(confidence)):
                raise RuntimeError("Ultralytics returned non-finite person detection values")
            x1, y1, x2, y2 = (float(value) for value in bbox)
            if x1 < 0.0 or y1 < 0.0 or x2 > width or y2 > height:
                raise RuntimeError(
                    "Ultralytics person bbox is outside original input pixel coordinates"
                )
            detections.append(PersonDetection(
                source_sequence_id=sequence_id,
                source_time_s=source_time_s,
                bbox_xyxy_px=(x1, y1, x2, y2),
                confidence=float(confidence),
                class_id=class_id,
                class_name=self._person_class_name,
            ))
        return tuple(detections)
