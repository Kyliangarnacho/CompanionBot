"""Source-aligned metric depth boundary for independent slow channels.

``source_time_s`` is the ColorFrame host read-complete time. It is not an
exposure timestamp. Values are metres; nonfinite and nonpositive samples are
represented by NaN. The model's internal inference resolution is not exposed.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from perception.camera import ColorFrame


@dataclass(frozen=True)
class DepthFrame:
    source_id: str
    source_sequence_id: int
    source_time_s: float
    width: int
    height: int
    depth_m: np.ndarray
    backend_name: str
    model_name: str

    def __post_init__(self) -> None:
        if not isinstance(self.source_id, str) or not self.source_id.strip():
            raise ValueError("source_id must not be empty")
        if (isinstance(self.source_sequence_id, bool)
                or not isinstance(self.source_sequence_id, int)
                or self.source_sequence_id < 0):
            raise ValueError("source_sequence_id must be a nonnegative integer")
        if not math.isfinite(float(self.source_time_s)):
            raise ValueError("source_time_s must be finite")
        for name in ("width", "height"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 1:
                raise ValueError(f"{name} must be a positive integer")
        if not isinstance(self.depth_m, np.ndarray):
            raise ValueError("depth_m must be a NumPy array")
        if self.depth_m.dtype != np.float32 or self.depth_m.shape != (self.height, self.width):
            raise ValueError("depth_m must be float32 H×W at source resolution")
        if np.isinf(self.depth_m).any() or np.any(self.depth_m[~np.isnan(self.depth_m)] <= 0):
            raise ValueError("depth_m must contain positive metres or NaN")
        if not isinstance(self.backend_name, str) or not self.backend_name.strip():
            raise ValueError("backend_name must not be empty")
        if not isinstance(self.model_name, str) or not self.model_name.strip():
            raise ValueError("model_name must not be empty")


@dataclass(frozen=True)
class YoloDepthConfig:
    model_path: str | Path = "yolo26n-depth.pt"
    imgsz: int = 768
    device: str | None = None

    def __post_init__(self) -> None:
        if not str(self.model_path):
            raise ValueError("model_path must not be empty")
        if isinstance(self.imgsz, bool) or not isinstance(self.imgsz, int) or self.imgsz < 1:
            raise ValueError("imgsz must be a positive integer")
        if self.device is not None and not self.device.strip():
            raise ValueError("device must be a nonempty string or None")


class YoloDepthAdapter:
    """Keep Ultralytics Results and torch tensors behind a source-frame check."""

    def __init__(self, config: YoloDepthConfig | None = None, *, model: object | None = None):
        self.config = config or YoloDepthConfig()
        if model is None:
            from ultralytics import YOLO

            model = YOLO(str(self.config.model_path))
        if getattr(model, "task", "depth") != "depth":
            raise ValueError("model must have the Ultralytics depth task")
        self._model = model

    def process(self, frame: ColorFrame) -> DepthFrame:
        if not isinstance(frame, ColorFrame):
            raise TypeError("frame must be a ColorFrame")
        arguments = {
            "source": frame.bgr,
            "imgsz": self.config.imgsz,
            "verbose": False,
            "stream": False,
        }
        if self.config.device is not None:
            arguments["device"] = self.config.device
        results = self._model.predict(**arguments)
        if len(results) != 1:
            raise RuntimeError(f"Expected one depth result, got {len(results)}")
        result = results[0]
        if tuple(getattr(result, "orig_shape", None) or ()) != (frame.height, frame.width):
            raise RuntimeError("depth result orig_shape does not match source ColorFrame")
        depth = getattr(result, "depth", None)
        if depth is None or getattr(depth, "data", None) is None:
            raise RuntimeError("Ultralytics depth result has no depth.data")
        values = depth.data
        if hasattr(values, "detach"):
            values = values.detach()
        if hasattr(values, "cpu"):
            values = values.cpu()
        if hasattr(values, "numpy"):
            values = values.numpy()
        depth_m = np.array(values, dtype=np.float32, copy=True)
        if depth_m.shape != (frame.height, frame.width):
            raise RuntimeError("depth map shape does not match source ColorFrame")
        depth_m[~np.isfinite(depth_m) | (depth_m <= 0)] = np.nan
        depth_m.setflags(write=False)
        return DepthFrame(
            source_id=frame.source_id,
            source_sequence_id=frame.sequence_id,
            source_time_s=frame.host_receive_time_s,
            width=frame.width,
            height=frame.height,
            depth_m=depth_m,
            backend_name="ultralytics.depth",
            model_name=Path(str(self.config.model_path)).name,
        )
