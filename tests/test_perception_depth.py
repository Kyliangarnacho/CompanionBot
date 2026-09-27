"""Hardware-free checks for source-frame depth and independent scheduling."""

from types import SimpleNamespace

import numpy as np
import pytest

from perception.camera import ColorFrame
from perception.depth import DepthFrame, YoloDepthAdapter, YoloDepthConfig


def _frame(seq: int = 7) -> ColorFrame:
    return ColorFrame(
        bgr=np.zeros((720, 1280, 3), dtype=np.uint8),
        sequence_id=seq, source_id="test-camera", width=1280, height=720,
        host_receive_time_s=100.25,
    )


class _Model:
    task = "depth"

    def __init__(self, shape=(720, 1280), orig_shape=(720, 1280)):
        self.shape = shape
        self.orig_shape = orig_shape

    def predict(self, **kwargs):
        assert kwargs["source"].shape == (720, 1280, 3)
        return [SimpleNamespace(
            orig_shape=self.orig_shape,
            depth=SimpleNamespace(data=np.full(self.shape, 2.5, dtype=np.float32)),
        )]


def test_depth_adapter_preserves_source_metadata_shape_and_metres():
    depth = YoloDepthAdapter(YoloDepthConfig(imgsz=640), model=_Model()).process(_frame())
    assert isinstance(depth, DepthFrame)
    assert (depth.source_id, depth.source_sequence_id, depth.source_time_s) == (
        "test-camera", 7, 100.25)
    assert depth.depth_m.shape == (720, 1280)
    assert depth.depth_m.dtype == np.float32
    assert depth.depth_m[200, 400] == 2.5
    assert not depth.depth_m.flags.writeable


@pytest.mark.parametrize("shape,orig_shape", [
    ((640, 640), (720, 1280)),
    ((720, 1280), (640, 640)),
])
def test_depth_adapter_rejects_network_space_or_wrong_source_shape(shape, orig_shape):
    with pytest.raises(RuntimeError, match="shape"):
        YoloDepthAdapter(model=_Model(shape, orig_shape)).process(_frame())


def test_invalid_depth_becomes_nan():
    model = _Model()
    invalid = np.full((720, 1280), 2, dtype=np.float32)
    invalid[0, :3] = [0, -1, np.inf]
    model.predict = lambda **_kwargs: [SimpleNamespace(
        orig_shape=(720, 1280), depth=SimpleNamespace(data=invalid))]
    depth = YoloDepthAdapter(model=model).process(_frame())
    assert np.isnan(depth.depth_m[0, :3]).all()
    assert depth.depth_m[0, 3] == 2
