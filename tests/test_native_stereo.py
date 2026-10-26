"""Native stereo contracts; synthetic fixtures are not rendered accuracy evidence."""

from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import threading

import numpy as np
import pytest

from companionbot_perception import _native_stereo as native


ROOT = Path(__file__).resolve().parents[1]


def config():
    value = native.StereoConfig()
    value.width, value.height = 320, 120
    value.fx = value.fy = 200.0
    value.cx, value.cy = 160.0, 60.0
    value.baseline_m = 0.15
    value.right_translation_from_left = [-0.15, 0.0, 0.0]
    value.calibration_id = "synthetic_rectified_fixture"
    return value


def stamp(sequence=0, source_time=0.0, epoch="synthetic_source", calibration=None,
          clock="declared_software_fixture"):
    return native.StereoFrameStamp(
        epoch, sequence, source_time, calibration or config().calibration_id, clock
    )


def images(value=None, disparity=20):
    value = value or config()
    random = np.random.default_rng(20261024)
    shape = (value.height, value.width, 3)
    left = random.integers(0, 256, size=shape, dtype=np.uint8)
    right = random.integers(0, 256, size=shape, dtype=np.uint8)
    right[:, :-disparity] = left[:, disparity:]
    return left, right


@pytest.mark.parametrize("disparity", [12, 20, 40])
def test_native_stereo_known_rgb_shift_obeys_original_metric_geometry(disparity):
    value = config()
    pair = images(value, disparity)
    result = native.StereoDepthProcessor(value).process(*pair, stamp(), stamp())
    valid = result.valid_mask[20:100, 100:240] != 0
    assert np.mean(valid) > 0.8
    expected_z = value.fx * value.baseline_m / disparity
    assert abs(np.median(result.depth_m[20:100, 100:240][valid]) - expected_z) < 0.01
    good = result.valid_mask != 0
    assert np.all(np.isfinite(result.depth_m[good]))
    assert np.all(result.depth_m[good] > 0)
    assert np.all(np.isnan(result.depth_m[~good]))
    assert np.all(result.rejection_mask[good] == 0)
    assert np.all(result.texture_mask[good] == 255)
    assert np.all(result.lr_consistency_mask[good] == 255)
    assert np.all(result.lr_error_px[good] <= value.lr_max_error_px)
    np.testing.assert_allclose(
        result.depth_m[good], value.fx * value.baseline_m / result.disparity_px[good],
        rtol=1e-7, atol=0,
    )
    diagnostics = result.diagnostics
    assert diagnostics["total_pixels"] == value.width * value.height
    assert diagnostics["valid_pixels"] == np.count_nonzero(good)
    assert diagnostics["valid_fraction"] == np.mean(good)
    assert diagnostics["input_snapshot_bytes"] == 2 * pair[0].nbytes
    assert diagnostics["input_snapshot_wall_s"] >= 0
    assert diagnostics["output_copy_wall_s"] >= 0
    assert diagnostics["binding_total_wall_s"] >= diagnostics["wall_s"] > 0
    assert diagnostics["fallback"] is False
    assert diagnostics["metric_convention"] == "left_optical_axial_z_m"
    durations = ("lock_wait_wall_s", "grayscale_wall_s", "matching_wall_s", "quality_wall_s")
    assert sum(diagnostics[key] for key in durations) == pytest.approx(diagnostics["wall_s"], abs=1e-12)


def test_native_stereo_flat_scene_is_nan_and_does_not_reuse_old_depth():
    value = config()
    processor = native.StereoDepthProcessor(value)
    textured = processor.process(*images(value), stamp(), stamp())
    previous = textured.depth_m.copy()
    flat = np.full((value.height, value.width, 3), 127, dtype=np.uint8)
    result = processor.process(flat, flat, stamp(1, 0.05), stamp(1, 0.05))
    assert result.diagnostics["valid_pixels"] == 0
    assert not np.any(result.valid_mask)
    assert not np.any(result.texture_mask)
    assert np.all(np.isnan(result.depth_m))
    assert np.all((result.rejection_mask & native.LOW_LEFT_TEXTURE) != 0)
    np.testing.assert_array_equal(textured.depth_m, previous)


def test_native_stereo_binding_owns_readonly_outputs_and_accepts_padded_rows():
    value = config()
    processor = native.StereoDepthProcessor(value)
    left, right = images(value)
    padded_left = np.zeros((value.height, value.width + 5, 3), np.uint8)
    padded_right = np.zeros_like(padded_left)
    padded_left[:, :value.width] = left
    padded_right[:, :value.width] = right
    result = processor.process(padded_left[:, :value.width], padded_right[:, :value.width], stamp(), stamp())
    arrays = [getattr(result, name) for name in (
        "depth_m", "disparity_px", "lr_error_px", "valid_mask", "rejection_mask",
        "texture_mask", "lr_consistency_mask",
    )]
    preserved = [array.copy() for array in arrays]
    padded_left[:] = 0
    padded_right[:] = 0
    processor.process(left, right, stamp(1, 0.05), stamp(1, 0.05))
    for index, array in enumerate(arrays):
        assert array.flags.owndata and array.base is None
        assert not array.flags.writeable and array.flags.c_contiguous
        assert array.shape == (value.height, value.width)
        assert array.dtype == (np.float32 if index < 3 else np.uint8)
        with pytest.raises(ValueError):
            array[0, 0] = 0
        np.testing.assert_array_equal(array, preserved[index])
        assert not np.shares_memory(array, padded_left)
        assert all(not np.shares_memory(array, other) for other in arrays[index + 1:])


@pytest.mark.parametrize("change", [
    {"epoch": "other_source"}, {"sequence": 2}, {"source_time": 0.051},
    {"calibration": "wrong_calibration"}, {"clock": "other_clock"},
])
def test_native_stereo_rejects_unpaired_frames_without_advancing_watermark(change):
    processor = native.StereoDepthProcessor(config())
    pair = images()
    processor.process(*pair, stamp(), stamp())
    right = {"sequence": 1, "source_time": 0.05, **change}
    with pytest.raises(ValueError, match="source_calibration_or_order"):
        processor.process(*pair, stamp(1, 0.05), stamp(**right))
    snapshot = processor.snapshot()
    assert snapshot["accepted_frames"] == 1
    assert snapshot["rejected_native_inputs"] == 1
    assert snapshot["last_sequence"] == 0
    processor.process(*pair, stamp(1, 0.05), stamp(1, 0.05))
    assert processor.snapshot()["accepted_frames"] == 2


@pytest.mark.parametrize("candidate", [
    lambda: stamp(), lambda: stamp(1, 0.0), lambda: stamp(0, 0.05),
    lambda: stamp(1, float("nan")), lambda: stamp(1, float("inf")),
    lambda: stamp(1, -0.1), lambda: stamp(1, 0.05, epoch=""),
    lambda: stamp(1, 0.05, epoch="new_epoch"),
    lambda: stamp(1, 0.05, clock="new_clock"),
])
def test_native_stereo_cannot_reset_processor_source_or_replay_old_capture(candidate):
    processor = native.StereoDepthProcessor(config())
    pair = images()
    processor.process(*pair, stamp(), stamp())
    next_stamp = candidate()
    with pytest.raises(ValueError):
        processor.process(*pair, next_stamp, next_stamp)
    assert processor.snapshot()["accepted_frames"] == 1


@pytest.mark.parametrize("field,bad_value", [
    ("baseline_m", 0.0), ("baseline_m", float("nan")), ("fx", float("inf")),
    ("calibration_id", ""), ("num_disparities", 17), ("min_disparity", -1),
    ("block_size", 2), ("p1", 0), ("p2", 32767),
    ("disp12_max_diff", 2**31 - 1), ("speckle_range", 2**31 - 1),
    ("right_translation_from_left", [0.15, 0.0, 0.0]),
    ("right_rotation_from_left", [1, 0.01, 0, 0, 1, 0, 0, 0, 1]),
    ("texture_min_gradient_rms", 0.0), ("lr_max_error_px", float("inf")),
])
def test_native_stereo_invalid_calibration_or_processor_config_is_rejected(field, bad_value):
    value = config()
    setattr(value, field, bad_value)
    with pytest.raises(ValueError):
        native.StereoDepthProcessor(value)


@pytest.mark.parametrize("transform", [
    lambda a: a.astype(np.float32), lambda a: a[:, :, ::-1],
    lambda a: a[::-1], lambda a: a[:, ::-1], lambda a: a[:, :, 0],
    lambda a: a[:, :-1], lambda a: a[:0],
])
def test_native_stereo_binding_rejects_wrong_dtype_shape_or_stride(transform):
    processor = native.StereoDepthProcessor(config())
    left, right = images()
    with pytest.raises(ValueError):
        processor.process(transform(left), right, stamp(), stamp())
    assert processor.snapshot()["accepted_frames"] == 0


def test_native_stereo_rejects_boolean_sequence_and_preserves_full_uint64():
    with pytest.raises(ValueError, match="unsigned_integer"):
        stamp(True)
    maximum = native.StereoFrameStamp("epoch", 2**64 - 1, 0.0, "calibration")
    assert maximum.sequence == 2**64 - 1


def test_native_stereo_float32_underflow_cannot_be_a_valid_metric_depth():
    value = config()
    value.baseline_m = 1e-47
    value.right_translation_from_left = [-value.baseline_m, 0.0, 0.0]
    processor = native.StereoDepthProcessor(value)
    result = processor.process(*images(value), stamp(), stamp())
    assert result.diagnostics["valid_pixels"] == 0
    assert result.diagnostics["invalid_metric_depth"] > 0
    assert not np.any(result.valid_mask)
    assert np.all(np.isnan(result.depth_m))


def test_native_stereo_actual_declared_camera_rectification_is_explicit():
    camera = json.loads((ROOT / "models/minisegway/native_runtime/config/stereo_camera.json").read_text())
    value = config()
    value.width, value.height = camera["width"], camera["height"]
    value.fx = camera["K"][0][0]
    value.fy = camera["K"][1][1]
    value.cx, value.cy = camera["K"][0][2], camera["K"][1][2]
    value.baseline_m = camera["baseline_m"]
    value.right_rotation_from_left = np.array(camera["right_from_left"]["rotation"]).ravel().tolist()
    value.right_translation_from_left = camera["right_from_left"]["translation_m"]
    audit = native.StereoDepthProcessor(value).calibration_audit()
    np.testing.assert_array_equal(np.array(audit["R1"]).reshape(3, 3), np.eye(3))
    np.testing.assert_array_equal(np.array(audit["R2"]).reshape(3, 3), np.eye(3))
    expected = np.column_stack((np.array(camera["K"]), np.zeros(3)))
    np.testing.assert_allclose(np.array(audit["P1"]).reshape(3, 4), expected, rtol=0, atol=1e-7)
    expected[0, 3] = -value.fx * value.baseline_m
    np.testing.assert_allclose(np.array(audit["P2"]).reshape(3, 4), expected, rtol=0, atol=1e-7)
    assert native.OPENCV_COMPILED_VERSION == native.OPENCV_RUNTIME_VERSION == "4.6.0"


def test_native_stereo_concurrent_duplicate_and_snapshot_are_serialized_without_gil_deadlock():
    processor = native.StereoDepthProcessor(config())
    left, right = images()
    left.setflags(write=False)
    right.setflags(write=False)
    barrier = threading.Barrier(3)

    def process():
        barrier.wait(timeout=5)
        try:
            return processor.process(left, right, stamp(), stamp()).diagnostics["valid_pixels"] > 0
        except ValueError:
            return False

    def snapshot():
        barrier.wait(timeout=5)
        return processor.snapshot()

    with ThreadPoolExecutor(max_workers=3) as executor:
        calls = [executor.submit(process), executor.submit(process), executor.submit(snapshot)]
        accepted = [future.result(timeout=10) for future in calls[:2]]
        observed = calls[2].result(timeout=10)
    assert sorted(accepted) == [False, True]
    assert observed["accepted_frames"] in (0, 1)
    assert processor.snapshot()["accepted_frames"] == 1
    assert processor.snapshot()["rejected_native_inputs"] == 1
