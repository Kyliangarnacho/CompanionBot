"""Hardware-free contracts for the Stage 7 camera and intrinsic foundation."""

from __future__ import annotations

import numpy as np
import pytest

from perception import camera
from perception.camera import CameraConfig, CameraStream, ColorFrame
from perception.camera_calibration import (
    CameraCalibration,
    load_camera_calibration,
    undistort_image_points,
    validate_calibration_resolution,
)


class FakeCapture:
    def __init__(self, frames=(), *, opened=True):
        self.frames = list(frames)
        self.opened = opened
        self.released = False
        self.properties = {
            camera.cv2.CAP_PROP_FRAME_WIDTH: 640.0,
            camera.cv2.CAP_PROP_FRAME_HEIGHT: 480.0,
            camera.cv2.CAP_PROP_FPS: 30.0,
        }
        self.set_calls = []

    def isOpened(self):
        return self.opened

    def getBackendName(self):
        return "FAKE"

    def get(self, prop):
        return self.properties.get(prop, 0.0)

    def set(self, prop, value):
        self.set_calls.append((prop, value))
        self.properties[prop] = float(value)
        return True

    def read(self):
        if not self.frames:
            return False, None
        return True, self.frames.pop(0)

    def release(self):
        self.released = True
        self.opened = False


def calibration(D=None):
    return CameraCalibration(
        K=np.array([[800.0, 0.0, 320.0],
                    [0.0, 810.0, 240.0],
                    [0.0, 0.0, 1.0]]),
        D=np.zeros(5) if D is None else D,
        frame_width=640,
        frame_height=480,
        source_path="synthetic",
        source_format="test",
    )


def test_camera_defaults_to_opencv_backend_selection_and_returns_bgr_frame(monkeypatch):
    capture = FakeCapture([np.zeros((480, 640, 3), dtype=np.uint8)])
    calls = []
    monkeypatch.setattr(camera.cv2, "VideoCapture",
                        lambda *args: calls.append(args) or capture)
    monkeypatch.setattr(camera.time, "perf_counter_ns", lambda: 12_500_000_000)
    stream = CameraStream(CameraConfig(device=2))

    profile = stream.open()
    frame = stream.read()

    assert calls == [(2,)]
    assert profile.backend_name == "FAKE"
    assert frame.bgr.shape == (480, 640, 3)
    assert frame.bgr.dtype == np.uint8
    assert frame.sequence_id == 0
    assert frame.source_id == "2"
    assert frame.host_receive_time_s == 12.5
    stream.release()
    assert capture.released


def test_explicit_backend_and_requested_profile_are_forwarded(monkeypatch):
    capture = FakeCapture([np.zeros((720, 1280, 3), dtype=np.uint8)])
    calls = []
    monkeypatch.setattr(camera.cv2, "VideoCapture",
                        lambda *args: calls.append(args) or capture)
    stream = CameraStream(CameraConfig(
        device="/dev/video2", requested_width=1280, requested_height=720,
        requested_fps=30.0, backend=17,
    ))

    profile = stream.open()

    assert calls == [("/dev/video2", 17)]
    assert profile.device_identifier == "/dev/video2"
    assert profile.requested_width == 1280
    assert profile.requested_height == 720
    assert profile.reported_width == 1280
    assert profile.reported_height == 720
    assert (camera.cv2.CAP_PROP_FRAME_WIDTH, 1280) in capture.set_calls
    assert (camera.cv2.CAP_PROP_FRAME_HEIGHT, 720) in capture.set_calls
    assert (camera.cv2.CAP_PROP_FPS, 30.0) in capture.set_calls


def test_stream_sequences_frames_and_release_is_idempotent(monkeypatch):
    capture = FakeCapture([
        np.zeros((2, 3, 3), dtype=np.uint8),
        np.ones((2, 3, 3), dtype=np.uint8),
    ])
    times = iter([1_000_000_000, 1_040_000_000])
    monkeypatch.setattr(camera.cv2, "VideoCapture", lambda *_args: capture)
    monkeypatch.setattr(camera.time, "perf_counter_ns", lambda: next(times))
    stream = CameraStream(CameraConfig(device=0))
    stream.open()

    first, second = stream.read(), stream.read()
    stream.release()
    stream.release()

    assert (first.sequence_id, second.sequence_id) == (0, 1)
    assert second.host_receive_time_s > first.host_receive_time_s
    assert capture.released
    assert not stream.is_open


def test_equal_clock_ticks_are_ordered_by_one_nanosecond(monkeypatch):
    capture = FakeCapture([
        np.zeros((2, 3, 3), dtype=np.uint8),
        np.ones((2, 3, 3), dtype=np.uint8),
    ])
    monkeypatch.setattr(camera.cv2, "VideoCapture", lambda *_args: capture)
    monkeypatch.setattr(camera.time, "perf_counter_ns", lambda: 1_000_000_000)
    stream = CameraStream(CameraConfig(device=0))
    stream.open()

    first, second = stream.read(), stream.read()

    assert second.host_receive_time_s > first.host_receive_time_s
    assert second.host_receive_time_s == 1.000000001
    stream.release()


def test_failed_open_releases_capture(monkeypatch):
    capture = FakeCapture(opened=False)
    monkeypatch.setattr(camera.cv2, "VideoCapture", lambda *_args: capture)
    stream = CameraStream(CameraConfig(device="camera-path"))

    with pytest.raises(RuntimeError, match="Could not open"):
        stream.open()

    assert capture.released
    assert not stream.is_open


@pytest.mark.parametrize(
    "image",
    [np.zeros((4, 5), dtype=np.uint8),
     np.zeros((4, 5, 4), dtype=np.uint8),
     np.zeros((4, 5, 3), dtype=np.float32)],
)
def test_noncanonical_capture_frame_is_rejected(monkeypatch, image):
    capture = FakeCapture([image])
    monkeypatch.setattr(camera.cv2, "VideoCapture", lambda *_args: capture)
    stream = CameraStream(CameraConfig(device=0))
    stream.open()

    with pytest.raises(RuntimeError, match="H×W×3 uint8"):
        stream.read()
    stream.release()


def test_color_frame_rejects_dimensions_that_do_not_match_array():
    with pytest.raises(ValueError, match="must match"):
        ColorFrame(np.zeros((4, 5, 3), dtype=np.uint8), 0, "source", 4, 4, 1.0)


def test_camera_config_rejects_partial_or_invalid_capture_profile():
    with pytest.raises(ValueError, match="must be set together"):
        CameraConfig(device=0, requested_width=1280)
    with pytest.raises(ValueError, match="positive"):
        CameraConfig(device=0, requested_fps=0.0)
    with pytest.raises(ValueError, match="nonnegative"):
        CameraConfig(device=-1)


def test_npz_calibration_round_trip_and_resolution_validation(tmp_path):
    K = calibration().K
    D = np.zeros((1, 5), dtype=np.float64)
    path = tmp_path / "intrinsics.npz"
    np.savez(path, camera_matrix=K, dist_coeffs=D,
             image_width=640, image_height=480)

    loaded = load_camera_calibration(path)

    assert loaded.source_format == "npz"
    assert loaded.K.flags.writeable is False
    assert loaded.D.shape == (1, 5)
    validate_calibration_resolution(loaded, 640, 480)
    with pytest.raises(ValueError, match="automatic K scaling"):
        validate_calibration_resolution(loaded, 1280, 720)


def test_opencv_yaml_loading_and_sparse_point_undistortion(tmp_path):
    path = tmp_path / "intrinsics.yml"
    storage = camera.cv2.FileStorage(str(path), camera.cv2.FILE_STORAGE_WRITE)
    storage.write("image_width", 640)
    storage.write("image_height", 480)
    storage.write("camera_matrix", calibration().K)
    storage.write("dist_coeffs", np.zeros((1, 5), dtype=np.float64))
    storage.release()

    loaded = load_camera_calibration(path)
    points = np.array([[0.0, 0.0], [320.0, 240.0], [639.0, 479.0]])
    result = undistort_image_points(points, loaded)

    assert loaded.source_format == "opencv_yaml"
    np.testing.assert_allclose(result, points, atol=1e-10)


@pytest.mark.parametrize(
    ("K", "D"),
    [(np.eye(2), np.zeros(5)),
     (np.diag([-1.0, 1.0, 1.0]), np.zeros(5)),
     (np.eye(3), np.zeros(6)),
     (np.full((3, 3), np.nan), np.zeros(5))],
)
def test_calibration_rejects_invalid_K_or_D(K, D):
    with pytest.raises(ValueError):
        CameraCalibration(K, D, 640, 480, "test", "test")


def test_undistort_rejects_malformed_or_nonfinite_points():
    camera_calibration = calibration()
    for points in (np.zeros((2, 3)), np.array([[np.nan, 0.0]])):
        with pytest.raises(ValueError, match="image_points"):
            undistort_image_points(points, camera_calibration)
