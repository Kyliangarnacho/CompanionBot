import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from perception.camera import ColorFrame
from perception.robot_geometry import (
    CameraExtrinsic, ConstantPitchProvider, PitchSample, camera_to_robot_observation,
)


R_BODY_CAMERA = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


def _frame():
    return ColorFrame(np.zeros((72, 128, 3), np.uint8), 42, "camera:test", 128, 72, 10.0)


def test_camera_right_is_robot_right_negative_y_and_camera_down_is_negative_z():
    mount = CameraExtrinsic(R_BODY_CAMERA, [0.08, 0.0, 0.35], True)
    result = camera_to_robot_observation([0.4, 0.2, 2], _frame(), 7, mount,
                                         ConstantPitchProvider(), depth_convention="axial_z")
    np.testing.assert_allclose([result.x_forward_m, result.y_left_m, result.z_up_m],
                               [2.08, -0.4, 0.15])
    assert (result.source_id, result.source_sequence_id, result.source_time_s) == ("camera:test", 42, 10.0)
    assert result.time_semantics == "host_read_complete"
    assert result.extrinsic_provisional and result.pitch_provisional
    assert result.frame_id == "robot_leveled_axle"


def test_pitch_rotates_translation_about_axle_and_nose_down_sign():
    mount = CameraExtrinsic(R_BODY_CAMERA, [0.1, 0, 0.5], True)
    result = camera_to_robot_observation([0, 0, 2], _frame(), 1, mount,
                                         ConstantPitchProvider(np.pi / 2), depth_convention="axial_z")
    np.testing.assert_allclose([result.x_forward_m, result.y_left_m, result.z_up_m],
                               [0.5, 0, -2.1], atol=1e-12)


def test_pitch_compensation_recovers_fixed_leveled_point_for_either_pitch_sign():
    mount = CameraExtrinsic(R_BODY_CAMERA, [0.08, 0, 0.35], True)
    target = np.array([2.0, 0.4, 0.8])
    for theta in (-0.3, 0.0, 0.3):
        body = Rotation.from_rotvec([0, theta, 0]).inv().apply(target)
        camera = R_BODY_CAMERA.T @ (body - mount.translation_body_camera_m)
        result = camera_to_robot_observation(camera, _frame(), 1, mount,
                                             ConstantPitchProvider(theta), depth_convention="axial_z")
        np.testing.assert_allclose([result.x_forward_m, result.y_left_m, result.z_up_m], target)


def test_missing_pitch_coverage_is_unavailable_and_latest_wrong_time_is_rejected():
    mount = CameraExtrinsic(R_BODY_CAMERA, [0, 0, 0], True)

    class Provider:
        def __init__(self, sample):
            self.sample = sample

        def at_source_time(self, source_time_s):
            assert source_time_s == 10.0
            return self.sample

    assert camera_to_robot_observation([0, 0, 2], _frame(), 1, mount,
                                       Provider(None), depth_convention="axial_z") is None
    for sample in (PitchSample(11.0, 0.0, "newest_wrong_time", False),
                   PitchSample(10.0, 0.0, "device_clock", False, clock_domain="device")):
        with pytest.raises(ValueError, match="exact source time"):
            camera_to_robot_observation([0, 0, 2], _frame(), 1, mount,
                                        Provider(sample), depth_convention="axial_z")


def test_reflection_is_not_accepted_as_camera_rotation():
    with pytest.raises(ValueError, match="proper orthonormal"):
        CameraExtrinsic(np.diag([1, -1, 1]), [0, 0, 0], True)


def test_provider_can_explicitly_map_exposure_to_host_clock_without_relabeling_receipt():
    class ExposureMappedProvider:
        def at_source_time(self, source_time_s):
            return PitchSample(source_time_s, 0.1, "test_mapped_history", False,
                               pitch_time_s=9.96, alignment_basis="capture_time_mapped_to_host")

    result = camera_to_robot_observation([0, 0, 2], _frame(), 1,
        CameraExtrinsic(R_BODY_CAMERA, [0, 0, 0], True), ExposureMappedProvider(),
        depth_convention="axial_z")
    assert result.source_time_s == 10.0
    assert result.pitch_time_s == 9.96
    assert result.pitch_alignment_basis == "capture_time_mapped_to_host"
    assert result.time_semantics == "host_read_complete"


def test_final_config_assets_exist_and_provisional_mount_is_proper_rotation():
    directory = Path(__file__).resolve().parents[1] / "models/minisegway/stage7/config"
    config = json.loads((directory / "final_demo.json").read_text())
    assert (directory / config["calibration_asset"]).is_file()
    assert (directory / config["reid_policy_asset"]).is_file()
    mount = config["camera_extrinsic"]
    CameraExtrinsic(mount["rotation_body_camera"], mount["translation_body_camera_m"], mount["provisional"])
    assert mount["provisional"] is True
    assert config["pitch"]["provisional"] is True
