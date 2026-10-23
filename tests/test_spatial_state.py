"""Physical and lifecycle invariants for the passive spatial data foundation."""
from dataclasses import replace
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from companionbot_contracts.spatial_geometry import DESCRIPTION, load_spatial_geometry, load_sim_lidar
from companionbot_control.estimation.longitudinal_estimation import load_longitudinal_estimator_config
from companionbot_control.estimation.spatial import PlanarOdometry, SpatialAttitude, SpatialSample


def sample(t=.02, sequence=10, **changes):
    result = SpatialSample("source-epoch", sequence, t, 0., 0., 0., 0., 0., 0., 0., 0.,
                           (0., 0.), (0., 0.), t-.001, t, .001)
    return replace(result, **changes)


def test_ros_axes_are_proper_rotation_and_historical_wheel_names_are_explicit():
    geometry = load_spatial_geometry()
    assert np.linalg.det(geometry.mj_to_ros) == 1.
    np.testing.assert_array_equal(geometry.vector_to_ros([0, -1, 0]), [1, 0, 0])
    np.testing.assert_array_equal(geometry.vector_to_ros([1, 0, 0]), [0, 1, 0])
    assert (geometry.ros_left_encoder_index, geometry.ros_right_encoder_index) == (1, 0)
    tree = ET.parse(DESCRIPTION.parents[2]/"models/minisegway/mini_segway.xml")
    native_left = np.fromstring(tree.find("worldbody/body/body[@name='left_wheel']").get("pos"), sep=" ")
    assert geometry.vector_to_ros(native_left)[1] < 0


def test_derived_geometry_cannot_silently_change_while_retaining_identity(tmp_path):
    raw = json.loads((DESCRIPTION/"config/minisegway.yaml").read_text())
    raw["wheel_radius_m"] *= 2
    altered = tmp_path/"altered.json"
    altered.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="digest"):
        load_spatial_geometry(altered)


def test_pitch_only_encoder_geometry_does_not_create_axle_translation():
    from companionbot_simulation.scenarios.check_encoder_displacement_geometry import pitch_only_case
    geometry = load_spatial_geometry()
    result = pitch_only_case(geometry.wheel_radius_m)
    assert result["gt_forward_displacement_m"] == 0.
    assert abs(result["naive_signed_error_m"]) > .003
    assert abs(result["corrected_signed_error_m"]) < .00006


def test_navigation_yaw_projection_handles_tilt_without_changing_control_yaw():
    attitude = SpatialAttitude(load_longitudinal_estimator_config().pitch)
    theta, rate = math.radians(10), 1.
    accel = [-9.81*math.sin(theta), 0., 9.81*math.cos(theta)]
    gyro = [-rate*math.sin(theta), 0., rate*math.cos(theta)]
    for index in range(501):
        result = attitude.update(accel, gyro, theta, .001+.002*index, .001)
    assert abs(result.roll_rad) < 1e-12
    assert result.yaw_rad == pytest.approx(1.001, abs=1e-12)
    assert result.yaw_rate_rad_s == pytest.approx(rate)
    assert 1-math.cos(theta) > .015  # Measured failure of the simpler body-z baseline.


def test_roll_and_pitch_joint_rotation_is_not_flattened():
    phi, theta, yaw_rate = .07, .12, .5
    rotation = Rotation.from_euler("xyz", [phi, theta, 0.]).as_matrix()
    accel = rotation.T @ np.array([0, 0, 9.81])
    gyro = rotation.T @ np.array([0, 0, yaw_rate])
    attitude = SpatialAttitude(load_longitudinal_estimator_config().pitch)
    for index in range(501):
        result = attitude.update(accel, gyro, theta, .001+.002*index, .001)
    assert result.roll_rad == pytest.approx(phi, abs=1e-12)
    assert result.yaw_rate_rad_s == pytest.approx(yaw_rate, abs=1e-12)


def test_finite_input_overflow_latches_instead_of_publishing_invalid_odometry():
    odom = PlanarOdometry()
    odom.update(sample(distance_m=-1e308))
    with np.errstate(over="ignore", invalid="ignore"):
        with pytest.raises(ValueError, match="nonfinite_derived"):
            odom.update(sample(.04, 20, distance_m=1e308))
    with pytest.raises(ValueError, match="latched"):
        odom.update(sample(.06, 30))


def test_planar_arc_length_integrates_into_a_circle_and_covariance_grows():
    odom = PlanarOdometry()
    odom.update(sample())
    for index in range(1, 101):
        t = .02 + .02*index
        result = odom.update(sample(t=t, sequence=10+index*10,
            distance_m=.01*index, yaw_rad=.01*index, velocity_m_s=.5, yaw_rate_rad_s=.5))
    assert result.x_m == pytest.approx(math.sin(1.), abs=1e-12)
    assert result.y_m == pytest.approx(1-math.cos(1.), abs=1e-12)
    assert result.yaw_rad == pytest.approx(1., abs=1e-12)
    assert np.linalg.eigvalsh(result.covariance).min() > 0


@pytest.mark.parametrize("change,reason", [
    ({"imu_stale": True}, "sensor_invalid"),
    ({"imu_valid": False}, "sensor_invalid"),
    ({"encoder_valid": False}, "sensor_invalid"),
    ({"distance_m": float("nan")}, "nonfinite"),
    ({"imu_sample_time_s": .03}, "source_time"),
    ({"imu_available_time_s": .03}, "source_time"),
    ({"imu_age_s": .006}, "source_time"),
    ({"pitch_rad": math.pi/3}, "tilt_envelope"),
    ({"roll_rad": math.pi/3}, "tilt_envelope"),
])
def test_faults_latch_without_relabeling_old_data_as_current(change, reason):
    odom = PlanarOdometry()
    odom.update(sample())
    with pytest.raises(ValueError, match=reason):
        odom.update(sample(.04, 20, **change))
    with pytest.raises(ValueError, match="latched"):
        odom.update(sample(.06, 30))


@pytest.mark.parametrize("bad", [sample(.01, 20), sample(.04, 10), sample(.3, 30), sample(.04, 20, epoch="reboot")])
def test_replay_gaps_and_device_restarts_require_explicit_new_owner(bad):
    odom = PlanarOdometry()
    odom.update(sample())
    with pytest.raises(ValueError):
        odom.update(bad)
    assert odom.previous.time_s == .02


def test_description_uses_measured_joint_angles_and_external_dynamic_body():
    import xacro
    document = xacro.process_file(str(DESCRIPTION/"urdf/companionbot.urdf.xacro"), mappings={
        "geometry_config": str(DESCRIPTION/"config/minisegway.yaml"),
        "lidar_config": str(DESCRIPTION/"config/sim_lidar.yaml"), "sim_lidar": "true"})
    xml = ET.fromstring(document.toxml())
    assert xml.find("link[@name='base_footprint']") is None
    assert xml.find("link[@name='camera_optical_frame']") is None
    assert xml.find("link[@name='lidar_link']") is not None
    for side in ("left", "right"):
        joint = xml.find(f"joint[@name='{side}_wheel_joint']")
        assert joint.get("type") == "continuous"
        assert joint.find("axis").get("xyz") == "0 1 0"


def test_indoor_lidar_is_three_dimensional_and_pitch_hits_the_floor(tmp_path):
    import mujoco
    from companionbot_simulation.core.minisegway import MiniSegwaySim
    from companionbot_simulation.core.spatial_scene import prepare_indoor_model, VirtualPlanarLidar
    from companionbot_contracts.paths import repository_root
    path = prepare_indoor_model(repository_root()/"models/minisegway/mini_segway.xml", tmp_path/"room.xml")
    sim = MiniSegwaySim(path)
    config = load_sim_lidar()
    config["range_noise_std_m"] = 0.
    lidar = VirtualPlanarLidar(sim.model, config)
    scan = lidar.sample(sim.model, sim.data)
    assert scan.ranges_m[180] == pytest.approx(6., abs=1e-12)
    assert scan.ranges_m[270] == pytest.approx(4., abs=1e-12)
    assert scan.ranges_m[0] == pytest.approx(2., abs=1e-12)
    sim.reset(pitch_rad=math.radians(10))
    mujoco.mj_forward(sim.model, sim.data)
    tilted = lidar.sample(sim.model, sim.data)
    height = .043 + .18*math.cos(math.radians(10))
    assert tilted.ranges_m[180] == pytest.approx(height/math.sin(math.radians(10)), abs=1e-12)
    assert tilted.ranges_m[180] < scan.ranges_m[180]/2


def test_session_and_description_share_selected_lidar_configuration(tmp_path):
    """An installed/custom sensor mount must reach ray casting and its TF alike."""
    import xacro
    from companionbot_simulation.runtime.spatial_session import SpatialSession

    config = load_sim_lidar()
    config.update(xyz_m=[.08, .03, .24], rpy_rad=[.02, .05, .17], rays=32,
                  period_s=.02, range_noise_std_m=0.)
    selected = tmp_path / "selected_lidar.yaml"
    selected.write_text(json.dumps(config))
    description = xacro.process_file(str(DESCRIPTION / "urdf/companionbot.urdf.xacro"), mappings={
        "geometry_config": str(DESCRIPTION / "config/minisegway.yaml"),
        "lidar_config": str(selected), "sim_lidar": "true"})
    origin = ET.fromstring(description.toxml()).find("joint[@name='sim_lidar_mount']/origin")
    frames, physical_mount = [], {}

    def capture_mount_posthoc(sim):
        if not physical_mount:
            site = sim.model.site("spatial_lidar")
            physical_mount["position"] = site.pos.copy()
            physical_mount["quaternion"] = site.quat.copy()

    session = SpatialSession(tmp_path / "session", scenario="balance", duration_s=.2,
                             lidar_config=selected, on_frame=frames.append,
                             posthoc=capture_mount_posthoc)
    session.run()
    c = session.geometry.mj_to_ros
    np.testing.assert_allclose(c @ physical_mount["position"],
                               np.fromstring(origin.get("xyz"), sep=" "), atol=1e-12)
    q = physical_mount["quaternion"]
    actual_rotation = c @ Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    expected_rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" ")).as_matrix()
    np.testing.assert_allclose(actual_rotation, expected_rotation, atol=1e-12)
    scans = [frame["scan"] for frame in frames if frame["scan"] is not None]
    assert len(scans) == 10
    assert all(len(scan.ranges_m) == 32 and scan.scan_time == .02 for scan in scans)
