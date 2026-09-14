"""Print deterministic raw IMU and wheel-angle sanity checks; no estimator."""

from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATHS = (
    ROOT / "models" / "minisegway" / "mini_segway.xml",
    ROOT / "models" / "minisegway" / "mini_segway_moving_payload.xml",
)
SITE_POS_M = np.array([0.0, 0.0, 0.020])
SITE_QUAT_WXYZ = np.array([1.0, 0.0, 0.0, 0.0])
PITCH_RATE_RAD_S = math.radians(30.0)
WHEEL_ANGLES_RAD = np.array([0.25, -0.40])


def read_sensor(data: mujoco.MjData, name: str) -> np.ndarray:
    return np.asarray(data.sensor(name).data, dtype=float).copy()


def sample(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    pitch_rad: float = 0.0,
    pitch_rate_rad_s: float = 0.0,
    wheel_angles_rad: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Sample a prescribed state held by an ideal external fixture."""

    mujoco.mj_resetDataKeyframe(model, data, 0)
    root = model.joint("root").id
    root_qpos = model.jnt_qposadr[root]
    root_dof = model.jnt_dofadr[root]
    half_pitch = 0.5 * pitch_rad
    data.qpos[root_qpos + 3 : root_qpos + 7] = [
        math.cos(half_pitch),
        math.sin(half_pitch),
        0.0,
        0.0,
    ]
    data.qvel[root_dof + 3] = pitch_rate_rad_s
    if wheel_angles_rad is not None:
        for joint_name, angle_rad in zip(
            ("left_wheel_hinge", "right_wheel_hinge"), wheel_angles_rad
        ):
            joint = model.joint(joint_name).id
            data.qpos[model.jnt_qposadr[joint]] = angle_rad

    mujoco.mj_forward(model, data)
    # Get the native acceleration reading for a motionless, fixture-held state.
    # This changes only MjData for this check; neither plant XML is constrained.
    data.qacc[:] = 0.0
    mujoco.mj_sensorAcc(model, data)
    wheel_angles = np.array(
        [
            read_sensor(data, "left_wheel_angle")[0],
            read_sensor(data, "right_wheel_angle")[0],
        ]
    )
    return (
        read_sensor(data, "imu_accelerometer"),
        read_sensor(data, "imu_gyro"),
        wheel_angles,
    )


def main() -> None:
    cases = (
        ("upright static", 0.0, 0.0, None),
        ("+5 deg pitch static", math.radians(5.0), 0.0, None),
        ("-5 deg pitch static", math.radians(-5.0), 0.0, None),
        ("+30 deg/s pitch motion", 0.0, PITCH_RATE_RAD_S, None),
        ("wheel rotation", 0.0, 0.0, WHEEL_ANGLES_RAD),
    )
    reference_samples = None
    for model_path in MODEL_PATHS:
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        site = model.site("imu_site")
        assert model.site_bodyid[site.id] == model.body("chassis").id
        assert np.array_equal(model.site_pos[site.id], SITE_POS_M)
        assert np.array_equal(model.site_quat[site.id], SITE_QUAT_WXYZ)

        samples = {}
        print(f"\n{model_path.name}: imu_site pos={SITE_POS_M}, quat={SITE_QUAT_WXYZ}")
        for name, pitch, pitch_rate, wheels in cases:
            accel, gyro, angles = sample(model, data, pitch, pitch_rate, wheels)
            samples[name] = (accel, gyro, angles)
            print(f"  {name:24s} accel={accel} gyro={gyro} wheel={angles}")

        theta = math.radians(5.0)
        assert np.allclose(samples["upright static"][0], [0.0, 0.0, 9.81])
        assert np.allclose(
            samples["+5 deg pitch static"][0],
            [0.0, 9.81 * math.sin(theta), 9.81 * math.cos(theta)],
        )
        assert np.allclose(
            samples["-5 deg pitch static"][0],
            [0.0, -9.81 * math.sin(theta), 9.81 * math.cos(theta)],
        )
        assert np.allclose(
            samples["+30 deg/s pitch motion"][1], [PITCH_RATE_RAD_S, 0.0, 0.0]
        )
        assert np.allclose(samples["wheel rotation"][2], WHEEL_ANGLES_RAD)
        if reference_samples is None:
            reference_samples = samples
        else:
            assert all(
                np.array_equal(actual, expected)
                for name in samples
                for actual, expected in zip(samples[name], reference_samples[name])
            )

    print("\nPASS: sensor frames, signs, magnitudes, and both-model parity verified")


if __name__ == "__main__":
    main()
