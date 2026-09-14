"""Validate wheel-count displacement geometry without estimating velocity/state."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import FixedLQR, design_from_files
from sim import MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
LQR_CONFIG_PATH = MODEL_DIR / "lqr_baseline_config.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
PITCH_ONLY_RAD = math.radians(5.0)
PURE_ROLL_DISPLACEMENT_M = 0.100
LQR_INITIAL_ERROR_DEG = 5.0
LQR_DURATION_S = 1.0


def chassis_pitch_rad(sim: MiniSegwaySim) -> float:
    root_qpos = sim.model.jnt_qposadr[sim.model.joint("root").id]
    w, x, y, z = sim.data.qpos[root_qpos + 3 : root_qpos + 7]
    return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))


def set_chassis_pitch(sim: MiniSegwaySim, pitch_rad: float) -> None:
    root_qpos = sim.model.jnt_qposadr[sim.model.joint("root").id]
    half_pitch = 0.5 * pitch_rad
    sim.data.qpos[root_qpos + 3 : root_qpos + 7] = [
        math.cos(half_pitch),
        math.sin(half_pitch),
        0.0,
        0.0,
    ]


def set_wheel_angles(sim: MiniSegwaySim, angles_rad: np.ndarray) -> None:
    for joint_name, angle_rad in zip(
        ("left_wheel_hinge", "right_wheel_hinge"), angles_rad
    ):
        joint = sim.model.joint(joint_name).id
        sim.data.qpos[sim.model.jnt_qposadr[joint]] = angle_rad


def raw_wheel_angles_rad(sim: MiniSegwaySim) -> np.ndarray:
    return np.array(
        [
            sim.data.sensor("left_wheel_angle").data[0],
            sim.data.sensor("right_wheel_angle").data[0],
        ]
    )


def wheel_world_pitch_rad(sim: MiniSegwaySim, body_name: str) -> float:
    rotation = sim.data.xmat[sim.model.body(body_name).id].reshape(3, 3)
    return math.atan2(rotation[2, 1], rotation[2, 2])


def capture(sim: MiniSegwaySim) -> dict[str, object]:
    root_qpos = sim.model.jnt_qposadr[sim.model.joint("root").id]
    return {
        "chassis_world_y_m": float(sim.data.qpos[root_qpos + 1]),
        "chassis_pitch_rad": chassis_pitch_rad(sim),
        "encoder_counts": sim.wheel_encoder_counts(),
        "raw_wheel_angles_rad": raw_wheel_angles_rad(sim),
        "wheel_world_pitch_rad": np.array(
            [
                wheel_world_pitch_rad(sim, "left_wheel"),
                wheel_world_pitch_rad(sim, "right_wheel"),
            ]
        ),
    }


def evaluate(
    name: str,
    sim: MiniSegwaySim,
    start: dict[str, object],
    wheel_radius_m: float,
) -> dict[str, object]:
    end = capture(sim)
    count_signs = np.array(
        [sim.encoder_profile.left_count_sign, sim.encoder_profile.right_count_sign]
    )
    delta_counts = end["encoder_counts"] - start["encoder_counts"]
    delta_phi = (
        2.0
        * math.pi
        * delta_counts
        / (count_signs * sim.encoder_profile.output_decoded_counts_per_rev)
    )
    delta_theta = math.atan2(
        math.sin(end["chassis_pitch_rad"] - start["chassis_pitch_rad"]),
        math.cos(end["chassis_pitch_rad"] - start["chassis_pitch_rad"]),
    )
    gt_displacement = -(
        end["chassis_world_y_m"] - start["chassis_world_y_m"]
    )
    naive = wheel_radius_m * float(np.mean(delta_phi))
    corrected = wheel_radius_m * (float(np.mean(delta_phi)) + delta_theta)

    raw_delta_phi = end["raw_wheel_angles_rad"] - start["raw_wheel_angles_rad"]
    delta_world_wheel_pitch = (
        end["wheel_world_pitch_rad"] - start["wheel_world_pitch_rad"]
    )
    composition_error = delta_world_wheel_pitch - (raw_delta_phi + delta_theta)
    raw_corrected = wheel_radius_m * (float(np.mean(raw_delta_phi)) + delta_theta)
    return {
        "name": name,
        "delta_encoder_counts": delta_counts.tolist(),
        "delta_relative_wheel_angle_from_counts_rad": delta_phi.tolist(),
        "delta_raw_relative_wheel_angle_oracle_rad": raw_delta_phi.tolist(),
        "delta_chassis_pitch_rad": delta_theta,
        "delta_wheel_world_pitch_rad": delta_world_wheel_pitch.tolist(),
        "world_angle_composition_error_rad": composition_error.tolist(),
        "gt_forward_displacement_m": gt_displacement,
        "naive_displacement_m": naive,
        "corrected_displacement_m": corrected,
        "naive_signed_error_m": naive - gt_displacement,
        "corrected_signed_error_m": corrected - gt_displacement,
        "raw_angle_oracle_corrected_error_m": raw_corrected - gt_displacement,
    }


def pitch_only_case(wheel_radius_m: float) -> dict[str, object]:
    sim = MiniSegwaySim()
    start = capture(sim)
    set_chassis_pitch(sim, PITCH_ONLY_RAD)
    set_wheel_angles(sim, np.full(2, -PITCH_ONLY_RAD))
    mujoco.mj_forward(sim.model, sim.data)
    return evaluate("pitch_only_fixed_axle", sim, start, wheel_radius_m)


def pure_roll_case(wheel_radius_m: float) -> dict[str, object]:
    sim = MiniSegwaySim()
    start = capture(sim)
    root_qpos = sim.model.jnt_qposadr[sim.model.joint("root").id]
    sim.data.qpos[root_qpos + 1] -= PURE_ROLL_DISPLACEMENT_M
    set_wheel_angles(sim, np.full(2, PURE_ROLL_DISPLACEMENT_M / wheel_radius_m))
    mujoco.mj_forward(sim.model, sim.data)
    return evaluate("pure_forward_roll_fixed_pitch", sim, start, wheel_radius_m)


def lqr_transient_case(wheel_radius_m: float) -> dict[str, object]:
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    config = json.loads(LQR_CONFIG_PATH.read_text(encoding="utf-8"))
    plant = json.loads(PLANT_PARAMETERS_PATH.read_text(encoding="utf-8"))
    theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    design = design_from_files(REDUCED_PATH, LQR_CONFIG_PATH)
    controller = FixedLQR(design, plant["known"]["wheel_torque_hard_peak_nm"])
    sim = MiniSegwaySim()
    sim.reset(theta_eq + math.radians(LQR_INITIAL_ERROR_DEG))
    start = capture(sim)
    held = controller.command(sim.longitudinal_state(theta_eq))
    for physics_step in range(round(LQR_DURATION_S / sim.physics_dt)):
        if physics_step % config["physics_steps_per_update"] == 0:
            held = controller.command(sim.longitudinal_state(theta_eq))
        sim.step(held.left_torque_nm, held.right_torque_nm)
    return evaluate("frozen_lqr_plus5deg_1s", sim, start, wheel_radius_m)


def main() -> None:
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    wheel_radius = float(reduced["parameters"]["wheel_radius_m"])
    cases = [
        pitch_only_case(wheel_radius),
        pure_roll_case(wheel_radius),
        lqr_transient_case(wheel_radius),
    ]
    half_count_distance = (
        math.pi
        * wheel_radius
        / MiniSegwaySim().encoder_profile.output_decoded_counts_per_rev
    )
    assert abs(cases[0]["corrected_signed_error_m"]) <= half_count_distance
    assert abs(cases[0]["naive_signed_error_m"]) > 1e-3
    assert abs(cases[1]["corrected_signed_error_m"]) <= half_count_distance
    assert abs(cases[2]["corrected_signed_error_m"]) < abs(
        cases[2]["naive_signed_error_m"]
    )
    assert all(
        np.allclose(case["world_angle_composition_error_rad"], 0.0, atol=1e-6)
        for case in cases
    )
    report = {
        "wheel_radius_m": wheel_radius,
        "forward_axis": "-Y",
        "count_to_relative_angle": "delta_phi_i = 2*pi*delta_count_i/(count_sign_i*counts_per_rev)",
        "validated_displacement": "delta_p = radius*(mean(delta_phi_left, delta_phi_right)+delta_theta_chassis)",
        "half_count_distance_m": half_count_distance,
        "cases": cases,
        "status": "PASS",
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
