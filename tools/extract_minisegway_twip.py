"""Extract the small-angle longitudinal TWIP model from plant mass properties."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "models" / "minisegway"


def main() -> None:
    plant = json.loads((MODEL_DIR / "plant_report.json").read_text(encoding="utf-8"))
    params = plant["parameters"]
    chassis = plant["chassis_aggregate"]

    body_mass = float(chassis["mass_kg"])
    body_com = np.asarray(chassis["center_m"], dtype=float)
    body_pitch_inertia = float(chassis["inertia_about_com_kg_m2"][0][0])
    wheel_mass = float(plant["single_wheel_rotating_mass_kg"])
    wheel_inertia = float(plant["single_wheel_axial_inertia_about_hinge_kg_m2"])
    wheel_radius = float(params["known"]["wheel_radius_m"])
    track_width = 2.0 * float(params["provisional"]["wheel_center_x_m"])

    # CAD axes: +X right/axle, +Y rear, +Z up. Forward is -Y. Positive
    # pitch is right-hand rotation about +X, i.e. nose-down.
    com_forward = -body_com[1]
    com_height = body_com[2]
    com_length = math.hypot(com_forward, com_height)
    theta_eq = math.atan2(body_com[1], body_com[2])

    # No-slip small-angle model about theta_eq.  u is tau_left + tau_right.
    equivalent_translation_mass = body_mass + 2.0 * wheel_mass + 2.0 * wheel_inertia / wheel_radius**2
    body_first_moment = body_mass * com_length
    body_pitch_about_axle = body_pitch_inertia + body_mass * com_length**2
    mass_matrix = np.array(
        [
            [equivalent_translation_mass, body_first_moment],
            [body_first_moment, body_pitch_about_axle],
        ]
    )
    gravity_vector = np.array([0.0, body_mass * 9.81 * com_length])
    torque_vector = np.array([1.0 / wheel_radius, -1.0])
    gravity_accel = np.linalg.solve(mass_matrix, gravity_vector)
    torque_accel = np.linalg.solve(mass_matrix, torque_vector)

    # x = [p, p_dot, theta_error, theta_dot], p positive toward CAD -Y.
    A = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, gravity_accel[0], 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, gravity_accel[1], 0.0],
        ]
    )
    B = np.array([[0.0], [torque_accel[0]], [0.0], [torque_accel[1]]])

    if np.linalg.det(mass_matrix) <= 0:
        raise RuntimeError("reduced mass matrix is not positive definite")
    if not (A[3, 2] > 0 and B[1, 0] > 0 and B[3, 0] < 0):
        raise RuntimeError("TWIP gravity/input signs do not match the declared coordinates")

    reduced = {
        "coordinate_convention": {
            "cad_axes": "+X right/wheel-axis, +Y rear, +Z up",
            "forward_position_p": "axle midpoint displacement along CAD -Y, metres",
            "pitch_theta": "right-hand rotation about +X; positive is nose-down, radians",
            "theta_error": "pitch_theta - theta_eq",
            "input_u": "tau_left + tau_right, positive wheel torque about +X, N*m"
        },
        "state": ["p_m", "p_dot_m_s", "theta_error_rad", "theta_dot_rad_s"],
        "input": ["wheel_torque_sum_nm"],
        "parameters": {
            "body_mass_kg": body_mass,
            "body_com_from_axle_m": body_com.tolist(),
            "body_com_forward_m": com_forward,
            "body_com_height_m": com_height,
            "body_com_length_m": com_length,
            "body_pitch_inertia_about_com_kg_m2": body_pitch_inertia,
            "single_wheel_rotating_mass_kg": wheel_mass,
            "single_wheel_axial_inertia_kg_m2": wheel_inertia,
            "wheel_radius_m": wheel_radius,
            "track_width_m": track_width,
            "theta_eq_rad": theta_eq,
            "theta_eq_deg": math.degrees(theta_eq),
            "equivalent_translation_mass_kg": equivalent_translation_mass,
            "body_pitch_inertia_about_axle_kg_m2": body_pitch_about_axle
        },
        "continuous_time": {
            "A": A.tolist(),
            "B": B.tolist(),
            "units": {
                "A_1_2": "m/s^2 per rad",
                "A_3_2": "rad/s^2 per rad",
                "B_1_0": "m/s^2 per N*m summed wheel torque",
                "B_3_0": "rad/s^2 per N*m summed wheel torque"
            }
        },
        "assumptions": [
            "small pitch error about theta_eq",
            "symmetric straight-line motion",
            "pure rolling without slip",
            "rigid chassis and rigid wheels",
            "no caster, motor electrical dynamics, backlash, compliance or drag",
            "wheel torque sum only; yaw/differential mode excluded"
        ]
    }
    output = MODEL_DIR / "reduced_twip.json"
    output.write_text(json.dumps(reduced, indent=2), encoding="utf-8")
    print(json.dumps(reduced["parameters"], indent=2))
    print("A=")
    print(A)
    print("B=")
    print(B)


if __name__ == "__main__":
    main()
