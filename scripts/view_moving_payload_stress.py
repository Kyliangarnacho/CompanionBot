"""Interactive replay of the final moving-payload collision scenarios."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import DiscreteStateSpaceModel
from moving_payload_benchmark import (
    EXPERIMENT_CONFIG_PATH,
    DR_CONFIG_PATH,
    MODEL_DIR,
    NOMINAL_OFFLINE_PATH,
    PLANT_PARAMETERS_PATH,
    REDUCED_PATH,
    configure_payload_box,
    run_case,
    set_payload_initial_longitudinal_state,
)
from sim import MiniSegwaySim


CASES = {
    "A": {
        "label": "A_frozen_nominal_id_lqr_collision_acceptance",
        "disturbance_rejection": False,
        "description": "Frozen nominal ID-LQR",
    },
    "Q": {
        "label": "sensorized_single_q_filtered_disturbance_rejection",
        "disturbance_rejection": True,
        "description": "Fixed nominal ID-LQR plus selected single Q-filter compensation",
    },
}


def load_inputs() -> tuple[dict, dict, dict, dict]:
    raw = json.loads(EXPERIMENT_CONFIG_PATH.read_text(encoding="utf-8"))
    dr_raw = json.loads(DR_CONFIG_PATH.read_text(encoding="utf-8"))
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    plant = json.loads(PLANT_PARAMETERS_PATH.read_text(encoding="utf-8"))
    offline = json.loads(NOMINAL_OFFLINE_PATH.read_text(encoding="utf-8"))
    nominal_model = DiscreteStateSpaceModel(
        np.asarray(offline["fit"]["A_identified"], dtype=float),
        np.asarray(offline["fit"]["B_identified"], dtype=float),
    )
    common = {
        "raw": raw,
        "dr_raw": dr_raw,
        "reduced": reduced,
        "nominal_model": nominal_model,
        "nominal_gain": np.asarray(
            offline["identified_lqr"]["K_id"], dtype=float
        ),
        "state_scales": np.asarray(offline["fit"]["state_scales"], dtype=float),
        "input_scale": float(offline["fit"]["input_scale_nm"]),
        "peak": float(plant["known"]["wheel_torque_hard_peak_nm"]),
    }
    return raw, dr_raw, reduced, common


def main() -> None:
    parser = argparse.ArgumentParser(
        description="View the final 0.20 kg moving-payload collision scenario."
    )
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=1.0,
        help="1.0 is real time; 0.5 is half speed; 2.0 is double speed.",
    )
    parser.add_argument(
        "--hide-walls",
        action="store_true",
        help="Keep collision walls invisible instead of translucent cyan.",
    )
    args = parser.parse_args()
    if args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be positive")

    selected = CASES[args.case]
    raw, dr_raw, reduced, common = load_inputs()
    collision = dr_raw["collision_acceptance"]
    payload_size = np.asarray(collision["payload_full_size_m"], dtype=float)

    print(f"Preparing deterministic case {args.case}: {selected['description']}")
    result = run_case(
        label=selected["label"],
        enable_disturbance_rejection=selected["disturbance_rejection"],
        payload_full_size_m=payload_size,
        basket_friction_override=float(collision["basket_contact_friction"]),
        initial_payload_longitudinal_position_m=float(
            collision["initial_payload_longitudinal_center_m"]
        ),
        initial_payload_longitudinal_velocity_m_s=float(
            collision["initial_payload_longitudinal_velocity_m_s"]
        ),
        capture_commands=True,
        **common,
    )
    commands = result["_held_sum_commands_nm"]

    stress = raw["moving_payload_stress"]
    sim = MiniSegwaySim(MODEL_DIR / stress["model_file"])
    configure_payload_box(sim, payload_size)
    for geom_name in (
        "basket_floor",
        "basket_front_wall",
        "basket_rear_wall",
        "basket_left_wall",
        "basket_right_wall",
        "moving_payload_collision",
    ):
        sim.model.geom(geom_name).friction[0] = float(
            collision["basket_contact_friction"]
        )

    nominal_theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim.reset(
        nominal_theta_eq
        + math.radians(float(stress["initial_pitch_error_deg"]))
    )
    set_payload_initial_longitudinal_state(
        sim,
        relative_position_m=float(collision["initial_payload_longitudinal_center_m"]),
        relative_velocity_m_s=float(
            collision["initial_payload_longitudinal_velocity_m_s"]
        ),
        relative_vertical_position_m=0.141 + 0.5 * float(payload_size[2]),
    )
    steps_per_update = int(raw["physics_steps_per_update"])

    if not args.hide_walls:
        for name in (
            "basket_front_wall",
            "basket_rear_wall",
            "basket_left_wall",
            "basket_right_wall",
        ):
            sim.model.geom(name).rgba[:] = [0.15, 0.65, 1.0, 0.18]

    payload = result["payload_gt"]
    print(
        f"Opening viewer: payload=0.20 kg, "
        f"size={payload_size[0]*1000:.0f}x{payload_size[1]*1000:.0f}x"
        f"{payload_size[2]*1000:.0f} mm, mu={collision['basket_contact_friction']:.3f}, "
        f"initial velocity={collision['initial_payload_longitudinal_velocity_m_s']:.2f} m/s"
    )
    print(
        f"Recorded result: collisions="
        f"{payload['force_bearing_wall_collision_episode_count']}, "
        f"payload contained={payload['remained_in_basket']}, "
        f"max wheel torque={result['max_wheel_torque_nm']:.3f} N m, "
        f"saturation={100*result['saturation_ratio']:.2f}%"
    )
    print("Translucent cyan boxes are collision walls shown for inspection.")

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        for held_sum in commands:
            if not viewer.is_running():
                break
            for _ in range(steps_per_update):
                wall_start = time.perf_counter()
                sim.step(held_sum / 2.0, held_sum / 2.0)
                viewer.sync()
                remaining = (
                    sim.physics_dt / args.realtime_factor
                    - (time.perf_counter() - wall_start)
                )
                if remaining > 0.0:
                    time.sleep(remaining)


if __name__ == "__main__":
    main()
