"""Low-speed wall-impact acceptance at the selected basket friction."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import DiscreteStateSpaceModel
from scripts.run_two_timescale_disturbance_benchmark import run_case, strip_repeat_state


MODEL_DIR = ROOT / "models" / "minisegway"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
DR_CONFIG_PATH = MODEL_DIR / "disturbance_rejection_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
NOMINAL_OFFLINE_PATH = MODEL_DIR / "full_state_identification_results.json"
RESULTS_PATH = MODEL_DIR / "moving_payload_collision_acceptance_results.json"


def main() -> None:
    raw = json.loads(EXPERIMENT_CONFIG_PATH.read_text(encoding="utf-8"))
    dr_raw = json.loads(DR_CONFIG_PATH.read_text(encoding="utf-8"))
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    plant = json.loads(PLANT_PARAMETERS_PATH.read_text(encoding="utf-8"))
    offline = json.loads(NOMINAL_OFFLINE_PATH.read_text(encoding="utf-8"))
    nominal_model = DiscreteStateSpaceModel(
        np.asarray(offline["fit"]["A_identified"], dtype=float),
        np.asarray(offline["fit"]["B_identified"], dtype=float),
    )
    nominal_gain = np.asarray(offline["identified_lqr"]["K_id"], dtype=float)
    collision = dr_raw["collision_acceptance"]
    payload_size = np.asarray(collision["payload_full_size_m"], dtype=float)
    peak = float(plant["known"]["wheel_torque_hard_peak_nm"])
    common = dict(
        raw=raw,
        dr_raw=dr_raw,
        reduced=reduced,
        nominal_model=nominal_model,
        nominal_gain=nominal_gain,
        state_scales=np.asarray(offline["fit"]["state_scales"], dtype=float),
        input_scale=float(offline["fit"]["input_scale_nm"]),
        peak=peak,
        enable_probe=False,
        payload_full_size_m=payload_size,
        basket_friction_override=float(collision["basket_contact_friction"]),
        initial_payload_longitudinal_position_m=float(
            collision["initial_payload_longitudinal_center_m"]
        ),
        initial_payload_longitudinal_velocity_m_s=float(
            collision["initial_payload_longitudinal_velocity_m_s"]
        ),
    )
    a = run_case(
        label="A_frozen_nominal_id_lqr_collision_acceptance",
        enable_slow=False,
        enable_fast=False,
        **common,
    )
    b = run_case(
        label="B_fixed_lqr_plus_slow_collision_acceptance",
        enable_slow=True,
        enable_fast=False,
        **common,
    )
    c = run_case(
        label="C_fixed_lqr_plus_slow_fast_collision_acceptance",
        enable_slow=True,
        enable_fast=True,
        **common,
    )
    repeat = run_case(
        label="C_fixed_lqr_plus_slow_fast_collision_acceptance",
        enable_slow=True,
        enable_fast=True,
        **common,
    )
    deterministic = bool(
        c["final_qpos"] == repeat["final_qpos"]
        and c["final_qvel"] == repeat["final_qvel"]
        and c["history_50hz"] == repeat["history_50hz"]
    )
    collision_reproduced = bool(
        1
        <= c["payload_gt"]["force_bearing_wall_collision_episode_count"]
        <= 5
        and c["payload_gt"]["remained_in_basket"]
    )
    motion_decayed = bool(
        c["payload_gt"]["motion_decay"]["terminal_to_first_rms_ratio"] < 0.5
    )
    actuator_headroom_pass = bool(c["saturation_ratio"] <= 0.01)
    control_performance_pass = bool(
        not c["fell"] and motion_decayed and actuator_headroom_pass
    )
    result = {
        "status": "PASS"
        if deterministic and collision_reproduced and control_performance_pass
        else "FAIL",
        "acceptance": {
            "C_one_to_five_nonzero_force_collisions_with_containment": collision_reproduced,
            "C_no_fall": not c["fell"],
            "C_payload_motion_decayed": motion_decayed,
            "C_actuator_headroom_pass": actuator_headroom_pass,
            "C_control_performance_pass": control_performance_pass,
            "C_deterministic_repeat_exact": deterministic,
        },
        "scenario": {
            "payload_mass_kg": raw["moving_payload_stress"]["payload_mass_kg"],
            "payload_full_size_m": payload_size.tolist(),
            "payload_size_scale_from_original": 0.8,
            "basket_contact_friction": collision["basket_contact_friction"],
            "initial_payload_longitudinal_center_m": collision[
                "initial_payload_longitudinal_center_m"
            ],
            "initial_payload_longitudinal_velocity_m_s": collision[
                "initial_payload_longitudinal_velocity_m_s"
            ],
            "probe_sum_torque_nm": 0.0,
            "Q_diag_unchanged": offline["identified_lqr"]["Q_diag_unchanged"],
            "R_unchanged": offline["identified_lqr"]["R_unchanged"],
            "per_wheel_peak_nm_unchanged": peak,
        },
        "A_frozen": strip_repeat_state(a),
        "B_slow": strip_repeat_state(b),
        "C_slow_fast": strip_repeat_state(c),
    }
    RESULTS_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    concise = {
        "status": result["status"],
        "acceptance": result["acceptance"],
        "scenario": result["scenario"],
        "cases": {},
    }
    for key in ("A_frozen", "B_slow", "C_slow_fast"):
        run = result[key]
        concise["cases"][key] = {
            "fell": run["fell"],
            "pitch_nominal_reference": run["pitch_nominal_reference"],
            "pitch_against_posthoc_instantaneous_gt": run[
                "pitch_against_posthoc_instantaneous_gt"
            ],
            "final_position_drift_m": run["final_position_drift_m"],
            "max_wheel_torque_nm": run["max_wheel_torque_nm"],
            "saturation_ratio": run["saturation_ratio"],
            "payload_gt": run["payload_gt"],
            "disturbance_rejection": run["disturbance_rejection"],
        }
    print(json.dumps(concise, indent=2))


if __name__ == "__main__":
    main()
