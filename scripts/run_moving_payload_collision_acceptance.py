"""Run the frozen final sensorized moving-payload collision acceptance once."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import DiscreteStateSpaceModel
from moving_payload_benchmark import SENSORIZED_STATE, run_case


MODEL_DIR = ROOT / "models" / "minisegway"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
DR_CONFIG_PATH = MODEL_DIR / "disturbance_rejection_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
NOMINAL_OFFLINE_PATH = MODEL_DIR / "stage2" / "results" / "full_state_identification_results.json"
RESULTS_PATH = MODEL_DIR / "stage2" / "results" / "moving_payload_timestamp_aligned_collision_results.json"


def key_metrics(run: dict) -> dict:
    payload = run["payload_gt"]
    pitch = run["pitch_against_posthoc_instantaneous_gt"]
    disturbance = run["disturbance_rejection"]
    return {
        "fell": bool(run["fell"]),
        "contained": bool(payload["remained_in_basket"]),
        "pitch_rms_deg": float(pitch["rms_deg"]),
        "pitch_peak_deg": float(pitch["peak_deg"]),
        "terminal_pitch_rms_deg": float(pitch["terminal_1s_rms_deg"]),
        "world_position_drift_m": float(run["final_position_drift_m"]),
        "effective_collision_count": int(
            payload["force_bearing_wall_collision_episode_count"]
        ),
        "payload_decay_ratio": float(
            payload["motion_decay"]["terminal_to_first_rms_ratio"]
        ),
        "max_wheel_torque_nm": float(run["max_wheel_torque_nm"]),
        "wheel_saturation_fraction": float(run["saturation_ratio"]),
        "u_dr_rms_nm": float(disturbance["u_dr_rms_nm"]),
        "u_dr_peak_nm": float(disturbance["u_dr_peak_nm"]),
        "u_dr_authority_hit_fraction": float(
            disturbance["authority_limit_hit_ratio"]
        ),
        "innovation_scaled_rms": float(disturbance["innovation_scaled_rms"]),
    }


def accepted(metrics: dict) -> bool:
    return bool(
        not metrics["fell"]
        and metrics["contained"]
        and 1 <= metrics["effective_collision_count"] <= 5
        and metrics["payload_decay_ratio"] < 0.5
        and metrics["wheel_saturation_fraction"] <= 0.01
    )


def main() -> None:
    raw = json.loads(EXPERIMENT_CONFIG_PATH.read_text(encoding="utf-8"))
    dr_raw = json.loads(DR_CONFIG_PATH.read_text(encoding="utf-8"))
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    plant = json.loads(PLANT_PARAMETERS_PATH.read_text(encoding="utf-8"))
    offline = json.loads(NOMINAL_OFFLINE_PATH.read_text(encoding="utf-8"))

    selected_cutoff_hz = float(dr_raw["q_filter"]["selected_cutoff_hz"])
    if selected_cutoff_hz != 2.0:
        raise RuntimeError(
            f"frozen final Q-filter cutoff changed: {selected_cutoff_hz} Hz"
        )
    collision = dr_raw["collision_acceptance"]
    payload_size = np.asarray(collision["payload_full_size_m"], dtype=float)
    nominal_model = DiscreteStateSpaceModel(
        np.asarray(offline["fit"]["A_identified"], dtype=float),
        np.asarray(offline["fit"]["B_identified"], dtype=float),
    )
    peak = float(plant["known"]["wheel_torque_hard_peak_nm"])

    # One production acceptance run: no sweep, legacy ablation, or repeat.
    run = run_case(
        label="sensorization_v1_final_single_q_2_hz_collision",
        raw=raw,
        dr_raw=dr_raw,
        reduced=reduced,
        nominal_model=nominal_model,
        nominal_gain=np.asarray(offline["identified_lqr"]["K_id"], dtype=float),
        state_scales=np.asarray(offline["fit"]["state_scales"], dtype=float),
        input_scale=float(offline["fit"]["input_scale_nm"]),
        peak=peak,
        enable_disturbance_rejection=True,
        payload_full_size_m=payload_size,
        basket_friction_override=float(collision["basket_contact_friction"]),
        initial_payload_longitudinal_position_m=float(
            collision["initial_payload_longitudinal_center_m"]
        ),
        initial_payload_longitudinal_velocity_m_s=float(
            collision["initial_payload_longitudinal_velocity_m_s"]
        ),
    )
    metrics = key_metrics(run)
    result = {
        "status": "PASS" if accepted(metrics) else "FAIL",
        "execution": {
            "final_scenario_run_count": 1,
            "cutoff_sweep_run": False,
            "legacy_ablation_run": False,
            "deterministic_repeat_run": False,
        },
        "frozen_configuration": {
            "controller_state_source": SENSORIZED_STATE,
            "q_filter_cutoff_hz": selected_cutoff_hz,
            "Q_diag": offline["identified_lqr"]["Q_diag_unchanged"],
            "R": offline["identified_lqr"]["R_unchanged"],
            "K_id": offline["identified_lqr"]["K_id"],
            "per_wheel_peak_nm": peak,
            "payload_full_size_m": payload_size.tolist(),
            "basket_contact_friction": collision["basket_contact_friction"],
        },
        "metrics": metrics,
        "estimator_error_at_500hz": run["estimator_error_at_500hz"],
        "imu_hardware_diagnostics": run["imu_hardware_diagnostics"],
        "controller": run["controller"],
        "disturbance_rejection": run["disturbance_rejection"],
        "payload_motion_decay": run["payload_gt"]["motion_decay"],
    }
    RESULTS_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({**result, "results_file": str(RESULTS_PATH)}, indent=2))


if __name__ == "__main__":
    main()
