"""Optimize Stage 3A case D with a dynamically projected full-state reference."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import (
    JerkLimitedLongitudinalReferenceGenerator,
    NominalDiscreteFeedforward,
    project_hidden_reference_and_input,
)
from sim import LongitudinalEstimator, load_longitudinal_estimator_config
from moving_payload_benchmark import has_chassis_floor_contact, held_initial_imu
from run_stage3a_commanded_motion import (
    build_sim,
    command_at,
    metric_triplet,
    rms,
    transition_metrics,
)


MODEL_DIR = ROOT / "models" / "minisegway"
CONFIG_PATH = MODEL_DIR / "stage3" / "config" / "stage3a_dynamic_nominal_reference_config.json"
MOTION_CONFIG_PATH = MODEL_DIR / "stage3" / "config" / "stage3a_commanded_motion_config.json"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
DR_CONFIG_PATH = MODEL_DIR / "disturbance_rejection_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
OFFLINE_PATH = MODEL_DIR / "stage2" / "results" / "full_state_identification_results.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
OLD_RESULTS_PATH = MODEL_DIR / "stage3" / "results" / "history" / "stage3a_reference_feedforward_ablation_results.json"
RESULTS_PATH = MODEL_DIR / "stage3" / "results" / "history" / "stage3a_dynamic_nominal_reference_results.json"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def generate_s_curve_profile(
    scenario: dict, config: dict, motion_config: dict
) -> np.ndarray:
    limits = motion_config["reference_limits"]
    generator = JerkLimitedLongitudinalReferenceGenerator(
        float(motion_config["controller_dt_s"]),
        float(limits["max_velocity_m_s"]),
        float(limits["max_acceleration_m_s2"]),
        float(config["max_jerk_m_s3"]),
    )
    dt_s = float(motion_config["controller_dt_s"])
    interval_count = round(float(scenario["duration_s"]) / dt_s)
    references = [generator.reference]
    for interval in range(interval_count):
        command = command_at(scenario["schedule"], interval * dt_s)
        if scenario["mode"] == "position":
            reference = generator.step_position(command)
        else:
            reference = generator.step_velocity(command)
        references.append(reference)
    if generator.plan_active:
        raise RuntimeError("S-curve did not complete within the frozen scenario duration")
    return np.asarray(
        [
            [item.position_m, item.velocity_m_s, item.acceleration_m_s2]
            for item in references
        ]
    )


def projection_segments(scenario: dict, dt_s: float, interval_count: int):
    starts = [round(float(event["time_s"]) / dt_s) for event in scenario["schedule"][1:]]
    ends = [*starts[1:], interval_count]
    return list(zip(starts, ends))


def history_decomposition(history: list[dict]) -> dict:
    velocity = np.asarray([item["v_ref_m_s"] for item in history])
    acceleration = np.asarray([item["a_ref_m_s2"] for item in history])
    u_ff = np.asarray([item["u_ff_applied_sum_nm"] for item in history])
    u_fb = np.asarray([item["u_fb_sum_nm"] for item in history])
    u_total = np.asarray([item["u_requested_sum_nm"] for item in history])
    nonzero_velocity = np.abs(velocity) > 1e-6
    constant_velocity = nonzero_velocity & (np.abs(acceleration) <= 1e-9)
    active_both = (np.abs(u_ff) > 1e-6) & (np.abs(u_fb) > 1e-6)
    denominator = float(np.dot(velocity, velocity))
    return {
        "sample_rate_hz": 50.0,
        "u_ff_vs_v_ref_slope_nm_per_m_s": (
            float(np.dot(velocity, u_ff) / denominator) if denominator > 0.0 else 0.0
        ),
        "u_ff_v_ref_correlation": (
            float(np.corrcoef(velocity, u_ff)[0, 1])
            if np.std(velocity) > 0.0 and np.std(u_ff) > 0.0
            else 0.0
        ),
        "constant_velocity_u_ff_rms_nm": (
            rms(u_ff[constant_velocity]) if np.any(constant_velocity) else 0.0
        ),
        "opposing_u_ff_u_fb_fraction": (
            float(np.mean(u_ff[active_both] * u_fb[active_both] < 0.0))
            if np.any(active_both)
            else 0.0
        ),
        "cancellation_ratio": rms(u_total) / max(rms(u_ff) + rms(u_fb), 1e-12),
    }


def run_case(
    scenario: dict,
    config: dict,
    motion_config: dict,
    experiment: dict,
    dr_raw: dict,
    reduced: dict,
    offline: dict,
    plant: dict,
) -> dict:
    dt_s = float(motion_config["controller_dt_s"])
    interval_count = round(float(scenario["duration_s"]) / dt_s)
    profile = generate_s_curve_profile(scenario, config, motion_config)
    fit = offline["fit"]
    A = np.asarray(fit["A_identified"], dtype=float)
    B = np.asarray(fit["B_identified"], dtype=float)
    state_scales = np.asarray(fit["state_scales"], dtype=float)
    segments = projection_segments(scenario, dt_s, interval_count)
    projection = project_hidden_reference_and_input(
        A,
        B,
        state_scales,
        float(fit["input_scale_nm"]),
        profile[:, :2],
        segments,
    )
    feedforward = NominalDiscreteFeedforward(A, B, state_scales)
    closed_form_u_ff = np.empty(interval_count)
    closed_form_residuals = np.empty((interval_count, 4))
    for interval in range(interval_count):
        command = feedforward.command(
            projection.reference_states[interval],
            projection.reference_states[interval + 1],
        )
        closed_form_u_ff[interval] = command.torque_nm
        closed_form_residuals[interval] = command.residual
    maximum_projection_input_difference = float(
        np.max(np.abs(closed_form_u_ff - projection.inputs_nm))
    )
    if maximum_projection_input_difference > 1e-8:
        raise RuntimeError("joint projection and scalar feedforward disagree")

    nominal_theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim, _, _ = build_sim(
        "empty", int(motion_config["imu_rng_seed"]), nominal_theta_eq, dr_raw
    )
    physics_steps = int(experiment["physics_steps_per_update"])
    if not math.isclose(sim.physics_dt, float(motion_config["physics_dt_s"]), abs_tol=1e-12):
        raise RuntimeError("physics timing differs from frozen Stage 3A baseline")
    if not math.isclose(physics_steps * sim.physics_dt, dt_s, abs_tol=1e-12):
        raise RuntimeError("controller timing differs from frozen Stage 3A baseline")
    estimator = LongitudinalEstimator(
        load_longitudinal_estimator_config(),
        sim.encoder_profile,
        float(reduced["parameters"]["wheel_radius_m"]),
    )
    accel, gyro, measurement_age_s, extrapolation_allowed = held_initial_imu(sim)
    estimate = estimator.reset(
        accel,
        gyro,
        sim.wheel_encoder_counts(),
        measurement_age_s=measurement_age_s,
        allow_kinematic_extrapolation=extrapolation_allowed,
    )
    K4 = np.asarray(offline["identified_lqr"]["K_id"], dtype=float)
    peak_per_wheel = float(plant["known"]["wheel_torque_hard_peak_nm"])
    history_period = round(
        1.0 / (dt_s * float(motion_config["history_frequency_hz"]))
    )
    logs = {
        name: []
        for name in (
            "time",
            "reference",
            "plant",
            "gt",
            "u_fb",
            "u_ff",
            "requested",
            "held",
            "actual",
        )
    }
    history = []
    fallen = False
    saturated_updates = 0

    for interval in range(interval_count):
        user_command = command_at(scenario["schedule"], float(sim.data.time))
        reference_state = projection.reference_states[interval]
        x_plant = estimate.plant_state(nominal_theta_eq)
        tracking_error = x_plant - reference_state
        u_fb = -float((K4 @ tracking_error).item())
        u_ff = float(closed_form_u_ff[interval])
        requested_sum = u_ff + u_fb
        held_sum = float(
            np.clip(requested_sum, -2.0 * peak_per_wheel, 2.0 * peak_per_wheel)
        )
        saturated_updates += int(
            not np.isclose(requested_sum, held_sum, atol=1e-12, rtol=0.0)
        )
        actual_samples = []
        next_gt = None
        for _ in range(physics_steps):
            snapshot = sim.step(held_sum / 2.0, held_sum / 2.0)
            next_gt = sim.longitudinal_state(nominal_theta_eq)
            actual_samples.append(float(np.sum(snapshot.applied_ctrl_nm)))
            fallen |= abs(next_gt[2]) >= math.radians(
                float(experiment["benchmark"]["fall_pitch_error_deg"])
            )
            fallen |= has_chassis_floor_contact(sim)
        assert next_gt is not None
        actual_sum = float(np.mean(actual_samples))
        accel, gyro, measurement_age_s, extrapolation_allowed = sim.imu_estimator_input()
        estimate = estimator.update(
            accel,
            gyro,
            sim.wheel_encoder_counts(),
            measurement_age_s=measurement_age_s,
            allow_kinematic_extrapolation=extrapolation_allowed,
        )
        next_plant = estimate.plant_state(nominal_theta_eq)
        next_profile = profile[interval + 1]
        next_reference_state = projection.reference_states[interval + 1]
        time_s = float(sim.data.time)
        logs["time"].append(time_s)
        logs["reference"].append(next_profile)
        logs["plant"].append(next_plant)
        logs["gt"].append(next_gt)
        logs["u_fb"].append(u_fb)
        logs["u_ff"].append(u_ff)
        logs["requested"].append(requested_sum)
        logs["held"].append(held_sum)
        logs["actual"].append(actual_sum)
        if interval % history_period == 0:
            history.append(
                {
                    "time_s": time_s,
                    "user_command": user_command,
                    "p_ref_m": float(next_profile[0]),
                    "p_hat_m": float(next_plant[0]),
                    "v_ref_m_s": float(next_profile[1]),
                    "v_hat_m_s": float(next_plant[1]),
                    "a_ref_m_s2": float(next_profile[2]),
                    "theta_ref_deg": math.degrees(float(next_reference_state[2])),
                    "theta_dot_ref_deg_s": math.degrees(float(next_reference_state[3])),
                    "pitch_hat_error_deg": math.degrees(float(next_plant[2])),
                    "gt_pitch_error_deg_posthoc": math.degrees(float(next_gt[2])),
                    "u_fb_sum_nm": u_fb,
                    "u_ff_applied_sum_nm": u_ff,
                    "u_requested_sum_nm": requested_sum,
                    "u_software_limited_sum_nm": held_sum,
                    "u_actual_applied_sum_nm": actual_sum,
                }
            )

    arrays = {name: np.asarray(values) for name, values in logs.items()}
    terminal = arrays["time"] >= float(scenario["duration_s"]) - float(
        motion_config["terminal_window_s"]
    )
    position_error = arrays["plant"][:, 0] - arrays["reference"][:, 0]
    velocity_error = arrays["plant"][:, 1] - arrays["reference"][:, 1]
    transients = transition_metrics(
        scenario,
        arrays["time"],
        arrays["reference"],
        arrays["plant"],
        motion_config["settling"],
        dt_s,
    )
    overshoots = [item["overshoot"] for item in transients if item["overshoot"] is not None]
    normalized_residual = closed_form_residuals / state_scales
    theta_reference = projection.reference_states[1:, 2]
    theta_dot_reference = projection.reference_states[1:, 3]
    active_acceleration = np.abs(profile[1:, 2]) > 1e-9
    lean_acceleration_correlation = (
        float(np.corrcoef(theta_reference[active_acceleration], profile[1:, 2][active_acceleration])[0, 1])
        if np.count_nonzero(active_acceleration) > 1
        else 0.0
    )
    result = {
        "scenario": scenario["name"],
        "mode": scenario["mode"],
        "case": config["new_case"],
        "fell": bool(fallen),
        "tracking": {
            "position_error_m": metric_triplet(position_error),
            "velocity_error_m_s": metric_triplet(velocity_error),
            "endpoint_position_error_m": float(position_error[-1]),
            "endpoint_velocity_error_m_s": float(velocity_error[-1]),
            "maximum_position_overshoot_m": float(max(overshoots, default=0.0)) if scenario["mode"] == "position" else None,
            "settled_transition_count": sum(item["settled_time_s"] is not None for item in transients),
            "transition_count": len(transients),
            "transients": transients,
        },
        "pitch": {
            "rms_deg": math.degrees(rms(arrays["gt"][:, 2])),
            "peak_abs_deg": math.degrees(float(np.max(np.abs(arrays["gt"][:, 2])))),
            "terminal_rms_deg": math.degrees(rms(arrays["gt"][terminal, 2])),
        },
        "nominal_lean_reference": {
            "theta_ref_rms_deg": math.degrees(rms(theta_reference)),
            "theta_ref_peak_deg": math.degrees(float(np.max(np.abs(theta_reference)))),
            "theta_dot_ref_rms_deg_s": math.degrees(rms(theta_dot_reference)),
            "theta_dot_ref_peak_deg_s": math.degrees(float(np.max(np.abs(theta_dot_reference)))),
            "theta_ref_acceleration_correlation": lean_acceleration_correlation,
        },
        "torque": {
            "feedback_sum_nm": metric_triplet(arrays["u_fb"]),
            "feedforward_sum_nm": metric_triplet(arrays["u_ff"]),
            "requested_sum_nm": metric_triplet(arrays["requested"]),
            "actual_applied_sum_nm": metric_triplet(arrays["actual"]),
            "wheel_saturation_fraction": saturated_updates / interval_count,
            "cancellation_ratio": rms(arrays["requested"]) / max(rms(arrays["u_ff"]) + rms(arrays["u_fb"]), 1e-12),
        },
        "feedforward_feasibility": {
            "normalized_residual_rms": float(np.sqrt(np.mean(normalized_residual**2))),
            "normalized_residual_step_peak": float(np.max(np.sqrt(np.mean(normalized_residual**2, axis=1)))),
            "projection_vs_closed_form_u_ff_peak_difference_nm": maximum_projection_input_difference,
            "segment_diagnostics": list(projection.segment_diagnostics),
        },
        "disturbance_rejection_enabled": False,
        "history_50hz": history,
    }
    result["feedforward_decomposition_50hz"] = history_decomposition(history)
    return result


def compact_summary(run: dict) -> dict:
    tracking = run["tracking"]
    return {
        "scenario": run["scenario"],
        "position_rmse_m": tracking["position_error_m"]["rms"],
        "position_peak_m": tracking["position_error_m"]["peak_abs"],
        "velocity_rmse_m_s": tracking["velocity_error_m_s"]["rms"],
        "velocity_peak_m_s": tracking["velocity_error_m_s"]["peak_abs"],
        "position_overshoot_m": tracking["maximum_position_overshoot_m"],
        "settled_transitions": f'{tracking["settled_transition_count"]}/{tracking["transition_count"]}',
        "pitch_rms_deg": run["pitch"]["rms_deg"],
        "pitch_peak_deg": run["pitch"]["peak_abs_deg"],
        "u_ff_rms_nm": run["torque"]["feedforward_sum_nm"]["rms"],
        "u_ff_peak_nm": run["torque"]["feedforward_sum_nm"]["peak_abs"],
        "u_fb_rms_nm": run["torque"]["feedback_sum_nm"]["rms"],
        "u_fb_peak_nm": run["torque"]["feedback_sum_nm"]["peak_abs"],
        "requested_rms_nm": run["torque"]["requested_sum_nm"]["rms"],
        "applied_peak_nm": run["torque"]["actual_applied_sum_nm"]["peak_abs"],
        "wheel_saturation_fraction": run["torque"]["wheel_saturation_fraction"],
        "ff_normalized_residual_rms": run["feedforward_feasibility"]["normalized_residual_rms"],
        "theta_ref_rms_peak_deg": [run["nominal_lean_reference"]["theta_ref_rms_deg"], run["nominal_lean_reference"]["theta_ref_peak_deg"]],
        "theta_dot_ref_rms_peak_deg_s": [run["nominal_lean_reference"]["theta_dot_ref_rms_deg_s"], run["nominal_lean_reference"]["theta_dot_ref_peak_deg_s"]],
    }


def old_d_summary(run: dict) -> dict:
    tracking = run["tracking"]
    return {
        "scenario": run["scenario"],
        "position_rmse_m": tracking["position_error_m"]["rms"],
        "position_peak_m": tracking["position_error_m"]["peak_abs"],
        "velocity_rmse_m_s": tracking["velocity_error_m_s"]["rms"],
        "velocity_peak_m_s": tracking["velocity_error_m_s"]["peak_abs"],
        "position_overshoot_m": tracking["maximum_position_overshoot_m"],
        "settled_transitions": f'{tracking["settled_transition_count"]}/{tracking["transition_count"]}',
        "pitch_rms_deg": run["pitch"]["rms_deg"],
        "pitch_peak_deg": run["pitch"]["peak_abs_deg"],
        "u_ff_rms_nm": run["feedforward"]["applied_sum_nm"]["rms"],
        "u_ff_peak_nm": run["feedforward"]["applied_sum_nm"]["peak_abs"],
        "u_fb_rms_nm": run["torque"]["feedback_sum_nm"]["rms"],
        "u_fb_peak_nm": run["torque"]["feedback_sum_nm"]["peak_abs"],
        "requested_rms_nm": run["torque"]["requested_sum_nm"]["rms"],
        "applied_peak_nm": run["torque"]["actual_applied_sum_nm"]["peak_abs"],
        "wheel_saturation_fraction": run["torque"]["wheel_saturation_fraction"],
        "ff_normalized_residual_rms": run["feedforward"]["normalized_feasibility_residual_rms"],
    }


def main() -> None:
    config = load_json(CONFIG_PATH)
    motion_config = load_json(MOTION_CONFIG_PATH)
    old_results = load_json(OLD_RESULTS_PATH) if OLD_RESULTS_PATH.exists() else None
    scenario_by_name = {item["name"]: item for item in motion_config["scenarios"]}
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=config["comparison_scenarios"])
    parser.add_argument("--no-write", action="store_true")
    args = parser.parse_args()
    experiment = load_json(EXPERIMENT_CONFIG_PATH)
    dr_raw = load_json(DR_CONFIG_PATH)
    reduced = load_json(REDUCED_PATH)
    offline = load_json(OFFLINE_PATH)
    plant = load_json(PLANT_PARAMETERS_PATH)
    scenario_names = [args.scenario] if args.scenario else config["comparison_scenarios"]
    runs = []
    for scenario_name in scenario_names:
        print(f"RUN empty/D-dynamic-reference/{scenario_name}", flush=True)
        run = run_case(scenario_by_name[scenario_name], config, motion_config, experiment, dr_raw, reduced, offline, plant)
        runs.append(run)
        summary = compact_summary(run)
        print(f"  p_rmse={summary['position_rmse_m']:.4f} m v_rmse={summary['velocity_rmse_m_s']:.4f} m/s pitch_peak={summary['pitch_peak_deg']:.2f} deg u_ff_rms={summary['u_ff_rms_nm']:.4f} N m sat={summary['wheel_saturation_fraction']:.4f}", flush=True)

    old_by_scenario = (
        {run["scenario"]: run for run in old_results["runs"] if run["case"] == "D"}
        if old_results is not None else {}
    )
    comparisons = []
    for run in runs:
        old_run = old_by_scenario.get(run["scenario"])
        if old_run is None:
            continue
        old_decomposition = history_decomposition(old_run["history_50hz"])
        old_decomposition["cancellation_ratio_full_rate_metrics"] = old_run["torque"]["requested_sum_nm"]["rms"] / max(old_run["feedforward"]["applied_sum_nm"]["rms"] + old_run["torque"]["feedback_sum_nm"]["rms"], 1e-12)
        comparisons.append({
            "scenario": run["scenario"],
            "old_D": old_d_summary(old_run),
            "new_D": compact_summary(run),
            "old_D_feedforward_decomposition": old_decomposition,
            "new_D_feedforward_decomposition": run["feedforward_decomposition_50hz"],
        })
    result = {
        "stage": config["stage"],
        "status": "COMPLETE",
        "construction": config["construction"],
        "frozen_invariants": {
            "K4": offline["identified_lqr"]["K_id"],
            "s_curve_unchanged": True,
            "estimator_unchanged": True,
            "pll_unchanged": True,
            "imu_timing_unchanged": True,
            "wheel_torque_limit_nm": plant["known"]["wheel_torque_hard_peak_nm"],
            "disturbance_rejection_enabled": False,
        },
        "run_summary": [compact_summary(run) for run in runs],
        "comparison_to_old_D": comparisons,
        "runs": runs,
    }
    if not args.no_write and not args.scenario:
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        RESULTS_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"WROTE {RESULTS_PATH}", flush=True)
    print(json.dumps({"status": result["status"], "run_count": len(runs), "run_summary": result["run_summary"]}, indent=2))


if __name__ == "__main__":
    main()
