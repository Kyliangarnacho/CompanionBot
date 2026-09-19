"""Stage 3A flat-ground commanded longitudinal-motion benchmark."""

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
    DiscreteStateSpaceModel,
    FilteredDisturbanceCompensator,
    LongitudinalReferenceGenerator,
    disturbance_rejection_config_from_dict,
)
from sim import (
    DynamicPayload,
    LongitudinalEstimator,
    MiniSegwaySim,
    load_longitudinal_estimator_config,
    load_payload_config,
)
from moving_payload_benchmark import (
    PayloadGroundTruthRecorder,
    configure_payload_box,
    force_bearing_contact_episodes,
    has_chassis_floor_contact,
    held_initial_imu,
    set_payload_initial_longitudinal_state,
)


MODEL_DIR = ROOT / "models" / "minisegway"
CONFIG_PATH = MODEL_DIR / "stage3" / "config" / "stage3a_commanded_motion_config.json"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
DR_CONFIG_PATH = MODEL_DIR / "disturbance_rejection_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
OFFLINE_PATH = MODEL_DIR / "stage2" / "results" / "full_state_identification_results.json"
RIGID_PAYLOAD_PATH = MODEL_DIR / "dynamic_payload_config.json"
RESULTS_PATH = MODEL_DIR / "stage3" / "results" / "history" / "stage3a_commanded_motion_results.json"
EMPTY_MODEL_PATH = MODEL_DIR / "mini_segway.xml"
FREE_MODEL_PATH = MODEL_DIR / "mini_segway_moving_payload.xml"
PAYLOAD_MODES = ("empty", "fixed", "free")
CONTROL_CASES = ("A", "Q")


def rms(values: np.ndarray) -> float:
    array = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(array * array)))


def wrap(angle_rad: float | np.ndarray) -> float | np.ndarray:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def command_at(schedule: list[dict], time_s: float) -> float:
    command = float(schedule[0]["command"])
    for event in schedule[1:]:
        if time_s + 1e-12 < float(event["time_s"]):
            break
        command = float(event["command"])
    return command


def first_dwell_time(
    times: np.ndarray,
    good: np.ndarray,
    start_s: float,
    end_s: float,
    dwell_s: float,
    dt_s: float,
) -> float | None:
    candidates = np.flatnonzero((times >= start_s) & (times < end_s))
    dwell_samples = max(1, round(dwell_s / dt_s))
    for index in candidates:
        if index + dwell_samples > len(good):
            break
        if times[index + dwell_samples - 1] >= end_s:
            break
        if np.all(good[index : index + dwell_samples]):
            return float(times[index])
    return None


def transition_metrics(
    scenario: dict,
    times: np.ndarray,
    references: np.ndarray,
    plant_states: np.ndarray,
    settling: dict,
    dt_s: float,
) -> list[dict]:
    results: list[dict] = []
    schedule = scenario["schedule"]
    position_error = plant_states[:, 0] - references[:, 0]
    velocity_error = plant_states[:, 1] - references[:, 1]
    good = (
        (np.abs(position_error) <= float(settling["position_error_band_m"]))
        & (np.abs(velocity_error) <= float(settling["velocity_error_band_m_s"]))
    )
    for event_index, event in enumerate(schedule[1:], start=1):
        event_time = float(event["time_s"])
        segment_end = (
            float(schedule[event_index + 1]["time_s"])
            if event_index + 1 < len(schedule)
            else float(scenario["duration_s"])
        )
        target = float(event["command"])
        prior = float(schedule[event_index - 1]["command"])
        segment = np.flatnonzero((times >= event_time) & (times < segment_end))
        if len(segment) == 0:
            continue
        if scenario["mode"] == "position":
            reached_mask = (
                (np.abs(references[:, 0] - target) <= 1e-9)
                & (np.abs(references[:, 1]) <= 1e-9)
                & (times >= event_time)
                & (times < segment_end)
            )
            tracking_signal = plant_states[:, 0]
        else:
            reached_mask = (
                (np.abs(references[:, 1] - target) <= 1e-9)
                & (times >= event_time)
                & (times < segment_end)
            )
            tracking_signal = plant_states[:, 1]
        reached_indices = np.flatnonzero(reached_mask)
        reached_time = (
            float(times[reached_indices[0]]) if len(reached_indices) else None
        )
        settle_start = reached_time if reached_time is not None else segment_end
        settled_time = first_dwell_time(
            times,
            good,
            settle_start,
            segment_end,
            float(settling["dwell_s"]),
            dt_s,
        )
        endpoint_index = int(segment[-1])
        overshoot = None
        direction = math.copysign(1.0, target - prior) if target != prior else 0.0
        if reached_time is not None and direction != 0.0:
            after_reached = (
                (times >= reached_time) & (times < segment_end)
            )
            overshoot = float(
                max(0.0, np.max(direction * (tracking_signal[after_reached] - target)))
            )
        results.append(
            {
                "command_time_s": event_time,
                "segment_end_s": segment_end,
                "user_command": target,
                "shaped_reference_reached_time_s": reached_time,
                "settled_time_s": settled_time,
                "settling_after_command_s": (
                    None if settled_time is None else settled_time - event_time
                ),
                "settling_after_shaped_reference_s": (
                    None
                    if settled_time is None or reached_time is None
                    else settled_time - reached_time
                ),
                "overshoot": overshoot,
                "endpoint_position_error_m": float(position_error[endpoint_index]),
                "endpoint_velocity_error_m_s": float(velocity_error[endpoint_index]),
            }
        )
    return results


def acceleration_window_metrics(
    reference_velocity: np.ndarray,
    reference_acceleration: np.ndarray,
    theta_acc_error: np.ndarray,
    theta_hat_error: np.ndarray,
    theta_gt_error: np.ndarray,
) -> dict:
    previous_speed = np.r_[0.0, np.abs(reference_velocity[:-1])]
    speed_delta = np.abs(reference_velocity) - previous_speed
    active = np.abs(reference_acceleration) > 1e-9
    masks = {
        "accelerating_speed": active & (speed_delta > 1e-12),
        "decelerating_speed": active & (speed_delta < -1e-12),
    }
    result = {}
    for name, mask in masks.items():
        if not np.any(mask):
            result[name] = {"sample_count": 0}
            continue
        scale = 180.0 / math.pi
        result[name] = {
            "sample_count": int(np.count_nonzero(mask)),
            "theta_acc_mean_deg": float(scale * np.mean(theta_acc_error[mask])),
            "theta_acc_rms_deg": float(scale * rms(theta_acc_error[mask])),
            "fused_theta_hat_mean_deg": float(scale * np.mean(theta_hat_error[mask])),
            "fused_theta_hat_rms_deg": float(scale * rms(theta_hat_error[mask])),
            "gt_pitch_mean_deg": float(scale * np.mean(theta_gt_error[mask])),
            "gt_pitch_rms_deg": float(scale * rms(theta_gt_error[mask])),
            "theta_acc_minus_gt_mean_deg": float(
                scale * np.mean(wrap(theta_acc_error[mask] - theta_gt_error[mask]))
            ),
            "theta_acc_minus_gt_rms_deg": float(
                scale * rms(wrap(theta_acc_error[mask] - theta_gt_error[mask]))
            ),
            "fused_minus_gt_mean_deg": float(
                scale * np.mean(wrap(theta_hat_error[mask] - theta_gt_error[mask]))
            ),
            "fused_minus_gt_rms_deg": float(
                scale * rms(wrap(theta_hat_error[mask] - theta_gt_error[mask]))
            ),
        }
    return result


def metric_triplet(values: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(values)),
        "rms": rms(values),
        "peak_abs": float(np.max(np.abs(values))),
    }


def estimator_metrics(errors: np.ndarray) -> dict:
    scales = np.asarray([1.0, 1.0, 180.0 / math.pi, 180.0 / math.pi])
    names = ("position_m", "velocity_m_s", "pitch_deg", "pitch_rate_deg_s")
    return {
        name: {
            "rms": rms(errors[:, index] * scales[index]),
            "peak": float(np.max(np.abs(errors[:, index] * scales[index]))),
            "final": float(errors[-1, index] * scales[index]),
        }
        for index, name in enumerate(names)
    }


def evaluate_case_gate(result: dict, gate: dict, payload_mode: str) -> tuple[bool, list[str]]:
    failures = []
    tracking = result["tracking"]
    pitch = result["pitch"]
    if result["fell"]:
        failures.append("fell")
    checks = (
        (tracking["position_error_m"]["rms"], gate["maximum_position_tracking_rmse_m"], "position_rmse"),
        (tracking["velocity_error_m_s"]["rms"], gate["maximum_velocity_tracking_rmse_m_s"], "velocity_rmse"),
        (abs(tracking["endpoint_position_error_m"]), gate["maximum_final_position_error_m"], "final_position_error"),
        (abs(tracking["endpoint_velocity_error_m_s"]), gate["maximum_final_velocity_error_m_s"], "final_velocity_error"),
        (pitch["peak_abs_deg"], gate["maximum_pitch_peak_deg"], "pitch_peak"),
        (pitch["terminal_rms_deg"], gate["maximum_terminal_pitch_rms_deg"], "terminal_pitch_rms"),
        (result["torque"]["wheel_saturation_fraction"], gate["maximum_wheel_saturation_fraction"], "wheel_saturation"),
    )
    failures.extend(name for value, limit, name in checks if value > limit)
    if gate.get("require_all_transitions_settled", False):
        if any(item["settled_time_s"] is None for item in tracking["transients"]):
            failures.append("unsettled_transition")
    payload = result.get("free_payload_posthoc")
    if payload_mode == "free" and payload is not None:
        if gate.get("require_free_payload_containment", False) and not payload["remained_in_basket"]:
            failures.append("payload_containment")
        decay_limit = gate.get("maximum_free_payload_terminal_to_first_speed_rms_ratio")
        if decay_limit is not None and payload["motion_decay"]["terminal_to_first_speed_rms_ratio"] > decay_limit:
            failures.append("payload_motion_decay")
    return not failures, failures


def build_sim(payload_mode: str, seed: int, nominal_theta_eq: float, dr_raw: dict):
    model_path = FREE_MODEL_PATH if payload_mode == "free" else EMPTY_MODEL_PATH
    sim = MiniSegwaySim(model_path, imu_seed=seed)
    rigid_payload = None
    if payload_mode == "fixed":
        rigid_payload = DynamicPayload(
            sim.model,
            sim.data,
            load_payload_config(RIGID_PAYLOAD_PATH),
        )
    sim.reset(nominal_theta_eq, imu_seed=seed)
    if rigid_payload is not None:
        rigid_payload.apply()
    sim.calibrate_imu_stationary()

    payload_setup = None
    payload_recorder = None
    if payload_mode == "free":
        acceptance = dr_raw["collision_acceptance"]
        full_size = np.asarray(acceptance["payload_full_size_m"], dtype=float)
        payload_box = configure_payload_box(sim, full_size)
        for geom_name in (
            "basket_floor",
            "basket_front_wall",
            "basket_rear_wall",
            "basket_left_wall",
            "basket_right_wall",
            "moving_payload_collision",
        ):
            sim.model.geom(geom_name).friction[0] = float(
                acceptance["basket_contact_friction"]
            )
        initial = set_payload_initial_longitudinal_state(
            sim,
            relative_position_m=0.0,
            relative_velocity_m_s=0.0,
            relative_vertical_position_m=0.141 + 0.5 * full_size[2],
        )
        payload_setup = {
            "box": payload_box,
            "basket_contact_friction": float(acceptance["basket_contact_friction"]),
            "initial_state": initial,
        }
        payload_recorder = PayloadGroundTruthRecorder(sim)
    elif rigid_payload is not None:
        payload_setup = {
            "source": str(RIGID_PAYLOAD_PATH.relative_to(ROOT)),
            "mass_kg": rigid_payload.config.mass_kg,
            "position_body_m": rigid_payload.config.position_body_m.tolist(),
            "full_size_m": rigid_payload.config.full_size_m.tolist(),
            "applied_before_run": True,
        }
    return sim, payload_setup, payload_recorder


def run_case(
    scenario: dict,
    payload_mode: str,
    control_case: str,
    config: dict,
    experiment: dict,
    dr_raw: dict,
    reduced: dict,
    offline: dict,
    plant: dict,
) -> dict:
    dt_s = float(config["controller_dt_s"])
    physics_steps = int(experiment["physics_steps_per_update"])
    nominal_theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim, payload_setup, payload_recorder = build_sim(
        payload_mode,
        int(config["imu_rng_seed"]),
        nominal_theta_eq,
        dr_raw,
    )
    if not math.isclose(sim.physics_dt, float(config["physics_dt_s"]), abs_tol=1e-12):
        raise RuntimeError("Stage 3A physics timing differs from frozen baseline")
    if not math.isclose(physics_steps * sim.physics_dt, dt_s, abs_tol=1e-12):
        raise RuntimeError("Stage 3A controller timing differs from frozen baseline")

    estimator_config = load_longitudinal_estimator_config()
    estimator = LongitudinalEstimator(
        estimator_config,
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
    fit = offline["fit"]
    nominal_model = DiscreteStateSpaceModel(
        A=np.asarray(fit["A_identified"], dtype=float),
        B=np.asarray(fit["B_identified"], dtype=float),
    )
    nominal_gain = np.asarray(offline["identified_lqr"]["K_id"], dtype=float)
    compensator = FilteredDisturbanceCompensator(
        nominal_model,
        np.asarray(fit["state_scales"], dtype=float),
        float(fit["input_scale_nm"]),
        disturbance_rejection_config_from_dict(dr_raw),
    )
    limits = config["reference_limits"]
    reference_generator = LongitudinalReferenceGenerator(
        dt_s,
        float(limits["max_velocity_m_s"]),
        float(limits["max_acceleration_m_s2"]),
    )
    peak = float(plant["known"]["wheel_torque_hard_peak_nm"])
    interval_count = round(float(scenario["duration_s"]) / dt_s)
    history_period = round(1.0 / (dt_s * float(config["history_frequency_hz"])))

    logs = {name: [] for name in (
        "time", "command", "reference", "plant", "tracking", "gt", "estimator_error",
        "theta_acc", "requested", "held", "actual", "u_lqr", "u_dr_requested",
        "u_dr", "innovation", "matched_fraction", "authority_hit",
    )}
    history = []
    payload_times = []
    payload_positions = []
    payload_contacts = []
    payload_forces = []
    fallen = False
    saturated_updates = 0

    for interval in range(interval_count):
        start_time = float(sim.data.time)
        command = command_at(scenario["schedule"], start_time)
        reference = reference_generator.reference
        x_plant = estimate.plant_state(nominal_theta_eq)
        e_track = estimate.tracking_error_state(
            nominal_theta_eq,
            reference.position_m,
            reference.velocity_m_s,
        )
        compensation = compensator.command(enabled=control_case == "Q")
        u_lqr = -float((nominal_gain @ e_track).item())
        requested_sum = u_lqr + compensation.u_dr_nm
        held_sum = float(np.clip(requested_sum, -2.0 * peak, 2.0 * peak))
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
            if payload_recorder is not None:
                rel_pos, _, contact, force = payload_recorder.sample(
                    snapshot.time_s,
                    float(next_gt[2]),
                    float(np.max(np.abs(snapshot.applied_ctrl_nm))),
                )
                payload_times.append(snapshot.time_s)
                payload_positions.append(rel_pos)
                payload_contacts.append(contact)
                payload_forces.append(force)
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
        observation = compensator.observe(x_plant, actual_sum, next_plant)
        if scenario["mode"] == "position":
            next_reference = reference_generator.step_position(command)
        else:
            next_reference = reference_generator.step_velocity(command)
        next_tracking = next_plant.copy()
        next_tracking[:2] -= [
            next_reference.position_m,
            next_reference.velocity_m_s,
        ]
        estimator_error = next_plant - next_gt
        estimator_error[2] = wrap(estimator_error[2])
        time_s = float(sim.data.time)

        logs["time"].append(time_s)
        logs["command"].append(command)
        logs["reference"].append([
            next_reference.position_m,
            next_reference.velocity_m_s,
            next_reference.acceleration_m_s2,
        ])
        logs["plant"].append(next_plant)
        logs["tracking"].append(next_tracking)
        logs["gt"].append(next_gt)
        logs["estimator_error"].append(estimator_error)
        logs["theta_acc"].append(wrap(estimate.theta_acc_rad - nominal_theta_eq))
        logs["requested"].append(requested_sum)
        logs["held"].append(held_sum)
        logs["actual"].append(actual_sum)
        logs["u_lqr"].append(u_lqr)
        logs["u_dr_requested"].append(compensation.requested_u_dr_nm)
        logs["u_dr"].append(compensation.u_dr_nm)
        logs["innovation"].append(observation.scaled_innovation_rms)
        logs["matched_fraction"].append(observation.matched_residual_fraction)
        logs["authority_hit"].append(compensation.authority_limited)
        if interval % history_period == 0:
            history.append({
                "time_s": time_s,
                "user_command": command,
                "p_ref_m": next_reference.position_m,
                "v_ref_m_s": next_reference.velocity_m_s,
                "a_ref_m_s2": next_reference.acceleration_m_s2,
                "x_plant_sensorized": next_plant.tolist(),
                "e_track": next_tracking.tolist(),
                "gt_plant_posthoc": next_gt.tolist(),
                "theta_acc_error_deg": math.degrees(float(logs["theta_acc"][-1])),
                "theta_hat_error_deg": math.degrees(float(next_plant[2])),
                "gt_pitch_error_deg": math.degrees(float(next_gt[2])),
                "requested_sum_torque_nm": requested_sum,
                "software_limited_sum_torque_nm": held_sum,
                "actual_applied_sum_torque_nm": actual_sum,
                "requested_u_dr_nm": compensation.requested_u_dr_nm,
                "u_dr_nm": compensation.u_dr_nm,
                "u_dr_authority_limited": compensation.authority_limited,
                "innovation_scaled_rms": observation.scaled_innovation_rms,
                "matched_residual_fraction": observation.matched_residual_fraction,
            })

    arrays = {name: np.asarray(values) for name, values in logs.items()}
    terminal = arrays["time"] >= float(scenario["duration_s"]) - float(
        config["terminal_window_s"]
    )
    position_error = arrays["tracking"][:, 0]
    velocity_error = arrays["tracking"][:, 1]
    transients = transition_metrics(
        scenario,
        arrays["time"],
        arrays["reference"],
        arrays["plant"],
        config["settling"],
        dt_s,
    )
    reference_kinematic_error = np.diff(
        np.r_[0.0, arrays["reference"][:, 0]]
    ) - 0.5 * dt_s * (
        np.r_[0.0, arrays["reference"][:-1, 1]] + arrays["reference"][:, 1]
    )
    result = {
        "scenario": scenario["name"],
        "mode": scenario["mode"],
        "payload_mode": payload_mode,
        "control_case": control_case,
        "disturbance_rejection_enabled": control_case == "Q",
        "fell": bool(fallen),
        "reference_semantics": {
            "x_plant": "[p_hat, v_hat, pitch_error, pitch_rate]",
            "reference": "[p_ref, v_ref, 0, 0]",
            "lqr_input": "e_track = x_plant - reference",
            "dob_input": "x_plant[k], actual held/applied sum torque[k], x_plant[k+1]",
            "ground_truth_boundary": "evaluator/post-hoc logger only",
        },
        "payload_setup": payload_setup,
        "tracking": {
            "position_error_m": metric_triplet(position_error),
            "velocity_error_m_s": metric_triplet(velocity_error),
            "endpoint_position_error_m": float(position_error[-1]),
            "endpoint_velocity_error_m_s": float(velocity_error[-1]),
            "transients": transients,
        },
        "pitch": {
            "rms_deg": math.degrees(rms(arrays["gt"][:, 2])),
            "peak_abs_deg": math.degrees(float(np.max(np.abs(arrays["gt"][:, 2])))),
            "terminal_rms_deg": math.degrees(rms(arrays["gt"][terminal, 2])),
        },
        "torque": {
            "requested_sum_nm": metric_triplet(arrays["requested"]),
            "software_limited_sum_nm": metric_triplet(arrays["held"]),
            "actual_applied_sum_nm": metric_triplet(arrays["actual"]),
            "requested_minus_actual_nm": metric_triplet(arrays["requested"] - arrays["actual"]),
            "maximum_actual_wheel_nm": float(np.max(np.abs(arrays["actual"])) / 2.0),
            "wheel_saturation_fraction": saturated_updates / interval_count,
        },
        "disturbance_rejection": {
            "q_filter_cutoff_hz": float(dr_raw["q_filter"]["selected_cutoff_hz"]),
            "augmentation_authority_bound_nm": float(dr_raw["augmentation_authority_bound_nm"]),
            "u_dr_nm": metric_triplet(arrays["u_dr"]),
            "requested_u_dr_nm": metric_triplet(arrays["u_dr_requested"]),
            "authority_hit_fraction": float(np.mean(arrays["authority_hit"])),
            "innovation_scaled_rms": rms(arrays["innovation"]),
            "matched_residual_fraction_mean": float(np.mean(arrays["matched_fraction"])),
        },
        "estimator_error_at_500hz": estimator_metrics(arrays["estimator_error"]),
        "acceleration_windows": acceleration_window_metrics(
            arrays["reference"][:, 1],
            arrays["reference"][:, 2],
            arrays["theta_acc"],
            arrays["plant"][:, 2],
            arrays["gt"][:, 2],
        ),
        "reference_validation": {
            "peak_abs_velocity_m_s": float(np.max(np.abs(arrays["reference"][:, 1]))),
            "peak_abs_acceleration_m_s2": float(np.max(np.abs(arrays["reference"][:, 2]))),
            "maximum_trapezoidal_integration_error_m": float(np.max(np.abs(reference_kinematic_error))),
        },
        "imu_hardware_diagnostics": sim.imu_diagnostics(include_hidden_truth=True),
        "history_50hz": history,
    }
    if payload_recorder is not None:
        payload_time = np.asarray(payload_times)
        payload_position = np.asarray(payload_positions)
        relative_velocity = np.gradient(payload_position, payload_time, axis=0)
        first_motion_time = float(scenario["schedule"][1]["time_s"]) if len(scenario["schedule"]) > 1 else 0.0
        early = (payload_time >= first_motion_time) & (payload_time < first_motion_time + 1.0)
        payload_terminal = payload_time >= float(scenario["duration_s"]) - 1.0
        front_wall = sim.model.geom("basket_front_wall")
        payload_geom = sim.model.geom("moving_payload_collision")
        center_limit = float(abs(front_wall.pos[1]) - front_wall.size[1] - payload_geom.size[1])
        contact = np.asarray(payload_contacts, dtype=bool)
        force = np.asarray(payload_forces, dtype=float)
        effective_episodes = force_bearing_contact_episodes(contact, force)
        result["free_payload_posthoc"] = {
            "remained_in_basket": bool(
                np.max(np.abs(payload_position[:, 1])) <= center_limit + 0.005
                and np.min(payload_position[:, 2]) >= 0.14
            ),
            "longitudinal_center_limit_m": center_limit,
            "relative_longitudinal_position_range_m": [
                float(np.min(payload_position[:, 1])),
                float(np.max(payload_position[:, 1])),
            ],
            "relative_longitudinal_speed_rms_m_s": rms(relative_velocity[:, 1]),
            "relative_longitudinal_speed_peak_m_s": float(np.max(np.abs(relative_velocity[:, 1]))),
            "collision_episode_count": len(payload_recorder.collision_episodes),
            "force_bearing_collision_episode_count": len(effective_episodes),
            "maximum_wall_normal_force_n": float(np.max(force)),
            "motion_decay": {
                "first_motion_1s_speed_rms_m_s": rms(relative_velocity[early, 1]),
                "terminal_1s_speed_rms_m_s": rms(relative_velocity[payload_terminal, 1]),
                "terminal_to_first_speed_rms_ratio": rms(relative_velocity[payload_terminal, 1])
                / max(rms(relative_velocity[early, 1]), 1e-12),
                "terminal_1s_position_span_m": float(np.ptp(payload_position[payload_terminal, 1])),
            },
        }
    gate = config["empty_gate"] if payload_mode == "empty" else config["payload_gate"]
    passed, failures = evaluate_case_gate(result, gate, payload_mode)
    result["gate"] = {"passed": passed, "failures": failures}
    return result


def failure_classification(runs: list[dict]) -> list[dict]:
    failures = []
    indexed = {(r["scenario"], r["payload_mode"], r["control_case"]): r for r in runs}
    for run in runs:
        if run["gate"]["passed"]:
            continue
        categories = []
        reasons = run["gate"]["failures"]
        reference = run["reference_validation"]
        if (
            reference["peak_abs_velocity_m_s"] > 0.200000001
            or reference["peak_abs_acceleration_m_s2"] > 0.250000001
            or reference["maximum_trapezoidal_integration_error_m"] > 1e-9
        ):
            categories.append("reference_semantics")
        estimator = run["estimator_error_at_500hz"]
        if estimator["velocity_m_s"]["rms"] > 0.05 or estimator["pitch_deg"]["rms"] > 3.0:
            categories.append("estimator_dynamic_error")
        empty = indexed.get((run["scenario"], "empty", run["control_case"]))
        fixed = indexed.get((run["scenario"], "fixed", run["control_case"]))
        if run["payload_mode"] == "fixed" and empty and empty["gate"]["passed"]:
            categories.append("nominal_model_mismatch")
        if run["payload_mode"] == "free" and fixed and fixed["gate"]["passed"]:
            categories.append("payload_hidden_dynamics")
        if run["control_case"] == "Q":
            baseline = indexed.get((run["scenario"], run["payload_mode"], "A"))
            if baseline:
                q_tracking = run["tracking"]
                a_tracking = baseline["tracking"]
                q_degraded = (
                    q_tracking["position_error_m"]["rms"]
                    > 1.10 * a_tracking["position_error_m"]["rms"]
                    or q_tracking["velocity_error_m_s"]["rms"]
                    > 1.10 * a_tracking["velocity_error_m_s"]["rms"]
                    or abs(q_tracking["endpoint_velocity_error_m_s"])
                    > 1.10 * abs(a_tracking["endpoint_velocity_error_m_s"])
                )
                if q_degraded:
                    categories.append("Q_interaction")
        if run["torque"]["wheel_saturation_fraction"] > 0.0 or run["disturbance_rejection"]["authority_hit_fraction"] > 0.25:
            categories.append("actuator_authority")
        if not categories:
            categories.append("nominal_model_mismatch")
        failures.append({
            "scenario": run["scenario"],
            "payload_mode": run["payload_mode"],
            "control_case": run["control_case"],
            "failed_metrics": reasons,
            "categories": categories,
            "alternative_explanations": [
                "A gate failure with low estimator error and no saturation is consistent with frozen nominal closed-loop bandwidth/model limits.",
                "A Q-only degradation is consistent with reference-transient energy entering the physical innovation through model mismatch, not with direct use of tracking error by the DOB.",
                "Free-payload degradation beyond fixed-payload behavior is consistent with hidden contact/unmatched dynamics.",
            ],
        })
    return failures


def load_inputs() -> tuple[dict, dict, dict, dict, dict, dict]:
    return tuple(
        json.loads(path.read_text(encoding="utf-8"))
        for path in (
            CONFIG_PATH,
            EXPERIMENT_CONFIG_PATH,
            DR_CONFIG_PATH,
            REDUCED_PATH,
            OFFLINE_PATH,
            PLANT_PARAMETERS_PATH,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=[item["name"] for item in json.loads(CONFIG_PATH.read_text(encoding="utf-8"))["scenarios"]])
    parser.add_argument("--payload", choices=PAYLOAD_MODES)
    parser.add_argument("--case", choices=CONTROL_CASES)
    parser.add_argument("--no-write", action="store_true")
    args = parser.parse_args()
    config, experiment, dr_raw, reduced, offline, plant = load_inputs()
    scenarios = config["scenarios"]
    if args.scenario:
        scenarios = [item for item in scenarios if item["name"] == args.scenario]
    payload_modes = (args.payload,) if args.payload else PAYLOAD_MODES
    control_cases = (args.case,) if args.case else CONTROL_CASES
    runs = []
    empty_gate_passed = True
    optional_empty_passed = True

    for payload_mode in payload_modes:
        if payload_mode != "empty" and not args.payload and not empty_gate_passed:
            break
        for scenario in scenarios:
            if (
                not scenario["required_for_empty_gate"]
                and payload_mode != "empty"
                and not optional_empty_passed
            ):
                continue
            for control_case in control_cases:
                print(f"RUN {payload_mode}/{control_case}/{scenario['name']}", flush=True)
                run = run_case(
                    scenario, payload_mode, control_case, config, experiment,
                    dr_raw, reduced, offline, plant,
                )
                runs.append(run)
                print(
                    f"  pass={run['gate']['passed']} "
                    f"p_rmse={run['tracking']['position_error_m']['rms']:.4f} m "
                    f"v_rmse={run['tracking']['velocity_error_m_s']['rms']:.4f} m/s "
                    f"pitch_peak={run['pitch']['peak_abs_deg']:.2f} deg",
                    flush=True,
                )
        if payload_mode == "empty":
            required = [
                run for run in runs
                if run["payload_mode"] == "empty"
                and next(item for item in config["scenarios"] if item["name"] == run["scenario"])["required_for_empty_gate"]
            ]
            optional = [
                run for run in runs
                if run["payload_mode"] == "empty"
                and not next(item for item in config["scenarios"] if item["name"] == run["scenario"])["required_for_empty_gate"]
            ]
            empty_gate_passed = bool(required) and all(run["gate"]["passed"] for run in required)
            optional_empty_passed = bool(optional) and all(run["gate"]["passed"] for run in optional)

    payload_matrix = {}
    for payload_mode in PAYLOAD_MODES:
        selected = [run for run in runs if run["payload_mode"] == payload_mode]
        payload_matrix[payload_mode] = {
            "status": (
                "NOT_RUN_EMPTY_GATE"
                if not selected and payload_mode != "empty"
                else "PASS"
                if selected and all(run["gate"]["passed"] for run in selected)
                else "FAIL"
            ),
            "run_count": len(selected),
            "passed_run_count": sum(run["gate"]["passed"] for run in selected),
        }
    result = {
        "stage": config["stage"],
        "status": "PASS" if runs and all(run["gate"]["passed"] for run in runs) else "FAIL",
        "execution": {
            "physics_dt_s": config["physics_dt_s"],
            "controller_dt_s": config["controller_dt_s"],
            "imu_rng_seed": config["imu_rng_seed"],
            "empty_required_gate_passed": empty_gate_passed,
            "optional_0p20_empty_passed": optional_empty_passed,
            "payload_stage_executed": any(run["payload_mode"] != "empty" for run in runs),
            "payload_stage_skip_reason": (
                None
                if any(run["payload_mode"] != "empty" for run in runs) or args.payload
                else "required empty-robot commanded-motion gate did not pass"
            ),
        },
        "frozen_invariants": {
            "K_id": offline["identified_lqr"]["K_id"],
            "q_filter_cutoff_hz": dr_raw["q_filter"]["selected_cutoff_hz"],
            "augmentation_authority_bound_nm": dr_raw["augmentation_authority_bound_nm"],
            "wheel_torque_limit_nm": plant["known"]["wheel_torque_hard_peak_nm"],
            "estimator_config_unchanged": True,
            "moving_payload_acceptance_unchanged": True,
        },
        "reference_limits": config["reference_limits"],
        "gates": {
            "empty": config["empty_gate"],
            "payload": config["payload_gate"],
        },
        "payload_matrix": payload_matrix,
        "run_summary": [
            {
                "scenario": run["scenario"],
                "payload_mode": run["payload_mode"],
                "control_case": run["control_case"],
                "status": "PASS" if run["gate"]["passed"] else "FAIL",
                "position_tracking_rmse_m": run["tracking"]["position_error_m"]["rms"],
                "velocity_tracking_rmse_m_s": run["tracking"]["velocity_error_m_s"]["rms"],
                "endpoint_position_error_m": run["tracking"]["endpoint_position_error_m"],
                "endpoint_velocity_error_m_s": run["tracking"]["endpoint_velocity_error_m_s"],
                "pitch_peak_deg": run["pitch"]["peak_abs_deg"],
                "terminal_pitch_rms_deg": run["pitch"]["terminal_rms_deg"],
                "wheel_saturation_fraction": run["torque"]["wheel_saturation_fraction"],
                "u_dr_rms_nm": run["disturbance_rejection"]["u_dr_nm"]["rms"],
                "u_dr_authority_hit_fraction": run["disturbance_rejection"]["authority_hit_fraction"],
                "innovation_scaled_rms": run["disturbance_rejection"]["innovation_scaled_rms"],
            }
            for run in runs
        ],
        "runs": runs,
        "failure_classification": failure_classification(runs),
    }
    if not args.no_write and not (args.scenario or args.payload or args.case):
        RESULTS_PATH.parent.mkdir(parents=True, exist_ok=True)
        RESULTS_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"WROTE {RESULTS_PATH}")
    print(json.dumps({
        "status": result["status"],
        "execution": result["execution"],
        "run_count": len(runs),
        "failure_classification": result["failure_classification"],
    }, indent=2))


if __name__ == "__main__":
    main()
