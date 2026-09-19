"""Run the Stage 3A Dynamic-D velocity feedforward handoff experiment."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import (
    CascadePID,
    JerkLimitedLongitudinalReferenceGenerator,
    NominalDiscreteFeedforward,
    VelocityLifecyclePhase,
    VelocityReferenceLifecycle,
    VelocityTransitionPlan,
    project_hidden_reference_and_input,
)
from control.cascade_pid import load_config as load_pid_config
from sim import LongitudinalEstimator, load_longitudinal_estimator_config
from sim.longitudinal_estimation import LongitudinalAccelerationCompensationConfig
from moving_payload_benchmark import has_chassis_floor_contact, held_initial_imu
from run_stage3a_commanded_motion import build_sim, metric_triplet, rms, transition_metrics
from run_stage3a_dynamic_nominal_reference import (
    DR_CONFIG_PATH,
    EXPERIMENT_CONFIG_PATH,
    MODEL_DIR,
    MOTION_CONFIG_PATH,
    OFFLINE_PATH,
    PLANT_PARAMETERS_PATH,
    REDUCED_PATH,
    compact_summary,
    load_json,
)


CONFIG_PATH = MODEL_DIR / "stage3" / "config" / "stage3a_velocity_feedforward_lean_tail_config.json"
DYNAMIC_CONFIG_PATH = MODEL_DIR / "stage3" / "config" / "stage3a_dynamic_nominal_reference_config.json"
BASELINE_RESULTS_PATH = (
    MODEL_DIR / "stage3" / "results" / "history" / "stage3a_velocity_feedforward_rearm_results.json"
)
RESULTS_PATH = MODEL_DIR / "stage3" / "results" / "stage3a_velocity_feedforward_lean_tail_results.json"
PID_CONFIG_PATH = MODEL_DIR / "cascade_pid_config.json"
BASELINE_MANIFEST_PATH = MODEL_DIR / "stage3" / "config" / "baseline.json"


def load_production_baseline() -> tuple[dict, dict, dict, dict, dict]:
    """Resolve the small Stage 3A manifest without duplicating model/gain data."""
    manifest = load_json(BASELINE_MANIFEST_PATH)
    config = load_json(MODEL_DIR / manifest["runtime_config"])
    dynamic_config = load_json(MODEL_DIR / manifest["dynamic_nominal_config"])
    motion_config = load_json(MODEL_DIR / manifest["motion_config"])
    offline = load_json(MODEL_DIR / manifest["feedback"]["source"])
    model_result = load_json(MODEL_DIR / manifest["nominal_model"]["source"])
    offline["fit"]["A_identified"] = model_result[manifest["nominal_model"]["A_field"]]
    offline["fit"]["B_identified"] = model_result[manifest["nominal_model"]["B_field"]]
    if not 0.0 <= float(config["lambda_ff"]) <= 1.0:
        raise RuntimeError("Stage 3A baseline lambda is outside [0, 1]")
    if "nominal_A_velocity_correction_c_v" in config:
        raise RuntimeError("experimental velocity-column patch is forbidden in production config")
    return manifest, config, dynamic_config, motion_config, offline


def build_velocity_transition_plan(
    start_reference: np.ndarray,
    target_velocity_m_s: float,
    dynamic_config: dict,
    motion_config: dict,
    A: np.ndarray,
    B: np.ndarray,
    state_scales: np.ndarray,
    input_scale_nm: float,
    feedforward_rearm_threshold_s: float,
    horizon_search: dict,
    start_acceleration_m_s2: float = 0.0,
) -> dict:
    dt_s = float(motion_config["controller_dt_s"])
    limits = motion_config["reference_limits"]
    generator = JerkLimitedLongitudinalReferenceGenerator(
        dt_s,
        float(limits["max_velocity_m_s"]),
        float(limits["max_acceleration_m_s2"]),
        float(dynamic_config["max_jerk_m_s3"]),
    )
    generator.reset(
        position_m=float(start_reference[0]),
        velocity_m_s=float(start_reference[1]),
        acceleration_m_s2=float(start_acceleration_m_s2),
    )
    references = [generator.reference]
    shaped_finished = [False]
    # Only the accepted target/current reference is used, never the timing
    # or value of a future benchmark command.
    while True:
        references.append(generator.step_velocity(target_velocity_m_s))
        shaped_finished.append(not generator.plan_active)
        if not generator.plan_active:
            break
    velocity_interval_count = len(references) - 1
    planned_transient_duration_s = float(velocity_interval_count * dt_s)
    if (
        abs(references[-1].velocity_m_s - target_velocity_m_s) > 1e-9
        or abs(references[-1].acceleration_m_s2) > 1e-9
    ):
        raise RuntimeError("shaped terminal must have target velocity and zero acceleration")
    feedforward_rearmed = bool(
        planned_transient_duration_s >= feedforward_rearm_threshold_s
    )
    velocity_profile = np.asarray(
        [
            [item.position_m, item.velocity_m_s, item.acceleration_m_s2]
            for item in references
        ]
    )
    velocity_delta = abs(
        float(target_velocity_m_s) - float(start_reference[1])
    )
    acceleration_limit = float(limits["max_acceleration_m_s2"])
    jerk_limit = float(dynamic_config["max_jerk_m_s3"])
    acceleration_limit_active = bool(
        velocity_delta + 1e-12 >= acceleration_limit**2 / jerk_limit
    )
    if acceleration_limit_active:
        jerk_ramp_duration_s = acceleration_limit / jerk_limit
        acceleration_limited_duration_s = max(
            0.0, velocity_delta / acceleration_limit - jerk_ramp_duration_s
        )
    else:
        jerk_ramp_duration_s = math.sqrt(velocity_delta / jerk_limit)
        acceleration_limited_duration_s = 0.0
    sampled_jerk = np.diff(velocity_profile[:, 2]) / dt_s
    reference_dynamics = {
        "requested_acceleration_limit_m_s2": acceleration_limit,
        "requested_jerk_limit_m_s3": jerk_limit,
        "realized_peak_abs_acceleration_m_s2": float(
            np.max(np.abs(velocity_profile[:, 2]))
        ),
        "realized_peak_abs_jerk_m_s3": float(
            np.max(np.abs(sampled_jerk)) if len(sampled_jerk) else 0.0
        ),
        "acceleration_limit_active": acceleration_limit_active,
        "acceleration_limited_duration_s": acceleration_limited_duration_s,
        "jerk_limited_duration_s": 2.0 * jerk_ramp_duration_s,
        "triangular_acceleration_profile": not acceleration_limit_active,
    }
    step_s = float(horizon_search["candidate_step_s"])
    maximum_tail_s = float(horizon_search["maximum_tail_s"])
    residual_threshold = float(horizon_search["normalized_residual_rms_max"])
    trials = []
    projection = None
    for candidate in range(round(maximum_tail_s / step_s) + 1):
        tail_intervals = round(candidate * step_s / dt_s)
        interval_count = velocity_interval_count + tail_intervals
        tail = np.empty((tail_intervals, 3))
        if tail_intervals:
            tail[:, 0] = velocity_profile[-1, 0] + (
                np.arange(1, tail_intervals + 1) * dt_s * target_velocity_m_s
            )
            tail[:, 1] = target_velocity_m_s
            tail[:, 2] = 0.0
        profile = np.vstack([velocity_profile, tail])
        trial = {"T_full_s": interval_count * dt_s, "tail_s": tail_intervals * dt_s}
        try:
            candidate_projection = project_hidden_reference_and_input(
                A, B, state_scales, input_scale_nm,
                profile[:, :2], [(0, interval_count)],
            )
            finite = bool(np.isfinite(np.r_[
                candidate_projection.reference_states.ravel(),
                candidate_projection.inputs_nm,
                candidate_projection.residuals.ravel(),
            ]).all())
            normalized_rms = float(np.sqrt(np.mean(
                (candidate_projection.residuals / state_scales) ** 2
            )))
            converged = all(
                d["lsqr_stop_code"] in (1, 2)
                for d in candidate_projection.segment_diagnostics
            )
            trial.update({
                "solver_converged": converged, "finite": finite,
                "normalized_residual_rms": normalized_rms,
                "feasible": converged and finite and normalized_rms <= residual_threshold,
            })
        except (RuntimeError, FloatingPointError) as error:
            trial.update({"feasible": False, "reason": str(error)})
        trials.append(trial)
        if trial["feasible"]:
            projection = candidate_projection
            break
    if projection is None:
        raise RuntimeError("velocity maneuver infeasible in permitted T_full search: " + json.dumps(trials))
    feedforward = NominalDiscreteFeedforward(A, B, state_scales)
    inputs = np.empty(interval_count)
    residuals = np.empty((interval_count, 4))
    for interval in range(interval_count):
        command = feedforward.command(
            projection.reference_states[interval],
            projection.reference_states[interval + 1],
        )
        inputs[interval] = command.torque_nm
        residuals[interval] = command.residual
    input_difference = float(np.max(np.abs(inputs - projection.inputs_nm)))
    if input_difference > 1e-8:
        raise RuntimeError("joint projection and scalar feedforward disagree")
    plan = VelocityTransitionPlan(
        target_velocity_m_s=float(target_velocity_m_s),
        planned_transient_duration_s=planned_transient_duration_s,
        feedforward_rearmed=feedforward_rearmed,
        reference_states=projection.reference_states,
        reference_accelerations_m_s2=profile[:, 2],
        shaped_reference_finished=np.r_[
            np.asarray(shaped_finished, dtype=bool),
            np.ones(tail_intervals, dtype=bool),
        ],
        feedforward_inputs_nm=inputs,
    )
    return {
        "plan": plan,
        "projection_residuals": residuals,
        "projection_input_difference_nm": input_difference,
        "projection_diagnostics": list(projection.segment_diagnostics),
        "planned_transient_duration_s": planned_transient_duration_s,
        "dynamic_nominal_horizon_s": interval_count * dt_s,
        "horizon_search_trials": trials,
        "normalized_residual_rms": trials[-1]["normalized_residual_rms"],
        "theta_ref_at_T_v_rad": float(projection.reference_states[velocity_interval_count, 2]),
        "theta_dot_ref_at_T_v_rad_s": float(projection.reference_states[velocity_interval_count, 3]),
        "feedforward_rearmed": feedforward_rearmed,
        "reference_dynamics": reference_dynamics,
    }


def next_event_end_s(schedule: list[dict], event_index: int, duration_s: float) -> float:
    return (
        float(schedule[event_index + 1]["time_s"])
        if event_index + 1 < len(schedule)
        else float(duration_s)
    )


def run_case(
    scenario: dict,
    config: dict,
    dynamic_config: dict,
    motion_config: dict,
    experiment: dict,
    dr_raw: dict,
    reduced: dict,
    offline: dict,
    plant: dict,
) -> dict:
    dt_s = float(motion_config["controller_dt_s"])
    interval_count = round(float(scenario["duration_s"]) / dt_s)
    fit = offline["fit"]
    A = np.asarray(fit["A_identified"], dtype=float)
    B = np.asarray(fit["B_identified"], dtype=float)
    state_scales = np.asarray(fit["state_scales"], dtype=float)
    input_scale_nm = float(fit["input_scale_nm"])
    K4 = np.asarray(offline["identified_lqr"]["K_id"], dtype=float)
    position_feedback_coefficient = -float(K4[0, 0])
    controller_mode = config.get("controller_mode", "lqr_ff")
    if controller_mode not in ("pid", "lqr", "lqr_ff"):
        raise ValueError("unsupported controller mode")
    pid = CascadePID(load_pid_config(PID_CONFIG_PATH)) if controller_mode == "pid" else None
    if pid is not None:
        if not math.isclose(pid.config.controller_dt_s, dt_s, abs_tol=1e-12):
            raise RuntimeError("PID timing differs from the common benchmark")
        if not math.isclose(
            pid.config.sum_torque_limit_nm,
            2.0 * float(plant["known"]["wheel_torque_hard_peak_nm"]),
            abs_tol=1e-12,
        ):
            raise RuntimeError("PID torque limit differs from the common benchmark")
    feedforward_enabled = controller_mode == "lqr_ff"
    lambda_ff = float(config.get("lambda_ff", 1.0))

    lifecycle_raw = config["velocity_lifecycle"]
    lifecycle = VelocityReferenceLifecycle(
        controller_dt_s=dt_s,
        feedforward_fade_s=float(lifecycle_raw["feedforward_fade_s"]),
    )
    lifecycle.reset(
        position_reference_m=0.0,
        velocity_reference_m_s=float(scenario["schedule"][0]["command"]),
    )

    nominal_theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim, _, _ = build_sim(
        "empty", int(motion_config["imu_rng_seed"]), nominal_theta_eq, dr_raw
    )
    physics_steps = int(experiment["physics_steps_per_update"])
    if not math.isclose(sim.physics_dt, float(motion_config["physics_dt_s"]), abs_tol=1e-12):
        raise RuntimeError("physics timing differs from frozen Stage 3A baseline")
    if not math.isclose(physics_steps * sim.physics_dt, dt_s, abs_tol=1e-12):
        raise RuntimeError("controller timing differs from frozen Stage 3A baseline")
    estimator_config = load_longitudinal_estimator_config()
    pitch_filter_mode = config.get("pitch_filter_mode", "current")
    if pitch_filter_mode not in ("current", "acceleration_compensated"):
        raise ValueError("unsupported pitch filter experiment mode")
    acceleration_compensation = (
        LongitudinalAccelerationCompensationConfig()
        if pitch_filter_mode == "acceleration_compensated" else None
    )
    estimator = LongitudinalEstimator(
        estimator_config,
        sim.encoder_profile,
        float(reduced["parameters"]["wheel_radius_m"]),
        acceleration_compensation=acceleration_compensation,
    )
    accel, gyro, measurement_age_s, extrapolation_allowed = held_initial_imu(sim)
    estimate = estimator.reset(
        accel,
        gyro,
        sim.wheel_encoder_counts(),
        measurement_age_s=measurement_age_s,
        allow_kinematic_extrapolation=extrapolation_allowed,
    )
    peak_per_wheel = float(plant["known"]["wheel_torque_hard_peak_nm"])
    history_period = round(
        1.0 / (dt_s * float(motion_config["history_frequency_hz"]))
    )
    logs = {
        name: []
        for name in (
            "time",
            "control_time",
            "command",
            "phase",
            "reference",
            "acceleration",
            "plant",
            "gt",
            "u_fb",
            "u_ff",
            "u_ff_raw",
            "u_ff_lifecycle",
            "fade_alpha",
            "requested",
            "held",
            "actual",
        )
    }
    history = []
    handoffs = []
    transition_plans = []
    transition_events = []
    schedule = scenario["schedule"]
    next_event_index = 1
    user_command = float(schedule[0]["command"])
    fallen = False
    saturated_updates = 0
    previous_requested_sum: float | None = None

    for interval in range(interval_count):
        control_time_s = float(sim.data.time)
        if (
            next_event_index < len(schedule)
            and control_time_s + 1e-12
            >= float(schedule[next_event_index]["time_s"])
        ):
            maneuver_interrupted = lifecycle.phase != VelocityLifecyclePhase.VELOCITY_HOLD
            if transition_events and maneuver_interrupted:
                transition_events[-1]["maneuver_interrupted_by_new_command"] = True
                transition_events[-1]["maneuver_interruption_time_s"] = control_time_s
            if lifecycle.feedforward_fade_active and handoffs:
                interrupted_command = lifecycle.command(control_time_s)
                handoffs[-1].update(
                    {
                        "fade_completed": False,
                        "fade_interrupted_by_new_command": True,
                        "fade_interruption_time_s": control_time_s,
                        "fade_elapsed_before_interruption_s": control_time_s
                        - float(handoffs[-1]["fade_start_time_s"]),
                        "u_ff_used_at_interruption_nm": (
                            lambda_ff * interrupted_command.feedforward_nm
                        ),
                    }
                )
                transition_events[-1].update(
                    {
                        "fade_interrupted_by_new_command": True,
                        "fade_interruption_time_s": control_time_s,
                    }
                )
            user_command = float(schedule[next_event_index]["command"])
            segment_end_s = next_event_end_s(
                schedule, next_event_index, float(scenario["duration_s"])
            )
            transition_plan = build_velocity_transition_plan(
                lifecycle.reference_state,
                user_command,
                dynamic_config,
                motion_config,
                A,
                B,
                state_scales,
                input_scale_nm,
                float(lifecycle_raw["feedforward_rearm_threshold_s"]),
                lifecycle_raw["horizon_search"],
                start_acceleration_m_s2=lifecycle.reference_acceleration_m_s2,
            )
            lifecycle.begin_transition(transition_plan["plan"])
            transition_plans.append(
                {
                    "event_index": next_event_index,
                    "command_time_s": control_time_s,
                    "segment_end_s": segment_end_s,
                    **transition_plan,
                }
            )
            transition_events.append(
                {
                    "schedule_event_index": next_event_index,
                    "command_time_s": control_time_s,
                    "accepted_target_velocity_m_s": user_command,
                    "start_reference_velocity_m_s": float(
                        transition_plan["plan"].reference_states[0, 1]
                    ),
                    "planned_transient_duration_s": transition_plan[
                        "planned_transient_duration_s"
                    ],
                    "dynamic_nominal_horizon_s": transition_plan[
                        "dynamic_nominal_horizon_s"
                    ],
                    "T_v_s": transition_plan["planned_transient_duration_s"],
                    "T_full_s": transition_plan["dynamic_nominal_horizon_s"],
                    "lean_recovery_tail_s": transition_plan["dynamic_nominal_horizon_s"]
                    - transition_plan["planned_transient_duration_s"],
                    "normalized_residual_rms": transition_plan["normalized_residual_rms"],
                    "horizon_search_trials": transition_plan["horizon_search_trials"],
                    "theta_ref_at_T_v_rad": transition_plan["theta_ref_at_T_v_rad"],
                    "theta_dot_ref_at_T_v_rad_s": transition_plan["theta_dot_ref_at_T_v_rad_s"],
                    "theta_ref_at_T_full_rad": float(transition_plan["plan"].reference_states[-1, 2]),
                    "theta_dot_ref_at_T_full_rad_s": float(transition_plan["plan"].reference_states[-1, 3]),
                    "u_ff_raw_at_T_full_nm": float(transition_plan["plan"].feedforward_inputs_nm[-1]),
                    "u_ff_scaled_at_T_full_nm": lambda_ff * float(transition_plan["plan"].feedforward_inputs_nm[-1]),
                    "u_ff_terminal_convention": "last solved interval (left-limit); no cruise input solve",
                    "feedforward_rearm_threshold_s": float(
                        lifecycle_raw["feedforward_rearm_threshold_s"]
                    ),
                    "feedforward_rearmed": transition_plan[
                        "feedforward_rearmed"
                    ],
                    "reference_dynamics": transition_plan[
                        "reference_dynamics"
                    ],
                    "expected_shaped_reference_reached_time_s": (
                        control_time_s
                        + transition_plan["planned_transient_duration_s"]
                    ),
                    "expected_T_full_time_s": control_time_s + transition_plan["dynamic_nominal_horizon_s"],
                    "maneuver_interrupted_by_new_command": False,
                    "fade_interrupted_by_new_command": False,
                }
            )
            next_event_index += 1

        x_plant = estimate.plant_state(nominal_theta_eq)
        if (
            transition_events and lifecycle.shaped_reference_reached
            and "actual_shaped_reference_reached_time_s" not in transition_events[-1]
        ):
            transition_events[-1]["actual_shaped_reference_reached_time_s"] = control_time_s
        lifecycle_command = lifecycle.command(control_time_s)
        reference_state = lifecycle_command.reference_state
        # The planner/lifecycle is identical in all modes. Only the existing
        # controller and actuator participation of feedforward are selected.
        tracking_error = x_plant - reference_state
        pid_command = pid.command(tracking_error) if pid is not None else None
        u_ff_raw = lifecycle.raw_feedforward_nm
        u_ff_lifecycle = lifecycle_command.feedforward_nm
        u_ff = lambda_ff * u_ff_lifecycle if feedforward_enabled else 0.0
        u_fb = (
            pid_command.raw_sum_torque_nm
            if pid_command is not None
            else -float((K4 @ tracking_error).item())
        )
        requested_sum = u_ff + u_fb
        if lifecycle_command.fade_started:
            reference_before = lifecycle_command.dynamic_reference_before_fade
            assert reference_before is not None
            u_ff_before = (
                lambda_ff * float(lifecycle_command.feedforward_exit_nm)
                if feedforward_enabled else 0.0
            )
            e_before = x_plant - reference_before
            e_after = x_plant - reference_state
            # Fade start preserves the reference. Do not call stateful PID
            # twice merely to compute this same-state diagnostic.
            u_fb_before = u_fb if pid_command is not None else -float((K4 @ e_before).item())
            u_total_before = u_ff_before + u_fb_before
            handoffs.append(
                {
                    "transition_index": len(handoffs),
                    "schedule_event_index": transition_events[-1][
                        "schedule_event_index"
                    ],
                    "target_velocity_m_s": user_command,
                    "planned_transient_duration_s": transition_events[-1][
                        "planned_transient_duration_s"
                    ],
                    "feedforward_rearmed": True,
                    "fade_start_time_s": control_time_s,
                    "scheduled_fade_duration_s": float(
                        lifecycle_raw["feedforward_fade_s"]
                    ),
                    "fade_completed": False,
                    "fade_interrupted_by_new_command": False,
                    "u_ff_exit_nm": u_ff_before,
                    "e_p_before_fade_m": float(e_before[0]),
                    "e_p_after_reference_manifold_switch_m": float(e_after[0]),
                    "p_ref_before_fade_m": float(reference_before[0]),
                    "p_ref_after_reference_manifold_switch_m": float(
                        reference_state[0]
                    ),
                    "u_total_before_fade_start_nm": u_total_before,
                    "u_total_at_fade_start_nm": requested_sum,
                    "fade_start_torque_jump_nm": requested_sum - u_total_before,
                    "theta_ref_before_deg": math.degrees(float(reference_before[2])),
                    "theta_ref_after_deg": math.degrees(float(reference_state[2])),
                    "theta_dot_ref_before_deg_s": math.degrees(
                        float(reference_before[3])
                    ),
                    "theta_dot_ref_after_deg_s": math.degrees(
                        float(reference_state[3])
                    ),
                    "v_ref_before_m_s": float(reference_before[1]),
                    "a_ref_before_m_s2": float(
                        lifecycle_command.dynamic_acceleration_before_fade_m_s2
                    ),
                }
            )
            transition_events[-1].update(
                {
                    "fade_start_time_s": control_time_s,
                    "fade_start_minus_expected_completion_s": control_time_s
                    - float(
                        transition_events[-1][
                            "expected_T_full_time_s"
                        ]
                    ),
                }
            )
        if lifecycle_command.fade_finished:
            transition_events[-1]["feedforward_fade_completed_time_s"] = control_time_s
            handoffs[-1].update(
                {
                    "feedforward_fade_completed_time_s": control_time_s,
                    "actual_fade_duration_s": control_time_s
                    - float(handoffs[-1]["fade_start_time_s"]),
                    "u_ff_after_fade_nm": u_ff,
                    "u_total_previous_sample_at_fade_end_nm": previous_requested_sum,
                    "u_total_at_fade_end_nm": requested_sum,
                    "fade_completed": True,
                    "theta_ref_at_fade_end_deg": math.degrees(float(reference_state[2])),
                    "theta_dot_ref_at_fade_end_deg_s": math.degrees(float(reference_state[3])),
                }
            )

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
        lean_reference_finished = lifecycle.advance()
        next_reference_state = lifecycle.reference_state
        next_acceleration = (
            lifecycle.reference_acceleration_m_s2
        )
        time_s = float(sim.data.time)
        if lean_reference_finished:
            transition_events[-1].update(
                {
                    "lean_reference_finished_time_s": time_s,
                    "transition_to_hold_time_s": time_s,
                    "terminal_reference_state": next_reference_state.tolist(),
                    "terminal_reference_acceleration_m_s2": next_acceleration,
                }
            )

        logs["time"].append(time_s)
        logs["control_time"].append(control_time_s)
        logs["command"].append(user_command)
        logs["phase"].append(lifecycle_command.phase.value)
        logs["reference"].append(next_reference_state)
        logs["acceleration"].append(next_acceleration)
        logs["plant"].append(next_plant)
        logs["gt"].append(next_gt)
        logs["u_fb"].append(u_fb)
        logs["u_ff"].append(u_ff)
        logs["u_ff_raw"].append(u_ff_raw)
        logs["u_ff_lifecycle"].append(u_ff_lifecycle)
        logs["fade_alpha"].append(lifecycle_command.fade_alpha)
        logs["requested"].append(requested_sum)
        logs["held"].append(held_sum)
        logs["actual"].append(actual_sum)
        if interval % history_period == 0:
            history.append(
                {
                    "time_s": time_s,
                    "control_time_s": control_time_s,
                    "user_velocity_command_m_s": user_command,
                    "velocity_lifecycle_phase": lifecycle_command.phase.value,
                    "feedforward_lifecycle_phase": lifecycle_command.feedforward_phase,
                    "controller_mode": controller_mode,
                    "feedforward_actuator_enabled": feedforward_enabled,
                    "feedforward_exit_fade_active": lifecycle_command.fade_active,
                    "feedforward_exit_fade_alpha": lifecycle_command.fade_alpha,
                    "feedforward_rearmed_for_active_transition": (
                        lifecycle_command.feedforward_rearmed
                    ),
                    "p_ref_m": float(next_reference_state[0]),
                    "v_ref_m_s": float(next_reference_state[1]),
                    "a_ref_m_s2": next_acceleration,
                    "theta_ref_deg": math.degrees(float(next_reference_state[2])),
                    "theta_dot_ref_deg_s": math.degrees(
                        float(next_reference_state[3])
                    ),
                    "x_plant_sensorized": next_plant.tolist(),
                    "gt_plant_posthoc": next_gt.tolist(),
                    "e_p_m": float(next_plant[0] - next_reference_state[0]),
                    "e_v_m_s": float(next_plant[1] - next_reference_state[1]),
                    "u_fb_sum_nm": u_fb,
                    "u_ff_sum_nm": u_ff,
                    "u_ff_raw_sum_nm": u_ff_raw,
                    "u_ff_lifecycle_sum_nm": u_ff_lifecycle,
                    "lambda_ff": lambda_ff,
                    "u_requested_sum_nm": requested_sum,
                    "u_software_limited_sum_nm": held_sum,
                    "u_actual_applied_sum_nm": actual_sum,
                }
            )
            if pid_command is not None:
                history[-1]["pid_command"] = asdict(pid_command)
            if acceleration_compensation is not None:
                history[-1]["acceleration_compensated_pitch"] = {
                    **estimator.acceleration_compensation_log_fields(),
                    "theta_hat_control_time_rad": estimate.theta_hat_rad,
                    "theta_hat_measurement_time_rad": estimate.theta_measurement_time_rad,
                    "theta_dot_hat_rad_s": estimate.theta_dot_hat_rad_s,
                    "imu_measurement_age_s": estimate.imu_measurement_age_s,
                    **sim.imu_raw_log_fields(),
                }
        previous_requested_sum = requested_sum

    arrays = {
        name: np.asarray(values)
        for name, values in logs.items()
    }
    position_error = arrays["plant"][:, 0] - arrays["reference"][:, 0]
    velocity_error = arrays["plant"][:, 1] - arrays["reference"][:, 1]
    transients = transition_metrics(
        scenario,
        arrays["time"],
        np.column_stack(
            [arrays["reference"][:, :2], arrays["acceleration"]]
        ),
        arrays["plant"],
        motion_config["settling"],
        dt_s,
    )
    terminal = arrays["time"] >= float(scenario["duration_s"]) - float(
        motion_config["terminal_window_s"]
    )
    theta_reference = arrays["reference"][:, 2]
    theta_dot_reference = arrays["reference"][:, 3]
    active_acceleration = np.abs(arrays["acceleration"]) > 1e-9
    lean_acceleration_correlation = (
        float(
            np.corrcoef(
                theta_reference[active_acceleration],
                arrays["acceleration"][active_acceleration],
            )[0, 1]
        )
        if np.count_nonzero(active_acceleration) > 1
        else 0.0
    )
    transition_residuals = np.vstack(
        [item["projection_residuals"] for item in transition_plans]
    )
    normalized_transition_residuals = transition_residuals / state_scales
    launch_diagnostics = []
    for event in transition_events:
        target = float(event["accepted_target_velocity_m_s"])
        if event["start_reference_velocity_m_s"] != 0.0 or target == 0.0:
            continue
        start = event["command_time_s"]
        shaped_time = event["actual_shaped_reference_reached_time_s"]
        after = (arrays["time"] >= shaped_time) & (arrays["time"] < shaped_time + 0.4)
        entry = {"target_velocity_m_s": target, "T_v_time_s": shaped_time}
        for name, states in (("estimated", arrays["plant"]), ("GT_posthoc", arrays["gt"])):
            entry[name] = {
                "max_velocity_dip_after_T_v_0p4s_m_s": float(max(
                    0.0, np.max(np.sign(target) * (target - states[after, 1]))
                )),
            }
            for window_name, lo, hi in (
                ("before_T_v", start, shaped_time),
                ("after_T_v_0p4s", shaped_time, shaped_time + 0.4),
            ):
                mask = (arrays["time"] >= lo) & (arrays["time"] < hi)
                window_times = arrays["time"][mask]
                theta = states[mask, 2]
                crossings = np.flatnonzero(theta[:-1] * theta[1:] < 0.0)
                entry[name][window_name] = {
                    "pitch_error_crossed_zero": bool(len(crossings)),
                    "pitch_error_zero_crossing_times_s": window_times[crossings + 1].tolist(),
                    "absolute_pitch_crossed_zero": bool(np.any(
                        (theta[:-1] + nominal_theta_eq) * (theta[1:] + nominal_theta_eq) < 0.0
                    )),
                }
        launch_diagnostics.append(entry)
    cruise_segments = []
    for transition_event in (
        item for item in transition_events if "transition_to_hold_time_s" in item
    ):
        event_index = int(transition_event["schedule_event_index"])
        end_s = next_event_end_s(
            schedule, event_index, float(scenario["duration_s"])
        )
        mask = (
            (
                arrays["control_time"]
                >= transition_event["transition_to_hold_time_s"] - 1e-12
            )
            & (arrays["control_time"] < end_s - 1e-12)
            & (arrays["phase"] == VelocityLifecyclePhase.VELOCITY_HOLD.value)
        )
        e_p = position_error[mask]
        e_v = velocity_error[mask]
        if not np.any(mask):
            # Nominal lean may finish exactly at the next scheduled command.
            # Do not manufacture a positive-duration HOLD/cruise measurement.
            continue
        cruise_segments.append(
            {
                "target_velocity_m_s": transition_event[
                    "accepted_target_velocity_m_s"
                ],
                "feedforward_rearmed": transition_event["feedforward_rearmed"],
                "start_time_s": transition_event["transition_to_hold_time_s"],
                "end_time_s": end_s,
                "duration_s": end_s
                - transition_event["transition_to_hold_time_s"],
                "sample_count": int(np.count_nonzero(mask)),
                "v_ref_m_s": metric_triplet(arrays["reference"][mask, 1]),
                "v_hat_m_s": metric_triplet(arrays["plant"][mask, 1]),
                "e_v_m_s": metric_triplet(e_v),
                "e_p_m": metric_triplet(e_p),
                "position_channel_correction_nm": metric_triplet(
                    position_feedback_coefficient * e_p
                ),
                "u_ff_sum_nm": metric_triplet(arrays["u_ff"][mask]),
                "u_fb_sum_nm": metric_triplet(arrays["u_fb"][mask]),
                "u_requested_sum_nm": metric_triplet(arrays["requested"][mask]),
            }
        )

    transition_torque = []
    post_fade_lean_tails = []
    nominal_reference_max_deviation = 0.0
    for item, event in zip(transition_plans, transition_events):
        start_interval = round(item["command_time_s"] / dt_s)
        count = len(item["plan"].feedforward_inputs_nm)
        nominal_reference_max_deviation = max(
            nominal_reference_max_deviation,
            float(np.max(np.abs(
                arrays["reference"][start_interval : start_interval + count]
                - item["plan"].reference_states[1:]
            ))),
        )
        fade_end = event.get("feedforward_fade_completed_time_s")
        if fade_end is None:
            continue
        transient_mask = (
            (arrays["control_time"] >= item["command_time_s"] - 1e-12)
            & (arrays["control_time"] < fade_end - 1e-12)
        )
        tail_mask = (
            (arrays["control_time"] >= fade_end - 1e-12)
            & (arrays["control_time"] < item["segment_end_s"] - 1e-12)
        )
        transition_torque.append({
            "target_velocity_m_s": event["accepted_target_velocity_m_s"],
            "window": "command_to_feedforward_fade_complete",
            "start_time_s": item["command_time_s"],
            "end_time_s": fade_end,
            "u_ff_sum_nm": metric_triplet(arrays["u_ff"][transient_mask]),
            "u_fb_sum_nm": metric_triplet(arrays["u_fb"][transient_mask]),
            "u_requested_sum_nm": metric_triplet(arrays["requested"][transient_mask]),
        })
        if np.any(tail_mask):
            post_fade_lean_tails.append({
                "target_velocity_m_s": event["accepted_target_velocity_m_s"],
                "start_time_s": fade_end,
                "end_time_s": item["segment_end_s"],
                "sample_count": int(np.count_nonzero(tail_mask)),
                "u_ff_sum_nm": metric_triplet(arrays["u_ff"][tail_mask]),
            })

    transient_masks = []
    for event in transition_events:
        fade_end = event.get("feedforward_fade_completed_time_s")
        if fade_end is None:
            continue
        transient_masks.append(
            (arrays["control_time"] >= event["command_time_s"] - 1e-12)
            & (arrays["control_time"] <= fade_end + 1e-12)
        )
    transient_mask = (
        np.logical_or.reduce(transient_masks)
        if transient_masks else np.zeros(len(arrays["time"]), dtype=bool)
    )

    result = {
        "scenario": scenario["name"],
        "mode": scenario["mode"],
        "case": config["case"],
        "controller_mode": controller_mode,
        "controller_parameters": (
            asdict(pid.config) if pid is not None else {"K4": K4.tolist()}
        ),
        "controller_input_semantics": "x_plant_sensorized - x_ref; state order [p, v, pitch_error, pitch_rate]",
        "feedforward_actuator_enabled": feedforward_enabled,
        "lambda_ff": lambda_ff,
        "nominal_model": {
            "A_used": A.tolist(), "B_unchanged": B.tolist(),
            "runtime_velocity_column_patch_applied": False,
            "scope": "dynamic nominal solver and feedforward only; K4 unchanged",
        },
        "benchmark_configuration": {
            "scenario": scenario,
            "payload_mode": "empty",
            "physics_dt_s": sim.physics_dt,
            "controller_dt_s": dt_s,
            "physics_steps_per_update": physics_steps,
            "imu_rng_seed": int(motion_config["imu_rng_seed"]),
            "reference_limits": motion_config["reference_limits"],
            "max_jerk_m_s3": dynamic_config["max_jerk_m_s3"],
            "reference_lifecycle": lifecycle_raw,
            "estimator": asdict(estimator_config),
            "wheel_torque_limit_nm": peak_per_wheel,
            "disturbance_rejection_enabled": False,
            "source_sha256": {
                str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (
                    CONFIG_PATH, DYNAMIC_CONFIG_PATH, MOTION_CONFIG_PATH,
                    EXPERIMENT_CONFIG_PATH, DR_CONFIG_PATH, REDUCED_PATH,
                    OFFLINE_PATH, PLANT_PARAMETERS_PATH, sim.model_path,
                    sim.encoder_profile_path, sim.imu_hardware_config_path,
                    MODEL_DIR / "longitudinal_estimator_config.json",
                )
            },
        },
        "fell": bool(fallen),
        "tracking": {
            "position_error_m": metric_triplet(position_error),
            "velocity_error_m_s": metric_triplet(velocity_error),
            "endpoint_position_error_m": float(position_error[-1]),
            "endpoint_velocity_error_m_s": float(velocity_error[-1]),
            "maximum_position_overshoot_m": None,
            "settled_transition_count": sum(
                item["settled_time_s"] is not None for item in transients
            ),
            "transition_count": len(transients),
            "transients": transients,
        },
        "pitch": {
            "rms_deg": math.degrees(rms(arrays["gt"][:, 2])),
            "peak_abs_deg": math.degrees(float(np.max(np.abs(arrays["gt"][:, 2])))),
            "terminal_rms_deg": math.degrees(rms(arrays["gt"][terminal, 2])),
            "tracking_rms_deg": math.degrees(
                rms(arrays["gt"][:, 2] - theta_reference)
            ),
        },
        "transient_performance": {
            "window_definition": "union of each command time through its completed feedforward fade",
            "sample_rate_hz": 1.0 / dt_s,
            "sample_count": int(np.count_nonzero(transient_mask)),
            "velocity_error_m_s": metric_triplet(velocity_error[transient_mask]),
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
            "raw_feedforward_sum_nm": metric_triplet(arrays["u_ff_raw"]),
            "lifecycle_feedforward_sum_nm": metric_triplet(arrays["u_ff_lifecycle"]),
            "requested_sum_nm": metric_triplet(arrays["requested"]),
            "software_limited_sum_nm": metric_triplet(arrays["held"]),
            "actual_applied_sum_nm": metric_triplet(arrays["actual"]),
            "requested_first_difference_rms_nm": rms(
                np.diff(arrays["requested"])
            ),
            "wheel_saturation_fraction": saturated_updates / interval_count,
            "FF_FB_cancellation": {
                "definition": "1 - mean(abs(u_total)) / mean(abs(u_ff_used) + abs(u_fb)); larger means more opposing cancellation",
                "absolute_cancellation_fraction": float(1.0 - np.mean(np.abs(arrays["requested"])) / np.mean(
                    np.abs(arrays["u_ff"]) + np.abs(arrays["u_fb"])
                )),
                "opposing_sign_fraction": float(np.mean(arrays["u_ff"] * arrays["u_fb"] < 0.0)),
            },
        },
        "launch_diagnostics": {
            "angle_convention": "pitch error relative to nominal theta_eq; absolute-pitch zero crossing is recorded separately",
            "velocity_dip_definition": "max(0, sign(v_cmd)*(v_cmd-v)) in [T_v, T_v+0.4s)",
            "sample_frequency_hz": 1.0 / dt_s,
            "transitions": launch_diagnostics,
        },
        "feedforward_feasibility": {
            "normalized_residual_rms": float(
                np.sqrt(np.mean(normalized_transition_residuals**2))
            ),
            "normalized_residual_step_peak": float(
                np.max(
                    np.sqrt(
                        np.mean(normalized_transition_residuals**2, axis=1)
                    )
                )
            ),
            "projection_vs_closed_form_u_ff_peak_difference_nm": max(
                item["projection_input_difference_nm"] for item in transition_plans
            ),
            "segment_diagnostics": [
                diagnostic
                for item in transition_plans
                for diagnostic in item["projection_diagnostics"]
            ],
        },
        "velocity_lifecycle": {
            "configuration": lifecycle_raw,
            "position_feedback_coefficient_nm_per_m": position_feedback_coefficient,
            "transition_events": transition_events,
            "handoffs": handoffs,
            "transition_torque": transition_torque,
            "post_fade_lean_tails": post_fade_lean_tails,
            "original_nominal_reference_peak_deviation": nominal_reference_max_deviation,
            "all_post_fade_u_ff_strictly_zero": all(
                tail["u_ff_sum_nm"]["peak_abs"] == 0.0
                for tail in post_fade_lean_tails
            ),
            "cruise_segments": cruise_segments,
            "all_cruise_u_ff_strictly_zero": all(
                segment["u_ff_sum_nm"]["peak_abs"] == 0.0
                for segment in cruise_segments
            ),
        },
        "disturbance_rejection_enabled": False,
        "all_logged_values_finite": bool(all(
            np.isfinite(value).all() for value in arrays.values()
            if np.issubdtype(value.dtype, np.number)
        )),
        "history_50hz": history,
    }
    if acceleration_compensation is not None:
        result["pitch_filter_experiment"] = {
            "mode": pitch_filter_mode,
            "parameters": asdict(acceleration_compensation),
            "default_frozen_estimator_unchanged": True,
            "gt_used_by_estimator_or_controller": False,
            "acceleration_source": "previous control-time odometry velocity difference, first-order low-pass; no a_ref",
            "acceleration_gain_factor": "1 / (1 + (a_odom_lpf / adaptive_acceleration_scale_m_s2)^2)",
            "compensation_frame": "world -Y longitudinal acceleration, rotated to chassis with gyro-predicted pitch",
            "upstream_responsibilities_adapted": [
                "https://github.com/CCNYRoboticsLab/imu_tools/blob/rolling/imu_complementary_filter/src/complementary_filter.cpp",
                "https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS_DCM.cpp",
            ],
            "implementation_sha256": {
                str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (Path(__file__), ROOT / "sim" / "longitudinal_estimation.py")
            },
        }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller-mode", choices=("pid", "lqr", "lqr_ff"), default="lqr_ff")
    parser.add_argument("--results-path", type=Path)
    parser.add_argument("--pitch-filter", choices=("current", "acceleration_compensated"))
    parser.add_argument("--lambda-ff", type=float)
    args = parser.parse_args()
    manifest, config, dynamic_config, motion_config, offline = load_production_baseline()
    if args.lambda_ff is None:
        args.lambda_ff = float(config["lambda_ff"])
    if not 0.0 <= args.lambda_ff <= 1.0:
        raise ValueError("lambda_ff must be in [0, 1]")
    if args.pitch_filter is None:
        args.pitch_filter = manifest["estimator"]["pitch_filter_mode"]
    config["controller_mode"] = args.controller_mode
    config["pitch_filter_mode"] = args.pitch_filter
    config["lambda_ff"] = args.lambda_ff
    if args.results_path is None:
        args.results_path = MODEL_DIR / "stage3" / "results" / "stage3a_longitudinal_baseline_results.json"
    experiment = load_json(EXPERIMENT_CONFIG_PATH)
    dr_raw = load_json(DR_CONFIG_PATH)
    reduced = load_json(REDUCED_PATH)
    plant = load_json(PLANT_PARAMETERS_PATH)
    scenario = next(
        item
        for item in motion_config["scenarios"]
        if item["name"] == config["scenario"]
    )
    print(f"RUN empty/{config['case']}/{config['scenario']}", flush=True)
    try:
        run = run_case(
            scenario, config, dynamic_config, motion_config, experiment,
            dr_raw, reduced, offline, plant,
        )
    except RuntimeError as error:
        if not str(error).startswith("velocity maneuver infeasible"):
            raise
        args.results_path.write_text(json.dumps({
            "status": "INFEASIBLE", "reason": str(error),
            "scenario": scenario, "lambda_ff": args.lambda_ff,
            "horizon_search": config["velocity_lifecycle"]["horizon_search"],
        }, indent=2), encoding="utf-8")
        raise
    result = {
        "stage": config["stage"],
        "status": "COMPLETE",
        "controller_mode": args.controller_mode,
        "lambda_ff": args.lambda_ff,
        "research_question": "Canonical Stage 3A longitudinal baseline regression",
        "frozen_invariants": {
            "K4": offline["identified_lqr"]["K_id"],
            "A_identified": offline["fit"]["A_identified"],
            "B_identified": offline["fit"]["B_identified"],
            "s_curve_unchanged": True,
            "dynamic_nominal_feedforward_formula_unchanged": True,
            "feedforward_rearm_threshold_s": config["velocity_lifecycle"][
                "feedforward_rearm_threshold_s"
            ],
            "position_lifecycle_unchanged": True,
            "estimator_unchanged": True,
            "disturbance_rejection_enabled": False,
            "wheel_torque_limit_nm": plant["known"]["wheel_torque_hard_peak_nm"],
        },
        "new": compact_summary(run),
        "run": run,
        "baseline_manifest": manifest,
    }
    if args.pitch_filter == "acceleration_compensated":
        result["pitch_filter_mode"] = args.pitch_filter
        result["frozen_invariants"]["estimator_unchanged"] = True
        result["frozen_invariants"]["default_frozen_estimator_unchanged"] = True
        result["baseline_runtime"] = {
            "dynamic_nominal_horizon": config["velocity_lifecycle"]["horizon_search"],
            "feedforward_actuator_contribution": "lambda_ff * u_ff_lifecycle; feedback and lean reference unchanged",
            "hold_feedforward_nm": 0.0,
            "dwell_threshold_unchanged": True,
            "unsupported_mid_maneuver_retarget": "nonzero-acceleration/nonzero-lean starts are not supported by the frozen shaper/zero-boundary solver; never reset them silently; fixed benchmark does not exercise this",
        }
    args.results_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"WROTE {args.results_path}", flush=True)
    print(
        json.dumps(
            {
                "status": result["status"],
                "handoffs": run["velocity_lifecycle"]["handoffs"],
                "transition_events": run["velocity_lifecycle"][
                    "transition_events"
                ],
                "new": result["new"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
