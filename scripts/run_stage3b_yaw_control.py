"""Stage 3B parallel yaw-PD tuning and fixed longitudinal integration tests."""

from __future__ import annotations

import argparse
import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from control import (
    GyroRelativeYawEstimator,
    ParallelYawPD,
    VelocityLifecyclePhase,
    VelocityReferenceLifecycle,
    allocate_longitudinal_priority,
    wrap_to_pi,
)
from sim import LongitudinalEstimator, load_longitudinal_estimator_config
from sim.longitudinal_estimation import LongitudinalAccelerationCompensationConfig
from moving_payload_benchmark import (
    force_bearing_contact_episodes,
    has_chassis_floor_contact,
    held_initial_imu,
    set_payload_initial_longitudinal_state,
)
import run_stage3a_velocity_feedforward_handoff as longitudinal
from run_stage3a_commanded_motion import build_sim, rms
from run_stage3a_dynamic_nominal_reference import load_json


MODEL_DIR = ROOT / "models" / "minisegway"
RESULT_PATH = MODEL_DIR / "stage3" / "results" / "stage3b_yaw_control_results.json"

# First pass only.  Up to three refine candidates may be appended after the
# coarse response is inspected; the script never permits more than six total.
TUNING_CANDIDATES = [
    {"name": "C1", "K_psi_nm_per_rad": 0.10, "K_r_nm_per_rad_s": 0.05},
    {"name": "C2", "K_psi_nm_per_rad": 0.25, "K_r_nm_per_rad_s": 0.10},
    {"name": "C3", "K_psi_nm_per_rad": 0.40, "K_r_nm_per_rad_s": 0.15},
    {"name": "C4", "K_psi_nm_per_rad": 0.55, "K_r_nm_per_rad_s": 0.20},
    {"name": "C5", "K_psi_nm_per_rad": 0.40, "K_r_nm_per_rad_s": 0.25},
]

MIXED_SCENARIO = {
    "name": "mixed_steering",
    "duration_s": 14.0,
    "linear_velocity_schedule": [
        {"time_s": 0.0, "command": 0.0},
        {"time_s": 1.0, "command": 0.4},
        {"time_s": 12.0, "command": 0.0},
    ],
    "yaw_rate_schedule": [
        {"time_s": 0.0, "command": 0.0},
        {"time_s": 4.0, "command": 0.5},
        {"time_s": 6.0, "command": 0.0},
        {"time_s": 8.0, "command": -0.5},
        {"time_s": 10.0, "command": 0.0},
    ],
}

MISMATCH_SCENARIO = {
    "name": "straight_motor_response_mismatch",
    "duration_s": 12.0,
    "linear_velocity_schedule": [
        {"time_s": 0.0, "command": 0.0},
        {"time_s": 1.0, "command": 0.4},
        {"time_s": 10.0, "command": 0.0},
    ],
    "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
}

MISMATCH = {
    "enabled": True,
    "model": "experiment-only exact-discrete first-order torque tracking lag",
    "left_time_constant_s": 0.100,
    "right_time_constant_s": 0.130,
    "relative_difference": 0.30,
}


def metric(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"mean": None, "rms": None, "peak_abs": None}
    return {
        "mean": float(np.mean(values)),
        "rms": rms(values),
        "peak_abs": float(np.max(np.abs(values))),
    }


def root_yaw_rad(sim) -> float:
    root = sim.model.joint("root").id
    adr = int(sim.model.jnt_qposadr[root])
    w, x, y, z = sim.data.qpos[adr + 3 : adr + 7]
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def root_lateral_position_m(sim) -> float:
    root = sim.model.joint("root").id
    adr = int(sim.model.jnt_qposadr[root])
    return float(sim.data.qpos[adr])


def root_forward_velocity_m_s(sim) -> float:
    """Project GT world velocity onto chassis-local forward (-Y)."""

    root = sim.model.joint("root").id
    dof_adr = int(sim.model.jnt_dofadr[root])
    chassis = sim.model.body("chassis").id
    local_to_world = np.asarray(sim.data.xmat[chassis], dtype=float).reshape(3, 3)
    forward_world = -local_to_world[:, 1]
    world_velocity = np.asarray(
        sim.data.qvel[dof_adr : dof_adr + 3], dtype=float
    )
    return float(forward_world @ world_velocity)


class ExperimentMotorLag:
    """Local validation-only actuator wrapper; disabled means exact pass-through."""

    def __init__(self, enabled: bool, physics_dt_s: float) -> None:
        self.enabled = bool(enabled)
        self.dt_s = float(physics_dt_s)
        self.state_nm = np.zeros(2, dtype=float)
        self.time_constants_s = np.asarray([
            MISMATCH["left_time_constant_s"],
            MISMATCH["right_time_constant_s"],
        ])

    def command(self, requested_nm: np.ndarray) -> np.ndarray:
        requested = np.asarray(requested_nm, dtype=float)
        if not self.enabled:
            self.state_nm = requested.copy()
            return requested
        alpha = 1.0 - np.exp(-self.dt_s / self.time_constants_s)
        self.state_nm += alpha * (requested - self.state_nm)
        return self.state_nm.copy()


def next_command(schedule: list[dict], index: int, duration_s: float) -> float:
    return (
        float(schedule[index + 1]["time_s"])
        if index + 1 < len(schedule) else float(duration_s)
    )


def settling_after_yaw_stops(
    times: np.ndarray,
    r_cmd: np.ndarray,
    heading_error_gt: np.ndarray,
    r_gt: np.ndarray,
    schedule: list[dict],
    dwell_s: float = 0.30,
) -> list[dict]:
    dt_s = float(np.median(np.diff(times)))
    dwell_samples = max(1, round(dwell_s / dt_s))
    output = []
    for index, event in enumerate(schedule):
        if index == 0 or abs(float(event["command"])) > 1e-12:
            continue
        if abs(float(schedule[index - 1]["command"])) <= 1e-12:
            continue
        start = float(event["time_s"])
        end = next_command(schedule, index, float(times[-1] + dt_s))
        indices = np.flatnonzero((times >= start - 1e-12) & (times < end - 1e-12))
        settled_time = None
        for offset in range(0, max(0, len(indices) - dwell_samples + 1)):
            window = indices[offset : offset + dwell_samples]
            if np.all(np.abs(heading_error_gt[window]) <= 0.05) and np.all(
                np.abs(r_gt[window]) <= 0.05
            ):
                settled_time = float(times[window[-1]])
                break
        local_error = heading_error_gt[indices]
        crossings = int(np.count_nonzero(local_error[:-1] * local_error[1:] < 0.0))
        output.append({
            "stop_time_s": start,
            "next_command_time_s": end,
            "settled_time_s": settled_time,
            "settling_after_stop_s": (
                None if settled_time is None else settled_time - start
            ),
            "heading_error_zero_crossings": crossings,
            "sustained_oscillation": crossings > 6,
        })
    return output


def run_case(
    scenario: dict,
    K_psi: float,
    K_r: float,
    *,
    yaw_enabled: bool,
    motor_mismatch_enabled: bool,
    common: tuple,
    keep_history: bool,
    payload_mode: str = "empty",
    use_accepted_free_payload_initial_state: bool = False,
    common_mode_augmentation=None,
    physics_step_callback=None,
    simulation_setup_callback=None,
    command_source=None,
    empty_model_path: Path | None = None,
    model_path_override: Path | None = None,
    history_diagnostic_callback=None,
    termination_guard=None,
    equilibrium_reference_callback=None,
    equilibrium_input_callback=None,
    control_observer=None,
    motion_limit_scale_callback=None,
    reference_source=None,
    reference_context_callback=None,
    mcu_local_state_callback=None,
    pre_reference_tick_callback=None,
) -> dict:
    (
        manifest, config, dynamic_config, motion_config, offline,
        experiment, dr_raw, reduced, plant,
    ) = common
    config = copy.deepcopy(config)
    dynamic_config = copy.deepcopy(dynamic_config)
    motion_config = copy.deepcopy(motion_config)
    dt_s = float(motion_config["controller_dt_s"])
    physics_steps = int(experiment["physics_steps_per_update"])
    interval_count = round(float(scenario["duration_s"]) / dt_s)
    A = np.asarray(offline["fit"]["A_identified"], dtype=float)
    B = np.asarray(offline["fit"]["B_identified"], dtype=float)
    state_scales = np.asarray(offline["fit"]["state_scales"], dtype=float)
    input_scale = float(offline["fit"]["input_scale_nm"])
    K4 = np.asarray(offline["identified_lqr"]["K_id"], dtype=float)
    lambda_ff = float(config["lambda_ff"])
    lifecycle_config = config["velocity_lifecycle"]
    lifecycle = VelocityReferenceLifecycle(
        dt_s, float(lifecycle_config["feedforward_fade_s"])
    )
    linear_schedule = scenario["linear_velocity_schedule"]
    yaw_schedule = scenario["yaw_rate_schedule"]
    lifecycle.reset(0.0, float(linear_schedule[0]["command"]))
    if reference_source is not None and command_source is not None:
        raise ValueError("reference_source and command_source are mutually exclusive")

    nominal_theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    theta_eq = nominal_theta_eq
    sim, payload_setup, payload_recorder = build_sim(
        payload_mode,
        int(motion_config["imu_rng_seed"]),
        nominal_theta_eq,
        dr_raw,
        empty_model_path=empty_model_path,
        model_path_override=model_path_override,
    )
    if simulation_setup_callback is not None:
        setup_override = simulation_setup_callback(sim)
        payload_setup = {
            **(payload_setup or {}),
            "mechanical_override": setup_override,
        }
        if (
            payload_setup.get("box") is not None
            and setup_override is not None
            and "payload_mass_kg" in setup_override
            and "payload_diagonal_inertia_kg_m2" in setup_override
        ):
            payload_setup["box"].update({
                "mass_kg": setup_override["payload_mass_kg"],
                "diagonal_inertia_kg_m2": setup_override[
                    "payload_diagonal_inertia_kg_m2"
                ],
            })
    if use_accepted_free_payload_initial_state:
        if payload_mode != "free":
            raise ValueError("accepted free-payload initial state requires payload_mode='free'")
        acceptance = dr_raw["collision_acceptance"]
        full_size = np.asarray(acceptance["payload_full_size_m"], dtype=float)
        payload_initial = set_payload_initial_longitudinal_state(
            sim,
            relative_position_m=float(
                acceptance["initial_payload_longitudinal_center_m"]
            ),
            relative_velocity_m_s=float(
                acceptance["initial_payload_longitudinal_velocity_m_s"]
            ),
            relative_vertical_position_m=0.141 + 0.5 * float(full_size[2]),
        )
        payload_setup = {**(payload_setup or {}), "initial_state": payload_initial}
    if not math.isclose(physics_steps * sim.physics_dt, dt_s, abs_tol=1e-12):
        raise RuntimeError("frozen physics/controller timing changed")
    estimator = LongitudinalEstimator(
        load_longitudinal_estimator_config(),
        sim.encoder_profile,
        float(reduced["parameters"]["wheel_radius_m"]),
        acceleration_compensation=LongitudinalAccelerationCompensationConfig(),
    )
    accel, gyro, measurement_age, extrapolation_allowed = held_initial_imu(sim)
    estimate = estimator.reset(
        accel, gyro, sim.wheel_encoder_counts(),
        measurement_age_s=measurement_age,
        allow_kinematic_extrapolation=extrapolation_allowed,
    )
    readout = sim.last_imu_readout
    assert readout is not None
    yaw_estimator = GyroRelativeYawEstimator()
    yaw_estimate = yaw_estimator.reset(
        float(gyro[2]), float(readout.packet.sample_time_s), measurement_age,
        allow_extrapolation=extrapolation_allowed,
    )
    psi_ref = yaw_estimate.psi_control_rad
    yaw_pd = ParallelYawPD(K_psi, K_r)
    limit = float(plant["known"]["wheel_torque_hard_peak_nm"])
    motor_lag = ExperimentMotorLag(motor_mismatch_enabled, sim.physics_dt)

    linear_index = 1
    yaw_index = 1
    v_cmd = float(linear_schedule[0]["command"])
    r_cmd = float(yaw_schedule[0]["command"])
    transition_events = []
    previous_gt_yaw = root_yaw_rad(sim)
    gt_psi = 0.0
    fallen = False
    logs: dict[str, list] = {name: [] for name in (
        "time", "v_cmd", "r_cmd", "reference", "plant", "gt_long",
        "psi_ref", "psi_hat", "r_hat", "psi_gt", "r_gt", "lateral_gt",
        "u_base", "u_eq", "u_sum_request", "u_sum", "u_ff", "u_fb", "u_diff_request",
        "u_diff_used", "u_left", "u_right", "actual_left", "actual_right",
        "diff_clipped", "guard_clipped", "ff_phase", "velocity_phase",
    )}
    history = []
    history_period = round(
        1.0 / (dt_s * float(motion_config["history_frequency_hz"]))
    )
    payload_times: list[float] = []
    payload_positions: list[np.ndarray] = []
    payload_contacts: list[bool] = []
    payload_forces: list[float] = []
    termination_reason: str | None = None

    def begin_velocity_transition(target_velocity_m_s: float, time_s: float) -> None:
        transition_motion_config = copy.deepcopy(motion_config)
        acceleration_scale = 1.0
        if motion_limit_scale_callback is not None:
            acceleration_scale = float(motion_limit_scale_callback({
                "sim": sim,
                "time_s": time_s,
                "target_velocity_m_s": target_velocity_m_s,
                "reference_state": lifecycle.reference_state.copy(),
                "reference_acceleration_m_s2": lifecycle.reference_acceleration_m_s2,
                "nominal_acceleration_limit_m_s2": float(
                    motion_config["reference_limits"]["max_acceleration_m_s2"]
                ),
            }))
            if not math.isfinite(acceleration_scale) or not 0.0 < acceleration_scale <= 1.0:
                raise ValueError("motion limit scale must be finite in (0, 1]")
            transition_motion_config["reference_limits"][
                "max_acceleration_m_s2"
            ] *= acceleration_scale
        transition = longitudinal.build_velocity_transition_plan(
            lifecycle.reference_state, target_velocity_m_s, dynamic_config,
            transition_motion_config, A, B, state_scales, input_scale,
            float(lifecycle_config["feedforward_rearm_threshold_s"]),
            lifecycle_config["horizon_search"],
            start_acceleration_m_s2=lifecycle.reference_acceleration_m_s2,
        )
        lifecycle.begin_transition(transition["plan"])
        transition_events.append({
            "command_time_s": time_s,
            "target_velocity_m_s": target_velocity_m_s,
            "acceleration_limit_scale": acceleration_scale,
            "T_v_s": transition["planned_transient_duration_s"],
            "T_full_s": transition["dynamic_nominal_horizon_s"],
            "normalized_residual_rms": transition["normalized_residual_rms"],
        })

    for interval in range(interval_count):
        control_time = float(sim.data.time)
        rolling_reference_command = None
        if reference_source is not None:
            if mcu_local_state_callback is not None:
                mcu_local_state_callback(control_time, estimate, yaw_estimate)
            if pre_reference_tick_callback is not None:
                pre_reference_tick_callback(control_time)
            if reference_context_callback is not None:
                reference_context_callback(control_time, estimate)
            rolling_reference_command = reference_source.command(control_time)
            v_cmd = float(
                rolling_reference_command.linear_velocity_target_m_s
            )
            next_r_cmd = float(rolling_reference_command.yaw_rate_target_rad_s)
            if not math.isclose(next_r_cmd, r_cmd, abs_tol=1e-12):
                r_cmd = next_r_cmd
                yaw_schedule.append({"time_s": control_time, "command": r_cmd})
        elif command_source is not None:
            next_v_cmd, next_r_cmd = command_source(sim, control_time)
            next_v_cmd = float(next_v_cmd)
            next_r_cmd = float(next_r_cmd)
            if not np.isfinite([next_v_cmd, next_r_cmd]).all():
                raise ValueError("runtime v/w command source returned non-finite data")
            if not math.isclose(next_v_cmd, v_cmd, abs_tol=1e-12):
                # The frozen lifecycle does not accept mid-plan retargeting.
                # A safety cap is queued until the active jerk-limited plan
                # reaches HOLD; balance/reference continuity takes priority.
                if not (
                    motion_limit_scale_callback is not None
                    and lifecycle.phase == VelocityLifecyclePhase.VELOCITY_TRANSIENT
                ):
                    v_cmd = next_v_cmd
                    begin_velocity_transition(v_cmd, control_time)
            if not math.isclose(next_r_cmd, r_cmd, abs_tol=1e-12):
                r_cmd = next_r_cmd
                yaw_schedule.append({"time_s": control_time, "command": r_cmd})
        else:
            if linear_index < len(linear_schedule) and control_time + 1e-12 >= float(
                linear_schedule[linear_index]["time_s"]
            ):
                v_cmd = float(linear_schedule[linear_index]["command"])
                begin_velocity_transition(v_cmd, control_time)
                linear_index += 1
            if yaw_index < len(yaw_schedule) and control_time + 1e-12 >= float(
                yaw_schedule[yaw_index]["time_s"]
            ):
                r_cmd = float(yaw_schedule[yaw_index]["command"])
                yaw_index += 1

        if rolling_reference_command is not None:
            x_ref = rolling_reference_command.reference_state
            u_ff_raw = rolling_reference_command.u_ff_raw_nm
            u_ff_after_lifecycle = (
                rolling_reference_command.u_ff_after_lifecycle_nm
            )
            feedforward_phase = rolling_reference_command.feedforward_phase
            velocity_phase = rolling_reference_command.velocity_phase
        else:
            lifecycle_command = lifecycle.command(control_time)
            x_ref = lifecycle_command.reference_state
            u_ff_raw = lifecycle.raw_feedforward_nm
            u_ff_after_lifecycle = lifecycle_command.feedforward_nm
            feedforward_phase = lifecycle_command.feedforward_phase
            velocity_phase = lifecycle_command.phase.value
        observer_output = (
            control_observer.output({
                "sim": sim,
                "time_s": control_time,
                "estimate": estimate,
                "reference_state": x_ref.copy(),
                "nominal_theta_eq_rad": nominal_theta_eq,
            })
            if control_observer is not None else {}
        )
        if equilibrium_reference_callback is not None:
            theta_eq = float(equilibrium_reference_callback({
                "sim": sim,
                "time_s": control_time,
                "estimate": estimate,
                "reference_state": x_ref.copy(),
                "nominal_theta_eq_rad": nominal_theta_eq,
                "control_observer_output": observer_output,
            }))
            if not math.isfinite(theta_eq):
                raise ValueError("runtime equilibrium reference must be finite")
        x_plant = estimate.plant_state(theta_eq)
        error = x_plant - x_ref
        u_fb = -float((K4 @ error).item())
        u_ff = lambda_ff * u_ff_after_lifecycle
        u_eq = 0.0
        if equilibrium_input_callback is not None:
            u_eq = float(equilibrium_input_callback({
                "sim": sim,
                "time_s": control_time,
                "estimate": estimate,
                "reference_state": x_ref.copy(),
                "nominal_theta_eq_rad": nominal_theta_eq,
                "theta_eq_rad": theta_eq,
                "control_observer_output": observer_output,
            }))
            if not math.isfinite(u_eq):
                raise ValueError("runtime equilibrium input must be finite")
        u_base = u_eq + u_fb + u_ff
        augmentation_command = (
            common_mode_augmentation.command(u_base, control_time)
            if common_mode_augmentation is not None else {}
        )
        u_sum_request = u_base + float(
            augmentation_command.get("u_Q_used_nm", 0.0)
        )
        u_sum = float(np.clip(u_sum_request, -2.0 * limit, 2.0 * limit))
        yaw_command = yaw_pd.command(
            psi_ref, r_cmd, yaw_estimate.psi_control_rad,
            yaw_estimate.r_hat_rad_s,
        )
        u_diff_request = yaw_command.u_diff_request_nm if yaw_enabled else 0.0
        allocation = allocate_longitudinal_priority(u_sum, u_diff_request, limit)

        actual_wheels = []
        next_gt_long = None
        requested_wheels = np.asarray([
            allocation.left_torque_nm, allocation.right_torque_nm
        ])
        for _ in range(physics_steps):
            motor_torque = motor_lag.command(requested_wheels)
            snapshot = sim.step(float(motor_torque[0]), float(motor_torque[1]))
            if physics_step_callback is not None:
                physics_step_callback(sim)
            actual_wheels.append(snapshot.applied_ctrl_nm.copy())
            next_gt_long = sim.longitudinal_state(theta_eq)
            next_gt_long[1] = root_forward_velocity_m_s(sim)
            if payload_recorder is not None:
                payload_position, _, payload_contact, payload_force = (
                    payload_recorder.sample(
                        snapshot.time_s,
                        float(next_gt_long[2]),
                        float(np.max(np.abs(snapshot.applied_ctrl_nm))),
                    )
                )
                payload_times.append(float(snapshot.time_s))
                payload_positions.append(payload_position.copy())
                payload_contacts.append(payload_contact)
                payload_forces.append(payload_force)
            fallen |= abs(float(next_gt_long[2])) >= math.radians(
                float(experiment["benchmark"]["fall_pitch_error_deg"])
            )
            fallen |= has_chassis_floor_contact(sim)
        assert next_gt_long is not None
        actual = np.mean(np.asarray(actual_wheels), axis=0)

        estimate_before_step = estimate
        accel, gyro, measurement_age, extrapolation_allowed = sim.imu_estimator_input()
        estimate = estimator.update(
            accel, gyro, sim.wheel_encoder_counts(),
            measurement_age_s=measurement_age,
            allow_kinematic_extrapolation=extrapolation_allowed,
        )
        readout = sim.last_imu_readout
        assert readout is not None
        yaw_estimate = yaw_estimator.update(
            float(gyro[2]), float(readout.packet.sample_time_s), measurement_age,
            valid_new_sample=not (readout.imu_invalid or readout.imu_stale),
            allow_extrapolation=extrapolation_allowed,
        )
        if reference_source is None:
            lifecycle.advance()
        psi_ref += r_cmd * dt_s
        next_reference = (
            lifecycle.reference_state
            if reference_source is None
            else reference_source.next_reference_state
        )
        current_gt_yaw = root_yaw_rad(sim)
        delta_gt_yaw = wrap_to_pi(current_gt_yaw - previous_gt_yaw)
        gt_psi += delta_gt_yaw
        r_gt = delta_gt_yaw / dt_s
        previous_gt_yaw = current_gt_yaw
        time_s = float(sim.data.time)
        observer_observation = (
            control_observer.observe({
                "sim": sim,
                "time_s": time_s,
                "dt_s": dt_s,
                "estimate_before": estimate_before_step,
                "estimate_after": estimate,
                "actual_wheels_nm": actual.copy(),
                "requested_wheels_nm": requested_wheels.copy(),
                "accelerometer_m_s2": np.asarray(accel, dtype=float).copy(),
                "gyro_rad_s": np.asarray(gyro, dtype=float).copy(),
                "saturated": bool(
                    allocation.final_guard_clipped
                    or abs(u_sum_request - u_sum) > 1e-12
                ),
                "reference_state": next_reference.copy(),
                "reference_acceleration_m_s2": float(
                    lifecycle.reference_acceleration_m_s2
                    if reference_source is None
                    else reference_source.next_reference_acceleration_m_s2
                ),
                "nominal_theta_eq_rad": nominal_theta_eq,
            })
            if control_observer is not None else {}
        )
        augmentation_observation = (
            common_mode_augmentation.observe(
                x_plant,
                float(np.sum(actual)),
                estimate.plant_state(theta_eq),
                time_s,
                actual.copy(),
            )
            if common_mode_augmentation is not None else {}
        )

        row = {
            "time": time_s,
            "v_cmd": v_cmd,
            "r_cmd": r_cmd,
            "reference": next_reference.copy(),
            "plant": estimate.plant_state(theta_eq),
            "gt_long": next_gt_long.copy(),
            "psi_ref": psi_ref,
            "psi_hat": yaw_estimate.psi_control_rad,
            "r_hat": yaw_estimate.r_hat_rad_s,
            "psi_gt": gt_psi,
            "r_gt": r_gt,
            "lateral_gt": root_lateral_position_m(sim),
            "u_base": u_base,
            "u_eq": u_eq,
            "u_sum_request": u_sum_request,
            "u_sum": u_sum,
            "u_ff": u_ff,
            "u_fb": u_fb,
            "u_diff_request": u_diff_request,
            "u_diff_used": allocation.u_diff_used_nm,
            "u_left": allocation.left_torque_nm,
            "u_right": allocation.right_torque_nm,
            "actual_left": float(actual[0]),
            "actual_right": float(actual[1]),
            "diff_clipped": allocation.differential_clipped,
            "guard_clipped": allocation.final_guard_clipped,
            "ff_phase": feedforward_phase,
            "velocity_phase": velocity_phase,
        }
        for name, value in row.items():
            logs[name].append(value)
        if keep_history and interval % history_period == 0:
            history.append({
                "t": time_s,
                "linear_velocity_cmd_m_s": v_cmd,
                "yaw_rate_cmd_rad_s": r_cmd,
                "psi_ref_rad": psi_ref,
                "r_ref_rad_s": r_cmd,
                "psi_hat_rad": yaw_estimate.psi_control_rad,
                "r_hat_rad_s": yaw_estimate.r_hat_rad_s,
                "psi_GT_rad": gt_psi,
                "r_GT_rad_s": r_gt,
                "u_sum_nm": u_sum,
                "u_base_nm": u_base,
                "u_eq_nm": u_eq,
                "u_fb_nm": u_fb,
                "u_ff_used_nm": u_ff,
                "u_diff_request_nm": u_diff_request,
                "u_diff_used_nm": allocation.u_diff_used_nm,
                "u_left_nm": allocation.left_torque_nm,
                "u_right_nm": allocation.right_torque_nm,
                "actual_left_nm": float(actual[0]),
                "actual_right_nm": float(actual[1]),
                "actual_sum_nm": float(np.sum(actual)),
                "sum_command_saturated": bool(abs(u_sum_request - u_sum) > 1e-12),
                "allocator_guard_clipped": bool(allocation.final_guard_clipped),
                "allocator_differential_clipped": bool(allocation.differential_clipped),
                "p_ref_m": float(next_reference[0]),
                "p_hat_m": float(estimate.position_hat_m),
                "p_GT_m": float(next_gt_long[0]),
                "v_ref_m_s": float(next_reference[1]),
                "v_hat_m_s": float(estimate.velocity_hat_m_s),
                "v_GT_m_s": float(next_gt_long[1]),
                "theta_ref_rad": float(next_reference[2]),
                "theta_eq_used_rad": float(theta_eq),
                "theta_hat_rad": float(estimate.theta_hat_rad),
                "theta_GT_rad": float(next_gt_long[2]),
                "theta_dot_ref_rad_s": float(next_reference[3]),
                "theta_dot_hat_rad_s": float(estimate.theta_dot_hat_rad_s),
                "lateral_GT_m": root_lateral_position_m(sim),
                "feedforward_phase": feedforward_phase,
                "velocity_lifecycle_phase": velocity_phase,
            })
            if rolling_reference_command is not None:
                history[-1].update({
                    "raw_linear_velocity_cmd_m_s": float(
                        rolling_reference_command.raw_linear_velocity_target_m_s
                        if rolling_reference_command.raw_linear_velocity_target_m_s
                        is not None
                        else rolling_reference_command.linear_velocity_target_m_s
                    ),
                    "scheduler_mode": rolling_reference_command.scheduler_mode,
                    "accepted_velocity_changed": bool(
                        rolling_reference_command.accepted_velocity_changed
                    ),
                    "planning_path": rolling_reference_command.planning_path,
                    "candidate_delta_v_m_s": float(
                        rolling_reference_command.candidate_delta_v_m_s
                    ),
                    "candidate_target_m_s": (
                        None
                        if rolling_reference_command.candidate_target_m_s is None
                        else float(rolling_reference_command.candidate_target_m_s)
                    ),
                    "candidate_stable": bool(
                        rolling_reference_command.candidate_stable
                    ),
                    "latest_pending_raw_v_cmd_m_s": (
                        rolling_reference_command.latest_pending_raw_v_cmd_m_s
                    ),
                    "pending_command_active": bool(
                        rolling_reference_command.pending_command_active
                    ),
                    "rolling_p_ref_applied_m": float(
                        rolling_reference_command.reference_state[0]
                    ),
                    "rolling_v_ref_applied_m_s": float(
                        rolling_reference_command.reference_state[1]
                    ),
                    "rolling_a_ref_applied_m_s2": float(
                        rolling_reference_command.reference_acceleration_m_s2
                    ),
                    "rolling_theta_ref_applied_rad": float(
                        rolling_reference_command.reference_state[2]
                    ),
                    "rolling_theta_dot_ref_applied_rad_s": float(
                        rolling_reference_command.reference_state[3]
                    ),
                    "rolling_u_ff_raw_nm": float(u_ff_raw),
                    "rolling_u_ff_after_lifecycle_nm": float(
                        u_ff_after_lifecycle
                    ),
                    "rolling_fade_alpha": float(
                        rolling_reference_command.fade_alpha
                    ),
                    "reference_block_id": int(
                        rolling_reference_command.block_id
                    ),
                    "reference_block_sample_index": int(
                        rolling_reference_command.block_sample_index
                    ),
                    "reference_block_start_time_s": float(
                        rolling_reference_command.block_start_time_s
                    ),
                    "intent_source_time_s": float(
                        rolling_reference_command.intent_source_time_s
                    ),
                    "rolling_replanned": bool(
                        rolling_reference_command.replanned
                    ),
                })
            if common_mode_augmentation is not None:
                history[-1].update(augmentation_command)
                history[-1].update(augmentation_observation)
            if control_observer is not None:
                history[-1].update(observer_output)
                history[-1].update(observer_observation)
            if history_diagnostic_callback is not None:
                diagnostic = history_diagnostic_callback(sim)
                if diagnostic is not None:
                    history[-1].update(diagnostic)
            if payload_recorder is not None and payload_positions:
                history[-1].update({
                    "payload_posthoc_relative_position_body_m": (
                        payload_positions[-1].tolist()
                    ),
                    "payload_posthoc_wall_contact": bool(payload_contacts[-1]),
                    "payload_posthoc_wall_normal_force_n": float(
                        payload_forces[-1]
                    ),
                })

        if termination_guard is not None:
            guard_result = termination_guard(sim)
            if guard_result:
                termination_reason = str(guard_result)
                break

    arrays = {name: np.asarray(values) for name, values in logs.items()}
    heading_error_gt = np.asarray([
        wrap_to_pi(ref - gt)
        for ref, gt in zip(arrays["psi_ref"], arrays["psi_gt"])
    ])
    heading_error_hat = np.asarray([
        wrap_to_pi(ref - hat)
        for ref, hat in zip(arrays["psi_ref"], arrays["psi_hat"])
    ])
    turning = np.abs(arrays["r_cmd"]) > 1e-9
    terminal = arrays["time"] >= max(0.0, float(arrays["time"][-1]) - 1.0)
    yaw_stops = settling_after_yaw_stops(
        arrays["time"], arrays["r_cmd"], heading_error_gt,
        arrays["r_gt"], yaw_schedule,
    )
    denominator = float(np.mean(
        np.abs(arrays["u_diff_used"]) + np.abs(arrays["u_fb"])
    ))
    mismatch_window = (
        (arrays["time"] >= 3.0) & (arrays["time"] < 9.5)
        & (np.abs(arrays["r_cmd"]) < 1e-12)
    )
    drift_slope = (
        float(np.polyfit(
            arrays["time"][mismatch_window], arrays["psi_gt"][mismatch_window], 1
        )[0]) if np.count_nonzero(mismatch_window) >= 2 else None
    )
    hold_no_fade = (
        (arrays["velocity_phase"] == VelocityLifecyclePhase.VELOCITY_HOLD.value)
        & (arrays["ff_phase"] != "FADING")
    )
    payload_posthoc = None
    if payload_recorder is not None and payload_positions:
        payload_time_array = np.asarray(payload_times, dtype=float)
        payload_position_array = np.asarray(payload_positions, dtype=float)
        payload_contact_array = np.asarray(payload_contacts, dtype=bool)
        payload_force_array = np.asarray(payload_forces, dtype=float)
        relative_speed = np.gradient(
            payload_position_array[:, 1], payload_time_array
        )
        first = payload_time_array <= payload_time_array[0] + 1.0
        last = payload_time_array >= payload_time_array[-1] - 1.0
        front_wall = sim.model.geom("basket_front_wall")
        payload_geom = sim.model.geom("moving_payload_collision")
        longitudinal_limit = float(
            abs(front_wall.pos[1]) - front_wall.size[1] - payload_geom.size[1]
        )
        force_episodes = force_bearing_contact_episodes(
            payload_contact_array, payload_force_array
        )
        payload_posthoc = {
            "data_boundary": "evaluator-only; never read by observer, gate, or controller",
            "setup": payload_setup,
            "remained_in_basket": bool(
                np.max(np.abs(payload_position_array[:, 1]))
                <= longitudinal_limit + 0.005
                and np.min(payload_position_array[:, 2]) >= 0.14
            ),
            "longitudinal_center_limit_m": longitudinal_limit,
            "relative_longitudinal_position_range_m": [
                float(np.min(payload_position_array[:, 1])),
                float(np.max(payload_position_array[:, 1])),
            ],
            "relative_longitudinal_speed_rms_m_s": rms(relative_speed),
            "relative_longitudinal_speed_peak_m_s": float(
                np.max(np.abs(relative_speed))
            ),
            "motion_decay": {
                "first_1s_speed_rms_m_s": rms(relative_speed[first]),
                "terminal_1s_speed_rms_m_s": rms(relative_speed[last]),
                "terminal_to_first_speed_rms_ratio": (
                    rms(relative_speed[last]) / max(rms(relative_speed[first]), 1e-12)
                ),
            },
            "wall_collision_episode_count": len(payload_recorder.collision_episodes),
            "force_bearing_wall_collision_episode_count": len(force_episodes),
            "wall_collision_times_s": [
                float(event["time_s"])
                for event in payload_recorder.collision_episodes
            ],
            "maximum_wall_normal_force_n": float(np.max(payload_force_array)),
            "resolved_contact_parameters": payload_recorder.resolved_contact_parameters,
        }

    result = {
        "scenario": scenario,
        "yaw_enabled": yaw_enabled,
        "gains": {"K_psi_nm_per_rad": K_psi, "K_r_nm_per_rad_s": K_r},
        "yaw_estimator": {
            "source": "startup-bias-corrected realistic IMU gyro z only",
            "integration": "new valid packet timestamps only; stale samples never reintegrated",
            "control_time_extrapolation": "psi_measurement + r_hat * measurement_age when valid",
            "encoder_magnetometer_GT_correction": False,
            "psi_estimator_error_rad": metric(np.asarray([
                wrap_to_pi(hat - gt)
                for hat, gt in zip(arrays["psi_hat"], arrays["psi_gt"])
            ])),
            "r_estimator_error_rad_s": metric(arrays["r_hat"] - arrays["r_gt"]),
        },
        "yaw_tracking": {
            "turning_yaw_rate_error_GT_rad_s": metric(
                arrays["r_gt"][turning] - arrays["r_cmd"][turning]
            ),
            "turning_yaw_rate_error_estimated_rad_s": metric(
                arrays["r_hat"][turning] - arrays["r_cmd"][turning]
            ),
            "heading_error_GT_rad": metric(heading_error_gt),
            "heading_error_estimated_rad": metric(heading_error_hat),
            "yaw_rate_overshoot_GT_rad_s": float(max(
                0.0,
                np.max(np.sign(arrays["r_cmd"][turning]) * (
                    arrays["r_gt"][turning] - arrays["r_cmd"][turning]
                )),
            )) if np.any(turning) else 0.0,
            "stops": yaw_stops,
            "heading_terminal_RMS_GT_rad": rms(heading_error_gt[terminal]),
            "sustained_yaw_oscillation": any(
                item["sustained_oscillation"] for item in yaw_stops
            ),
            "final_heading_error_GT_rad": float(heading_error_gt[-1]),
            "heading_drift_rate_GT_rad_s": drift_slope,
            "final_lateral_GT_m": float(arrays["lateral_gt"][-1]),
        },
        "longitudinal": {
            "velocity_error_estimated_m_s": metric(
                arrays["plant"][:, 1] - arrays["reference"][:, 1]
            ),
            "velocity_error_GT_m_s": metric(
                arrays["gt_long"][:, 1] - arrays["reference"][:, 1]
            ),
            "pitch_peak_GT_deg": math.degrees(float(np.max(
                np.abs(arrays["gt_long"][:, 2])
            ))),
            "pitch_terminal_RMS_GT_deg": math.degrees(
                rms(arrays["gt_long"][terminal, 2])
            ),
            "fell": fallen,
        },
        "torque_allocation": {
            "u_sum_nm": metric(arrays["u_sum"]),
            "u_diff_request_nm": metric(arrays["u_diff_request"]),
            "u_diff_used_nm": metric(arrays["u_diff_used"]),
            "u_diff_first_difference_RMS_nm": rms(np.diff(arrays["u_diff_used"])),
            "differential_clipping_fraction": float(np.mean(arrays["diff_clipped"])),
            "final_guard_clipping_fraction": float(np.mean(arrays["guard_clipped"])),
            "requested_wheel_peak_nm": float(np.max(np.abs(np.column_stack([
                arrays["u_left"], arrays["u_right"]
            ])))),
            "actual_wheel_peak_nm": float(np.max(np.abs(np.column_stack([
                arrays["actual_left"], arrays["actual_right"]
            ])))),
            "per_wheel_saturation_fraction": float(np.mean(
                (np.abs(arrays["u_left"]) >= limit - 1e-12)
                | (np.abs(arrays["u_right"]) >= limit - 1e-12)
            )),
            "yaw_feedback_opposes_longitudinal_feedback_fraction": float(np.mean(
                arrays["u_diff_used"] * arrays["u_fb"] < 0.0
            )),
            "combined_cancellation_fraction_diagnostic": (
                0.0 if denominator == 0.0 else float(
                    1.0 - np.mean(np.abs(arrays["u_sum"] + arrays["u_diff_used"]))
                    / denominator
                )
            ),
        },
        "motor_mismatch": {
            **MISMATCH,
            "enabled": motor_mismatch_enabled,
            "production_default": False,
        },
        "payload_posthoc": payload_posthoc,
        "frozen_longitudinal": {
            "A": A.tolist(), "B": B.tolist(), "K4": K4.tolist(),
            "lambda_ff": lambda_ff,
            "reference_limits": motion_config["reference_limits"],
            "max_jerk_m_s3": dynamic_config["max_jerk_m_s3"],
            "Q_disturbance_rejection_enabled": False,
            "transition_events": transition_events,
            "post_fade_HOLD_u_ff_strictly_zero": bool(
                np.all(arrays["u_ff"][hold_no_fade] == 0.0)
            ),
        },
        "finite": bool(all(
            np.isfinite(value).all() for value in arrays.values()
            if np.issubdtype(value.dtype, np.number)
        )),
        "GT_runtime_dependency": False,
        "simulation_termination": {
            "terminated_early": termination_reason is not None,
            "reason": termination_reason,
            "actual_duration_s": float(arrays["time"][-1]),
            "requested_duration_s": float(scenario["duration_s"]),
        },
        "history_50hz": history,
    }
    return result


def tuning_gate(result: dict) -> dict:
    yaw = result["yaw_tracking"]
    long = result["longitudinal"]
    torque = result["torque_allocation"]
    checks = {
        "no_fall": not long["fell"],
        "no_per_wheel_saturation": torque["per_wheel_saturation_fraction"] == 0.0,
        "differential_clipping_below_1pct": torque[
            "differential_clipping_fraction"
        ] < 0.01,
        "all_heading_stops_settle": all(
            item["settled_time_s"] is not None for item in yaw["stops"]
        ),
        "no_sustained_yaw_oscillation": not yaw["sustained_yaw_oscillation"],
        "finite": result["finite"],
        "yaw_rate_RMSE_below_0p15": yaw[
            "turning_yaw_rate_error_GT_rad_s"
        ]["rms"] <= 0.15,
        "heading_peak_below_0p30": yaw["heading_error_GT_rad"]["peak_abs"] <= 0.30,
        "velocity_RMSE_below_0p06": long[
            "velocity_error_estimated_m_s"
        ]["rms"] <= 0.06,
        "pitch_peak_below_12deg": long["pitch_peak_GT_deg"] <= 12.0,
        "HOLD_feedforward_zero": result["frozen_longitudinal"][
            "post_fade_HOLD_u_ff_strictly_zero"
        ],
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "failed_checks": [name for name, passed in checks.items() if not passed],
    }


def candidate_key(item: dict) -> tuple:
    result = item["summary"]
    yaw = result["yaw_tracking"]
    torque = result["torque_allocation"]
    return (
        yaw["turning_yaw_rate_error_GT_rad_s"]["rms"],
        yaw["heading_error_GT_rad"]["rms"],
        torque["u_diff_used_nm"]["rms"],
        torque["u_diff_first_difference_RMS_nm"],
        torque["differential_clipping_fraction"],
    )


def slim(result: dict) -> dict:
    return {key: value for key, value in result.items() if key != "history_50hz"}


def load_common() -> tuple:
    manifest, config, dynamic, motion, offline = longitudinal.load_production_baseline()
    return (
        manifest, config, dynamic, motion, offline,
        load_json(longitudinal.EXPERIMENT_CONFIG_PATH),
        load_json(longitudinal.DR_CONFIG_PATH),
        load_json(longitudinal.REDUCED_PATH),
        load_json(longitudinal.PLANT_PARAMETERS_PATH),
    )


def base_result() -> dict:
    return {
        "stage": "Stage 3B V1 parallel yaw / steering control",
        "status": "TUNING",
        "command_semantics": {
            "user_command": "(linear_velocity_cmd_m_s, yaw_rate_cmd_rad_s)",
            "psi_ref_update": "psi_ref[k+1] = psi_ref[k] + r_ref[k] * controller_dt",
            "zero_rate": "r_ref=0 and psi_ref holds its last continuous value",
            "heading_error": "wrap_to_pi(psi_ref - psi_hat)",
            "bumpless_enable": "psi_ref initialized from psi_hat at enable",
        },
        "controller_semantics": {
            "equation": "u_diff_request = K_psi*e_psi + K_r*e_r",
            "integral_or_feedforward": False,
            "mixer_sign": "positive u_diff => left torque increases, right torque decreases => positive MuJoCo +Z yaw",
            "allocator": "u_diff_available=max(0,2*tau_lim-|u_sum|); common-mode longitudinal priority",
        },
        "sign_sanity_check": {
            "initial_formula_left_negative_right_positive_produced_negative_yaw": True,
            "observed_delta_yaw_rad": -0.014891375331919754,
            "corrected_convention": "left=(u_sum+u_diff)/2; right=(u_sum-u_diff)/2",
            "gain_sign_changed": False,
        },
        "maximum_tuning_simulations": 6,
        "tuning_simulation_count": 0,
        "tuning_history": [],
        "selected_candidate": None,
        "symmetric_validation": None,
        "mismatch_comparison": None,
        "final_production_regression": None,
        "production_baseline": None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tuning-only", action="store_true")
    parser.add_argument("--mismatch-only", action="store_true")
    parser.add_argument("--final-only", action="store_true")
    args = parser.parse_args()
    if sum((args.tuning_only, args.mismatch_only, args.final_only)) > 1:
        parser.error("select at most one execution mode")
    common = load_common()
    result = read_result = (
        json.loads(RESULT_PATH.read_text(encoding="utf-8"))
        if RESULT_PATH.exists() else base_result()
    )
    if args.mismatch_only:
        selected = result["selected_candidate"]
        if selected is None:
            raise RuntimeError("mismatch validation requires a selected candidate")
        prior = result.get("mismatch_comparison")
        if prior is not None:
            result["mismatch_pilot_rejected"] = {
                "reason": "40/50 ms lag produced heading drift below gyro-only integration error floor",
                "configuration": prior["configuration"],
                "yaw_OFF": slim(prior["yaw_OFF"]),
                "yaw_ON": slim(prior["yaw_ON"]),
            }
        kp = float(selected["K_psi_nm_per_rad"])
        kr = float(selected["K_r_nm_per_rad_s"])
        print("VALIDATE revised mismatch yaw OFF", flush=True)
        mismatch_off = run_case(
            MISMATCH_SCENARIO, kp, kr, yaw_enabled=False,
            motor_mismatch_enabled=True, common=common, keep_history=True,
        )
        print("VALIDATE revised mismatch yaw ON", flush=True)
        mismatch_on = run_case(
            MISMATCH_SCENARIO, kp, kr, yaw_enabled=True,
            motor_mismatch_enabled=True, common=common, keep_history=True,
        )
        result["mismatch_comparison"] = {
            "configuration": MISMATCH,
            "yaw_OFF": mismatch_off,
            "yaw_ON": mismatch_on,
        }
        RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"WROTE {RESULT_PATH}", flush=True)
        return
    if args.final_only:
        selected = result["selected_candidate"]
        if selected is None:
            raise RuntimeError("final validation requires a selected candidate")
        prior = result.get("final_production_regression")
        if prior is not None:
            result["invalidated_production_regression"] = {
                "reason": "post-hoc GT velocity used fixed world -Y instead of chassis-forward projection; runtime control was unaffected",
                "summary": slim(prior),
            }
        kp = float(selected["K_psi_nm_per_rad"])
        kr = float(selected["K_r_nm_per_rad_s"])
        print("RUN coordinate-correct final production regression", flush=True)
        final = run_case(
            MIXED_SCENARIO, kp, kr, yaw_enabled=True,
            motor_mismatch_enabled=False, common=common, keep_history=True,
        )
        result["final_production_regression"] = final
        result["symmetric_validation"]["yaw_ON"] = slim(final)
        result["symmetric_validation"]["yaw_ON_source"] = (
            "coordinate-correct final production regression"
        )
        result["production_baseline"]["motor_mismatch_enabled"] = False
        RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"WROTE {RESULT_PATH}", flush=True)
        return
    existing = {item["name"] for item in result["tuning_history"]}
    for candidate in TUNING_CANDIDATES:
        if candidate["name"] in existing:
            continue
        if result["tuning_simulation_count"] >= result["maximum_tuning_simulations"]:
            raise RuntimeError("yaw tuning simulation budget exhausted")
        print(f"TUNE {candidate}", flush=True)
        run = run_case(
            MIXED_SCENARIO,
            candidate["K_psi_nm_per_rad"], candidate["K_r_nm_per_rad_s"],
            yaw_enabled=True, motor_mismatch_enabled=False,
            common=common, keep_history=False,
        )
        item = {**candidate, "summary": slim(run)}
        item["gate"] = tuning_gate(run)
        result["tuning_history"].append(item)
        result["tuning_simulation_count"] += 1
        RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    if args.tuning_only:
        print(f"WROTE TUNING CHECKPOINT {RESULT_PATH}", flush=True)
        return

    passing = [item for item in result["tuning_history"] if item["gate"]["passed"]]
    if not passing:
        result["status"] = "NO_TUNING_CANDIDATE_PASSED"
        RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(f"STOPPED {result['status']} {RESULT_PATH}", flush=True)
        return
    passing.sort(key=candidate_key)
    best_rate = candidate_key(passing[0])[0]
    close = [item for item in passing if candidate_key(item)[0] <= 1.05 * best_rate]
    selected = min(close, key=lambda item: (
        item["summary"]["torque_allocation"]["u_diff_used_nm"]["rms"],
        item["summary"]["torque_allocation"]["u_diff_first_difference_RMS_nm"],
        candidate_key(item),
    ))
    result["selected_candidate"] = {
        "name": selected["name"],
        "K_psi_nm_per_rad": selected["K_psi_nm_per_rad"],
        "K_r_nm_per_rad_s": selected["K_r_nm_per_rad_s"],
        "selection": "minimum yaw-rate RMSE; within 5%, lower u_diff RMS/chatter",
    }
    kp, kr = selected["K_psi_nm_per_rad"], selected["K_r_nm_per_rad_s"]

    print("VALIDATE symmetric yaw OFF", flush=True)
    symmetric_off = run_case(
        MIXED_SCENARIO, kp, kr, yaw_enabled=False,
        motor_mismatch_enabled=False, common=common, keep_history=False,
    )
    result["symmetric_validation"] = {
        "yaw_OFF": slim(symmetric_off),
        "yaw_ON": selected["summary"],
        "yaw_ON_source": "selected symmetric tuning run reused without rerun",
    }

    print("VALIDATE mismatch yaw OFF", flush=True)
    mismatch_off = run_case(
        MISMATCH_SCENARIO, kp, kr, yaw_enabled=False,
        motor_mismatch_enabled=True, common=common, keep_history=True,
    )
    print("VALIDATE mismatch yaw ON", flush=True)
    mismatch_on = run_case(
        MISMATCH_SCENARIO, kp, kr, yaw_enabled=True,
        motor_mismatch_enabled=True, common=common, keep_history=True,
    )
    result["mismatch_comparison"] = {
        "configuration": MISMATCH,
        "yaw_OFF": mismatch_off,
        "yaw_ON": mismatch_on,
    }

    print("RUN final production regression", flush=True)
    final = run_case(
        MIXED_SCENARIO, kp, kr, yaw_enabled=True,
        motor_mismatch_enabled=False, common=common, keep_history=True,
    )
    result["final_production_regression"] = final
    result["production_baseline"] = {
        "yaw_enabled": True,
        "K_psi_nm_per_rad": kp,
        "K_r_nm_per_rad_s": kr,
        "motor_mismatch_enabled": False,
        "longitudinal_baseline_unchanged": True,
        "GT_runtime_dependency": False,
    }
    result["status"] = "COMPLETE"
    RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"WROTE {RESULT_PATH}", flush=True)


if __name__ == "__main__":
    main()
