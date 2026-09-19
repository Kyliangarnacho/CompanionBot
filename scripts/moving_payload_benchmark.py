"""Moving-payload benchmark for fixed LQR plus filtered disturbance rejection."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import (
    DiscreteStateSpaceModel,
    FilteredDisturbanceCompensator,
    disturbance_rejection_config_from_dict,
)
from sim import LongitudinalEstimator, MiniSegwaySim, load_longitudinal_estimator_config


MODEL_DIR = ROOT / "models" / "minisegway"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
DR_CONFIG_PATH = MODEL_DIR / "disturbance_rejection_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
NOMINAL_OFFLINE_PATH = MODEL_DIR / "stage2" / "results" / "full_state_identification_results.json"
SENSORIZED_STATE = "sensorized_x_hat"


def has_chassis_floor_contact(sim: MiniSegwaySim) -> bool:
    for index in range(sim.data.ncon):
        contact = sim.data.contact[index]
        names = {
            mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
            mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
        }
        if "floor" in names and any(
            name and name.endswith("chassis_collision") for name in names
        ):
            return True
    return False


def held_initial_imu(
    sim: MiniSegwaySim,
) -> tuple[np.ndarray, np.ndarray, float, bool]:
    """Initialize attitude from a motionless held chassis without reading GT state."""

    sim.data.qacc[:] = 0.0
    mujoco.mj_sensorAcc(sim.model, sim.data)
    return sim.imu_estimator_input()


def window_metrics(
    times: np.ndarray,
    positions: np.ndarray,
    pitch: np.ndarray,
    start: float,
    end: float,
) -> dict:
    mask = (times >= start) & (times < end)
    if not np.any(mask):
        return {}
    values = pitch[mask]
    return {
        "pitch_rms_deg": float(np.degrees(np.sqrt(np.mean(values**2)))),
        "peak_abs_pitch_deg": float(np.degrees(np.max(np.abs(values)))),
        "position_change_m": float(positions[mask][-1] - positions[mask][0]),
    }


class PayloadGroundTruthRecorder:
    """Post-hoc payload/contact observer; its values never enter control."""

    WALL_NAMES = {
        "basket_front_wall",
        "basket_rear_wall",
        "basket_left_wall",
        "basket_right_wall",
    }

    def __init__(self, sim: MiniSegwaySim):
        self.sim = sim
        self.payload_body = sim.model.body("moving_payload").id
        self.chassis_body = sim.model.body("chassis").id
        self.payload_geom = sim.model.geom("moving_payload_collision").id
        self.wall_ids = {sim.model.geom(name).id: name for name in self.WALL_NAMES}
        self.floor_id = sim.model.geom("basket_floor").id
        self.previous_walls: set[str] = set()
        self.collision_episodes: list[dict] = []
        self.resolved_contact_parameters: dict[str, dict] = {}

    def sample(
        self, time_s: float, nominal_pitch_error: float, torque: float
    ) -> tuple[np.ndarray, float, bool, float]:
        sim = self.sim
        rotation = sim.data.xmat[self.chassis_body].reshape(3, 3)
        chassis_origin = sim.data.xpos[self.chassis_body]
        relative_position = rotation.T @ (
            sim.data.xpos[self.payload_body] - chassis_origin
        )
        masses = np.asarray(sim.model.body_mass, dtype=float)
        active = np.flatnonzero(masses > 0.0)
        total_mass = float(np.sum(masses[active]))
        system_com_world = np.sum(
            masses[active, None] * sim.data.xipos[active], axis=0
        ) / total_mass
        system_com_body = rotation.T @ (system_com_world - chassis_origin)
        theta_eq = math.atan2(system_com_body[1], system_com_body[2])

        active_walls: set[str] = set()
        maximum_wall_force = 0.0
        for index in range(sim.data.ncon):
            contact = sim.data.contact[index]
            if contact.geom1 == self.payload_geom:
                other = contact.geom2
            elif contact.geom2 == self.payload_geom:
                other = contact.geom1
            else:
                continue
            if other not in self.wall_ids:
                if other == self.floor_id:
                    self.resolved_contact_parameters["basket_floor"] = {
                        "friction": np.asarray(contact.friction).tolist(),
                        "solref": np.asarray(contact.solref).tolist(),
                        "solimp": np.asarray(contact.solimp).tolist(),
                    }
                continue
            wall_name = self.wall_ids[other]
            self.resolved_contact_parameters[wall_name] = {
                "friction": np.asarray(contact.friction).tolist(),
                "solref": np.asarray(contact.solref).tolist(),
                "solimp": np.asarray(contact.solimp).tolist(),
            }
            active_walls.add(wall_name)
            force = np.zeros(6, dtype=float)
            mujoco.mj_contactForce(sim.model, sim.data, index, force)
            maximum_wall_force = max(maximum_wall_force, abs(float(force[0])))

        for wall_name in sorted(active_walls - self.previous_walls):
            self.collision_episodes.append(
                {
                    "time_s": float(time_s),
                    "wall": wall_name,
                    "relative_position_body_m": relative_position.tolist(),
                    "nominal_pitch_error_deg": math.degrees(nominal_pitch_error),
                    "wheel_torque_nm": float(torque),
                    "initial_normal_force_n": maximum_wall_force,
                }
            )
        self.previous_walls = active_walls
        return relative_position, theta_eq, bool(active_walls), maximum_wall_force


def collision_transient_metrics(
    times: np.ndarray,
    pitch_error: np.ndarray,
    torques: np.ndarray,
    collision_mask: np.ndarray,
    window_s: float,
) -> dict:
    rising = np.flatnonzero(collision_mask & ~np.r_[False, collision_mask[:-1]])
    if len(rising) == 0:
        return {
            "episode_count": 0,
            "mean_pre_pitch_rms_deg": None,
            "mean_post_pitch_rms_deg": None,
            "worst_post_peak_pitch_deg": None,
            "maximum_post_wheel_torque_nm": None,
        }
    pre_rms: list[float] = []
    post_rms: list[float] = []
    post_peak: list[float] = []
    post_torque: list[float] = []
    for index in rising:
        center = times[index]
        pre = (times >= center - window_s) & (times < center)
        post = (times >= center) & (times < center + window_s)
        if np.any(pre):
            pre_rms.append(math.degrees(float(np.sqrt(np.mean(pitch_error[pre] ** 2)))))
        if np.any(post):
            post_rms.append(
                math.degrees(float(np.sqrt(np.mean(pitch_error[post] ** 2))))
            )
            post_peak.append(math.degrees(float(np.max(np.abs(pitch_error[post])))))
            post_torque.append(float(np.max(torques[post])))
    return {
        "episode_count": int(len(rising)),
        "mean_pre_pitch_rms_deg": float(np.mean(pre_rms)) if pre_rms else None,
        "mean_post_pitch_rms_deg": float(np.mean(post_rms)) if post_rms else None,
        "worst_post_peak_pitch_deg": float(np.max(post_peak)) if post_peak else None,
        "maximum_post_wheel_torque_nm": float(np.max(post_torque)) if post_torque else None,
    }


def force_bearing_contact_episodes(
    contact_mask: np.ndarray,
    normal_force: np.ndarray,
    threshold_n: float = 0.01,
) -> list[dict]:
    """Group contiguous contact samples and retain force-bearing episodes."""

    mask = np.asarray(contact_mask, dtype=bool)
    force = np.asarray(normal_force, dtype=float)
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    episodes = []
    for start in starts:
        end = int(start)
        while end + 1 < len(mask) and mask[end + 1]:
            end += 1
        peak = float(np.max(force[start : end + 1]))
        if peak >= threshold_n:
            episodes.append(
                {"start_index": int(start), "end_index": end, "peak_force_n": peak}
            )
    return episodes


def set_payload_initial_longitudinal_state(
    sim: MiniSegwaySim,
    *,
    relative_position_m: float | None,
    relative_velocity_m_s: float | None,
    relative_vertical_position_m: float | None = None,
) -> dict | None:
    """Set a test-only payload initial condition in the chassis frame.

    The controller remains blind to this information.  Existing lateral and
    vertical placement and payload orientation are preserved.
    """

    if (
        relative_position_m is None
        and relative_velocity_m_s is None
        and relative_vertical_position_m is None
    ):
        return None
    payload_body = sim.model.body("moving_payload").id
    chassis_body = sim.model.body("chassis").id
    payload_joint = sim.model.joint("moving_payload_free").id
    qpos_address = int(sim.model.jnt_qposadr[payload_joint])
    dof_address = int(sim.model.jnt_dofadr[payload_joint])
    rotation = sim.data.xmat[chassis_body].reshape(3, 3)
    chassis_origin = sim.data.xpos[chassis_body].copy()
    relative_position = rotation.T @ (
        sim.data.xpos[payload_body] - chassis_origin
    )
    if relative_position_m is not None:
        relative_position[1] = float(relative_position_m)
    if relative_vertical_position_m is not None:
        relative_position[2] = float(relative_vertical_position_m)
    if relative_position_m is not None or relative_vertical_position_m is not None:
        sim.data.qpos[qpos_address : qpos_address + 3] = (
            chassis_origin + rotation @ relative_position
        )
    relative_velocity = np.zeros(3, dtype=float)
    if relative_velocity_m_s is not None:
        relative_velocity[1] = float(relative_velocity_m_s)
        sim.data.qvel[dof_address : dof_address + 3] = rotation @ relative_velocity
    mujoco.mj_forward(sim.model, sim.data)
    actual_relative_position = rotation.T @ (
        sim.data.xpos[payload_body] - chassis_origin
    )
    return {
        "requested_relative_longitudinal_position_m": relative_position_m,
        "requested_relative_longitudinal_velocity_m_s": relative_velocity_m_s,
        "requested_relative_vertical_position_m": relative_vertical_position_m,
        "actual_relative_position_body_m": actual_relative_position.tolist(),
        "actual_initial_linear_velocity_world_m_s": sim.data.qvel[
            dof_address : dof_address + 3
        ].tolist(),
    }


def configure_payload_box(
    sim: MiniSegwaySim, full_size_m: np.ndarray | None
) -> dict | None:
    """Resize the test payload at fixed mass and update its box inertia."""

    if full_size_m is None:
        return None
    size = np.asarray(full_size_m, dtype=float)
    if size.shape != (3,) or np.any(size <= 0.0):
        raise ValueError("payload full size must contain three positive values")
    body_id = sim.model.body("moving_payload").id
    geom_id = sim.model.geom("moving_payload_collision").id
    mass = float(sim.model.body_mass[body_id])
    x, y, z = size
    inertia = mass / 12.0 * np.array(
        [y * y + z * z, x * x + z * z, x * x + y * y], dtype=float
    )
    sim.model.geom_size[geom_id, :3] = size / 2.0
    sim.model.body_inertia[body_id] = inertia
    return {
        "full_size_m": size.tolist(),
        "mass_kg": mass,
        "diagonal_inertia_kg_m2": inertia.tolist(),
    }


def attribute_final_saturation(
    channel_requests_nm: dict[str, float], actual_sum_torque_nm: float
) -> dict[str, float]:
    """Attribute net final-limit loss without changing control allocation.

    Only contributors pointing in the saturated net direction are reduced.  The
    loss is distributed in proportion to their requested magnitudes.  This is a
    post-hoc logging convention; MuJoCo receives only the clipped total torque.
    """

    requested_total = float(sum(channel_requests_nm.values()))
    loss = requested_total - float(actual_sum_torque_nm)
    attributed = {name: 0.0 for name in channel_requests_nm}
    if abs(loss) <= 1e-12 or abs(requested_total) <= 1e-12:
        return attributed
    direction = math.copysign(1.0, requested_total)
    eligible = {
        name: abs(value)
        for name, value in channel_requests_nm.items()
        if value * direction > 0.0
    }
    total_weight = float(sum(eligible.values()))
    if total_weight <= 1e-12:
        return attributed
    for name, weight in eligible.items():
        attributed[name] = loss * weight / total_weight
    return attributed


def run_case(
    *,
    label: str,
    enable_disturbance_rejection: bool,
    raw: dict,
    dr_raw: dict,
    reduced: dict,
    nominal_model: DiscreteStateSpaceModel,
    nominal_gain: np.ndarray,
    state_scales: np.ndarray,
    input_scale: float,
    peak: float,
    initial_payload_longitudinal_position_m: float | None = None,
    initial_payload_longitudinal_velocity_m_s: float | None = None,
    payload_full_size_m: np.ndarray | None = None,
    basket_friction_override: float | None = None,
    capture_commands: bool = False,
) -> dict:
    stress = raw["moving_payload_stress"]
    benchmark = raw["benchmark"]
    controller_hz = float(raw["controller_frequency_hz"])
    physics_steps = int(raw["physics_steps_per_update"])
    interval_count = round(float(stress["duration_s"]) * controller_hz)
    history_period = round(controller_hz / float(stress["history_frequency_hz"]))

    sim = MiniSegwaySim(MODEL_DIR / stress["model_file"])
    payload_box = configure_payload_box(sim, payload_full_size_m)
    for geom_name in (
        "basket_floor",
        "basket_front_wall",
        "basket_rear_wall",
        "basket_left_wall",
        "basket_right_wall",
        "moving_payload_collision",
    ):
        sim.model.geom(geom_name).friction[0] = float(
            basket_friction_override
            if basket_friction_override is not None
            else stress["basket_contact_friction"]
        )
    nominal_theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim.reset(
        nominal_theta_eq
        + math.radians(float(stress["initial_pitch_error_deg"]))
    )
    sim.calibrate_imu_stationary()
    payload_vertical_position = (
        0.141 + 0.5 * float(np.asarray(payload_full_size_m)[2])
        if payload_full_size_m is not None
        else None
    )
    initial_payload_state = set_payload_initial_longitudinal_state(
        sim,
        relative_position_m=initial_payload_longitudinal_position_m,
        relative_velocity_m_s=initial_payload_longitudinal_velocity_m_s,
        relative_vertical_position_m=payload_vertical_position,
    )
    estimator_config = load_longitudinal_estimator_config()
    expected_physics_steps = round(
        estimator_config.pitch.sample_period_s / sim.physics_dt
    )
    if expected_physics_steps != physics_steps:
        raise RuntimeError("estimator, controller, and physics sample periods disagree")
    imu_ticks_per_estimator = round(
        estimator_config.pitch.sample_period_s / sim.imu_sample_period_s
    )
    if imu_ticks_per_estimator < 1 or not math.isclose(
        estimator_config.pitch.sample_period_s,
        imu_ticks_per_estimator * sim.imu_sample_period_s,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(
            "estimator period must be an integer multiple of the IMU period"
        )
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
    gt = PayloadGroundTruthRecorder(sim)
    disturbance_config = disturbance_rejection_config_from_dict(dr_raw)
    compensator = FilteredDisturbanceCompensator(
        nominal_model,
        state_scales,
        input_scale,
        disturbance_config,
    )
    times: list[float] = []
    positions: list[float] = []
    nominal_pitch_errors: list[float] = []
    pitch_rates: list[float] = []
    wheel_torques: list[float] = []
    sum_torques: list[float] = []
    payload_positions: list[np.ndarray] = []
    theta_eq_gt: list[float] = []
    wall_contact: list[bool] = []
    wall_force: list[float] = []
    lqr_commands: list[float] = []
    requested_u_dr_commands: list[float] = []
    u_dr_commands: list[float] = []
    raw_disturbances: list[float] = []
    projected_disturbances: list[float] = []
    innovation_rms: list[float] = []
    matched_fractions: list[float] = []
    estimator_errors = {
        "position_m": [],
        "velocity_m_s": [],
        "pitch_rad": [],
        "pitch_rate_rad_s": [],
    }
    requested_sum_torques: list[float] = []
    held_sum_torques: list[float] = []
    actual_interval_sum_torques: list[float] = []
    saturation_eaten = {
        "lqr": [],
        "disturbance_rejection": [],
    }
    histories: list[dict] = []
    saturated_updates = 0
    fallen = False

    for interval in range(interval_count):
        start_time = float(sim.data.time)
        control_state = estimate.controller_state(nominal_theta_eq)
        compensation = compensator.command(enabled=enable_disturbance_rejection)
        lqr_sum = -float((nominal_gain @ control_state).item())
        raw_sum = lqr_sum + compensation.u_dr_nm
        held_sum = float(np.clip(raw_sum, -2.0 * peak, 2.0 * peak))
        saturated_updates += int(
            not np.isclose(raw_sum, held_sum, rtol=0.0, atol=1e-12)
        )
        next_gt_state = None
        actual_sum = held_sum
        for _ in range(physics_steps):
            snapshot = sim.step(held_sum / 2.0, held_sum / 2.0)
            next_gt_state = sim.longitudinal_state(nominal_theta_eq)
            actual_wheel_torque = float(np.max(np.abs(snapshot.applied_ctrl_nm)))
            actual_sum = float(np.sum(snapshot.applied_ctrl_nm))
            payload_position, theta_gt, contacting, force = gt.sample(
                snapshot.time_s, next_gt_state[2], actual_wheel_torque
            )
            times.append(snapshot.time_s)
            positions.append(next_gt_state[0])
            nominal_pitch_errors.append(next_gt_state[2])
            pitch_rates.append(next_gt_state[3])
            wheel_torques.append(actual_wheel_torque)
            sum_torques.append(actual_sum)
            payload_positions.append(payload_position)
            theta_eq_gt.append(theta_gt)
            wall_contact.append(contacting)
            wall_force.append(force)
            fallen |= abs(next_gt_state[2]) >= math.radians(
                benchmark["fall_pitch_error_deg"]
            )
            fallen |= has_chassis_floor_contact(sim)

        assert next_gt_state is not None
        # Read the latest already-generated packet available at this time. GT is
        # not passed into estimator or controller state assembly.
        (
            accel,
            gyro,
            measurement_age_s,
            extrapolation_allowed,
        ) = sim.imu_estimator_input()
        imu_raw_log_fields = sim.imu_raw_log_fields()
        estimate = estimator.update(
            accel,
            gyro,
            sim.wheel_encoder_counts(),
            measurement_age_s=measurement_age_s,
            allow_kinematic_extrapolation=extrapolation_allowed,
        )
        next_control_state = estimate.controller_state(nominal_theta_eq)
        estimator_errors["position_m"].append(
            float(next_control_state[0] - next_gt_state[0])
        )
        estimator_errors["velocity_m_s"].append(
            float(next_control_state[1] - next_gt_state[1])
        )
        estimator_errors["pitch_rad"].append(
            float(
                (next_control_state[2] - next_gt_state[2] + math.pi)
                % (2.0 * math.pi)
                - math.pi
            )
        )
        estimator_errors["pitch_rate_rad_s"].append(
            float(next_control_state[3] - next_gt_state[3])
        )

        observation = compensator.observe(
            control_state, actual_sum, next_control_state
        )
        channel_requests = {
            "lqr": lqr_sum,
            "disturbance_rejection": compensation.u_dr_nm,
        }
        attributed_loss = attribute_final_saturation(
            channel_requests, actual_sum
        )
        lqr_commands.append(lqr_sum)
        requested_u_dr_commands.append(compensation.requested_u_dr_nm)
        u_dr_commands.append(compensation.u_dr_nm)
        raw_disturbances.append(observation.matched_disturbance_raw_nm)
        projected_disturbances.append(
            observation.matched_disturbance_projected_nm
        )
        innovation_rms.append(observation.scaled_innovation_rms)
        matched_fractions.append(observation.matched_residual_fraction)
        requested_sum_torques.append(raw_sum)
        held_sum_torques.append(held_sum)
        actual_interval_sum_torques.append(actual_sum)
        for channel, value in attributed_loss.items():
            saturation_eaten[channel].append(value)

        if interval % history_period == 0:
            histories.append(
                {
                    "time_s": start_time,
                    "controller_state_source": SENSORIZED_STATE,
                    "controller_state": control_state.tolist(),
                    "next_controller_state": next_control_state.tolist(),
                    "scaled_innovation_rms": observation.scaled_innovation_rms,
                    "matched_disturbance_projected_nm": observation.matched_disturbance_projected_nm,
                    "u_lqr_nm": lqr_sum,
                    "requested_u_dr_nm": compensation.requested_u_dr_nm,
                    "u_dr_nm": compensation.u_dr_nm,
                    "q_filter_estimate_nm": compensation.q_filter_estimate_nm,
                    "u_dr_authority_limited": compensation.authority_limited,
                    "u_dr_slew_limited": compensation.slew_limited,
                    "requested_sum_torque_nm": raw_sum,
                    "software_limited_sum_torque_nm": held_sum,
                    "actual_applied_sum_torque_nm": actual_sum,
                    "final_saturation_eaten_nm": attributed_loss,
                    "next_theta_hat_measurement_time_rad": (
                        estimate.theta_measurement_time_rad
                    ),
                    "next_theta_hat_control_time_rad": estimate.theta_hat_rad,
                    "next_theta_dot_hat_control_time_rad_s": (
                        estimate.theta_dot_hat_rad_s
                    ),
                    "next_imu_extrapolation_age_s": (
                        estimate.imu_extrapolation_age_s
                    ),
                    "next_imu_kinematic_extrapolation_applied": (
                        estimate.imu_kinematic_extrapolation_applied
                    ),
                    **imu_raw_log_fields,
                }
            )

    arrays = {
        "times": np.asarray(times),
        "positions": np.asarray(positions),
        "nominal_pitch": np.asarray(nominal_pitch_errors),
        "pitch_rate": np.asarray(pitch_rates),
        "wheel_torque": np.asarray(wheel_torques),
        "sum_torque": np.asarray(sum_torques),
        "payload_position": np.asarray(payload_positions),
        "theta_eq_gt": np.asarray(theta_eq_gt),
        "wall_contact": np.asarray(wall_contact, dtype=bool),
        "wall_force": np.asarray(wall_force),
    }
    relative_velocity = np.gradient(
        arrays["payload_position"], arrays["times"], axis=0
    )
    early_motion_mask = arrays["times"] < 1.0
    terminal_motion_mask = arrays["times"] >= float(stress["duration_s"]) - 1.0
    gt_offset = arrays["theta_eq_gt"] - nominal_theta_eq
    instantaneous_gt_pitch_error = arrays["nominal_pitch"] - gt_offset
    collision_metrics = collision_transient_metrics(
        arrays["times"],
        instantaneous_gt_pitch_error,
        arrays["wheel_torque"],
        arrays["wall_contact"],
        float(stress["collision_transient_window_s"]),
    )
    force_bearing_episodes = force_bearing_contact_episodes(
        arrays["wall_contact"], arrays["wall_force"]
    )

    def rms(values: np.ndarray) -> float:
        return float(np.sqrt(np.mean(values**2)))

    def estimator_error_metrics(values: list[float], scale: float = 1.0) -> dict:
        array = scale * np.asarray(values, dtype=float)
        return {
            "rms": rms(array),
            "peak": float(np.max(np.abs(array))),
            "final": float(array[-1]),
        }

    requested_sum_array = np.asarray(requested_sum_torques)
    held_sum_array = np.asarray(held_sum_torques)
    actual_interval_sum_array = np.asarray(actual_interval_sum_torques)

    phase_metrics = {
        "full": window_metrics(
            arrays["times"],
            arrays["positions"],
            instantaneous_gt_pitch_error,
            0.0,
            float(stress["duration_s"]),
        )
    }
    terminal_mask = arrays["times"] >= float(stress["duration_s"]) - 1.0
    payload_position_min = np.min(arrays["payload_position"], axis=0)
    payload_position_max = np.max(arrays["payload_position"], axis=0)
    front_wall = sim.model.geom("basket_front_wall")
    payload_geom = sim.model.geom("moving_payload_collision")
    longitudinal_limit = float(
        abs(front_wall.pos[1]) - front_wall.size[1] - payload_geom.size[1]
    )
    payload_remained_in_basket = bool(
        np.max(np.abs(arrays["payload_position"][:, 1]))
        <= longitudinal_limit + 0.005
        and np.min(arrays["payload_position"][:, 2]) >= 0.14
    )

    result = {
        "label": label,
        "controller_state_source": SENSORIZED_STATE,
        "fell": bool(fallen),
        "basket_contact_friction": float(
            basket_friction_override
            if basket_friction_override is not None
            else stress["basket_contact_friction"]
        ),
        "initial_payload_state": initial_payload_state,
        "payload_box_override": payload_box,
        "controller": {
            "fixed_nominal_K": nominal_gain.reshape(-1).tolist(),
            "state_dataflow": "current integer encoder counts -> encoder PLL; delayed calibrated IMU packet -> complementary filter at measurement time -> guarded constant-rate extrapolation to control time; aligned encoder PLL + IMU output -> x_hat_control_time; LQR and matched-disturbance innovation consume that same state only",
            "ground_truth_boundary": "GT is evaluator/logger-only after command assembly",
            "theta_reference_changed": False,
            "K_changed": False,
            "disturbance_rejection_enabled": enable_disturbance_rejection,
            "imu_hardware_profile": sim.imu_profile_name,
            "imu_rng_seed": sim.imu_seed,
            "sensorized_state_timestamp": "current controller horizon",
            "dob_model_horizon": "frozen A_2ms/B_2ms",
        },
        "pitch_nominal_reference": {
            "rms_deg": math.degrees(rms(arrays["nominal_pitch"])),
            "peak_deg": math.degrees(float(np.max(np.abs(arrays["nominal_pitch"])))),
        },
        "pitch_against_posthoc_instantaneous_gt": {
            "rms_deg": math.degrees(rms(instantaneous_gt_pitch_error)),
            "peak_deg": math.degrees(float(np.max(np.abs(instantaneous_gt_pitch_error)))),
            "terminal_1s_rms_deg": math.degrees(
                rms(instantaneous_gt_pitch_error[terminal_mask])
            ),
            "phase_metrics": phase_metrics,
        },
        "final_position_drift_m": float(arrays["positions"][-1]),
        "max_wheel_torque_nm": float(np.max(arrays["wheel_torque"])),
        "saturation_ratio": saturated_updates / interval_count,
        "torque_authority": {
            "attribution_method": "logging-only proportional attribution across same-direction requested channels; control allocation is unchanged",
            "requested_sum_rms_nm": rms(requested_sum_array),
            "requested_sum_peak_nm": float(np.max(np.abs(requested_sum_array))),
            "software_limited_sum_rms_nm": rms(held_sum_array),
            "software_limited_sum_peak_nm": float(np.max(np.abs(held_sum_array))),
            "actual_applied_sum_rms_nm": rms(actual_interval_sum_array),
            "actual_applied_sum_peak_nm": float(
                np.max(np.abs(actual_interval_sum_array))
            ),
            "requested_minus_actual_rms_nm": rms(
                requested_sum_array - actual_interval_sum_array
            ),
            "requested_minus_actual_peak_nm": float(
                np.max(np.abs(requested_sum_array - actual_interval_sum_array))
            ),
            "channels_eaten_by_final_saturation": {
                channel: {
                    "rms_nm": rms(np.asarray(values)),
                    "peak_abs_nm": float(np.max(np.abs(values))),
                    "mean_abs_nm": float(np.mean(np.abs(values))),
                }
                for channel, values in saturation_eaten.items()
            },
        },
        "payload_gt": {
            "remained_in_basket": payload_remained_in_basket,
            "longitudinal_center_limit_m": longitudinal_limit,
            "containment_check": "post-hoc only: |longitudinal center| <= geometry-derived center limit + 0.005 m contact tolerance and body-frame center height >= 0.14 m",
            "relative_position_body_min_m": payload_position_min.tolist(),
            "relative_position_body_max_m": payload_position_max.tolist(),
            "relative_longitudinal_position_range_m": [
                float(np.min(arrays["payload_position"][:, 1])),
                float(np.max(arrays["payload_position"][:, 1])),
            ],
            "relative_longitudinal_speed_rms_m_s": rms(relative_velocity[:, 1]),
            "relative_longitudinal_speed_peak_m_s": float(
                np.max(np.abs(relative_velocity[:, 1]))
            ),
            "motion_decay": {
                "first_1s_speed_rms_m_s": rms(
                    relative_velocity[early_motion_mask, 1]
                ),
                "terminal_1s_speed_rms_m_s": rms(
                    relative_velocity[terminal_motion_mask, 1]
                ),
                "terminal_to_first_rms_ratio": rms(
                    relative_velocity[terminal_motion_mask, 1]
                )
                / max(rms(relative_velocity[early_motion_mask, 1]), 1e-12),
                "terminal_1s_position_span_m": float(
                    np.ptp(arrays["payload_position"][terminal_motion_mask, 1])
                ),
            },
            "wall_collision_episode_count": len(gt.collision_episodes),
            "force_bearing_wall_collision_episode_count": len(
                force_bearing_episodes
            ),
            "force_bearing_threshold_n": 0.01,
            "wall_collision_times_s": [
                event["time_s"] for event in gt.collision_episodes
            ],
            "maximum_wall_normal_force_n": float(np.max(arrays["wall_force"])),
            "theta_eq_gt_min_deg": math.degrees(float(np.min(arrays["theta_eq_gt"]))),
            "theta_eq_gt_max_deg": math.degrees(float(np.max(arrays["theta_eq_gt"]))),
            "collision_transient": collision_metrics,
        },
        "disturbance_rejection": {
            "architecture": "bounded matched innovation -> single Q-filter -> augmentation authority bound -> optional final slew",
            "q_filter_cutoff_hz": disturbance_config.q_filter_cutoff_hz,
            "augmentation_authority_bound_nm": disturbance_config.augmentation_authority_bound_nm,
            "slew_limiter_enabled": disturbance_config.augmentation_slew_rate_nm_s
            is not None,
            "augmentation_slew_rate_nm_s": disturbance_config.augmentation_slew_rate_nm_s,
            "observation_count": compensator.observation_count,
            "projection_clip_count": compensator.clipped_observation_count,
            "projection_clip_ratio": compensator.clipped_observation_count
            / max(compensator.observation_count, 1),
            "innovation_scaled_rms": rms(np.asarray(innovation_rms)),
            "matched_residual_fraction_mean": float(np.mean(matched_fractions)),
            "raw_equivalent_disturbance_rms_nm": rms(np.asarray(raw_disturbances)),
            "projected_equivalent_disturbance_rms_nm": rms(
                np.asarray(projected_disturbances)
            ),
            "u_lqr_rms_nm": rms(np.asarray(lqr_commands)),
            "requested_u_dr_rms_nm": rms(np.asarray(requested_u_dr_commands)),
            "requested_u_dr_peak_nm": float(
                np.max(np.abs(requested_u_dr_commands))
            ),
            "u_dr_rms_nm": rms(np.asarray(u_dr_commands)),
            "u_dr_peak_nm": float(np.max(np.abs(u_dr_commands))),
            "authority_limit_count": compensator.authority_limit_count,
            "authority_limit_hit_ratio": compensator.authority_limit_count
            / max(compensator.command_count, 1),
            "slew_limit_count": compensator.slew_limit_count,
            "slew_limit_hit_ratio": compensator.slew_limit_count
            / max(compensator.command_count, 1),
        },
        "history_50hz": histories,
        "final_qpos": sim.data.qpos.tolist(),
        "final_qvel": sim.data.qvel.tolist(),
    }
    result["estimator_error_at_500hz"] = {
        "position_m": estimator_error_metrics(estimator_errors["position_m"]),
        "velocity_m_s": estimator_error_metrics(estimator_errors["velocity_m_s"]),
        "pitch_deg": estimator_error_metrics(
            estimator_errors["pitch_rad"], 180.0 / math.pi
        ),
        "pitch_rate_deg_s": estimator_error_metrics(
            estimator_errors["pitch_rate_rad_s"], 180.0 / math.pi
        ),
    }
    result["imu_hardware_diagnostics"] = sim.imu_diagnostics(
        include_hidden_truth=True
    )
    if capture_commands:
        result["_held_sum_commands_nm"] = held_sum_torques
    return result
