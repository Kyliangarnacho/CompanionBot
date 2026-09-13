"""Moving-payload benchmark for fixed LQR plus two-timescale disturbance rejection."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import (
    AutoProbeManager,
    DiscreteStateSpaceModel,
    TwoTimescaleDisturbanceCompensator,
    auto_probe_config_from_dict,
    disturbance_rejection_config_from_dict,
)
from sim import MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
DR_CONFIG_PATH = MODEL_DIR / "disturbance_rejection_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
NOMINAL_OFFLINE_PATH = MODEL_DIR / "full_state_identification_results.json"
RESULTS_PATH = MODEL_DIR / "two_timescale_disturbance_results.json"


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


def pe_diagnostics(window: deque, dimension: int) -> tuple[float, float, np.ndarray]:
    if not window:
        return 0.0, float("inf"), np.zeros(dimension)
    matrix = np.asarray(window, dtype=float)
    column_rms = np.sqrt(np.mean(matrix**2, axis=0))
    normalized = matrix / np.maximum(column_rms, 1e-12)
    gram = normalized.T @ normalized / len(normalized)
    eigenvalues = np.linalg.eigvalsh(gram)
    minimum = float(max(eigenvalues[0], 0.0))
    condition = (
        float(eigenvalues[-1] / minimum) if minimum > 0.0 else float("inf")
    )
    return minimum, condition, column_rms


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
        self.previous_walls: set[str] = set()
        self.collision_episodes: list[dict] = []

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
                continue
            wall_name = self.wall_ids[other]
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
    enable_probe: bool,
    enable_slow: bool,
    enable_fast: bool,
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
    pe_config = raw["discrete_identification"]
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
    gt = PayloadGroundTruthRecorder(sim)
    compensator = TwoTimescaleDisturbanceCompensator(
        nominal_model,
        state_scales,
        input_scale,
        disturbance_rejection_config_from_dict(dr_raw),
    )
    probe_manager = (
        AutoProbeManager(
            auto_probe_config_from_dict(
                raw,
                probe_seed=int(raw["probe"]["seed"]),
            )
        )
        if enable_probe
        else None
    )
    information_window = deque(
        maxlen=round(
            float(pe_config["identifiability_window_s"]) * controller_hz
        )
    )
    minimum_samples = int(pe_config["minimum_window_samples"])

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
    slow_commands: list[float] = []
    fast_commands: list[float] = []
    probe_commands: list[float] = []
    raw_disturbances: list[float] = []
    projected_disturbances: list[float] = []
    innovation_rms: list[float] = []
    matched_fractions: list[float] = []
    requested_sum_torques: list[float] = []
    held_sum_torques: list[float] = []
    actual_interval_sum_torques: list[float] = []
    saturation_eaten = {
        "lqr": [],
        "slow": [],
        "fast": [],
        "probe": [],
    }
    histories: list[dict] = []
    saturated_updates = 0
    fallen = False
    previous_probe_active = False

    for interval in range(interval_count):
        start_time = float(sim.data.time)
        state = sim.longitudinal_state(nominal_theta_eq)
        compensation = compensator.command(
            enable_slow=enable_slow, enable_fast=enable_fast
        )
        lqr_sum = -float((nominal_gain @ state).item())
        probe_active = bool(probe_manager.active) if probe_manager else False
        probe_sum = probe_manager.probe_value(start_time) if probe_manager else 0.0
        raw_sum = lqr_sum + compensation.total_nm + probe_sum
        held_sum = float(np.clip(raw_sum, -2.0 * peak, 2.0 * peak))
        saturated_updates += int(
            not np.isclose(raw_sum, held_sum, rtol=0.0, atol=1e-12)
        )

        next_state = state
        actual_sum = held_sum
        for _ in range(physics_steps):
            snapshot = sim.step(held_sum / 2.0, held_sum / 2.0)
            next_state = sim.longitudinal_state(nominal_theta_eq)
            actual_wheel_torque = float(np.max(np.abs(snapshot.applied_ctrl_nm)))
            actual_sum = float(np.sum(snapshot.applied_ctrl_nm))
            payload_position, theta_gt, contacting, force = gt.sample(
                snapshot.time_s, next_state[2], actual_wheel_torque
            )
            times.append(snapshot.time_s)
            positions.append(next_state[0])
            nominal_pitch_errors.append(next_state[2])
            pitch_rates.append(next_state[3])
            wheel_torques.append(actual_wheel_torque)
            sum_torques.append(actual_sum)
            payload_positions.append(payload_position)
            theta_eq_gt.append(theta_gt)
            wall_contact.append(contacting)
            wall_force.append(force)
            fallen |= abs(next_state[2]) >= math.radians(
                benchmark["fall_pitch_error_deg"]
            )
            fallen |= has_chassis_floor_contact(sim)

        observation = compensator.observe(state, actual_sum, next_state)
        channel_requests = {
            "lqr": lqr_sum,
            "slow": compensation.slow_nm,
            "fast": compensation.fast_nm,
            "probe": probe_sum,
        }
        attributed_loss = attribute_final_saturation(
            channel_requests, actual_sum
        )
        lqr_commands.append(lqr_sum)
        slow_commands.append(compensation.slow_nm)
        fast_commands.append(compensation.fast_nm)
        probe_commands.append(probe_sum)
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

        pe_qualified = False
        minimum_eigenvalue = 0.0
        condition_number = float("inf")
        if probe_active and not previous_probe_active:
            information_window.clear()
        if probe_active:
            regressor = np.r_[state / state_scales, actual_sum / input_scale, 1.0]
            information_window.append(regressor)
            minimum_eigenvalue, condition_number, _ = pe_diagnostics(
                information_window, 6
            )
            pe_qualified = bool(
                len(information_window) >= minimum_samples
                and minimum_eigenvalue
                >= float(pe_config["minimum_normalized_gram_eigenvalue"])
                and condition_number
                <= float(pe_config["maximum_normalized_gram_condition_number"])
            )
        else:
            information_window.clear()

        if probe_manager is not None:
            # The old manager remains a stress-excitation asset only.  No fake
            # RLS updates are reported, so it cannot claim learning convergence;
            # its unchanged maximum-duration safety stop ends the session.
            probe_manager.observe(
                start_time,
                observation.normalized_innovation,
                pe_qualified=pe_qualified,
                accepted_updates=0,
                parameter_vector=None,
            )
        if interval % history_period == 0:
            histories.append(
                {
                    "time_s": start_time,
                    "probe_active": probe_active,
                    "scaled_innovation_rms": observation.scaled_innovation_rms,
                    "matched_disturbance_projected_nm": observation.matched_disturbance_projected_nm,
                    "u_lqr_nm": lqr_sum,
                    "u_slow_nm": compensation.slow_nm,
                    "u_fast_nm": compensation.fast_nm,
                    "u_probe_nm": probe_sum,
                    "requested_sum_torque_nm": raw_sum,
                    "software_limited_sum_torque_nm": held_sum,
                    "actual_applied_sum_torque_nm": actual_sum,
                    "final_saturation_eaten_nm": attributed_loss,
                    "minimum_gram_eigenvalue": minimum_eigenvalue,
                    "condition_number": condition_number,
                }
            )
        previous_probe_active = probe_active

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
    probe_start = probe_manager.session_start_s if probe_manager else None
    probe_end = probe_manager.session_end_s if probe_manager else None

    def rms(values: np.ndarray) -> float:
        return float(np.sqrt(np.mean(values**2)))

    requested_sum_array = np.asarray(requested_sum_torques)
    held_sum_array = np.asarray(held_sum_torques)
    actual_interval_sum_array = np.asarray(actual_interval_sum_torques)

    if probe_start is None or probe_end is None:
        phase_metrics = {
            "full": window_metrics(
                arrays["times"],
                arrays["positions"],
                instantaneous_gt_pitch_error,
                0.0,
                float(stress["duration_s"]),
            )
        }
    else:
        phase_metrics = {
            "pre_probe": window_metrics(
                arrays["times"],
                arrays["positions"],
                instantaneous_gt_pitch_error,
                0.0,
                probe_start,
            ),
            "probe": window_metrics(
                arrays["times"],
                arrays["positions"],
                instantaneous_gt_pitch_error,
                probe_start,
                probe_end,
            ),
            "post_probe": window_metrics(
                arrays["times"],
                arrays["positions"],
                instantaneous_gt_pitch_error,
                probe_end,
                float(stress["duration_s"]),
            ),
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
            "theta_reference_changed": False,
            "K_changed": False,
            "slow_enabled": enable_slow,
            "fast_enabled": enable_fast,
        },
        "probe_session": (
            {
                "start_s": probe_start,
                "end_s": probe_end,
                "stop_reason": probe_manager.stop_reason,
                "events": probe_manager.events,
                "amplitude_sum_torque_nm": float(
                    raw["probe"]["amplitude_sum_torque_nm"]
                ),
            }
            if probe_manager
            else None
        ),
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
            "u_slow_rms_nm": rms(np.asarray(slow_commands)),
            "u_fast_rms_nm": rms(np.asarray(fast_commands)),
            "u_slow_peak_nm": float(np.max(np.abs(slow_commands))),
            "u_fast_peak_nm": float(np.max(np.abs(fast_commands))),
            "u_total_compensation_rms_nm": rms(
                np.asarray(slow_commands) + np.asarray(fast_commands)
            ),
            "u_total_compensation_peak_nm": float(
                np.max(
                    np.abs(np.asarray(slow_commands) + np.asarray(fast_commands))
                )
            ),
            "final_slow_estimate_nm": compensator.slow_estimate_nm,
            "final_fast_band_estimate_nm": (
                compensator.fast_lowpass_estimate_nm
                - compensator.slow_estimate_nm
            ),
        },
        "history_50hz": histories,
        "final_qpos": sim.data.qpos.tolist(),
        "final_qvel": sim.data.qvel.tolist(),
    }
    if capture_commands:
        result["_held_sum_commands_nm"] = held_sum_torques
    return result


def strip_repeat_state(result: dict) -> dict:
    clean = dict(result)
    clean.pop("final_qpos", None)
    clean.pop("final_qvel", None)
    return clean


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
    state_scales = np.asarray(offline["fit"]["state_scales"], dtype=float)
    input_scale = float(offline["fit"]["input_scale_nm"])
    peak = float(plant["known"]["wheel_torque_hard_peak_nm"])
    common = dict(
        raw=raw,
        dr_raw=dr_raw,
        reduced=reduced,
        nominal_model=nominal_model,
        nominal_gain=nominal_gain,
        state_scales=state_scales,
        input_scale=input_scale,
        peak=peak,
    )
    a = run_case(
        label="A_frozen_nominal_id_lqr",
        enable_probe=False,
        enable_slow=False,
        enable_fast=False,
        **common,
    )
    b = run_case(
        label="B_frozen_nominal_id_lqr_same_auto_probe",
        enable_probe=True,
        enable_slow=False,
        enable_fast=False,
        **common,
    )
    c = run_case(
        label="C_fixed_lqr_plus_slow_fast_disturbance_rejection",
        enable_probe=True,
        enable_slow=True,
        enable_fast=True,
        **common,
    )
    repeat = run_case(
        label="C_fixed_lqr_plus_slow_fast_disturbance_rejection",
        enable_probe=True,
        enable_slow=True,
        enable_fast=True,
        **common,
    )
    deterministic = bool(
        c["final_qpos"] == repeat["final_qpos"]
        and c["final_qvel"] == repeat["final_qvel"]
        and c["history_50hz"] == repeat["history_50hz"]
    )
    result = {
        "status": "PASS" if not c["fell"] and deterministic else "FAIL",
        "architecture": {
            "control_law": "u = -K_nominal*x + u_slow + u_fast + u_probe",
            "nominal_K_fixed": True,
            "nominal_theta_reference_fixed": True,
            "online_ABc_or_theta_equilibrium_not_used": True,
            "payload_ground_truth_hidden_from_control": True,
            "state_order": ["p", "p_dot", "theta_error_nominal", "theta_dot"],
            "input": "tau_left_plus_tau_right",
            "controller_hz": float(raw["controller_frequency_hz"]),
            "physics_hz": 1.0 / 0.001,
            "disturbance_config": dr_raw,
        },
        "scenario": {
            "model": raw["moving_payload_stress"]["model_file"],
            "payload_mass_kg": raw["moving_payload_stress"]["payload_mass_kg"],
            "basket_contact_friction": raw["moving_payload_stress"]["basket_contact_friction"],
            "duration_s": raw["moving_payload_stress"]["duration_s"],
            "Q_diag_unchanged": offline["identified_lqr"]["Q_diag_unchanged"],
            "R_unchanged": offline["identified_lqr"]["R_unchanged"],
            "per_wheel_peak_nm_unchanged": peak,
        },
        "A_frozen": strip_repeat_state(a),
        "B_frozen_same_auto_probe": strip_repeat_state(b),
        "C_two_timescale": strip_repeat_state(c),
        "C_deterministic_repeat_exact": deterministic,
    }
    RESULTS_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    concise = {
        "status": result["status"],
        "deterministic": deterministic,
        "cases": {},
    }
    for key in ("A_frozen", "B_frozen_same_auto_probe", "C_two_timescale"):
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
            "probe_session": run["probe_session"],
        }
    print(json.dumps(concise, indent=2))


if __name__ == "__main__":
    main()
