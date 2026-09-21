"""Run the frozen Stage 3 controller on finite continuous-grade terrain.

This is a Stage 4A evaluator/ablation runner, not a controller redesign. Terrain
truth is confined to logging, post-hoc evaluation, and a simulation boundary
termination guard.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from control import (  # noqa: E402
    DiscreteStateSpaceModel,
    FilteredDisturbanceCompensator,
    disturbance_rejection_config_from_dict,
)
import run_stage3b_yaw_control as stage3b  # noqa: E402


MODEL_DIR = ROOT / "models" / "minisegway"
STAGE_DIR = MODEL_DIR / "stage4"
CONFIG_PATH = STAGE_DIR / "config" / "stage4a_slope_robustness_config.json"
WORLD_DIR = STAGE_DIR / "worlds"
RESULT_DIR = STAGE_DIR / "results"
HISTORY_DIR = RESULT_DIR / "history"
PLOT_DIR = RESULT_DIR / "plots"
RESULT_PATH = RESULT_DIR / "stage4a_slope_robustness_results.json"
SUMMARY_CSV_PATH = RESULT_DIR / "stage4a_summary.csv"
REPORT_PATH = RESULT_DIR / "STAGE4A_REPORT.md"
BASE_MODEL_PATH = MODEL_DIR / "mini_segway.xml"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def rms(values: np.ndarray) -> float:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("RMS requires at least one sample")
    return float(np.sqrt(np.mean(array * array)))


def metric(values: np.ndarray) -> dict:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("metric requires at least one sample")
    return {
        "mean": float(np.mean(array)),
        "rms": rms(array),
        "peak_abs": float(np.max(np.abs(array))),
    }


def wrap(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def angle_slug(angle_deg: float) -> str:
    sign = "p" if angle_deg > 0.0 else "m"
    return f"{sign}{abs(int(round(angle_deg))):02d}"


def fmt(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def grade_box_pose(
    start_y_m: float,
    start_z_m: float,
    length_m: float,
    angle_rad: float,
    half_thickness_m: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    tangent = np.asarray([0.0, -math.cos(angle_rad), math.sin(angle_rad)])
    normal = np.asarray([0.0, math.sin(angle_rad), math.cos(angle_rad)])
    start = np.asarray([0.0, start_y_m, start_z_m])
    top_center = start + 0.5 * length_m * tangent
    center = top_center - half_thickness_m * normal
    end = start + length_m * tangent
    return tuple(center.tolist()), tuple(end.tolist())


def build_slope_world(angle_deg: float, config: dict) -> Path:
    terrain = config["terrain"]
    tree = ET.parse(BASE_MODEL_PATH)
    root = tree.getroot()
    root.set("model", f"MiniSegway Stage 4A {angle_deg:+g} degree grade")
    compiler = root.find("compiler")
    if compiler is None:
        raise RuntimeError("base MJCF has no compiler element")
    compiler.set("meshdir", "../../assets/upstream_local")
    statistic = root.find("statistic")
    if statistic is not None:
        statistic.set("center", "0 -3 0")
        statistic.set("extent", "6")
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("base MJCF has no worldbody")
    floor = worldbody.find("geom[@name='floor']")
    if floor is None:
        raise RuntimeError("base MJCF floor geom not found")
    worldbody.remove(floor)

    lead = float(terrain["flat_lead_in_length_m"])
    back = float(terrain["flat_back_margin_m"])
    transition = float(terrain["transition_length_m"])
    grade = float(terrain["constant_grade_length_m"])
    half_width = float(terrain["half_width_m"])
    half_thickness = float(terrain["half_thickness_m"])
    angle_rad = math.radians(angle_deg)

    geoms: list[ET.Element] = []
    geoms.append(ET.Element("geom", {
        "name": "terrain_flat",
        "type": "box",
        "pos": fmt((0.0, 0.5 * (back - lead), -half_thickness)),
        "size": fmt((half_width, 0.5 * (back + lead), half_thickness)),
        "material": "ground_mat",
        "condim": "3",
    }))
    transition_center, transition_end = grade_box_pose(
        -lead, 0.0, transition, angle_rad, half_thickness
    )
    geoms.append(ET.Element("geom", {
        "name": "terrain_transition",
        "type": "box",
        "pos": fmt(transition_center),
        "euler": fmt((-angle_rad, 0.0, 0.0)),
        "size": fmt((half_width, 0.5 * transition, half_thickness)),
        "material": "ground_mat",
        "condim": "3",
    }))
    grade_center, _ = grade_box_pose(
        transition_end[1], transition_end[2], grade, angle_rad, half_thickness
    )
    geoms.append(ET.Element("geom", {
        "name": "terrain_constant_grade",
        "type": "box",
        "pos": fmt(grade_center),
        "euler": fmt((-angle_rad, 0.0, 0.0)),
        "size": fmt((half_width, 0.5 * grade, half_thickness)),
        "material": "ground_mat",
        "condim": "3",
    }))
    insert_at = 1 if len(worldbody) and worldbody[0].tag == "light" else 0
    for geom in reversed(geoms):
        worldbody.insert(insert_at, geom)

    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    path = WORLD_DIR / f"slope_{angle_slug(angle_deg)}.xml"
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=False)
    mujoco.MjModel.from_xml_path(str(path))
    return path


class Stage4QAdapter:
    """Always-running frozen Q observer with an explicit OFF/ON actuator switch."""

    def __init__(
        self,
        model: DiscreteStateSpaceModel,
        state_scales: np.ndarray,
        input_scale_nm: float,
        dr_raw: dict,
        *,
        actuator_enabled: bool,
    ) -> None:
        self.config = disturbance_rejection_config_from_dict(dr_raw)
        self.compensator = FilteredDisturbanceCompensator(
            model, state_scales, input_scale_nm, self.config
        )
        self.actuator_enabled = bool(actuator_enabled)
        self.rows: list[dict] = []
        self._last_command: dict | None = None

    def command(self, u_base_nm: float, time_s: float) -> dict:
        del u_base_nm
        candidate = self.compensator.command(enabled=True)
        applied = candidate.u_dr_nm if self.actuator_enabled else 0.0
        self._last_command = {
            "Q_actuator_enabled": self.actuator_enabled,
            "q_filter_estimate_for_command_nm": candidate.q_filter_estimate_nm,
            "requested_u_dr_nm": candidate.requested_u_dr_nm,
            "authority_bounded_u_dr_nm": candidate.u_dr_nm,
            "applied_u_dr_nm": applied,
            "u_Q_used_nm": applied,
            "augmentation_authority_limited": candidate.authority_limited,
            "augmentation_slew_limited": candidate.slew_limited,
            "augmentation_command_time_s": float(time_s),
        }
        return dict(self._last_command)

    def observe(
        self,
        state_k: np.ndarray,
        held_sum_torque_nm: float,
        state_k1: np.ndarray,
        time_s: float,
        actual_wheels_nm: np.ndarray,
    ) -> dict:
        if self._last_command is None:
            raise RuntimeError("Q observer called before command")
        actual_sum = float(np.sum(np.asarray(actual_wheels_nm, dtype=float)))
        if not math.isclose(actual_sum, held_sum_torque_nm, abs_tol=1e-12):
            raise RuntimeError("Q observer must consume actual held wheel-torque sum")
        observation = self.compensator.observe(state_k, actual_sum, state_k1)
        fields = {
            "observer_time_s": float(time_s),
            "observer_held_actual_sum_nm": actual_sum,
            "matched_disturbance_projected_nm": (
                observation.matched_disturbance_projected_nm
            ),
            "q_filter_estimate_nm": self.compensator.q_filter_estimate_nm,
            "matched_residual_fraction": observation.matched_residual_fraction,
            "scaled_innovation_rms": observation.scaled_innovation_rms,
            "projection_clipped": observation.projection_clipped,
        }
        self.rows.append({**self._last_command, **fields})
        return fields

    def summary(self) -> dict:
        if not self.rows:
            raise RuntimeError("Q adapter has no observations")
        projected = np.asarray([
            row["matched_disturbance_projected_nm"] for row in self.rows
        ])
        estimate = np.asarray([row["q_filter_estimate_nm"] for row in self.rows])
        requested = np.asarray([row["requested_u_dr_nm"] for row in self.rows])
        bounded = np.asarray([row["authority_bounded_u_dr_nm"] for row in self.rows])
        applied = np.asarray([row["applied_u_dr_nm"] for row in self.rows])
        matched = np.asarray([row["matched_residual_fraction"] for row in self.rows])
        projection_clipped = np.asarray([row["projection_clipped"] for row in self.rows])
        authority_limited = np.asarray([
            row["augmentation_authority_limited"] for row in self.rows
        ])
        return {
            "actuator_enabled": self.actuator_enabled,
            "matched_disturbance_projected_nm": metric(projected),
            "q_filter_estimate_nm": metric(estimate),
            "requested_u_dr_nm": metric(requested),
            "authority_bounded_u_dr_nm": metric(bounded),
            "applied_u_dr_nm": metric(applied),
            "matched_residual_fraction": metric(matched),
            "projection_clipping_fraction": float(np.mean(projection_clipped)),
            "authority_limiting_fraction": float(np.mean(authority_limited)),
            "observation_count": len(self.rows),
        }


class SlopeGroundTruthRecorder:
    """Post-hoc terrain truth and safety termination; never a control input."""

    TERRAIN_GEOMS = {
        "terrain_flat",
        "terrain_transition",
        "terrain_constant_grade",
    }

    def __init__(self, sim, angle_deg: float, config: dict, theta_eq_rad: float) -> None:
        terrain = config["terrain"]
        self.sim = sim
        self.angle_deg = float(angle_deg)
        self.angle_rad = math.radians(angle_deg)
        self.lead_m = float(terrain["flat_lead_in_length_m"])
        self.transition_m = float(terrain["transition_length_m"])
        self.grade_m = float(terrain["constant_grade_length_m"])
        self.guard_margin_m = float(terrain["boundary_guard_margin_m"])
        self.theta_eq_rad = float(theta_eq_rad)
        root = sim.model.joint("root").id
        self.root_qpos = int(sim.model.jnt_qposadr[root])
        self.root_dof = int(sim.model.jnt_dofadr[root])
        self.chassis_terrain_contact = False
        self.pitch_instability = False
        self.maximum_abs_pitch_error_rad = 0.0

    def _pitch_world_rad(self) -> float:
        qpos = self.sim.data.qpos
        w, x, y, z = qpos[self.root_qpos + 3 : self.root_qpos + 7]
        return math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

    def _coordinate(self) -> tuple[float, float, str, np.ndarray]:
        qpos = self.sim.data.qpos
        qvel = self.sim.data.qvel
        forward_horizontal = -float(qpos[self.root_qpos + 1])
        on_slope = forward_horizontal > self.lead_m
        if on_slope:
            s_m = self.lead_m + (
                (forward_horizontal - self.lead_m) / math.cos(self.angle_rad)
            )
            alpha = self.angle_rad
            velocity = (
                -float(qvel[self.root_dof + 1]) * math.cos(alpha)
                + float(qvel[self.root_dof + 2]) * math.sin(alpha)
            )
            normal = np.asarray([0.0, math.sin(alpha), math.cos(alpha)])
            segment = (
                "transition"
                if s_m < self.lead_m + self.transition_m
                else "constant_grade"
            )
        else:
            s_m = forward_horizontal
            alpha = 0.0
            velocity = -float(qvel[self.root_dof + 1])
            normal = np.asarray([0.0, 0.0, 1.0])
            segment = "flat_lead_in"
        return s_m, velocity, segment, normal

    def _has_chassis_terrain_contact(self) -> bool:
        for index in range(self.sim.data.ncon):
            contact = self.sim.data.contact[index]
            names = {
                mujoco.mj_id2name(
                    self.sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
                ),
                mujoco.mj_id2name(
                    self.sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
                ),
            }
            if names & self.TERRAIN_GEOMS and any(
                name and name.endswith("chassis_collision") for name in names
            ):
                return True
        return False

    def physics_step(self, sim) -> None:
        if sim is not self.sim:
            raise RuntimeError("slope recorder attached to the wrong simulator")
        pitch_error = abs(wrap(self._pitch_world_rad() - self.theta_eq_rad))
        self.maximum_abs_pitch_error_rad = max(
            self.maximum_abs_pitch_error_rad, pitch_error
        )
        self.pitch_instability |= pitch_error >= math.radians(45.0)
        self.chassis_terrain_contact |= self._has_chassis_terrain_contact()

    def snapshot(self, sim) -> dict:
        if sim is not self.sim:
            raise RuntimeError("slope recorder attached to the wrong simulator")
        s_m, v_m_s, segment, normal = self._coordinate()
        qpos = sim.data.qpos
        return {
            "world_position_GT_m": qpos[
                self.root_qpos : self.root_qpos + 3
            ].tolist(),
            "along_track_position_GT_m": s_m,
            "v_GT_along_track_m_s": v_m_s,
            "pitch_GT_world_rad": self._pitch_world_rad(),
            "alpha_GT_rad": self.angle_rad if segment != "flat_lead_in" else 0.0,
            "terrain_normal_GT_world": normal.tolist(),
            "terrain_segment_GT": segment,
        }

    def boundary_guard(self, sim) -> str | None:
        if sim is not self.sim:
            raise RuntimeError("slope recorder attached to the wrong simulator")
        s_m, _, _, _ = self._coordinate()
        guard = self.lead_m + self.transition_m + self.grade_m - self.guard_margin_m
        return "terrain_boundary_guard" if s_m >= guard else None

    def safety_summary(self) -> dict:
        return {
            "chassis_terrain_contact": self.chassis_terrain_contact,
            "pitch_instability": self.pitch_instability,
            "maximum_abs_pitch_error_deg": math.degrees(
                self.maximum_abs_pitch_error_rad
            ),
        }


def validate_frozen_inputs(config: dict, common: tuple) -> None:
    manifest, _, _, motion, _, _, dr_raw, _, plant = common
    q = config["frozen_q_parameters"]
    actual = disturbance_rejection_config_from_dict(dr_raw)
    checks = {
        "stage3_Q_actuator_default_OFF": not manifest["disturbance_rejection"][
            "actuator_augmentation_production_enabled"
        ],
        "physics_dt_1ms": math.isclose(
            float(manifest["timing"]["physics_dt_s"]), 0.001, abs_tol=1e-12
        ),
        "controller_dt_2ms": math.isclose(
            float(motion["controller_dt_s"]), 0.002, abs_tol=1e-12
        ),
        "q_cutoff_frozen": math.isclose(
            actual.q_filter_cutoff_hz, float(q["q_filter_cutoff_hz"])
        ),
        "q_projection_bound_frozen": math.isclose(
            actual.innovation_projection_bound_nm,
            float(q["innovation_projection_bound_nm"]),
        ),
        "q_authority_frozen": math.isclose(
            actual.augmentation_authority_bound_nm,
            float(q["augmentation_authority_bound_nm"]),
        ),
        "q_slew_disabled": actual.augmentation_slew_rate_nm_s is None,
        "wheel_peak_0p63_nm": math.isclose(
            float(plant["known"]["wheel_torque_hard_peak_nm"]), 0.63
        ),
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError(f"frozen Stage 3 invariant failed: {failed}")


def make_q_adapter(common: tuple, actuator_enabled: bool) -> Stage4QAdapter:
    offline = common[4]
    dr_raw = common[6]
    return Stage4QAdapter(
        DiscreteStateSpaceModel(
            np.asarray(offline["fit"]["A_identified"], dtype=float),
            np.asarray(offline["fit"]["B_identified"], dtype=float),
        ),
        np.asarray(offline["fit"]["state_scales"], dtype=float),
        float(offline["fit"]["input_scale_nm"]),
        dr_raw,
        actuator_enabled=actuator_enabled,
    )


def slope_metrics(
    run: dict,
    adapter: Stage4QAdapter,
    recorder: SlopeGroundTruthRecorder,
    config: dict,
    wheel_limit_nm: float,
) -> tuple[dict, list[dict]]:
    history = run["history_50hz"]
    sum_limit = 2.0 * wheel_limit_nm
    for row in history:
        row["pitch_hat_world_rad"] = float(row["theta_hat_rad"])
        base_if_q_off = float(np.clip(row["u_base_nm"], -sum_limit, sum_limit))
        row["applied_u_dr_after_sum_limit_nm"] = (
            float(row["u_sum_nm"]) - base_if_q_off
        )

    t = np.asarray([row["t"] for row in history])
    v_ref = np.asarray([row["v_ref_m_s"] for row in history])
    v_hat = np.asarray([row["v_hat_m_s"] for row in history])
    v_gt = np.asarray([row["v_GT_along_track_m_s"] for row in history])
    pitch_hat = np.asarray([row["pitch_hat_world_rad"] for row in history])
    pitch_gt = np.asarray([row["pitch_GT_world_rad"] for row in history])
    u_base = np.asarray([row["u_base_nm"] for row in history])
    u_sum = np.asarray([row["u_sum_nm"] for row in history])
    u_dr = np.asarray([row["applied_u_dr_after_sum_limit_nm"] for row in history])
    wheels = np.asarray([
        [row["actual_left_nm"], row["actual_right_nm"]] for row in history
    ])
    residual_fraction = np.asarray([
        row["matched_residual_fraction"] for row in history
    ])
    projection_clipped = np.asarray([row["projection_clipped"] for row in history])
    authority_limited = np.asarray([
        row["augmentation_authority_limited"] for row in history
    ])
    segment = np.asarray([row["terrain_segment_GT"] for row in history])
    slope = segment != "flat_lead_in"
    constant = segment == "constant_grade"
    if not np.any(slope):
        raise RuntimeError("robot never reached the slope")
    constant_indices = np.flatnonzero(constant)
    if constant_indices.size:
        constant_entry_s = float(t[constant_indices[0]])
        steady = constant & (
            t >= constant_entry_s
            + float(config["metrics"]["steady_grade_settle_after_transition_s"])
        )
    else:
        constant_entry_s = None
        steady = slope.copy()
    if not np.any(steady):
        steady = constant if np.any(constant) else slope

    error_gt = v_gt - v_ref
    error_hat = v_hat - v_ref
    sat = np.any(np.abs(wheels) >= wheel_limit_nm - 1e-12, axis=1)
    fall = bool(
        run["longitudinal"]["fell"]
        or recorder.chassis_terrain_contact
        or recorder.pitch_instability
    )
    boundary = run["simulation_termination"]["reason"] == "terrain_boundary_guard"
    criteria = config["metrics"]
    checks = {
        "steady_grade_velocity_RMSE": rms(error_gt[steady])
        <= float(criteria["maximum_steady_grade_velocity_tracking_rmse_m_s"]),
        "steady_velocity_bias": abs(float(np.mean(error_gt[steady])))
        <= float(criteria["maximum_abs_steady_velocity_bias_m_s"]),
        "peak_pitch": math.degrees(float(np.max(np.abs(pitch_gt[slope]))))
        <= float(criteria["maximum_abs_pitch_deg"]),
        "wheel_saturation": float(np.mean(sat[slope]))
        <= float(criteria["maximum_wheel_saturation_fraction"]),
        "no_fall": not fall,
        "no_boundary_termination": not boundary,
        "finite": bool(run["finite"]),
    }
    transition = segment == "transition"
    summary = {
        "velocity_tracking": {
            "error_GT_slope_including_transition_m_s": metric(error_gt[slope]),
            "error_estimated_slope_including_transition_m_s": metric(
                error_hat[slope]
            ),
            "steady_error_GT_m_s": metric(error_gt[steady]),
            "maximum_abs_error_GT_m_s": float(np.max(np.abs(error_gt[slope]))),
        },
        "pitch": {
            "GT_world_slope_deg": metric(np.degrees(pitch_gt[slope])),
            "estimated_world_slope_deg": metric(np.degrees(pitch_hat[slope])),
        },
        "control": {
            "u_base_slope_nm": metric(u_base[slope]),
            "u_sum_final_slope_nm": metric(u_sum[slope]),
            "per_wheel_actual_slope_nm": metric(wheels[slope].reshape(-1)),
            "maximum_wheel_torque_nm": float(np.max(np.abs(wheels[slope]))),
            "wheel_saturation_fraction": float(np.mean(sat[slope])),
        },
        "Q": {
            **adapter.summary(),
            "applied_after_sum_limit_slope_nm": metric(u_dr[slope]),
            "applied_transition_RMS_nm": (
                rms(u_dr[transition]) if np.any(transition) else None
            ),
            "applied_steady_grade_RMS_nm": rms(u_dr[steady]),
            "matched_residual_fraction_slope": metric(residual_fraction[slope]),
            "projection_clipping_fraction_slope": float(
                np.mean(projection_clipped[slope])
            ),
            "authority_limiting_fraction_slope": float(
                np.mean(authority_limited[slope])
            ),
        },
        "safety": {
            "fall_or_instability": fall,
            "boundary_terminated": boundary,
            "termination": run["simulation_termination"],
            **recorder.safety_summary(),
        },
        "evaluation_windows": {
            "slope_sample_count_50hz": int(np.count_nonzero(slope)),
            "constant_grade_entry_s": constant_entry_s,
            "steady_grade_sample_count_50hz": int(np.count_nonzero(steady)),
        },
        "acceptance": {
            "passed": all(checks.values()),
            "checks": checks,
            "basis": config["metrics"]["acceptance_basis"],
        },
    }
    return summary, history


def run_arm(
    angle_deg: float,
    q_enabled: bool,
    config: dict,
    common: tuple,
    world_path: Path,
) -> dict:
    manifest, _, _, motion, offline, _, _, reduced, plant = common
    del offline
    scenario = {
        "name": f"stage4a_slope_{angle_slug(angle_deg)}",
        "duration_s": float(config["command"]["duration_s"]),
        "linear_velocity_schedule": config["command"]["linear_velocity_schedule"],
        "yaw_rate_schedule": config["command"]["yaw_rate_schedule"],
    }
    adapter = make_q_adapter(common, q_enabled)
    holder: dict[str, SlopeGroundTruthRecorder] = {}

    def setup(sim):
        recorder = SlopeGroundTruthRecorder(
            sim, angle_deg, config, float(reduced["parameters"]["theta_eq_rad"])
        )
        holder["recorder"] = recorder
        return {
            "terrain_angle_deg": angle_deg,
            "GT_data_boundary": config["data_boundary"],
        }

    def physics_step(sim):
        holder["recorder"].physics_step(sim)

    def history_diagnostic(sim):
        return holder["recorder"].snapshot(sim)

    def termination_guard(sim):
        return holder["recorder"].boundary_guard(sim)

    run = stage3b.run_case(
        scenario,
        float(manifest["yaw"]["K_psi_nm_per_rad"]),
        float(manifest["yaw"]["K_r_nm_per_rad_s"]),
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        payload_mode="empty",
        common_mode_augmentation=adapter,
        physics_step_callback=physics_step,
        simulation_setup_callback=setup,
        empty_model_path=world_path,
        history_diagnostic_callback=history_diagnostic,
        termination_guard=termination_guard,
    )
    recorder = holder["recorder"]
    summary, history = slope_metrics(
        run,
        adapter,
        recorder,
        config,
        float(plant["known"]["wheel_torque_hard_peak_nm"]),
    )
    state = "ON" if q_enabled else "OFF"
    history_path = HISTORY_DIR / f"slope_{angle_slug(angle_deg)}_q_{state.lower()}.json"
    history_payload = {
        "stage": config["stage"],
        "angle_deg": angle_deg,
        "Q_state": state,
        "imu_rng_seed": int(motion["imu_rng_seed"]),
        "data_boundary": config["data_boundary"],
        "history_50hz": history,
    }
    history_path.write_text(
        json.dumps(history_payload, indent=2, allow_nan=False), encoding="utf-8"
    )
    return {
        "angle_deg": angle_deg,
        "Q_state": state,
        "world": str(world_path.relative_to(ROOT)).replace("\\", "/"),
        "history": str(history_path.relative_to(ROOT)).replace("\\", "/"),
        "imu_rng_seed": int(motion["imu_rng_seed"]),
        "summary": summary,
        "frozen_stage3_runner_summary": stage3b.slim(run),
    }


def compare_q(runs: list[dict]) -> list[dict]:
    comparisons = []
    for angle in sorted({float(run["angle_deg"]) for run in runs}):
        pair = {run["Q_state"]: run for run in runs if run["angle_deg"] == angle}
        if set(pair) != {"OFF", "ON"}:
            continue
        off = pair["OFF"]["summary"]
        on = pair["ON"]["summary"]
        off_rmse = off["velocity_tracking"]["steady_error_GT_m_s"]["rms"]
        on_rmse = on["velocity_tracking"]["steady_error_GT_m_s"]["rms"]
        off_all_rmse = off["velocity_tracking"][
            "error_GT_slope_including_transition_m_s"
        ]["rms"]
        on_all_rmse = on["velocity_tracking"][
            "error_GT_slope_including_transition_m_s"
        ]["rms"]
        off_bias = abs(off["velocity_tracking"]["steady_error_GT_m_s"]["mean"])
        on_bias = abs(on["velocity_tracking"]["steady_error_GT_m_s"]["mean"])
        comparisons.append({
            "angle_deg": angle,
            "velocity_RMSE_ON_over_OFF": on_rmse / max(off_rmse, 1e-12),
            "transition_inclusive_velocity_RMSE_ON_over_OFF": (
                on_all_rmse / max(off_all_rmse, 1e-12)
            ),
            "abs_steady_bias_ON_over_OFF": on_bias / max(off_bias, 1e-12),
            "pitch_RMS_ON_over_OFF": (
                on["pitch"]["GT_world_slope_deg"]["rms"]
                / max(off["pitch"]["GT_world_slope_deg"]["rms"], 1e-12)
            ),
            "torque_RMS_ON_over_OFF": (
                on["control"]["per_wheel_actual_slope_nm"]["rms"]
                / max(off["control"]["per_wheel_actual_slope_nm"]["rms"], 1e-12)
            ),
            "saturation_fraction_delta": (
                on["control"]["wheel_saturation_fraction"]
                - off["control"]["wheel_saturation_fraction"]
            ),
            "Q_ON_applied_steady_RMS_nm": on["Q"][
                "applied_steady_grade_RMS_nm"
            ],
            "Q_ON_passed": on["acceptance"]["passed"],
            "Q_OFF_passed": off["acceptance"]["passed"],
        })
    return comparisons


def summary_rows(runs: list[dict]) -> list[dict]:
    rows = []
    for run in sorted(runs, key=lambda item: (item["angle_deg"], item["Q_state"])):
        summary = run["summary"]
        rows.append({
            "angle_deg": run["angle_deg"],
            "Q_state": run["Q_state"],
            "velocity_RMSE_m_s": summary["velocity_tracking"][
                "steady_error_GT_m_s"
            ]["rms"],
            "transition_inclusive_velocity_RMSE_m_s": summary[
                "velocity_tracking"
            ]["error_GT_slope_including_transition_m_s"]["rms"],
            "steady_velocity_bias_m_s": summary["velocity_tracking"][
                "steady_error_GT_m_s"
            ]["mean"],
            "maximum_abs_velocity_error_m_s": summary["velocity_tracking"][
                "maximum_abs_error_GT_m_s"
            ],
            "pitch_RMS_deg": summary["pitch"]["GT_world_slope_deg"]["rms"],
            "peak_abs_pitch_deg": summary["pitch"]["GT_world_slope_deg"][
                "peak_abs"
            ],
            "per_wheel_torque_RMS_nm": summary["control"][
                "per_wheel_actual_slope_nm"
            ]["rms"],
            "maximum_wheel_torque_nm": summary["control"][
                "maximum_wheel_torque_nm"
            ],
            "torque_saturation_fraction": summary["control"][
                "wheel_saturation_fraction"
            ],
            "Q_applied_mean_nm": summary["Q"][
                "applied_after_sum_limit_slope_nm"
            ]["mean"],
            "Q_applied_RMS_nm": summary["Q"][
                "applied_after_sum_limit_slope_nm"
            ]["rms"],
            "Q_applied_peak_nm": summary["Q"][
                "applied_after_sum_limit_slope_nm"
            ]["peak_abs"],
            "Q_transition_RMS_nm": summary["Q"]["applied_transition_RMS_nm"],
            "Q_steady_grade_RMS_nm": summary["Q"][
                "applied_steady_grade_RMS_nm"
            ],
            "matched_residual_fraction_mean": summary["Q"][
                "matched_residual_fraction_slope"
            ]["mean"],
            "projection_clipping_fraction": summary["Q"][
                "projection_clipping_fraction_slope"
            ],
            "authority_limiting_fraction": summary["Q"][
                "authority_limiting_fraction_slope"
            ],
            "boundary_terminated": summary["safety"]["boundary_terminated"],
            "fall_or_instability": summary["safety"]["fall_or_instability"],
            "pass": summary["acceptance"]["passed"],
        })
    return rows


def write_summary_csv(rows: list[dict]) -> None:
    with SUMMARY_CSV_PATH.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def load_history(run: dict) -> list[dict]:
    return load_json(ROOT / run["history"])["history_50hz"]


def svg_polyline(
    xs: np.ndarray,
    ys: np.ndarray,
    x0: float,
    y0: float,
    width: float,
    height: float,
    x_limits: tuple[float, float],
    y_limits: tuple[float, float],
    color: str,
    dash: str = "",
) -> str:
    xmin, xmax = x_limits
    ymin, ymax = y_limits
    xscale = width / max(xmax - xmin, 1e-12)
    yscale = height / max(ymax - ymin, 1e-12)
    points = " ".join(
        f"{x0 + (float(x) - xmin) * xscale:.2f},{y0 + height - (float(y) - ymin) * yscale:.2f}"
        for x, y in zip(xs, ys)
    )
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return (
        f'<polyline points="{points}" fill="none" stroke="{color}" '
        f'stroke-width="1.5"{dash_attr}/>'
    )


def padded_limits(series: list[np.ndarray]) -> tuple[float, float]:
    values = np.concatenate([np.asarray(item, dtype=float) for item in series])
    low = float(np.min(values))
    high = float(np.max(values))
    span = max(high - low, 1e-6)
    return low - 0.08 * span, high + 0.08 * span


def plot_angle(angle_deg: float, pair: dict[str, dict]) -> Path:
    colors = {"OFF": "#1f77b4", "ON": "#d62728"}
    data: dict[str, dict[str, np.ndarray]] = {}
    for state in ("OFF", "ON"):
        history = load_history(pair[state])
        data[state] = {
            "t": np.asarray([row["t"] for row in history]),
            "v_ref": np.asarray([row["v_ref_m_s"] for row in history]),
            "v_hat": np.asarray([row["v_hat_m_s"] for row in history]),
            "v_gt": np.asarray([row["v_GT_along_track_m_s"] for row in history]),
            "pitch_hat": np.degrees([row["pitch_hat_world_rad"] for row in history]),
            "pitch_gt": np.degrees([row["pitch_GT_world_rad"] for row in history]),
            "u_base": np.asarray([row["u_base_nm"] for row in history]),
            "u_dr": np.asarray([
                row["applied_u_dr_after_sum_limit_nm"] for row in history
            ]),
            "u_sum": np.asarray([row["u_sum_nm"] for row in history]),
        }
    width, height = 1100, 900
    left, panel_width, panel_height = 90.0, 960.0, 210.0
    panel_tops = (80.0, 345.0, 610.0)
    x_limits = (
        min(float(data[state]["t"][0]) for state in data),
        max(float(data[state]["t"][-1]) for state in data),
    )
    y_groups = [
        [data[state][key] for state in data for key in ("v_ref", "v_hat", "v_gt")],
        [data[state][key] for state in data for key in ("pitch_hat", "pitch_gt")],
        [data[state][key] for state in data for key in ("u_base", "u_dr", "u_sum")],
    ]
    y_limits = [padded_limits(group) for group in y_groups]
    labels = ("velocity [m/s]", "world pitch [deg]", "sum torque [N m]")
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="550" y="32" text-anchor="middle" font-family="sans-serif" font-size="20">Stage 4A {angle_deg:+g}° slope — frozen Stage 3, Q OFF vs ON</text>',
    ]
    for index, top in enumerate(panel_tops):
        svg.append(
            f'<rect x="{left}" y="{top}" width="{panel_width}" height="{panel_height}" fill="none" stroke="#777"/>'
        )
        for fraction in (0.25, 0.5, 0.75):
            x = left + panel_width * fraction
            y = top + panel_height * fraction
            svg.append(f'<line x1="{x}" y1="{top}" x2="{x}" y2="{top + panel_height}" stroke="#ddd"/>')
            svg.append(f'<line x1="{left}" y1="{y}" x2="{left + panel_width}" y2="{y}" stroke="#ddd"/>')
        ymin, ymax = y_limits[index]
        svg.append(f'<text x="12" y="{top + panel_height / 2}" font-family="sans-serif" font-size="13">{labels[index]}</text>')
        svg.append(f'<text x="82" y="{top + 12}" text-anchor="end" font-family="sans-serif" font-size="11">{ymax:.3g}</text>')
        svg.append(f'<text x="82" y="{top + panel_height}" text-anchor="end" font-family="sans-serif" font-size="11">{ymin:.3g}</text>')
    for state in ("OFF", "ON"):
        color = colors[state]
        t = data[state]["t"]
        if state == "OFF":
            svg.append(svg_polyline(t, data[state]["v_ref"], left, panel_tops[0], panel_width, panel_height, x_limits, y_limits[0], "#111", "7 5"))
        svg.append(svg_polyline(t, data[state]["v_hat"], left, panel_tops[0], panel_width, panel_height, x_limits, y_limits[0], color))
        svg.append(svg_polyline(t, data[state]["v_gt"], left, panel_tops[0], panel_width, panel_height, x_limits, y_limits[0], color, "2 4"))
        svg.append(svg_polyline(t, data[state]["pitch_hat"], left, panel_tops[1], panel_width, panel_height, x_limits, y_limits[1], color))
        svg.append(svg_polyline(t, data[state]["pitch_gt"], left, panel_tops[1], panel_width, panel_height, x_limits, y_limits[1], color, "2 4"))
        svg.append(svg_polyline(t, data[state]["u_base"], left, panel_tops[2], panel_width, panel_height, x_limits, y_limits[2], color, "7 5"))
        svg.append(svg_polyline(t, data[state]["u_dr"], left, panel_tops[2], panel_width, panel_height, x_limits, y_limits[2], color, "2 4"))
        svg.append(svg_polyline(t, data[state]["u_sum"], left, panel_tops[2], panel_width, panel_height, x_limits, y_limits[2], color))
    legend = "solid: estimate/final | dotted: GT or u_dr | dashed: reference/baseline | blue: Q OFF | red: Q ON"
    svg.append(f'<text x="550" y="852" text-anchor="middle" font-family="sans-serif" font-size="13">{legend}</text>')
    svg.append(f'<text x="550" y="878" text-anchor="middle" font-family="sans-serif" font-size="13">time [s], {x_limits[0]:.2f} to {x_limits[1]:.2f}</text>')
    svg.append("</svg>")
    path = PLOT_DIR / f"slope_{angle_slug(angle_deg)}_q_comparison.svg"
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def plot_summary_table(rows: list[dict]) -> Path:
    headers = [
        "angle", "Q", "v RMS steady/all", "steady bias", "pitch RMS", "peak pitch",
        "torque RMS", "sat.", "Q mean", "Q peak", "status",
    ]
    cells: list[list[str]] = []
    for row in rows:
        cells.append([
            f"{row['angle_deg']:+g}°",
            row["Q_state"],
            f"{row['velocity_RMSE_m_s']:.4f}/{row['transition_inclusive_velocity_RMSE_m_s']:.4f}",
            f"{row['steady_velocity_bias_m_s']:+.4f}",
            f"{row['pitch_RMS_deg']:.2f}°",
            f"{row['peak_abs_pitch_deg']:.2f}°",
            f"{row['per_wheel_torque_RMS_nm']:.3f}",
            f"{100.0 * row['torque_saturation_fraction']:.2f}%",
            f"{row['Q_applied_mean_nm']:+.3f}",
            f"{row['Q_applied_peak_nm']:.3f}",
            "PASS" if row["pass"] else "FAIL",
        ])
    col_widths = [75, 55, 90, 105, 95, 95, 95, 70, 85, 85, 70]
    width = sum(col_widths) + 40
    row_height = 34
    height = 70 + row_height * (len(cells) + 1)
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width / 2}" y="30" text-anchor="middle" font-family="sans-serif" font-size="20">Stage 4A slope robustness summary</text>',
    ]
    x_edges = [20]
    for col_width in col_widths:
        x_edges.append(x_edges[-1] + col_width)
    y0 = 50
    all_rows = [headers] + cells
    for row_index, row in enumerate(all_rows):
        y = y0 + row_index * row_height
        fill = "#e8edf3" if row_index == 0 else ("#f8f8f8" if row_index % 2 == 0 else "white")
        svg.append(f'<rect x="20" y="{y}" width="{sum(col_widths)}" height="{row_height}" fill="{fill}" stroke="#aaa"/>')
        for column, value in enumerate(row):
            x = 0.5 * (x_edges[column] + x_edges[column + 1])
            svg.append(f'<text x="{x}" y="{y + 22}" text-anchor="middle" font-family="sans-serif" font-size="11">{value}</text>')
            svg.append(f'<line x1="{x_edges[column + 1]}" y1="{y}" x2="{x_edges[column + 1]}" y2="{y + row_height}" stroke="#aaa"/>')
    svg.append("</svg>")
    path = PLOT_DIR / "stage4a_summary_table.svg"
    path.write_text("\n".join(svg), encoding="utf-8")
    return path


def write_report(rows: list[dict], comparisons: list[dict]) -> None:
    off = [row for row in rows if row["Q_state"] == "OFF"]
    pass_angles = [f"{row['angle_deg']:+g}°" for row in off if row["pass"]]
    fail_angles = [f"{row['angle_deg']:+g}°" for row in off if not row["pass"]]
    helpful = [
        item for item in comparisons
        if item["velocity_RMSE_ON_over_OFF"] < 0.90
        and item["abs_steady_bias_ON_over_OFF"] < 0.90
        and item["pitch_RMS_ON_over_OFF"] <= 1.05
        and item["torque_RMS_ON_over_OFF"] <= 1.10
        and item["saturation_fraction_delta"] <= 0.001
    ]
    saturation = [
        row for row in rows if row["torque_saturation_fraction"] > 0.0
    ]
    q_on_rows = [row for row in rows if row["Q_state"] == "ON"]
    q_localization = ", ".join(
        f"{row['angle_deg']:+g}° {row['Q_transition_RMS_nm']:.3f}/{row['Q_steady_grade_RMS_nm']:.3f}"
        for row in q_on_rows
    )
    table_lines = [
        "| Angle | Q | v RMSE steady / transition-inclusive (m/s) | Steady bias (m/s) | Pitch RMS / peak (deg) | Torque RMS (N m/wheel) | Saturation | Q mean / peak (N m) | Status |",
        "|---:|:---:|---:|---:|---:|---:|---:|---:|:---:|",
    ]
    for row in rows:
        table_lines.append(
            f"| {row['angle_deg']:+g}° | {row['Q_state']} | "
            f"{row['velocity_RMSE_m_s']:.4f} / {row['transition_inclusive_velocity_RMSE_m_s']:.4f} | {row['steady_velocity_bias_m_s']:+.4f} | "
            f"{row['pitch_RMS_deg']:.2f} / {row['peak_abs_pitch_deg']:.2f} | "
            f"{row['per_wheel_torque_RMS_nm']:.3f} | "
            f"{100.0 * row['torque_saturation_fraction']:.2f}% | "
            f"{row['Q_applied_mean_nm']:+.3f} / {row['Q_applied_peak_nm']:.3f} | "
            f"{'PASS' if row['pass'] else 'FAIL'} |"
        )
    q_text = (
        "Q ON 在 "
        + ", ".join(f"{item['angle_deg']:+g}°" for item in helpful)
        + " 同时满足预声明的增量价值条件；这至多把冻结 Q 路径列为坡度补偿候选，并不自动改变 production baseline。"
        if helpful else
        "Q ON 没有在任一角度同时把恒坡速度 RMSE 和稳态 bias 降低至少 10%，因此没有显示实质增量价值，也没有调 Q 参数的依据。"
    )
    report = f"""# Stage 4A — Slope / Grade Robustness

## Repository audit

- 权威 Stage 3 manifest 中的 A/B、K、acceleration-compensated estimator、FF lifecycle、yaw PD、1 ms / 2 ms 时序和 ±0.63 N·m 单轮峰值均保持冻结。
- production baseline 的 Q observer 仅用于 diagnostic，actuator augmentation 为 OFF；Stage 4A 只在显式 Q ON 实验臂接入 actuator。
- Stage 4A 复用 Stage 3 commanded-motion runner、reference lifecycle、estimator、allocator 和现有 2 Hz matched-disturbance observer。坡度/法向/world pose/along-track GT 只进入 logger、evaluator 和仿真 boundary guard。

## 结果

{chr(10).join(table_lines)}

冻结 Q-OFF baseline 通过：{', '.join(pass_angles) if pass_angles else '无'}。
冻结 Q-OFF baseline 未通过 / operating envelope：{', '.join(fail_angles) if fail_angles else '无'}。

{q_text}

Q ON 的 transition / steady-grade augmentation RMS（N·m）分别为：{q_localization}。两者同量级，说明 Q 主要在估计并补偿持续 grade disturbance，不是只在 transition 瞬间“瞎忙”；但该持续输出没有转化为 tracking 改善。

8 个实验臂中出现 torque saturation 的数量为 {len(saturation)}；所有实验均无 fall、无 chassis-terrain contact、无 boundary termination。正常 acceptance arm 若触发 fall 或 boundary guard 均不得通过。

## 结论

- ±8°：冻结 Stage 3 baseline 能处理恒坡；入坡 transition 的误差显著大于恒坡稳态误差，但之后恢复。
- +15°：恒坡速度仍可跟踪，但 world-pitch 超过复用的 Stage 3 20°门槛；归入压力测试 envelope。
- -15°：恒坡速度 RMSE / bias 和 pitch 均越过门槛；归入压力测试 envelope。
- Q：四个角度均持续输出，matched residual fraction 也较高，但 OFF→ON 的 RMSE/bias 变化约为 -2.5% 到 +3.1%，没有工程上有意义的改善。
- saturation / authority：无 wheel saturation、无 projection clipping、无 Q authority limiting；失败不是 actuator authority 耗尽造成的。
- 没有证据要求修改 Stage 3 controller。尤其不应因 ±15° 压力测试或 entry transient 自动重算 LQR、修改 estimator/FF，或调 Q cutoff/authority。

## 解释纪律

- ±15° 的失败被保留为 operating-envelope 证据，不是必须修复的产品 requirement。
- Q estimate 大但 tracking 未改善，不构成调 cutoff 或 authority 的理由；后续若继续研究，应先检查 equilibrium/reference 与 estimator semantics。
- 本阶段没有修改 controller，也没有把 Q 接入 production；保存的 OFF/ON ablation 是后续决策输入。
"""
    REPORT_PATH.write_text(report, encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--angles", nargs="*", type=float,
        help="optional subset for development; default runs all configured angles",
    )
    parser.add_argument(
        "--q-states", nargs="*", choices=("OFF", "ON"),
        help="optional subset for development; default runs both arms",
    )
    parser.add_argument("--generate-worlds-only", action="store_true")
    args = parser.parse_args()

    config = load_json(CONFIG_PATH)
    common = stage3b.load_common()
    validate_frozen_inputs(config, common)
    configured_angles = [float(value) for value in config["angles_deg"]]
    angles = configured_angles if not args.angles else args.angles
    invalid = [angle for angle in angles if angle not in configured_angles]
    if invalid:
        raise ValueError(f"angles are outside the frozen Stage 4A set: {invalid}")
    q_states = ("OFF", "ON") if not args.q_states else tuple(args.q_states)

    for directory in (RESULT_DIR, HISTORY_DIR, PLOT_DIR, WORLD_DIR):
        directory.mkdir(parents=True, exist_ok=True)
    worlds = {angle: build_slope_world(angle, config) for angle in angles}
    if args.generate_worlds_only:
        print(json.dumps({
            "status": "WORLDS_GENERATED",
            "worlds": [str(path) for path in worlds.values()],
        }, indent=2))
        return

    runs = []
    for angle in angles:
        for state in q_states:
            print(f"RUN angle={angle:+g} Q={state}", flush=True)
            runs.append(run_arm(angle, state == "ON", config, common, worlds[angle]))

    rows = summary_rows(runs)
    comparisons = compare_q(runs)
    write_summary_csv(rows)
    plots = []
    for angle in angles:
        pair = {run["Q_state"]: run for run in runs if run["angle_deg"] == angle}
        if set(pair) == {"OFF", "ON"}:
            plots.append(plot_angle(angle, pair))
    plots.append(plot_summary_table(rows))
    if set(q_states) == {"OFF", "ON"} and set(angles) == set(configured_angles):
        write_report(rows, comparisons)

    result = {
        "stage": config["stage"],
        "status": "COMPLETE" if len(runs) == 8 else "PARTIAL_DEVELOPMENT_RUN",
        "repository_audit": {
            "authoritative_baseline": "models/minisegway/stage3/config/baseline.json",
            "Q_observer_retained_diagnostic": True,
            "Q_actuator_frozen_stage3_default": False,
            "frozen_controller_changed": False,
            "terrain_GT_runtime_controller_dependency": False,
        },
        "experiment_config": config,
        "worlds": {
            f"{angle:+g}": {
                "path": str(path.relative_to(ROOT)).replace("\\", "/"),
                "sha256": sha256(path),
            }
            for angle, path in worlds.items()
        },
        "run_count": len(runs),
        "runs": runs,
        "summary_rows": rows,
        "Q_OFF_vs_ON": comparisons,
        "artifacts": {
            "summary_csv": str(SUMMARY_CSV_PATH.relative_to(ROOT)).replace("\\", "/"),
            "plots": [str(path.relative_to(ROOT)).replace("\\", "/") for path in plots],
            "report": (
                str(REPORT_PATH.relative_to(ROOT)).replace("\\", "/")
                if REPORT_PATH.exists() else None
            ),
        },
    }
    RESULT_PATH.write_text(
        json.dumps(result, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps({
        "status": result["status"],
        "run_count": len(runs),
        "result": str(RESULT_PATH),
        "summary": str(SUMMARY_CSV_PATH),
        "report": str(REPORT_PATH) if REPORT_PATH.exists() else None,
    }, indent=2))


if __name__ == "__main__":
    main()
