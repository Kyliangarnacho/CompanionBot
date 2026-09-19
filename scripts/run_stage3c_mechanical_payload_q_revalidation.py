"""Stage 3C-R mechanical payload conditioning and frozen gated-Q revalidation."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from control import DiscreteStateSpaceModel
import run_stage3b_yaw_control as stage3b
from run_stage3c_dynamic_payload_q import SCENARIO, Stage3CQAdapter, arm_summary


MODEL_DIR = ROOT / "models" / "minisegway"
OLD_RESULT_PATH = MODEL_DIR / "stage3" / "results" / "stage3c_failed_payload_summary.json"
RESULT_PATH = MODEL_DIR / "stage3" / "results" / "stage3c_mechanical_payload_q_revalidation_results.json"
LEARNING_LOG_PATH = ROOT / "LEARNING_LOG.md"

PAYLOAD_MASS_KG = 0.10
SLIDING_FRICTION = 0.35
WALL_SOLREF = np.asarray([0.020, 1.50], dtype=float)
WALL_SOLIMP = np.asarray([0.70, 0.95, 0.005, 0.50, 2.0], dtype=float)
WALL_PRIORITY = 1
FROZEN_THRESHOLDS_NM = (0.0055667353, 0.0033400412)
WALL_NAMES = (
    "basket_front_wall",
    "basket_rear_wall",
    "basket_left_wall",
    "basket_right_wall",
)


def metric(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "rms": float(np.sqrt(np.mean(values**2))),
        "peak_abs": float(np.max(np.abs(values))),
    }


def configure_mechanical_payload(sim) -> dict:
    payload_body = sim.model.body("moving_payload").id
    payload_geom = sim.model.geom("moving_payload_collision").id
    floor_geom = sim.model.geom("basket_floor").id
    full_size = 2.0 * np.asarray(sim.model.geom_size[payload_geom, :3], dtype=float)
    x, y, z = full_size
    inertia = PAYLOAD_MASS_KG / 12.0 * np.asarray(
        [y * y + z * z, x * x + z * z, x * x + y * y], dtype=float
    )
    previous = {
        "payload_mass_kg": float(sim.model.body_mass[payload_body]),
        "payload_inertia_kg_m2": sim.model.body_inertia[payload_body].tolist(),
        "payload_friction": sim.model.geom_friction[payload_geom].tolist(),
        "floor_friction": sim.model.geom_friction[floor_geom].tolist(),
        "wall_solref": sim.model.geom_solref[sim.model.geom(WALL_NAMES[0]).id].tolist(),
        "wall_solimp": sim.model.geom_solimp[sim.model.geom(WALL_NAMES[0]).id].tolist(),
        "wall_priority": int(sim.model.geom_priority[sim.model.geom(WALL_NAMES[0]).id]),
    }

    sim.model.body_mass[payload_body] = PAYLOAD_MASS_KG
    sim.model.body_inertia[payload_body] = inertia
    friction = np.asarray([SLIDING_FRICTION, 0.005, 0.0001], dtype=float)
    sim.model.geom_friction[payload_geom] = friction
    sim.model.geom_friction[floor_geom] = friction
    for name in WALL_NAMES:
        geom = sim.model.geom(name).id
        sim.model.geom_friction[geom] = friction
        sim.model.geom_priority[geom] = WALL_PRIORITY
        sim.model.geom_solref[geom] = WALL_SOLREF
        sim.model.geom_solimp[geom] = WALL_SOLIMP
    mujoco.mj_setConst(sim.model, sim.data)
    mujoco.mj_forward(sim.model, sim.data)
    return {
        "payload_mass_kg": PAYLOAD_MASS_KG,
        "payload_full_size_m": full_size.tolist(),
        "payload_diagonal_inertia_kg_m2": inertia.tolist(),
        "floor_and_payload_friction": friction.tolist(),
        "wall_friction": friction.tolist(),
        "wall_contact_priority": WALL_PRIORITY,
        "wall_padding": {
            "implementation": "existing wall geometry with MuJoCo native compliant/damped contact; no controller force and no payload constraint",
            "geometry_thickness_added_m": 0.0,
            "solref": WALL_SOLREF.tolist(),
            "solref_semantics": "20 ms positive-format time constant, damping ratio 1.5",
            "solimp": WALL_SOLIMP.tolist(),
            "solimp_semantics": "reduced minimum impedance with 5 mm impedance transition width",
        },
        "previous_stage3c_runtime_values": previous,
    }


def add_payload_velocity(history: list[dict]) -> None:
    times = np.asarray([row["t"] for row in history], dtype=float)
    positions = np.asarray([
        row["payload_posthoc_relative_position_body_m"][1] for row in history
    ], dtype=float)
    relative_velocity = np.gradient(positions, times)
    for row, velocity in zip(history, relative_velocity):
        row["payload_posthoc_relative_longitudinal_velocity_m_s"] = float(velocity)


def stop_go_diagnostic(history: list[dict], collisions: list[float]) -> dict:
    add_payload_velocity(history)
    times = np.asarray([row["t"] for row in history], dtype=float)
    v_ref = np.asarray([row["v_ref_m_s"] for row in history], dtype=float)
    v_hat = np.asarray([row["v_hat_m_s"] for row in history], dtype=float)
    v_gt = np.asarray([row["v_GT_m_s"] for row in history], dtype=float)
    relative_velocity = np.asarray([
        row["payload_posthoc_relative_longitudinal_velocity_m_s"]
        for row in history
    ])
    theta = np.asarray([row["theta_hat_rad"] for row in history], dtype=float)
    u_fb = np.asarray([row["u_fb_nm"] for row in history], dtype=float)
    hold = np.asarray([
        row["velocity_lifecycle_phase"] == "VELOCITY_HOLD"
        and abs(row["v_ref_m_s"]) > 1e-6
        for row in history
    ])
    padded = np.r_[False, hold, False]
    starts = np.flatnonzero(~padded[:-1] & padded[1:])
    stops = np.flatnonzero(padded[:-1] & ~padded[1:])
    drops: list[dict] = []
    segments: list[dict] = []
    for start, stop in zip(starts, stops):
        start_time = float(times[start])
        end_time = float(times[stop - 1])
        target = float(np.median(v_ref[start:stop]))
        direction = math.copysign(1.0, target)
        segment_collisions = [
            float(value) for value in collisions
            if start_time <= float(value) <= end_time
        ]
        segments.append({
            "start_time_s": start_time,
            "end_time_s": end_time,
            "v_ref_m_s": target,
            "collision_times_s": segment_collisions,
            "body_forward_GT_velocity": metric(v_gt[start:stop]),
            "estimated_velocity": metric(v_hat[start:stop]),
            "payload_relative_longitudinal_velocity": metric(
                relative_velocity[start:stop]
            ),
            "pitch_hat_deg": metric(np.degrees(theta[start:stop])),
            "u_fb_nm": metric(u_fb[start:stop]),
        })
        for collision_time in segment_collisions:
            before = (
                (times >= collision_time - 0.15)
                & (times < collision_time)
                & hold
            )
            after = (
                (times >= collision_time)
                & (times <= collision_time + 0.25)
                & hold
            )
            if not np.any(before) or not np.any(after):
                continue
            pre_progress = float(np.median(direction * v_gt[before]))
            post_min = float(np.min(direction * v_gt[after]))
            drop = max(0.0, pre_progress - post_min)
            drops.append({
                "collision_time_s": collision_time,
                "v_ref_m_s": target,
                "pre_collision_progress_velocity_m_s": pre_progress,
                "post_collision_min_progress_velocity_m_s": post_min,
                "velocity_drop_m_s": drop,
                "large_drop": drop >= 0.05,
            })
    return {
        "definition": "within constant nonzero VELOCITY_HOLD, compare 0.15 s pre-collision median progress velocity with minimum over 0.25 s post-collision; large drop >=0.05 m/s",
        "largest_collision_correlated_velocity_drop_m_s": max(
            (item["velocity_drop_m_s"] for item in drops), default=0.0
        ),
        "large_collision_correlated_drop_count": sum(
            item["large_drop"] for item in drops
        ),
        "collision_windows": drops,
        "constant_velocity_segments": segments,
    }


def run_arm(
    common: tuple,
    model: DiscreteStateSpaceModel,
    state_scales: np.ndarray,
    input_scale: float,
    sum_limit: float,
    K_psi: float,
    K_r: float,
    *,
    q_enabled: bool,
) -> tuple[dict, Stage3CQAdapter]:
    dr_raw = common[6]
    adapter = Stage3CQAdapter(
        model, state_scales, input_scale, dr_raw, sum_limit,
        thresholds=FROZEN_THRESHOLDS_NM,
        actuator_enabled=q_enabled,
    )
    run = stage3b.run_case(
        SCENARIO, K_psi, K_r,
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        payload_mode="free",
        use_accepted_free_payload_initial_state=True,
        common_mode_augmentation=adapter,
        simulation_setup_callback=configure_mechanical_payload,
    )
    return run, adapter


def summarize_arm(run: dict, adapter: Stage3CQAdapter, motion_config: dict) -> dict:
    summary = arm_summary(run, adapter, motion_config)
    collisions = summary["payload_evaluator_posthoc"]["wall_collision_times_s"]
    summary["stop_go_posthoc"] = stop_go_diagnostic(
        run["history_50hz"], collisions
    )
    return summary


def mechanical_health_gate(summary: dict) -> dict:
    checks = {
        "no_fall": not summary["balance"]["fall"],
        "payload_contained": summary["payload_evaluator_posthoc"][
            "remained_in_basket"
        ],
        "saturation_at_most_1pct": summary["control"][
            "per_wheel_saturation_fraction"
        ] <= 0.01,
        "velocity_RMSE_at_most_0p08": summary["tracking"][
            "velocity_error_estimated_m_s"
        ]["rms"] <= 0.08,
        "pitch_peak_at_most_25deg": summary["balance"][
            "pitch_peak_GT_deg"
        ] <= 25.0,
        "no_repeated_large_collision_correlated_drops": summary[
            "stop_go_posthoc"
        ]["large_collision_correlated_drop_count"] <= 2,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "failed_checks": [key for key, value in checks.items() if not value],
    }


def mechanical_comparison(old: dict, new: dict) -> dict:
    old_payload = old["payload_evaluator_posthoc"]
    new_payload = new["payload_evaluator_posthoc"]
    return {
        "old_stage3c_Q_OFF": {
            "fall": old["balance"]["fall"],
            "contained": old_payload["remained_in_basket"],
            "velocity_RMSE_m_s": old["tracking"]["velocity_error_estimated_m_s"]["rms"],
            "pitch_RMS_deg": old["balance"]["pitch_RMS_GT_deg"],
            "pitch_peak_deg": old["balance"]["pitch_peak_GT_deg"],
            "saturation_fraction": old["control"]["per_wheel_saturation_fraction"],
            "collision_count": old_payload["force_bearing_wall_collision_episode_count"],
            "maximum_wall_normal_force_n": old_payload["maximum_wall_normal_force_n"],
        },
        "new_mechanical_Q_OFF": {
            "fall": new["balance"]["fall"],
            "contained": new_payload["remained_in_basket"],
            "velocity_RMSE_m_s": new["tracking"]["velocity_error_estimated_m_s"]["rms"],
            "pitch_RMS_deg": new["balance"]["pitch_RMS_GT_deg"],
            "pitch_peak_deg": new["balance"]["pitch_peak_GT_deg"],
            "saturation_fraction": new["control"]["per_wheel_saturation_fraction"],
            "collision_count": new_payload["force_bearing_wall_collision_episode_count"],
            "maximum_wall_normal_force_n": new_payload["maximum_wall_normal_force_n"],
            "large_collision_correlated_velocity_drop_count": new[
                "stop_go_posthoc"
            ]["large_collision_correlated_drop_count"],
        },
    }


def q_acceptance(q_off: dict, q_on: dict) -> dict:
    position_ratio = (
        q_on["tracking"]["persistent_position_error_slow_RMS_m"]
        / max(q_off["tracking"]["persistent_position_error_slow_RMS_m"], 1e-12)
    )
    feedback_ratio = q_on["control"]["u_fb_slow_RMS_nm"] / max(
        q_off["control"]["u_fb_slow_RMS_nm"], 1e-12
    )
    checks = {
        "no_fall": not q_on["balance"]["fall"],
        "payload_contained": q_on["payload_evaluator_posthoc"]["remained_in_basket"],
        "no_meaningful_additional_saturation": q_on["control"][
            "per_wheel_saturation_fraction"
        ] <= q_off["control"]["per_wheel_saturation_fraction"] + 0.001,
        "velocity_RMSE_within_5pct": q_on["tracking"][
            "velocity_error_estimated_m_s"
        ]["rms"] <= 1.05 * q_off["tracking"]["velocity_error_estimated_m_s"]["rms"],
        "pitch_RMS_within_5pct": q_on["balance"]["pitch_RMS_GT_deg"]
        <= 1.05 * q_off["balance"]["pitch_RMS_GT_deg"],
        "pitch_peak_within_5pct": q_on["balance"]["pitch_peak_GT_deg"]
        <= 1.05 * q_off["balance"]["pitch_peak_GT_deg"],
        "no_Q_induced_chatter": q_on["control"]["u_sum_first_difference_RMS_nm"]
        <= 1.10 * q_off["control"]["u_sum_first_difference_RMS_nm"],
        "one_slow_burden_metric_improves_at_least_10pct": min(
            position_ratio, feedback_ratio
        ) <= 0.90,
        "other_slow_burden_metric_not_worse_over_5pct": max(
            position_ratio, feedback_ratio
        ) <= 1.05,
    }
    accepted = all(checks.values())
    return {
        "accepted": accepted,
        "decision": (
            "gated Q may enter production"
            if accepted else "retain Q observer diagnostic-only; actuator augmentation rejected"
        ),
        "persistent_position_error_slow_RMS_ratio": position_ratio,
        "u_fb_slow_RMS_ratio": feedback_ratio,
        "checks": checks,
        "failed_checks": [key for key, value in checks.items() if not value],
    }


def append_learning_log(result: dict) -> None:
    marker = "## Stage 3C-R — mechanically conditioned moving payload"
    existing = LEARNING_LOG_PATH.read_text(encoding="utf-8")
    if marker in existing:
        return
    off = result["arm_A_mechanical_Q_OFF"]["summary"]
    on = result.get("arm_B_mechanical_gated_Q_ON")
    q_sentence = (
        "Arm B was not run because the mechanical Q-OFF health gate failed."
        if on is None else result["Q_acceptance"]["decision"] + "."
    )
    note = f"""

{marker}

- The prior commanded free-payload failure was treated as repeated rigid-wall impact/slosh outside the Q observer's intended slow matched-disturbance responsibility.
- One fixed passive redesign changed the V1 payload to 0.10 kg with consistent box inertia, raised basket/payload sliding friction to 0.35, and used unified native MuJoCo compliant/damped wall contact; controller, estimator, A/B/K/FF/yaw/Q/gate remained frozen.
- Mechanical Q-OFF outcome: fall={off['balance']['fall']}, contained={off['payload_evaluator_posthoc']['remained_in_basket']}, saturation={off['control']['per_wheel_saturation_fraction']:.6f}, large collision-correlated drops={off['stop_go_posthoc']['large_collision_correlated_drop_count']}.
- {q_sentence} No friction, padding, payload-mass, Q, or gate sweep was performed.
"""
    LEARNING_LOG_PATH.write_text(existing.rstrip() + note, encoding="utf-8")


def main() -> None:
    common = stage3b.load_common()
    _, config, dynamic, motion, offline, _, dr_raw, _, plant = common
    A = np.asarray(offline["fit"]["A_identified"], dtype=float)
    B = np.asarray(offline["fit"]["B_identified"], dtype=float)
    model = DiscreteStateSpaceModel(A, B)
    state_scales = np.asarray(offline["fit"]["state_scales"], dtype=float)
    input_scale = float(offline["fit"]["input_scale_nm"])
    sum_limit = 2.0 * float(plant["known"]["wheel_torque_hard_peak_nm"])
    old_result = json.loads(OLD_RESULT_PATH.read_text(encoding="utf-8"))
    yaw_config = json.loads(
        (MODEL_DIR / "stage3" / "config" / "stage3b_yaw_control_config.json").read_text(
            encoding="utf-8"
        )
    )
    yaw = yaw_config["yaw_PD"]
    K_psi = float(yaw["K_psi_nm_per_rad"])
    K_r = float(yaw["K_r_nm_per_rad_s"])

    off_run, off_adapter = run_arm(
        common, model, state_scales, input_scale, sum_limit, K_psi, K_r,
        q_enabled=False,
    )
    off_summary = summarize_arm(off_run, off_adapter, motion)
    health = mechanical_health_gate(off_summary)
    old_off = old_result["old_stage3c_Q_OFF_summary"]
    mechanical_change = off_run["payload_posthoc"]["setup"]["mechanical_override"]
    result = {
        "stage": "Stage 3C-R mechanically conditioned moving-payload gated-Q revalidation",
        "status": "MECHANICAL_BASELINE_FAIL" if not health["passed"] else "RUNNING",
        "scenario": SCENARIO,
        "frozen_controller": {
            "A": A.tolist(), "B": B.tolist(),
            "K_old": offline["identified_lqr"]["K_id"],
            "lambda_ff": config["lambda_ff"],
            "reference_limits": motion["reference_limits"],
            "j_max_m_s3": dynamic["max_jerk_m_s3"],
            "yaw_PD": yaw,
            "per_wheel_limit_nm": sum_limit / 2.0,
            "Q_and_gate_unchanged": True,
            "gate_thresholds_nm": {
                "d_on": FROZEN_THRESHOLDS_NM[0],
                "d_off": FROZEN_THRESHOLDS_NM[1],
            },
            "sensor_seed": motion["imu_rng_seed"],
        },
        "mechanical_design": mechanical_change,
        "changes_vs_previous_stage3c": {
            "payload_mass_kg": {"old": 0.20, "new": PAYLOAD_MASS_KG},
            "payload_geometry_changed": False,
            "basket_geometry_changed": False,
            "sliding_friction": {"old": 0.040, "new": SLIDING_FRICTION},
            "wall_contact": {
                "old": "default rigid contact",
                "new": mechanical_change["wall_padding"],
            },
            "initial_payload_position_or_velocity_changed": False,
            "chassis_mass_or_inertia_changed": False,
        },
        "arm_A_mechanical_Q_OFF": {
            "summary": off_summary,
            "mechanical_health_gate": health,
            "history_50hz": off_run["history_50hz"],
        },
        "old_vs_new_mechanical_Q_OFF": mechanical_comparison(old_off, off_summary),
        "arm_B_mechanical_gated_Q_ON": None,
        "new_Q_OFF_vs_gated_Q_ON": None,
        "Q_acceptance": None,
        "final_production_decision": (
            "mechanical redesign insufficient; Q actuator comparison not run"
            if not health["passed"] else None
        ),
    }
    RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    if not health["passed"]:
        append_learning_log(result)
        print(json.dumps({
            "status": result["status"],
            "result_path": str(RESULT_PATH),
            "mechanical_health_gate": health,
        }, indent=2))
        return

    on_run, on_adapter = run_arm(
        common, model, state_scales, input_scale, sum_limit, K_psi, K_r,
        q_enabled=True,
    )
    on_summary = summarize_arm(on_run, on_adapter, motion)
    acceptance = q_acceptance(off_summary, on_summary)
    result["status"] = "COMPLETE"
    result["arm_B_mechanical_gated_Q_ON"] = {
        "summary": on_summary,
        "history_50hz": on_run["history_50hz"],
    }
    result["new_Q_OFF_vs_gated_Q_ON"] = {
        "Q_OFF": off_summary,
        "gated_Q_ON": on_summary,
    }
    result["Q_acceptance"] = acceptance
    result["final_production_decision"] = acceptance["decision"]
    RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    append_learning_log(result)
    print(json.dumps({
        "status": result["status"],
        "result_path": str(RESULT_PATH),
        "mechanical_health_gate": health,
        "Q_acceptance": acceptance,
    }, indent=2))


if __name__ == "__main__":
    main()
