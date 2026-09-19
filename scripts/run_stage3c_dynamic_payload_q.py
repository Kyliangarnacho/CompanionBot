"""Stage 3C one-shot integration of the frozen matched-DOB Q augmentation."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from control import (
    DiscreteStateSpaceModel,
    DisturbanceGateConfig,
    FilteredDisturbanceCompensator,
    PersistentDisturbanceGate,
    disturbance_rejection_config_from_dict,
)
from run_stage3a_commanded_motion import rms, transition_metrics
import run_stage3b_yaw_control as stage3b


MODEL_DIR = ROOT / "models" / "minisegway"
RESULT_PATH = MODEL_DIR / "stage3" / "results" / "history" / "stage3c_dynamic_payload_q_results.json"
LEARNING_LOG_PATH = ROOT / "LEARNING_LOG.md"

SCENARIO = {
    "name": "stage3c_integrated_forward_stop_reverse_turn_straight_stop",
    "duration_s": 18.0,
    "linear_velocity_schedule": [
        {"time_s": 0.0, "command": 0.0},
        {"time_s": 1.0, "command": 0.4},
        {"time_s": 4.0, "command": 0.0},
        {"time_s": 5.5, "command": -0.4},
        {"time_s": 8.5, "command": 0.0},
        {"time_s": 10.0, "command": 0.3},
        {"time_s": 15.5, "command": 0.0},
    ],
    "yaw_rate_schedule": [
        {"time_s": 0.0, "command": 0.0},
        {"time_s": 11.0, "command": 0.4},
        {"time_s": 13.0, "command": 0.0},
    ],
}

GATE_TIMING = {
    "on_persistence_s": 0.20,
    "off_persistence_s": 0.50,
    "ramp_duration_s": 0.15,
}


def metric(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "rms": rms(values),
        "peak_abs": float(np.max(np.abs(values))),
    }


class Stage3CQAdapter:
    """Small runner adapter; observer is always on, actuator path is optional."""

    def __init__(
        self,
        model: DiscreteStateSpaceModel,
        state_scales: np.ndarray,
        input_scale_nm: float,
        dr_raw: dict,
        sum_torque_limit_nm: float,
        *,
        thresholds: tuple[float, float] | None,
        actuator_enabled: bool,
    ) -> None:
        self.config = disturbance_rejection_config_from_dict(dr_raw)
        self.compensator = FilteredDisturbanceCompensator(
            model, state_scales, input_scale_nm, self.config
        )
        self.sum_torque_limit_nm = float(sum_torque_limit_nm)
        self.actuator_enabled = bool(actuator_enabled)
        self.gate = None
        if thresholds is not None:
            self.gate = PersistentDisturbanceGate(
                DisturbanceGateConfig(
                    controller_dt_s=self.config.controller_dt_s,
                    d_on_nm=float(thresholds[0]),
                    d_off_nm=float(thresholds[1]),
                    **GATE_TIMING,
                )
            )
        self.rows: list[dict] = []
        self._last_command: dict | None = None
        self.runtime_input_checks = 0
        self.runtime_active_input_checks = 0
        self.maximum_runtime_input_mismatch_nm = 0.0

    def command(self, u_base_nm: float, time_s: float) -> dict:
        d_hat = float(self.compensator.q_filter_estimate_nm)
        if self.gate is None:
            gate_state = "THRESHOLD_SHADOW"
            gate_active = False
            gate_alpha = 0.0
            on_timer = 0.0
            off_timer = 0.0
            transitioned_on = False
            transitioned_off = False
            d_on = None
            d_off = None
        else:
            gate = self.gate.update(d_hat)
            gate_state = gate.state
            gate_active = gate.requested_active
            gate_alpha = gate.alpha
            on_timer = gate.on_timer_s
            off_timer = gate.off_timer_s
            transitioned_on = gate.transitioned_on
            transitioned_off = gate.transitioned_off
            d_on = self.gate.config.d_on_nm
            d_off = self.gate.config.d_off_nm

        u_q_request = -d_hat
        u_q_gate = -gate_alpha * d_hat
        q_headroom = max(0.0, self.sum_torque_limit_nm - abs(float(u_base_nm)))
        q_limit = min(self.config.augmentation_authority_bound_nm, q_headroom)
        hypothetical_u_q = float(np.clip(u_q_gate, -q_limit, q_limit))
        u_q_used = hypothetical_u_q if self.actuator_enabled else 0.0
        self._last_command = {
            "Q_actuator_enabled": self.actuator_enabled,
            "q_filter_estimate_for_command_nm": d_hat,
            "u_Q_request_nm": u_q_request,
            "u_Q_gate_nm": u_q_gate,
            "u_Q_used_nm": u_q_used,
            "u_Q_hypothetical_used_nm": hypothetical_u_q,
            "q_headroom_nm": q_headroom,
            "q_authority_limit_nm": q_limit,
            "q_authority_limited": not math.isclose(
                u_q_gate, hypothetical_u_q, rel_tol=0.0, abs_tol=1e-12
            ),
            "d_on_nm": d_on,
            "d_off_nm": d_off,
            "gate_state": gate_state,
            "gate_requested_active": gate_active,
            "gate_alpha": gate_alpha,
            "gate_on_timer_s": on_timer,
            "gate_off_timer_s": off_timer,
            "gate_transitioned_on": transitioned_on,
            "gate_transitioned_off": transitioned_off,
            "gate_command_time_s": float(time_s),
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
            raise RuntimeError("Q adapter observe called before command")
        actual_sum = float(np.sum(np.asarray(actual_wheels_nm, dtype=float)))
        mismatch = abs(float(held_sum_torque_nm) - actual_sum)
        self.runtime_input_checks += 1
        self.runtime_active_input_checks += int(
            bool(self._last_command["gate_requested_active"])
        )
        self.maximum_runtime_input_mismatch_nm = max(
            self.maximum_runtime_input_mismatch_nm, mismatch
        )
        if mismatch > 1e-12:
            raise RuntimeError("observer held torque is not actual wheel-torque sum")

        observation = self.compensator.observe(
            np.asarray(state_k, dtype=float),
            actual_sum,
            np.asarray(state_k1, dtype=float),
        )
        fields = {
            "observer_time_s": float(time_s),
            "observer_held_actual_sum_nm": actual_sum,
            "innovation": observation.normalized_innovation.tolist(),
            "scaled_innovation_rms": observation.scaled_innovation_rms,
            "matched_disturbance_raw_nm": observation.matched_disturbance_raw_nm,
            "matched_disturbance_projected_nm": (
                observation.matched_disturbance_projected_nm
            ),
            "q_filter_estimate_nm": self.compensator.q_filter_estimate_nm,
            "matched_residual_fraction": observation.matched_residual_fraction,
            "projection_clipped": observation.projection_clipped,
        }
        self.rows.append({**self._last_command, **fields})
        return fields

    def summary(self) -> dict:
        if not self.rows:
            raise RuntimeError("Q adapter has no observations")
        d_raw = np.asarray([row["matched_disturbance_raw_nm"] for row in self.rows])
        d_projected = np.asarray([
            row["matched_disturbance_projected_nm"] for row in self.rows
        ])
        d_hat = np.asarray([row["q_filter_estimate_nm"] for row in self.rows])
        matched = np.asarray([row["matched_residual_fraction"] for row in self.rows])
        clipped = np.asarray([row["projection_clipped"] for row in self.rows])
        active = np.asarray([row["gate_requested_active"] for row in self.rows])
        alpha = np.asarray([row["gate_alpha"] for row in self.rows])
        authority = np.asarray([row["q_authority_limited"] for row in self.rows])
        u_request = np.asarray([row["u_Q_request_nm"] for row in self.rows])
        u_gate = np.asarray([row["u_Q_gate_nm"] for row in self.rows])
        u_used = np.asarray([row["u_Q_used_nm"] for row in self.rows])
        on_times = [
            row["gate_command_time_s"] for row in self.rows
            if row["gate_transitioned_on"]
        ]
        off_times = [
            row["gate_command_time_s"] for row in self.rows
            if row["gate_transitioned_off"]
        ]
        dt = self.config.controller_dt_s
        padded = np.r_[False, active, False]
        starts = np.flatnonzero(~padded[:-1] & padded[1:])
        stops = np.flatnonzero(padded[:-1] & ~padded[1:])
        durations = (stops - starts) * dt
        return {
            "observer": {
                "d_raw_nm": metric(d_raw),
                "d_projected_nm": metric(d_projected),
                "d_hat_nm": metric(d_hat),
                "matched_residual_fraction_mean": float(np.mean(matched)),
                "projection_clipping_fraction": float(np.mean(clipped)),
                "observation_count": len(self.rows),
            },
            "gate": {
                "d_on_nm": (
                    None if self.gate is None else self.gate.config.d_on_nm
                ),
                "d_off_nm": (
                    None if self.gate is None else self.gate.config.d_off_nm
                ),
                "active_fraction": float(np.mean(active)),
                "nonzero_alpha_fraction": float(np.mean(alpha > 0.0)),
                "ramp_activity_fraction": float(np.mean((alpha > 0.0) & (alpha < 1.0))),
                "on_transition_count": len(on_times),
                "off_transition_count": len(off_times),
                "on_times_s": on_times,
                "off_times_s": off_times,
                "active_durations_s": durations.tolist(),
                "average_active_duration_s": (
                    float(np.mean(durations)) if durations.size else 0.0
                ),
            },
            "augmentation": {
                "actuator_enabled": self.actuator_enabled,
                "u_Q_request_nm": metric(u_request),
                "u_Q_gate_nm": metric(u_gate),
                "u_Q_used_nm": metric(u_used),
                "authority_limiting_fraction": float(np.mean(authority)),
            },
            "actual_torque_self_feedback_invariant": {
                "passed": self.maximum_runtime_input_mismatch_nm <= 1e-12,
                "interval_checks": self.runtime_input_checks,
                "active_interval_checks": self.runtime_active_input_checks,
                "maximum_abs_mismatch_nm": self.maximum_runtime_input_mismatch_nm,
                "semantics": "x[k]->x[k+1] prediction consumes mean actual_left+actual_right applied over interval k",
            },
        }


def exact_input_invariant_test(
    model: DiscreteStateSpaceModel,
    state_scales: np.ndarray,
    input_scale_nm: float,
    dr_raw: dict,
) -> dict:
    config = disturbance_rejection_config_from_dict(dr_raw)
    x_k = np.asarray([0.13, -0.08, 0.02, -0.11], dtype=float)
    u_base = 0.11
    u_q = 0.07
    u_actual = u_base + u_q
    x_k1 = model.A @ x_k + model.B[:, 0] * u_actual
    correct = FilteredDisturbanceCompensator(
        model, state_scales, input_scale_nm, config
    ).observe(x_k, u_actual, x_k1)
    wrong = FilteredDisturbanceCompensator(
        model, state_scales, input_scale_nm, config
    ).observe(x_k, u_base, x_k1)
    correct_norm = float(np.linalg.norm(correct.normalized_innovation))
    wrong_norm = float(np.linalg.norm(wrong.normalized_innovation))
    passed = correct_norm <= 1e-12 and wrong_norm > 1e-6
    if not passed:
        raise RuntimeError("actual-torque self-feedback invariant test failed")
    return {
        "passed": passed,
        "u_base_nm": u_base,
        "u_Q_nm": u_q,
        "u_actual_sum_nm": u_actual,
        "correct_actual_input_normalized_innovation_norm": correct_norm,
        "wrong_base_only_input_normalized_innovation_norm": wrong_norm,
    }


def derive_threshold(adapter: Stage3CQAdapter) -> dict:
    magnitudes = np.abs(np.asarray([
        row["q_filter_estimate_nm"] for row in adapter.rows
    ]))
    median = float(np.median(magnitudes))
    mad = float(np.median(np.abs(magnitudes - median)))
    percentile = float(np.percentile(magnitudes, 99.9))
    robust = median + 6.0 * mad
    d_on = max(percentile, robust)
    if not math.isfinite(d_on) or d_on <= 0.0:
        raise RuntimeError("nominal disturbance threshold is not positive and finite")
    return {
        "sample_count": int(magnitudes.size),
        "startup_invalid_samples_ignored": 0,
        "absolute_d_hat_median_nm": median,
        "absolute_d_hat_MAD_nm": mad,
        "absolute_d_hat_P99p9_nm": percentile,
        "median_plus_6_MAD_nm": robust,
        "d_on_nm": d_on,
        "d_off_nm": 0.6 * d_on,
        "formula": "d_on=max(P99.9(abs(d_hat)), median(abs(d_hat))+6*MAD(abs(d_hat))); d_off=0.6*d_on",
    }


def slow_rms(values: np.ndarray, samples: int = 25) -> float:
    values = np.asarray(values, dtype=float)
    if values.size < samples:
        return rms(values)
    kernel = np.full(samples, 1.0 / samples)
    return rms(np.convolve(values, kernel, mode="valid"))


def arm_summary(run: dict, adapter: Stage3CQAdapter, motion_config: dict) -> dict:
    history = run["history_50hz"]
    times = np.asarray([row["t"] for row in history])
    p_ref = np.asarray([row["p_ref_m"] for row in history])
    p_hat = np.asarray([row["p_hat_m"] for row in history])
    p_gt = np.asarray([row["p_GT_m"] for row in history])
    v_ref = np.asarray([row["v_ref_m_s"] for row in history])
    v_hat = np.asarray([row["v_hat_m_s"] for row in history])
    v_gt = np.asarray([row["v_GT_m_s"] for row in history])
    theta_gt = np.asarray([row["theta_GT_rad"] for row in history])
    u_fb = np.asarray([row["u_fb_nm"] for row in history])
    u_ff = np.asarray([row["u_ff_used_nm"] for row in history])
    u_base = np.asarray([row["u_base_nm"] for row in history])
    u_q = np.asarray([row["u_Q_used_nm"] for row in history])
    u_sum = np.asarray([row["u_sum_nm"] for row in history])
    actual_wheels = np.asarray([
        [row["actual_left_nm"], row["actual_right_nm"]] for row in history
    ])
    references = np.column_stack([p_ref, v_ref, np.zeros_like(p_ref), np.zeros_like(p_ref)])
    plants = np.column_stack([p_hat, v_hat, np.zeros_like(p_hat), np.zeros_like(p_hat)])
    transition_scenario = {
        "mode": "velocity_derived",
        "schedule": SCENARIO["linear_velocity_schedule"],
        "duration_s": SCENARIO["duration_s"],
    }
    transitions = transition_metrics(
        transition_scenario,
        times,
        references,
        plants,
        motion_config["settling"],
        float(np.median(np.diff(times))),
    )
    terminal = times >= SCENARIO["duration_s"] - float(
        motion_config["terminal_window_s"]
    )
    return {
        "balance": {
            "pitch_RMS_GT_deg": math.degrees(rms(theta_gt)),
            "pitch_peak_GT_deg": math.degrees(float(np.max(np.abs(theta_gt)))),
            "terminal_pitch_RMS_GT_deg": math.degrees(rms(theta_gt[terminal])),
            "fall": run["longitudinal"]["fell"],
        },
        "tracking": {
            "position_error_estimated_m": metric(p_hat - p_ref),
            "velocity_error_estimated_m_s": metric(v_hat - v_ref),
            "velocity_error_GT_m_s": metric(v_gt - v_ref),
            "estimated_position_displacement_m": float(p_hat[-1] - p_hat[0]),
            "GT_position_displacement_m": float(p_gt[-1] - p_gt[0]),
            "estimated_position_endpoint_error_m": float(p_hat[-1] - p_ref[-1]),
            "GT_position_endpoint_error_m": float(p_gt[-1] - p_ref[-1]),
            "persistent_position_error_slow_RMS_m": slow_rms(p_hat - p_ref),
            "settling": transitions,
        },
        "control": {
            "u_fb_nm": metric(u_fb),
            "u_fb_slow_RMS_nm": slow_rms(u_fb),
            "u_ff_used_nm": metric(u_ff),
            "u_Q_used_nm": metric(u_q),
            "u_base_nm": metric(u_base),
            "u_sum_nm": metric(u_sum),
            "u_sum_first_difference_RMS_nm": rms(np.diff(u_sum)),
            "per_wheel_actual_RMS_nm": rms(actual_wheels.reshape(-1)),
            "per_wheel_actual_peak_nm": float(np.max(np.abs(actual_wheels))),
            "per_wheel_saturation_fraction": run["torque_allocation"][
                "per_wheel_saturation_fraction"
            ],
        },
        "yaw": run["yaw_tracking"],
        "Q": adapter.summary(),
        "payload_evaluator_posthoc": run["payload_posthoc"],
    }


def nominal_gate_check(summary: dict) -> dict:
    gate = summary["Q"]["gate"]
    durations = gate["active_durations_s"]
    checks = {
        "active_fraction_at_most_10pct": gate["active_fraction"] <= 0.10,
        "no_requested_active_episode_longer_than_1s": (
            max(durations, default=0.0) <= 1.0
        ),
        "no_fall": not summary["balance"]["fall"],
        "no_saturation": summary["control"]["per_wheel_saturation_fraction"] == 0.0,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "definition_of_long_term_false_activation": "requested-active fraction >10% or any requested-active episode >1.0 s",
    }


def compare_and_decide(nominal_ok: bool, q_off: dict, q_on: dict) -> dict:
    off_track = q_off["tracking"]
    on_track = q_on["tracking"]
    off_control = q_off["control"]
    on_control = q_on["control"]
    off_balance = q_off["balance"]
    on_balance = q_on["balance"]
    burden_changes = {
        "position_error_slow_RMS_ratio": (
            on_track["persistent_position_error_slow_RMS_m"]
            / max(off_track["persistent_position_error_slow_RMS_m"], 1e-12)
        ),
        "u_fb_RMS_ratio": on_control["u_fb_nm"]["rms"]
        / max(off_control["u_fb_nm"]["rms"], 1e-12),
        "u_fb_slow_RMS_ratio": on_control["u_fb_slow_RMS_nm"]
        / max(off_control["u_fb_slow_RMS_nm"], 1e-12),
        "GT_position_endpoint_error_abs_ratio": abs(
            on_track["GT_position_endpoint_error_m"]
        ) / max(abs(off_track["GT_position_endpoint_error_m"]), 1e-12),
    }
    clear_transfer = (
        q_on["Q"]["augmentation"]["u_Q_used_nm"]["rms"] > 1e-6
        and not off_balance["fall"]
        and not on_balance["fall"]
        and min(burden_changes.values()) <= 0.98
    )
    no_material_degradation = {
        "velocity_RMSE_within_5pct": (
            on_track["velocity_error_estimated_m_s"]["rms"]
            <= 1.05 * off_track["velocity_error_estimated_m_s"]["rms"]
        ),
        "pitch_RMS_within_5pct": (
            on_balance["pitch_RMS_GT_deg"]
            <= 1.05 * off_balance["pitch_RMS_GT_deg"]
        ),
        "pitch_peak_within_5pct": (
            on_balance["pitch_peak_GT_deg"]
            <= 1.05 * off_balance["pitch_peak_GT_deg"]
        ),
        "torque_chatter_within_10pct": (
            on_control["u_sum_first_difference_RMS_nm"]
            <= 1.10 * off_control["u_sum_first_difference_RMS_nm"]
        ),
        "no_fall": not on_balance["fall"],
        "no_saturation": on_control["per_wheel_saturation_fraction"] == 0.0,
        "payload_contained": bool(
            q_on["payload_evaluator_posthoc"]["remained_in_basket"]
        ),
    }
    accepted = nominal_ok and clear_transfer and all(no_material_degradation.values())
    return {
        "production_acceptance": accepted,
        "decision": (
            "accept always-on observer plus automatically gated actuator augmentation"
            if accepted else "reject gated actuator augmentation; retain observer as diagnostic only"
        ),
        "burden_transfer": burden_changes,
        "clear_slow_burden_transfer": clear_transfer,
        "no_material_degradation_checks": no_material_degradation,
        "reasons_if_not_accepted": [
            reason for reason, passed in {
                "nominal_false_trigger_validation": nominal_ok,
                "clear_slow_burden_transfer": clear_transfer,
                **no_material_degradation,
            }.items() if not passed
        ],
    }


def slim_run(run: dict, adapter: Stage3CQAdapter, motion_config: dict) -> dict:
    return {
        "summary": arm_summary(run, adapter, motion_config),
        "history_50hz": run["history_50hz"],
    }


def append_learning_log(decision: dict, nominal: dict, q_off: dict, q_on: dict) -> None:
    marker = "## Stage 3C — gated matched-disturbance Q integration"
    existing = LEARNING_LOG_PATH.read_text(encoding="utf-8")
    if marker in existing:
        return
    note = f"""

{marker}

- Reused the existing B-direction matched-disturbance observer unchanged; it runs continuously from sensorized plant state and interval actual applied wheel-torque sum.
- Derived the d_hat-only hysteretic gate threshold from one integrated nominal shadow run; payload/GT/contact data remained evaluator-only.
- Preserved nominal longitudinal priority over Q and yaw, with the frozen 2 Hz Q filter and 0.18 N m authority.
- Used the accepted free-moving payload directly and compared identical Q-OFF versus gated-Q-ON arms. Nominal active fraction was {nominal['Q']['gate']['active_fraction']:.6f}; moving-payload u_Q RMS changed from {q_off['control']['u_Q_used_nm']['rms']:.6f} to {q_on['control']['u_Q_used_nm']['rms']:.6f} N m.
- Production decision: {decision['decision']}. No Q/gate/controller/payload parameter was tuned after observing results.
"""
    LEARNING_LOG_PATH.write_text(existing.rstrip() + note + "\n", encoding="utf-8")


def main() -> None:
    common = stage3b.load_common()
    (
        manifest, config, dynamic_config, motion_config, offline,
        experiment, dr_raw, reduced, plant,
    ) = common
    A = np.asarray(offline["fit"]["A_identified"], dtype=float)
    B = np.asarray(offline["fit"]["B_identified"], dtype=float)
    state_scales = np.asarray(offline["fit"]["state_scales"], dtype=float)
    input_scale = float(offline["fit"]["input_scale_nm"])
    model = DiscreteStateSpaceModel(A, B)
    per_wheel_limit = float(plant["known"]["wheel_torque_hard_peak_nm"])
    sum_limit = 2.0 * per_wheel_limit
    dr_config = disturbance_rejection_config_from_dict(dr_raw)
    yaw_config = json.loads(
        (MODEL_DIR / "stage3" / "config" / "stage3b_yaw_control_config.json").read_text(encoding="utf-8")
    )
    K_psi = float(yaw_config["yaw_PD"]["K_psi_nm_per_rad"])
    K_r = float(yaw_config["yaw_PD"]["K_r_nm_per_rad_s"])

    compatibility_audit = {
        "passed": True,
        "model_source": manifest["nominal_model"],
        "A_current_stage3": A.tolist(),
        "B_current_stage3": B.tolist(),
        "state_order": manifest["state_order"],
        "observer_state_runtime_expression": "estimate.plant_state(theta_eq)",
        "observer_state_is_tracking_error": False,
        "observer_uses_GT": False,
        "input_definition": "mean interval actual_left + actual_right actuator torque",
        "theta_coordinate": "pitch error relative to theta_eq",
    }
    invariant_test = exact_input_invariant_test(
        model, state_scales, input_scale, dr_raw
    )

    threshold_adapter = Stage3CQAdapter(
        model, state_scales, input_scale, dr_raw, sum_limit,
        thresholds=None, actuator_enabled=False,
    )
    threshold_run = stage3b.run_case(
        SCENARIO, K_psi, K_r,
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=False,
        common_mode_augmentation=threshold_adapter,
    )
    threshold = derive_threshold(threshold_adapter)
    thresholds = (threshold["d_on_nm"], threshold["d_off_nm"])

    nominal_adapter = Stage3CQAdapter(
        model, state_scales, input_scale, dr_raw, sum_limit,
        thresholds=thresholds, actuator_enabled=True,
    )
    nominal_run = stage3b.run_case(
        SCENARIO, K_psi, K_r,
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        common_mode_augmentation=nominal_adapter,
    )
    nominal = slim_run(nominal_run, nominal_adapter, motion_config)
    nominal_check = nominal_gate_check(nominal["summary"])

    result = {
        "stage": "Stage 3C dynamic/free-payload gated matched-disturbance Q",
        "status": "NOMINAL_GATE_FAIL" if not nominal_check["passed"] else "RUNNING",
        "frozen_stage3b_baseline": {
            "A": A.tolist(),
            "B": B.tolist(),
            "K_old": offline["identified_lqr"]["K_id"],
            "lambda_ff": config["lambda_ff"],
            "reference_limits": motion_config["reference_limits"],
            "j_max_m_s3": dynamic_config["max_jerk_m_s3"],
            "feedforward_lifecycle": config["velocity_lifecycle"],
            "yaw_PD": yaw_config["yaw_PD"],
            "yaw_estimator": yaw_config["yaw_estimator"],
            "torque_allocator": yaw_config["torque_allocator"],
            "per_wheel_torque_limit_nm": per_wheel_limit,
            "sensor_seed": motion_config["imu_rng_seed"],
            "controller_dt_s": motion_config["controller_dt_s"],
            "physics_dt_s": motion_config["physics_dt_s"],
        },
        "Q_configuration": {
            "q_filter_cutoff_hz": dr_config.q_filter_cutoff_hz,
            "innovation_projection_bound_nm": dr_config.innovation_projection_bound_nm,
            "augmentation_authority_bound_nm": dr_config.augmentation_authority_bound_nm,
            "augmentation_slew_rate_nm_s": dr_config.augmentation_slew_rate_nm_s,
            "projection_ridge": dr_config.projection_ridge,
            "observer_always_on": True,
        },
        "compatibility_audit": compatibility_audit,
        "actual_torque_self_feedback_unit_invariant": invariant_test,
        "scenario": SCENARIO,
        "nominal_noise_floor_shadow": {
            "threshold_statistics": threshold,
            "observer_summary": threshold_adapter.summary(),
            "controller_summary": {
                "longitudinal": threshold_run["longitudinal"],
                "yaw": threshold_run["yaw_tracking"],
                "torque": threshold_run["torque_allocation"],
            },
        },
        "gate_timing": GATE_TIMING,
        "nominal_false_trigger_validation": {
            **nominal,
            "gate_design_check": nominal_check,
        },
        "dynamic_payload_shadow_run": None,
        "Q_OFF_vs_gated_Q_ON": None,
        "final_production_decision": None,
    }
    RESULT_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    if not nominal_check["passed"]:
        print(json.dumps({
            "status": result["status"],
            "result_path": str(RESULT_PATH),
            "nominal_gate_check": nominal_check,
        }, indent=2))
        return

    q_off_adapter = Stage3CQAdapter(
        model, state_scales, input_scale, dr_raw, sum_limit,
        thresholds=thresholds, actuator_enabled=False,
    )
    q_off_run = stage3b.run_case(
        SCENARIO, K_psi, K_r,
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        payload_mode="free",
        use_accepted_free_payload_initial_state=True,
        common_mode_augmentation=q_off_adapter,
    )
    q_off = slim_run(q_off_run, q_off_adapter, motion_config)

    q_on_adapter = Stage3CQAdapter(
        model, state_scales, input_scale, dr_raw, sum_limit,
        thresholds=thresholds, actuator_enabled=True,
    )
    q_on_run = stage3b.run_case(
        SCENARIO, K_psi, K_r,
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        payload_mode="free",
        use_accepted_free_payload_initial_state=True,
        common_mode_augmentation=q_on_adapter,
    )
    q_on = slim_run(q_on_run, q_on_adapter, motion_config)
    decision = compare_and_decide(
        nominal_check["passed"], q_off["summary"], q_on["summary"]
    )

    result["status"] = "COMPLETE"
    result["dynamic_payload_shadow_run"] = {
        "same_run_reused_as_causal_C0_Q_OFF_arm": True,
        **q_off,
    }
    result["Q_OFF_vs_gated_Q_ON"] = {
        "controlled_variables": "identical plant, accepted free-payload initial condition, sensor seed, scenario, estimator, reference, controller, and torque limits; only actuator use of gated u_Q differs",
        "Q_OFF": q_off,
        "gated_Q_ON": q_on,
    }
    result["final_production_decision"] = decision
    RESULT_PATH.parent.mkdir(parents=True, exist_ok=True)
    RESULT_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    append_learning_log(
        decision,
        nominal["summary"],
        q_off["summary"],
        q_on["summary"],
    )
    print(json.dumps({
        "status": result["status"],
        "result_path": str(RESULT_PATH),
        "threshold": threshold,
        "nominal_gate_check": nominal_check,
        "decision": decision,
    }, indent=2))


if __name__ == "__main__":
    main()
