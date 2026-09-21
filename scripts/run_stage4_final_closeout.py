"""Stage 4 final simplified runtime and two frozen-baseline smoke tests."""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage4a_slope_robustness as stage4a  # noqa: E402
import run_stage4c_slope_estimation as stage4c  # noqa: E402
from control.stage4_runtime import (  # noqa: E402
    EnvironmentMode, NaturalTransientPayloadId, PayloadIdConfig,
    PayloadLifecycle, PayloadPlantBuilder, QChangeTriggerConfig,
    SagittalPayloadParameters, SlopeSupervisor, SlopeSupervisorConfig,
)
from control.full_state_identification import DiscreteStateSpaceModel  # noqa: E402
from sim.dynamic_payload import DynamicPayload, PayloadConfig  # noqa: E402
from sim.slope_estimation import (  # noqa: E402
    TwipSlopeEkf, analytic_theta_eq, equilibrium_sum_torque_nm,
)


MODEL_DIR = ROOT / "models" / "minisegway"
STAGE_DIR = MODEL_DIR / "stage4"
CONFIG_PATH = STAGE_DIR / "config" / "stage4_final_baseline_config.json"
STAGE4C_CONFIG_PATH = STAGE_DIR / "config" / "stage4c_slope_estimation_config.json"
RESULT_DIR = STAGE_DIR / "results" / "final_baseline"
METRICS_PATH = RESULT_DIR / "stage4_final_baseline_metrics.json"
HISTORY_PATH = RESULT_DIR / "stage4_final_baseline_history.json"
REPORT_PATH = RESULT_DIR / "STAGE4_FINAL_BASELINE.md"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value, *, compact: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        value, ensure_ascii=False, allow_nan=False,
        separators=(",", ":") if compact else None,
        indent=None if compact else 2,
    ), encoding="utf-8")


def make_common(config: dict, seed: int) -> tuple:
    stage4c_config = load_json(STAGE4C_CONFIG_PATH)
    return stage4c.make_common({
        **stage4c_config,
        "history_frequency_hz": float(config["history_frequency_hz"]),
    }, seed)


def validate_frozen_baseline(config: dict, stage4c_config: dict) -> dict:
    common = make_common(config, int(config["seed"]))
    manifest = common[0]
    checks = {
        "physics_dt_1ms": float(manifest["timing"]["physics_dt_s"]) == 0.001,
        "controller_dt_2ms": float(manifest["timing"]["controller_dt_s"]) == 0.002,
        "payload_id_period_20ms": float(config["payload_id"]["update_period_s"]) == 0.02,
        "stage4c_ekf_period_10ms": float(stage4c_config["twip_ekf"]["update_period_s"]) == 0.01,
        "q_actuator_off": (
            config["q_actuator_enabled"] is False
            and manifest["disturbance_rejection"][
                "actuator_augmentation_production_enabled"
            ] is False
        ),
        "slip_off": config["slip_enabled"] is False,
        "slope_enter_frozen": (
            float(config["slope_supervisor"]["enter_error_deg"]) == 3.0
            and float(config["slope_supervisor"]["enter_persistence_s"]) == 0.20
        ),
        "slope_exit_frozen": (
            float(config["slope_supervisor"]["exit_abs_alpha_deg"]) == 0.7
            and float(config["slope_supervisor"]["exit_abs_error_deg"]) == 3.0
            and float(config["slope_supervisor"]["exit_persistence_s"]) == 0.75
        ),
    }
    if not all(checks.values()):
        failed = [name for name, passed in checks.items() if not passed]
        raise RuntimeError(f"frozen baseline check failed: {failed}")
    return checks


def payload_config(config: dict, lateral_m: float) -> PayloadConfig:
    raw = config["fixed_payload"]
    return PayloadConfig(
        target_body="chassis", add_time_s=0.0,
        mass_kg=float(raw["mass_kg"]),
        position_body_m=np.asarray([
            float(lateral_m), -float(raw["forward_position_m"]),
            float(raw["height_m"]),
        ], dtype=float),
        full_size_m=np.asarray(raw["full_size_m"], dtype=float),
    )


class FinalRuntime:
    def __init__(self, *, reduced: dict, config: dict, stage4c_config: dict, q_adapter) -> None:
        self.config = config
        self.q_adapter = q_adapter
        self.builder = PayloadPlantBuilder(
            reduced, float(config["payload_id"]["known_basket_payload_height_m"]),
            np.asarray(config["fixed_payload"]["full_size_m"], dtype=float),
        )
        self.current_model = self.builder.empty()
        self.supervisor = SlopeSupervisor(SlopeSupervisorConfig(**config["slope_supervisor"]))
        trigger = dict(config["q_change_trigger"])
        self.lifecycle = PayloadLifecycle(QChangeTriggerConfig(**trigger))
        id_raw = dict(config["payload_id"])
        id_raw.pop("derivative_filter_time_constant_s", None)
        id_raw.pop("known_basket_payload_height_m")
        self.payload_id = NaturalTransientPayloadId(
            PayloadIdConfig(**id_raw), self.builder,
            q_adapter.compensator.state_scales,
        )
        _, ekf_config = stage4c.configs(stage4c_config)
        self.ekf_config = ekf_config
        self.ekf = TwipSlopeEkf(self.plant, ekf_config, 0.0)
        self.previous_reference_velocity = 0.0
        self.latest_reference_acceleration = 0.0
        self.alpha_control_rad = 0.0
        self.transition_log: list[dict] = []
        self.id_results: list[dict] = []
        self._fields = self._snapshot(None, None)

    @property
    def plant(self):
        return self.current_model.slope_plant

    def output(self, context: dict) -> dict:
        del context
        return dict(self._fields)

    def observe(self, context: dict) -> dict:
        estimate = context["estimate_after"]
        dt = float(context["dt_s"])
        velocity = float(estimate.velocity_hat_m_s)
        pitch_rate = float(estimate.theta_dot_hat_rad_s)
        reference_velocity = float(context["reference_state"][1])
        self.latest_reference_acceleration = (
            reference_velocity - self.previous_reference_velocity
        ) / dt
        self.previous_reference_velocity = reference_velocity
        actual_sum = float(np.sum(context["actual_wheels_nm"]))

        if self.supervisor.mode is EnvironmentMode.SLOPE:
            self.ekf.plant = self.plant
            slope = self.ekf.update(
                velocity_m_s=velocity,
                theta_world_rad=float(estimate.theta_hat_rad),
                pitch_rate_rad_s=pitch_rate,
                actual_sum_torque_nm=actual_sum,
                reference_acceleration_m_s2=self.latest_reference_acceleration,
                saturated=bool(context["saturated"]), normal_traction=True, dt_s=dt,
            )
        else:
            slope = self.ekf.last
        supervisor = self.supervisor.update(
            dt_s=dt, theta_hat_rad=float(estimate.theta_hat_rad),
            theta_eq_rad=self.plant.theta_flat_rad,
            theta_dyn_ref_rad=float(context["reference_state"][2]),
            velocity_hat_m_s=velocity, alpha_hat_rad=slope.alpha_hat_rad,
            alpha_std_deg=math.degrees(math.sqrt(slope.alpha_variance_rad2)),
        )
        self.alpha_control_rad = supervisor.alpha_control_rad
        if supervisor.transition_reason is not None:
            self.transition_log.append({
                "time_s": float(context["time_s"]),
                "from": supervisor.previous_mode.value,
                "to": supervisor.mode.value,
                "reason": supervisor.transition_reason,
            })
            if supervisor.mode is EnvironmentMode.SLOPE:
                self.payload_id.abort()

        transient = abs(self.latest_reference_acceleration) > float(
            self.config["payload_id"]["minimum_abs_reference_acceleration_m_s2"]
        )
        q_value = float(self.q_adapter.compensator.q_filter_estimate_nm)
        lifecycle_event = self.lifecycle.observe(
            time_s=float(context["time_s"]), q_filtered_nm=q_value,
            flat=supervisor.mode is EnvironmentMode.FLAT, transient=transient,
        )
        if lifecycle_event == "payload_id_started":
            self.payload_id.start()
        if supervisor.mode is EnvironmentMode.SLOPE and self.payload_id.active:
            self.payload_id.abort()

        id_result = self.payload_id.observe(
            dt_s=dt,
            reference_acceleration_m_s2=self.latest_reference_acceleration,
            saturated=bool(context["saturated"]),
            state_absolute=np.asarray([
                float(estimate.position_hat_m), velocity,
                float(estimate.theta_hat_rad), pitch_rate,
            ], dtype=float),
            actual_sum_torque_nm=(
                actual_sum
                - self.builder.empty().slope_plant.hinge_loss_sum_nm(
                    velocity, pitch_rate
                )
                - self.builder.empty().slope_plant.rolling_loss_sum_nm(velocity)
            ),
        )
        if id_result is not None:
            if id_result.accepted and id_result.parameters is not None:
                self.current_model = self.builder.build(id_result.parameters)
                self.ekf = TwipSlopeEkf(self.plant, self.ekf_config, 0.0)
                q_a, q_b, _ = self.builder.discrete_state_space(
                    id_result.parameters, float(self.config["controller_dt_s"])
                )
                self.q_adapter.compensator.update_model(
                    DiscreteStateSpaceModel(q_a, q_b)
                )
            self.lifecycle.finish_identification(
                float(context["time_s"]), id_result.accepted
            )
            self.id_results.append({
                "time_s": float(context["time_s"]),
                "accepted": id_result.accepted,
                "reason": id_result.reason,
                "update_count": id_result.update_count,
                "rank": id_result.rank,
                "condition_number": id_result.condition_number,
                "normalized_residual_rms": id_result.normalized_residual_rms,
                "estimated_mass_kg": id_result.estimated_mass_kg,
                "estimated_forward_position_m": id_result.estimated_forward_position_m,
                "mass_kg": (
                    None if id_result.parameters is None
                    else id_result.parameters.mass_kg
                ),
                "forward_position_m": (
                    None if id_result.parameters is None
                    else id_result.parameters.forward_m
                ),
            })
        self._fields = self._snapshot(supervisor, lifecycle_event)
        return dict(self._fields)

    def _snapshot(self, supervisor, lifecycle_event) -> dict:
        slope = self.ekf.last
        payload = self.current_model.payload
        return {
            "environment_mode": self.supervisor.mode.value,
            "slope_transition_reason": (
                None if supervisor is None else supervisor.transition_reason
            ),
            "slope_enter_timer_s": self.supervisor.enter_timer_s,
            "slope_exit_timer_s": self.supervisor.exit_timer_s,
            "alpha_hat_ekf_deg": math.degrees(slope.alpha_hat_rad),
            "alpha_std_deg": math.degrees(math.sqrt(slope.alpha_variance_rad2)),
            "alpha_control_deg": math.degrees(self.alpha_control_rad),
            "slope_ekf_updated": slope.updated,
            "slope_ekf_active": self.supervisor.mode is EnvironmentMode.SLOPE,
            "payload_id_pending": self.lifecycle.payload_id_pending,
            "payload_id_active": self.lifecycle.payload_id_active,
            "payload_id_update_count": self.payload_id.update_count,
            "payload_lifecycle_event": lifecycle_event,
            "q_baseline_nm": self.lifecycle.q_baseline_nm,
            "delta_q_nm": self.lifecycle.delta_q_nm,
            "q_mismatch_ewma_nm": self.lifecycle.ewma_abs_delta_q_nm,
            "payload_mass_kg": payload.mass_kg,
            "payload_forward_position_m": (
                None if payload.mass_kg <= 0.0 else payload.forward_m
            ),
            "payload_known_height_m": payload.known_height_m,
            "Q_actuator_enabled": False,
            "slip_enabled": False,
        }


def run_payload_id(config: dict, stage4c_config: dict) -> tuple[dict, FinalRuntime]:
    common = make_common(config, int(config["seed"]))
    q_adapter = stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = FinalRuntime(
        reduced=common[7], config=config, stage4c_config=stage4c_config,
        q_adapter=q_adapter,
    )
    raw = config["payload_id_episode"]
    actual_config = payload_config(config, 0.0)
    holder: dict = {"events": []}

    def setup(sim):
        holder["payload"] = DynamicPayload(sim.model, sim.data, actual_config)
        return {"stage4_final_payload_id": True}

    def physics_step(sim):
        if (
            not holder["payload"].is_loaded
            and sim.data.time + 1e-12 >= float(raw["payload_add_time_s"])
        ):
            holder["payload"].apply()
            holder["events"].append({
                "time_s": float(sim.data.time), "event": "payload_added_GT_posthoc",
            })

    def diagnostic(sim):
        del sim
        return {"payload_loaded_GT_posthoc": holder["payload"].is_loaded}

    scenario = {
        "name": "stage4_final_payload_id",
        "duration_s": float(raw["duration_s"]),
        "linear_velocity_schedule": raw["linear_velocity_schedule"],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    manifest = common[0]
    run = stage3b.run_case(
        scenario, float(manifest["yaw"]["K_psi_nm_per_rad"]),
        float(manifest["yaw"]["K_r_nm_per_rad_s"]), yaw_enabled=True,
        motor_mismatch_enabled=False, common=common, keep_history=True,
        payload_mode="empty", common_mode_augmentation=q_adapter,
        physics_step_callback=physics_step, simulation_setup_callback=setup,
        history_diagnostic_callback=diagnostic,
        equilibrium_reference_callback=lambda context: analytic_theta_eq(
            runtime.alpha_control_rad, runtime.plant
        ),
        equilibrium_input_callback=lambda context: equilibrium_sum_torque_nm(
            runtime.alpha_control_rad,
            float(context["estimate"].velocity_hat_m_s),
            float(context["estimate"].theta_dot_hat_rad_s), runtime.plant,
        ),
        control_observer=runtime,
    )
    summary = {key: value for key, value in run.items() if key != "history_50hz"}
    truth = config["fixed_payload"]
    final_payload = runtime.current_model.payload
    return {
        "result": runtime.id_results[-1] if runtime.id_results else {
            "accepted": False, "reason": "no_id_session", "update_count": 0,
        },
        "session_count": len(runtime.id_results),
        "pending_event_count": sum(
            event["event"] == "payload_id_pending" for event in runtime.lifecycle.events
        ),
        "lifecycle_events": runtime.lifecycle.events,
        "q_observer_definition": "r[k] = x[k+1] - (A*x[k] + B*u[k]); existing matched projection and filtered Q",
        "truth_posthoc": {
            "mass_kg": truth["mass_kg"],
            "forward_position_m": truth["forward_position_m"],
        },
        "final_model": {
            "mass_kg": final_payload.mass_kg,
            "forward_position_m": (
                None if final_payload.mass_kg <= 0.0 else final_payload.forward_m
            ),
            "theta_flat_deg": math.degrees(runtime.plant.theta_flat_rad),
        },
        "run_summary": summary,
        "history": run["history_50hz"],
        "payload_events_GT_posthoc": holder["events"],
    }, runtime


def run_fixed_payload_case(
    *, name: str, lateral_m: float, config: dict, plant,
    seed: int, impulse_wheel: str | None = None,
) -> dict:
    common = make_common(config, seed)
    q_adapter = stage4a.make_q_adapter(common, actuator_enabled=False)
    fixed = payload_config(config, lateral_m)
    holder: dict = {}
    impulse = config["single_wheel_impulse"]

    def setup(sim):
        holder["payload"] = DynamicPayload(sim.model, sim.data, fixed)
        holder["payload"].apply()
        if impulse_wheel is not None:
            holder["wheel_body"] = sim.model.body(f"{impulse_wheel}_wheel").id
        return {
            "fixed_payload_lateral_offset_m": lateral_m,
            "impulse_wheel": impulse_wheel,
        }

    def physics_step(sim):
        if impulse_wheel is None:
            return
        sim.data.xfrc_applied[int(holder["wheel_body"]), :] = 0.0
        if float(impulse["start_time_s"]) <= sim.data.time < (
            float(impulse["start_time_s"]) + float(impulse["duration_s"])
        ):
            sim.data.xfrc_applied[int(holder["wheel_body"]), 1] = -float(
                impulse["force_n"]
            )

    if impulse_wheel is None:
        profile = config["robustness_profile"]
        schedule = profile["linear_velocity_schedule"]
        duration = float(profile["duration_s"])
    else:
        schedule = [
            {"time_s": 0.0, "command": 0.0},
            {"time_s": 0.8, "command": float(impulse["linear_velocity_m_s"])},
        ]
        duration = float(impulse["episode_duration_s"])
    scenario = {
        "name": name, "duration_s": duration,
        "linear_velocity_schedule": schedule,
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    manifest = common[0]
    run = stage3b.run_case(
        scenario, float(manifest["yaw"]["K_psi_nm_per_rad"]),
        float(manifest["yaw"]["K_r_nm_per_rad_s"]), yaw_enabled=True,
        motor_mismatch_enabled=False, common=common, keep_history=True,
        payload_mode="empty", common_mode_augmentation=q_adapter,
        physics_step_callback=physics_step, simulation_setup_callback=setup,
        equilibrium_reference_callback=lambda context: analytic_theta_eq(0.0, plant),
        equilibrium_input_callback=lambda context: equilibrium_sum_torque_nm(
            0.0, float(context["estimate"].velocity_hat_m_s),
            float(context["estimate"].theta_dot_hat_rad_s), plant,
        ),
    )
    return {"run": run, "history": run["history_50hz"]}


def metric(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "rms": float(np.sqrt(np.mean(values * values))),
        "peak_abs": float(np.max(np.abs(values))),
        "final": float(values[-1]),
    }


def lateral_metrics(case: dict, lateral_m: float) -> dict:
    run = case["run"]
    rows = case["history"]
    yaw_error = np.asarray([row["psi_ref_rad"] - row["psi_GT_rad"] for row in rows])
    velocity_error = np.asarray([row["v_GT_m_s"] - row["v_ref_m_s"] for row in rows])
    time = np.asarray([row["t"] for row in rows])
    yaw = np.asarray([row["psi_GT_rad"] for row in rows])
    drift_mask = time >= 2.0
    return {
        "lateral_offset_m": lateral_m,
        "fell": run["longitudinal"]["fell"],
        "wheel_saturation_fraction": run["torque_allocation"]["per_wheel_saturation_fraction"],
        "max_abs_pitch_deg": run["longitudinal"]["pitch_peak_GT_deg"],
        "yaw_error_deg": {
            key: math.degrees(value) for key, value in metric(yaw_error).items()
        },
        "yaw_drift_rate_deg_s": math.degrees(float(np.polyfit(
            time[drift_mask], yaw[drift_mask], 1
        )[0])),
        "velocity_tracking_m_s": metric(velocity_error),
        "actual_left_wheel_torque_nm": metric(np.asarray([
            row["actual_left_nm"] for row in rows
        ])),
        "actual_right_wheel_torque_nm": metric(np.asarray([
            row["actual_right_nm"] for row in rows
        ])),
    }


def impulse_metrics(case: dict, wheel: str, config: dict) -> dict:
    run = case["run"]
    rows = case["history"]
    raw = config["single_wheel_impulse"]
    start = float(raw["start_time_s"])
    end = float(raw["start_time_s"]) + float(raw["duration_s"])
    pre = [
        row for row in rows
        if start - 0.5 <= float(row["t"]) < start
    ]
    post = [row for row in rows if float(row["t"]) >= end]
    baseline_pitch = float(np.median([row["theta_GT_rad"] for row in pre]))
    baseline_yaw = float(np.median([row["psi_GT_rad"] for row in pre]))
    baseline_velocity_error = float(np.median([
        row["v_GT_m_s"] - row["v_ref_m_s"] for row in pre
    ]))
    pitch = np.asarray([row["theta_GT_rad"] for row in post])
    pitch_deviation = pitch - baseline_pitch
    yaw_deviation = np.asarray([
        math.atan2(
            math.sin(float(row["psi_GT_rad"]) - baseline_yaw),
            math.cos(float(row["psi_GT_rad"]) - baseline_yaw),
        )
        for row in post
    ])
    velocity_error = np.asarray([row["v_GT_m_s"] - row["v_ref_m_s"] for row in post])
    velocity_deviation = velocity_error - baseline_velocity_error
    times = np.asarray([row["t"] for row in post])
    required = max(1, round(
        float(raw["settling_window_s"]) * float(config["history_frequency_hz"])
    ))
    settled_time = None
    good = (
        np.abs(np.degrees(pitch_deviation)) < float(raw["settled_pitch_error_deg"])
    ) & (
        np.abs(np.degrees(yaw_deviation)) < float(raw["settled_yaw_error_deg"])
    ) & (
        np.abs(velocity_deviation) < float(raw["settled_velocity_error_m_s"])
    )
    for index in range(0, max(0, len(good) - required + 1)):
        if bool(np.all(good[index:index + required])):
            settled_time = float(times[index] - end)
            break
    return {
        "wheel": wheel,
        "force_n": float(raw["force_n"]),
        "duration_s": float(raw["duration_s"]),
        "impulse_n_s": float(raw["force_n"]) * float(raw["duration_s"]),
        "fell": run["longitudinal"]["fell"],
        "wheel_saturation_fraction": run["torque_allocation"]["per_wheel_saturation_fraction"],
        "peak_abs_pitch_deg": float(np.max(np.abs(np.degrees(pitch)))),
        "peak_pitch_deviation_from_pre_impulse_deg": float(
            np.max(np.abs(np.degrees(pitch_deviation)))
        ),
        "peak_abs_yaw_deviation_deg": float(
            np.max(np.abs(np.degrees(yaw_deviation)))
        ),
        "peak_abs_velocity_deviation_m_s": float(np.max(np.abs(velocity_deviation))),
        "settling_time_s": settled_time,
        "terminal_pitch_deviation_deg": float(math.degrees(pitch_deviation[-1])),
        "terminal_yaw_deviation_deg": float(math.degrees(yaw_deviation[-1])),
        "terminal_velocity_deviation_m_s": float(velocity_deviation[-1]),
    }


def write_report(metrics: dict) -> None:
    payload = metrics["payload_id"]
    lines = [
        "# Stage4 final baseline closeout",
        "",
        "## 1. 删除/保留了什么",
        "",
        "删除 PAYLOAD_ID environment state、continuous/shadow RLS、candidate/accepted 双层模型及多重 acceptance gates。保留两态 FLAT/SLOPE、Q one-step innovation 的 matched projection/filtered estimate、短时自然 transient batch ID。",
        "",
        "## 2. 最终 runtime 流程",
        "",
        "Q delta 经 EWMA+persistence+hysteresis 只置 pending；下一次自然 transient 置 active；50 Hz 收集一段数据并一次 least-squares；只做 rank/condition 与物理合法性检查，合格即更新 plant 并 freeze；首次 FLAT 非 transient 刷新 q_baseline 后 re-arm。",
        "",
        "## 3. payload ID update count 和结果",
        "",
        f"Sessions: {payload['session_count']}; updates: {payload['result']['update_count']}; accepted: {payload['result']['accepted']}; reason: {payload['result']['reason']}. Estimated mass {payload['final_model']['mass_kg']:.4f} kg versus post-hoc truth {payload['truth_posthoc']['mass_kg']:.4f} kg; estimated forward position {payload['final_model']['forward_position_m']:.5f} m versus post-hoc truth {payload['truth_posthoc']['forward_position_m']:.4f} m. The mass bias is retained as a baseline limitation; no extra acceptance layer or tuning was added.",
        "",
        "## 4. 左/右偏载结果",
        "",
        "| Case | Offset m | Fall | Saturation | Max pitch deg | Peak yaw deg | Yaw drift deg/s | Velocity RMS m/s | Left/Right torque peak N m |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for name, item in metrics["lateral_payload"].items():
        lines.append(
            f"| {name} | {item['lateral_offset_m']:+.3f} | {item['fell']} | "
            f"{item['wheel_saturation_fraction']:.4f} | {item['max_abs_pitch_deg']:.3f} | "
            f"{item['yaw_error_deg']['peak_abs']:.3f} | {item['yaw_drift_rate_deg_s']:.4f} | "
            f"{item['velocity_tracking_m_s']['rms']:.4f} | "
            f"{item['actual_left_wheel_torque_nm']['peak_abs']:.3f} / {item['actual_right_wheel_torque_nm']['peak_abs']:.3f} |"
        )
    lines += [
        "",
        "## 5. 左/右单轮冲激结果",
        "",
        "Recovery is measured relative to the 0.5 s pre-impulse steady baseline; absolute peak pitch is retained separately.",
        "",
        "| Wheel | Fall | Saturation | Peak pitch abs/dev deg | Peak yaw dev deg | Peak velocity dev m/s | Settling s |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, item in metrics["single_wheel_impulse"].items():
        settling = (
            f"{item['settling_time_s']:.3f}"
            if item["settling_time_s"] is not None else "not settled"
        )
        lines.append(
            f"| {name} | {item['fell']} | {item['wheel_saturation_fraction']:.4f} | "
            f"{item['peak_abs_pitch_deg']:.3f} / {item['peak_pitch_deviation_from_pre_impulse_deg']:.3f} | "
            f"{item['peak_abs_yaw_deviation_deg']:.3f} | "
            f"{item['peak_abs_velocity_deviation_m_s']:.4f} | "
            f"{settling} |"
        )
    lines += [
        "",
        "## 6. 最终 frozen baseline",
        "",
        "500 Hz main control; 50 Hz short-session payload ID; 100 Hz Stage4C EKF in SLOPE only; Q actuator OFF; slip OFF; LQR/FF/yaw/allocator/torque limits/friction frozen.",
    ]
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    config = load_json(CONFIG_PATH)
    stage4c_config = load_json(STAGE4C_CONFIG_PATH)
    frozen_checks = validate_frozen_baseline(config, stage4c_config)
    print("FINAL payload lifecycle", flush=True)
    payload_run, runtime = run_payload_id(config, stage4c_config)
    frozen_plant = runtime.plant

    lateral = {}
    lateral_histories = {}
    for index, (name, offset) in enumerate(
        config["fixed_payload"]["lateral_offsets_m"].items()
    ):
        print(f"SMOKE lateral {name} offset={offset:+.3f} m", flush=True)
        case = run_fixed_payload_case(
            name=f"fixed_payload_{name}", lateral_m=float(offset), config=config,
            plant=frozen_plant, seed=int(config["seed"]) + 100 + index,
        )
        lateral[name] = lateral_metrics(case, float(offset))
        lateral_histories[name] = case["history"]

    impulses = {}
    impulse_histories = {}
    for index, wheel in enumerate(("left", "right")):
        print(f"SMOKE impulse {wheel}", flush=True)
        case = run_fixed_payload_case(
            name=f"single_wheel_impulse_{wheel}", lateral_m=0.0,
            config=config, plant=frozen_plant,
            seed=int(config["seed"]) + 200 + index, impulse_wheel=wheel,
        )
        impulses[wheel] = impulse_metrics(case, wheel, config)
        impulse_histories[wheel] = case["history"]

    metrics = {
        "stage": config["stage"],
        "payload_id": {key: value for key, value in payload_run.items() if key != "history"},
        "lateral_payload": lateral,
        "single_wheel_impulse": impulses,
        "frozen_baseline": config["frozen_baseline"],
        "frozen_baseline_checks": frozen_checks,
        "slope_supervisor": config["slope_supervisor"],
        "fixed_payload_geometry": config["fixed_payload"],
        "Q_actuator_enabled": False,
        "slip_enabled": False,
        "GT_runtime_dependency": False,
    }
    histories = {
        "payload_id": payload_run["history"],
        "lateral_payload": lateral_histories,
        "single_wheel_impulse": impulse_histories,
    }
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    write_json(METRICS_PATH, metrics)
    write_json(HISTORY_PATH, histories, compact=True)
    write_report(metrics)
    print(json.dumps({
        "payload_id": payload_run["result"],
        "lateral": lateral,
        "impulses": impulses,
        "report": str(REPORT_PATH.relative_to(ROOT)),
    }, indent=2), flush=True)


if __name__ == "__main__":
    main()
