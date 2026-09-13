"""Offline full-state identification, independent validation, and K_id test."""

from __future__ import annotations

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
    CenteredPRBS,
    DiscreteStateSpaceModel,
    FixedLQR,
    PRBSConfig,
    design_discrete_lqr,
    design_from_files,
    fit_full_state_ridge,
)
from sim import DynamicPayload, MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"
EXPERIMENT_CONFIG_PATH = MODEL_DIR / "experiment_config.json"
LQR_CONFIG_PATH = MODEL_DIR / "lqr_baseline_config.json"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
PLANT_PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
FIXED_RESULTS_PATH = MODEL_DIR / "lqr_baseline_results.json"
RESULTS_PATH = MODEL_DIR / "full_state_identification_results.json"
CASES_DEG = [-5.0, -2.0, 2.0, 5.0]


def collect_trajectory(
    raw,
    reduced,
    design,
    per_wheel_peak,
    seed,
    payload_config=None,
):
    probe_raw = raw["probe"]
    probe_config = PRBSConfig(
        float(probe_raw["amplitude_sum_torque_nm"]),
        float(probe_raw["nominal_start_s"]),
        float(probe_raw["nominal_duration_s"]),
        float(probe_raw["bit_period_s"]),
        int(probe_raw["lfsr_bits"]),
        int(seed),
    )
    probe = CenteredPRBS(probe_config)
    sim = MiniSegwaySim()
    if payload_config is not None:
        DynamicPayload(sim.model, sim.data, payload_config).apply()
    theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim.reset(theta_eq)
    fixed = FixedLQR(design, per_wheel_peak)
    interval_count = round(
        float(raw["identification_tests"]["duration_s"])
        * float(raw["controller_frequency_hz"])
    )
    times = []
    states = []
    inputs = []
    probe_inputs = []
    probe_active = []
    next_states = []
    dt_errors = []
    torque_errors = []
    for _ in range(interval_count):
        start_step = sim.step_index
        start_time = float(sim.data.time)
        x_k = sim.longitudinal_state(theta_eq)
        nominal = fixed.command(x_k).limited_sum_torque_nm
        probe_torque = probe.value(start_time)
        commanded_u = float(
            np.clip(
                nominal + probe_torque,
                -2.0 * per_wheel_peak,
                2.0 * per_wheel_peak,
            )
        )
        snapshots = [
            sim.step(commanded_u / 2.0, commanded_u / 2.0)
            for _ in range(int(raw["physics_steps_per_update"]))
        ]
        held_u = float(np.sum(snapshots[-1].applied_ctrl_nm))
        x_next = sim.longitudinal_state(theta_eq)
        end_time = float(sim.data.time)
        times.append(start_time)
        states.append(x_k)
        inputs.append(held_u)
        probe_inputs.append(probe_torque)
        probe_active.append(probe.is_active(start_time))
        next_states.append(x_next)
        dt_errors.append(abs(end_time - start_time - design.controller_dt_s))
        torque_errors.extend(
            abs(float(np.sum(snapshot.applied_ctrl_nm)) - commanded_u)
            for snapshot in snapshots
        )
        if sim.step_index - start_step != int(raw["physics_steps_per_update"]):
            raise RuntimeError("sample alignment violated")
    return {
        "times": np.asarray(times),
        "states": np.asarray(states),
        "inputs": np.asarray(inputs),
        "probe_inputs": np.asarray(probe_inputs),
        "probe_active": np.asarray(probe_active, dtype=bool),
        "next_states": np.asarray(next_states),
        "seed": int(seed),
        "probe_mean_nm": probe.scheduled_mean_nm,
        "probe_rms_nm": probe.scheduled_rms_nm,
        "maximum_dt_error_s": float(np.max(dt_errors)),
        "maximum_held_torque_error_nm": float(np.max(torque_errors)),
    }


def subset(data, mask):
    result = {
        "states": data["states"][mask],
        "inputs": data["inputs"][mask],
        "next_states": data["next_states"][mask],
    }
    for optional in ("probe_inputs", "probe_active"):
        if optional in data:
            result[optional] = data[optional][mask]
    return result


def correlation_rows(residual, regressors):
    names = ["p", "p_dot", "theta_error", "theta_dot", "u"]
    rows = []
    for output in residual.T:
        values = []
        for regressor in regressors.T:
            if np.std(output) <= 1e-15 or np.std(regressor) <= 1e-15:
                values.append(None)
            else:
                values.append(float(np.corrcoef(output, regressor)[0, 1]))
        rows.append(values)
    return {"regressor_order": names, "by_output": rows}


def one_step_metrics(model, data, state_scales, affine=None):
    prediction = model.predict(data["states"], data["inputs"])
    if affine is not None:
        prediction = prediction + np.asarray(affine, dtype=float)
    residual = data["next_states"] - prediction
    residual_rms = np.sqrt(np.mean(residual**2, axis=0))
    lag_one = [
        float(np.corrcoef(output[:-1], output[1:])[0, 1])
        for output in residual.T
    ]
    regressors = np.column_stack([data["states"], data["inputs"]])
    return {
        "prediction_rms_by_state": residual_rms.tolist(),
        "scaled_aggregate_rms": float(
            np.sqrt(np.mean((residual / state_scales) ** 2))
        ),
        "residual_lag1_correlation_by_state": lag_one,
        "residual_regressor_correlation": correlation_rows(residual, regressors),
    }


def rollout_metrics(model, data, state_scales, dt, horizons_s, stride, affine=None):
    result = {}
    for horizon_s in horizons_s:
        horizon = round(float(horizon_s) / dt)
        errors = []
        for start in range(0, len(data["states"]) - horizon, stride):
            estimate = data["states"][start].copy()
            for offset in range(horizon):
                estimate = model.A @ estimate + model.B[:, 0] * data["inputs"][start + offset]
                if affine is not None:
                    estimate = estimate + affine
            errors.append(data["next_states"][start + horizon - 1] - estimate)
        errors = np.asarray(errors)
        result[f"{float(horizon_s):g}_s"] = {
            "endpoint_rms_by_state": np.sqrt(np.mean(errors**2, axis=0)).tolist(),
            "scaled_aggregate_endpoint_rms": float(
                np.sqrt(np.mean((errors / state_scales) ** 2))
            ),
            "rollout_count": len(errors),
        }
    estimate = data["states"][0].copy()
    full_errors = []
    for index, u_k in enumerate(data["inputs"]):
        estimate = model.A @ estimate + model.B[:, 0] * u_k
        if affine is not None:
            estimate = estimate + affine
        full_errors.append(data["next_states"][index] - estimate)
    full_errors = np.asarray(full_errors)
    result["full_validation_sequence"] = {
        "rms_by_state": np.sqrt(np.mean(full_errors**2, axis=0)).tolist(),
        "scaled_aggregate_rms": float(
            np.sqrt(np.mean((full_errors / state_scales) ** 2))
        ),
        "finite": bool(np.isfinite(full_errors).all()),
    }
    return result


def has_chassis_floor_contact(sim):
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


def first_settling_time(times, pitch_errors, pitch_rates, config, physics_dt):
    pitch_band = math.radians(config["settling_pitch_band_deg"])
    rate_band = math.radians(config["settling_pitch_rate_band_deg_s"])
    dwell_steps = round(float(config["settling_dwell_s"]) / physics_dt)
    good = (np.abs(pitch_errors) <= pitch_band) & (np.abs(pitch_rates) <= rate_band)
    for start in range(len(good) - dwell_steps + 1):
        if np.all(good[start : start + dwell_steps]):
            return float(times[start])
    return None


def run_lqr_case(initial_error_deg, gain, config, reduced, per_wheel_peak):
    sim = MiniSegwaySim()
    theta_eq = float(reduced["parameters"]["theta_eq_rad"])
    sim.reset(theta_eq + math.radians(initial_error_deg))
    duration_steps = round(float(config["test_duration_s"]) / sim.physics_dt)
    held_sum = 0.0
    saturated_updates = 0
    controller_updates = 0
    fallen = False
    max_wheel_torque = 0.0
    times = []
    pitch_errors = []
    pitch_rates = []
    for physics_step in range(duration_steps):
        if physics_step % int(config["physics_steps_per_update"]) == 0:
            raw_sum = -float((gain @ sim.longitudinal_state(theta_eq)).item())
            held_sum = float(
                np.clip(raw_sum, -2.0 * per_wheel_peak, 2.0 * per_wheel_peak)
            )
            saturated_updates += int(
                not np.isclose(raw_sum, held_sum, rtol=0.0, atol=1e-12)
            )
            controller_updates += 1
        snapshot = sim.step(held_sum / 2.0, held_sum / 2.0)
        state = sim.longitudinal_state(theta_eq)
        times.append(snapshot.time_s)
        pitch_errors.append(state[2])
        pitch_rates.append(state[3])
        max_wheel_torque = max(
            max_wheel_torque, float(np.max(np.abs(snapshot.applied_ctrl_nm)))
        )
        fallen |= abs(state[2]) >= math.radians(config["fall_pitch_error_deg"])
        fallen |= has_chassis_floor_contact(sim)
    times = np.asarray(times)
    pitch_errors = np.asarray(pitch_errors)
    pitch_rates = np.asarray(pitch_rates)
    final_state = sim.longitudinal_state(theta_eq)
    return {
        "initial_pitch_error_deg": initial_error_deg,
        "fell": bool(fallen),
        "settling_time_s": first_settling_time(
            times, pitch_errors, pitch_rates, config, sim.physics_dt
        ),
        "pitch_error_rms_deg": float(
            np.degrees(np.sqrt(np.mean(pitch_errors**2)))
        ),
        "max_wheel_torque_nm": max_wheel_torque,
        "saturation_ratio": saturated_updates / controller_updates,
        "final_position_drift_m": float(final_state[0]),
        "final_pitch_error_deg": float(np.degrees(final_state[2])),
        "final_qpos": sim.data.qpos.tolist(),
        "final_qvel": sim.data.qvel.tolist(),
    }


def main():
    raw = json.loads(EXPERIMENT_CONFIG_PATH.read_text(encoding="utf-8"))
    config = raw["full_state_identification"]
    lqr_config = json.loads(LQR_CONFIG_PATH.read_text(encoding="utf-8"))
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    plant = json.loads(PLANT_PARAMETERS_PATH.read_text(encoding="utf-8"))
    analytic_design = design_from_files(REDUCED_PATH, LQR_CONFIG_PATH)
    peak = float(plant["known"]["wheel_torque_hard_peak_nm"])
    train_raw = collect_trajectory(
        raw, reduced, analytic_design, peak, config["train_prbs_seed"]
    )
    validation_raw = collect_trajectory(
        raw, reduced, analytic_design, peak, config["validation_prbs_seed"]
    )
    train_mask = (
        (train_raw["times"] >= float(config["fit_start_s"]))
        & (train_raw["times"] < float(config["fit_end_s"]))
    )
    validation_mask = (
        (validation_raw["times"] >= float(config["fit_start_s"]))
        & (validation_raw["times"] < float(config["fit_end_s"]))
    )
    train = subset(train_raw, train_mask)
    validation = subset(validation_raw, validation_mask)
    fit = fit_full_state_ridge(
        train["states"],
        train["inputs"],
        train["next_states"],
        float(config["ridge_lambda_normalized"]),
        np.asarray(config["minimum_scale"], dtype=float),
        float(config["minimum_input_scale_nm"]),
    )
    analytic = DiscreteStateSpaceModel(
        analytic_design.A_discrete, analytic_design.B_discrete
    )
    models = {
        "identified_full_state": (fit.model, None),
        "analytic_twip_zoh": (analytic, None),
    }
    comparisons = {}
    for name, (model, affine) in models.items():
        comparisons[name] = {
            "train_one_step": one_step_metrics(
                model, train, fit.state_scales, affine
            ),
            "validation_one_step": one_step_metrics(
                model, validation, fit.state_scales, affine
            ),
            "validation_rollout": rollout_metrics(
                model,
                validation,
                fit.state_scales,
                analytic_design.controller_dt_s,
                config["rollout_horizons_s"],
                int(config["rollout_start_stride_intervals"]),
                affine,
            ),
        }

    identified_validation = comparisons["identified_full_state"][
        "validation_one_step"
    ]["scaled_aggregate_rms"]
    identified_rollout_01 = comparisons["identified_full_state"][
        "validation_rollout"
    ]["0.1_s"]["scaled_aggregate_endpoint_rms"]
    offline_pass = bool(
        fit.regressor_rank == 5
        and identified_validation
        <= float(config["offline_validation_max_scaled_one_step_rms"])
        and identified_rollout_01
        <= float(config["offline_validation_max_scaled_rollout_rms_at_0_1_s"])
    )

    try:
        K_id, poles, controllability_rank = design_discrete_lqr(
            fit.model, analytic_design.Q, analytic_design.R
        )
        dare_status = "SUCCESS"
        cases = []
        for initial_error in CASES_DEG:
            first = run_lqr_case(initial_error, K_id, lqr_config, reduced, peak)
            repeat = run_lqr_case(initial_error, K_id, lqr_config, reduced, peak)
            first["deterministic_repeat_exact"] = bool(
                first["final_qpos"] == repeat["final_qpos"]
                and first["final_qvel"] == repeat["final_qvel"]
            )
            first.pop("final_qpos")
            first.pop("final_qvel")
            cases.append(first)
        lqr_pass = bool(
            controllability_rank == 4
            and all(
                not case["fell"]
                and case["settling_time_s"] is not None
                and case["deterministic_repeat_exact"]
                for case in cases
            )
        )
    except Exception as error:
        K_id = None
        poles = np.array([], dtype=complex)
        controllability_rank = 0
        dare_status = f"FAILED: {type(error).__name__}: {error}"
        cases = []
        lqr_pass = False

    fixed_results = json.loads(FIXED_RESULTS_PATH.read_text(encoding="utf-8"))
    result = {
        "status": "PASS" if offline_pass and lqr_pass else "FAIL",
        "state_order": config["state_order"],
        "input": config["input"],
        "sample_period_s": analytic_design.controller_dt_s,
        "data_separation": {
            "train": {
                "prbs_seed": train_raw["seed"],
                "sample_count": len(train["states"]),
            },
            "validation": {
                "prbs_seed": validation_raw["seed"],
                "sample_count": len(validation["states"]),
            },
            "same_probe_amplitude_and_timing": True,
            "train_validation_overlap": False,
        },
        "sample_alignment": {
            "physics_steps_per_interval": int(raw["physics_steps_per_update"]),
            "train_maximum_dt_error_s": train_raw["maximum_dt_error_s"],
            "validation_maximum_dt_error_s": validation_raw["maximum_dt_error_s"],
            "train_maximum_held_torque_error_nm": train_raw[
                "maximum_held_torque_error_nm"
            ],
            "validation_maximum_held_torque_error_nm": validation_raw[
                "maximum_held_torque_error_nm"
            ],
        },
        "fit": {
            "method": "scaled ridge least-squares via scipy.linalg.lstsq on an augmented system",
            "ridge_lambda_normalized": fit.ridge_lambda,
            "state_scales": fit.state_scales.tolist(),
            "input_scale_nm": fit.input_scale,
            "normalized_regressor_condition_number": fit.normalized_regressor_condition_number,
            "normalized_regressor_singular_values": fit.normalized_regressor_singular_values.tolist(),
            "regressor_rank": fit.regressor_rank,
            "A_identified": fit.model.A.tolist(),
            "B_identified": fit.model.B.tolist(),
        },
        "reference_models": {
            "A_analytic_zoh": analytic.A.tolist(),
            "B_analytic_zoh": analytic.B.tolist(),
        },
        "prediction_comparison": comparisons,
        "offline_acceptance": {
            "passed": offline_pass,
            "criteria": {
                "regressor_rank_required": 5,
                "validation_scaled_one_step_rms_max": config[
                    "offline_validation_max_scaled_one_step_rms"
                ],
                "validation_scaled_rollout_rms_at_0_1_s_max": config[
                    "offline_validation_max_scaled_rollout_rms_at_0_1_s"
                ],
            },
        },
        "identified_lqr": {
            "dare_status": dare_status,
            "K_id": K_id.tolist() if K_id is not None else None,
            "closed_loop_poles": [
                [float(value.real), float(value.imag)] for value in poles
            ],
            "controllability_rank": controllability_rank,
            "Q_diag_unchanged": np.diag(analytic_design.Q).tolist(),
            "R_unchanged": float(analytic_design.R[0, 0]),
            "passed": lqr_pass,
            "cases": cases,
            "fixed_lqr_reference_cases": fixed_results["attempts"][0]["cases"],
        },
        "stage_gate": {
            "identified_lqr": "ACCEPTED" if offline_pass and lqr_pass else "REJECTED"
        },
    }
    RESULTS_PATH.write_text(json.dumps(result, indent=2), encoding="utf-8")
    summary = {
        "status": result["status"],
        "fit": result["fit"],
        "data_separation": result["data_separation"],
        "sample_alignment": result["sample_alignment"],
        "prediction_comparison": result["prediction_comparison"],
        "offline_acceptance": result["offline_acceptance"],
        "identified_lqr": result["identified_lqr"],
        "stage_gate": result["stage_gate"],
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
