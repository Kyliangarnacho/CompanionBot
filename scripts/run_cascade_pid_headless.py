"""Run the four deterministic cascade-PID acceptance cases headlessly."""

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

from control import CascadePID, load_config
from sim import MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
CONFIG_PATH = MODEL_DIR / "cascade_pid_config.json"
RESULTS_PATH = MODEL_DIR / "stage1" / "results" / "cascade_pid_results.json"
CASES_DEG = [-5.0, -2.0, 2.0, 5.0]


def has_chassis_floor_contact(sim: MiniSegwaySim) -> bool:
    for index in range(sim.data.ncon):
        contact = sim.data.contact[index]
        names = {
            mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
            mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
        }
        if "floor" in names and any(name and name.endswith("chassis_collision") for name in names):
            return True
    return False


def first_settling_time(times, pitch_errors, pitch_rates, config, physics_dt):
    pitch_band = math.radians(config["settling_pitch_band_deg"])
    rate_band = math.radians(config["settling_pitch_rate_band_deg_s"])
    dwell_steps = round(config["settling_dwell_s"] / physics_dt)
    good = (np.abs(pitch_errors) <= pitch_band) & (np.abs(pitch_rates) <= rate_band)
    for start in range(0, len(good) - dwell_steps + 1):
        if np.all(good[start : start + dwell_steps]):
            return float(times[start])
    return None


def run_case(initial_error_deg, controller_config, config, reduced):
    sim = MiniSegwaySim()
    if not np.isclose(sim.physics_dt, 0.001, rtol=0.0, atol=1e-12):
        raise RuntimeError(f"expected 0.001 s physics timestep, got {sim.physics_dt}")
    expected_steps = int(round(controller_config.controller_dt_s / sim.physics_dt))
    if expected_steps != config["physics_steps_per_update"]:
        raise RuntimeError("controller frequency is not an integer multiple of the physics timestep")
    if not np.isclose(controller_config.sum_torque_limit_nm / 2.0, config["limits"]["per_wheel_hard_peak_nm"]):
        raise RuntimeError("summed torque limit must equal twice the per-wheel hard peak")

    theta_eq = reduced["parameters"]["theta_eq_rad"]
    sim.reset(theta_eq + math.radians(initial_error_deg))
    controller = CascadePID(controller_config)

    duration_steps = round(config["test_duration_s"] / sim.physics_dt)
    held_command = None
    controller_updates = 0
    torque_saturated_updates = 0
    theta_reference_saturated_updates = 0
    outer_antiwindup_blocked_updates = 0
    inner_antiwindup_blocked_updates = 0
    fallen = False
    max_wheel_torque = 0.0
    max_theta_reference = 0.0
    max_position_integral_fraction = 0.0
    max_pitch_integral_fraction = 0.0
    times = []
    pitch_errors = []
    pitch_rates = []

    for physics_step in range(duration_steps):
        if physics_step % config["physics_steps_per_update"] == 0:
            held_command = controller.command(sim.longitudinal_state(theta_eq))
            controller_updates += 1
            torque_saturated_updates += int(held_command.torque_saturated)
            theta_reference_saturated_updates += int(held_command.theta_reference_saturated)
            outer_antiwindup_blocked_updates += int(held_command.outer_antiwindup_blocked)
            inner_antiwindup_blocked_updates += int(held_command.inner_antiwindup_blocked)
            max_theta_reference = max(max_theta_reference, abs(held_command.theta_reference_rad))
            max_position_integral_fraction = max(
                max_position_integral_fraction,
                abs(held_command.position_integral_m_s) / controller_config.position_integral_limit_m_s,
            )
            max_pitch_integral_fraction = max(
                max_pitch_integral_fraction,
                abs(held_command.pitch_integral_rad_s) / controller_config.pitch_integral_limit_rad_s,
            )

        if held_command is None:
            raise RuntimeError("controller was not called at the initial physics step")
        snapshot = sim.step(held_command.left_torque_nm, held_command.right_torque_nm)
        state = sim.longitudinal_state(theta_eq)
        times.append(snapshot.time_s)
        pitch_errors.append(state[2])
        pitch_rates.append(state[3])
        max_wheel_torque = max(max_wheel_torque, float(np.max(np.abs(snapshot.applied_ctrl_nm))))
        fallen |= abs(state[2]) >= math.radians(config["fall_pitch_error_deg"])
        fallen |= has_chassis_floor_contact(sim)

    times = np.asarray(times)
    pitch_errors = np.asarray(pitch_errors)
    pitch_rates = np.asarray(pitch_rates)
    final_state = sim.longitudinal_state(theta_eq)
    settling_time = first_settling_time(times, pitch_errors, pitch_rates, config, sim.physics_dt)
    threshold = config["windup_limit_fraction_threshold"]
    windup_detected = max(max_position_integral_fraction, max_pitch_integral_fraction) >= threshold
    return {
        "initial_pitch_error_deg": initial_error_deg,
        "fell": bool(fallen),
        "settling_time_s": settling_time,
        "pitch_error_rms_deg": float(np.degrees(np.sqrt(np.mean(pitch_errors**2)))),
        "max_wheel_torque_nm": max_wheel_torque,
        "saturation_ratio": torque_saturated_updates / controller_updates,
        "final_position_drift_m": float(final_state[0]),
        "final_pitch_error_deg": float(np.degrees(final_state[2])),
        "max_theta_reference_correction_deg": float(np.degrees(max_theta_reference)),
        "theta_reference_saturation_ratio": theta_reference_saturated_updates / controller_updates,
        "integral_windup_detected": bool(windup_detected),
        "max_position_integral_limit_fraction": max_position_integral_fraction,
        "max_pitch_integral_limit_fraction": max_pitch_integral_fraction,
        "outer_antiwindup_blocked_updates": outer_antiwindup_blocked_updates,
        "inner_antiwindup_blocked_updates": inner_antiwindup_blocked_updates,
        "final_position_integral_m_s": controller.position_integral_m_s,
        "final_pitch_integral_rad_s": controller.pitch_integral_rad_s,
        "controller_updates": controller_updates,
        "physics_steps": duration_steps,
        "final_qpos": sim.data.qpos.tolist(),
        "final_qvel": sim.data.qvel.tolist(),
    }


def main() -> None:
    raw_config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    controller_config = load_config(CONFIG_PATH)

    cases = []
    for initial_error_deg in CASES_DEG:
        first = run_case(initial_error_deg, controller_config, raw_config, reduced)
        repeat = run_case(initial_error_deg, controller_config, raw_config, reduced)
        exact_repeat = first["final_qpos"] == repeat["final_qpos"] and first["final_qvel"] == repeat["final_qvel"]
        first["deterministic_repeat_exact"] = exact_repeat
        first.pop("final_qpos")
        first.pop("final_qvel")
        cases.append(first)

    results = {
        "source_model": str(REDUCED_PATH),
        "physics_dt_s": MiniSegwaySim().physics_dt,
        "controller_dt_s": controller_config.controller_dt_s,
        "structure": {
            "outer": "theta_ref = clamp(Kp_position*(-p) + Ki_position*integral(-p) - Kd_velocity*p_dot)",
            "inner": "tau_sum = clamp(Kp_pitch*(theta_error-theta_ref) + Ki_pitch*integral(error) + Kd_pitch*theta_dot)",
            "allocation": "tau_left = tau_right = tau_sum / 2",
            "anti_windup": "conditional integration plus hard integral-state limits",
        },
        "gains": raw_config["gains"],
        "limits": raw_config["limits"],
        "tuning_record": raw_config["tuning_record"],
        "metric_definition": {
            "fell": f"abs(pitch error) >= {raw_config['fall_pitch_error_deg']} deg or chassis collision touches floor",
            "settling_time": (
                f"first {raw_config['settling_dwell_s']} s interval continuously within "
                f"+/-{raw_config['settling_pitch_band_deg']} deg and "
                f"+/-{raw_config['settling_pitch_rate_band_deg_s']} deg/s"
            ),
            "pitch_error_rms": "RMS over the full test duration",
            "saturation_ratio": "sum-torque saturated controller updates / all controller updates",
            "integral_windup_detected": (
                f"either integral state reaches >= {100*raw_config['windup_limit_fraction_threshold']:.0f}% "
                "of its configured hard limit"
            ),
        },
        "attempts": [
            {
                "name": raw_config["design_basis"]["attempt_name"],
                "design_basis": raw_config["design_basis"],
                "cases": cases,
            }
        ],
    }
    RESULTS_PATH.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))

    if not all(case["deterministic_repeat_exact"] for case in cases):
        raise SystemExit("FAIL: repeated deterministic simulations diverged")


if __name__ == "__main__":
    main()
