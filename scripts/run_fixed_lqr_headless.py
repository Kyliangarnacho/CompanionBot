"""Run the four deterministic fixed-LQR acceptance cases headlessly."""

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

from control import FixedLQR, design_from_files
from sim import MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
CONFIG_PATH = MODEL_DIR / "lqr_baseline_config.json"
PARAMETERS_PATH = MODEL_DIR / "plant_parameters.json"
RESULTS_PATH = MODEL_DIR / "lqr_baseline_results.json"
CASES_DEG = [-5.0, -2.0, 2.0, 5.0]


def has_chassis_floor_contact(sim: MiniSegwaySim) -> bool:
    for index in range(sim.data.ncon):
        contact = sim.data.contact[index]
        names = {
            mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
            mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
        }
        if "floor" in names and any(name.endswith("chassis_collision") for name in names):
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


def run_case(initial_error_deg, design, config, reduced, plant_params):
    sim = MiniSegwaySim()
    expected_steps = int(round((1.0 / config["controller_frequency_hz"]) / sim.physics_dt))
    if expected_steps != config["physics_steps_per_update"]:
        raise RuntimeError("controller frequency is not an integer multiple of the physics timestep")
    theta_eq = reduced["parameters"]["theta_eq_rad"]
    sim.reset(theta_eq + math.radians(initial_error_deg))
    controller = FixedLQR(design, plant_params["known"]["wheel_torque_hard_peak_nm"])

    duration_steps = round(config["test_duration_s"] / sim.physics_dt)
    held_command = controller.command(sim.longitudinal_state(theta_eq))
    controller_updates = 0
    saturated_updates = 0
    fallen = False
    max_wheel_torque = 0.0
    times = []
    pitch_errors = []
    pitch_rates = []

    for physics_step in range(duration_steps):
        if physics_step % config["physics_steps_per_update"] == 0:
            held_command = controller.command(sim.longitudinal_state(theta_eq))
            controller_updates += 1
            saturated_updates += int(held_command.saturated)
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
    return {
        "initial_pitch_error_deg": initial_error_deg,
        "fell": bool(fallen),
        "settling_time_s": settling_time,
        "pitch_error_rms_deg": float(np.degrees(np.sqrt(np.mean(pitch_errors**2)))),
        "max_wheel_torque_nm": max_wheel_torque,
        "saturation_ratio": saturated_updates / controller_updates,
        "final_position_drift_m": float(final_state[0]),
        "final_pitch_error_deg": float(np.degrees(final_state[2])),
        "controller_updates": controller_updates,
        "physics_steps": duration_steps,
        "final_qpos": sim.data.qpos.tolist(),
        "final_qvel": sim.data.qvel.tolist(),
    }


def main() -> None:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
    plant_params = json.loads(PARAMETERS_PATH.read_text(encoding="utf-8"))
    design = design_from_files(REDUCED_PATH, CONFIG_PATH)

    cases = []
    for initial_error_deg in CASES_DEG:
        first = run_case(initial_error_deg, design, config, reduced, plant_params)
        repeat = run_case(initial_error_deg, design, config, reduced, plant_params)
        exact_repeat = first["final_qpos"] == repeat["final_qpos"] and first["final_qvel"] == repeat["final_qvel"]
        first["deterministic_repeat_exact"] = exact_repeat
        first.pop("final_qpos")
        first.pop("final_qvel")
        cases.append(first)

    results = {
        "source_model": str(REDUCED_PATH),
        "discretization": "scipy.signal.cont2discrete, zero-order hold",
        "riccati_solver": "scipy.linalg.solve_discrete_are",
        "physics_dt_s": 0.001,
        "controller_dt_s": design.controller_dt_s,
        "Q_diag": np.diag(design.Q).tolist(),
        "R": float(design.R[0, 0]),
        "K": design.K.tolist(),
        "closed_loop_poles": [[float(value.real), float(value.imag)] for value in design.closed_loop_poles],
        "metric_definition": {
            "fell": f"abs(pitch error) >= {config['fall_pitch_error_deg']} deg or chassis collision touches floor",
            "settling_time": (
                f"first {config['settling_dwell_s']} s interval continuously within "
                f"+/-{config['settling_pitch_band_deg']} deg and +/-{config['settling_pitch_rate_band_deg_s']} deg/s"
            ),
            "pitch_error_rms": "RMS over the full test duration",
            "saturation_ratio": "saturated controller updates / all controller updates",
        },
        "attempts": [
            {
                "name": "initial_bryson_physical_scales",
                "design_basis": config["design_basis"],
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
