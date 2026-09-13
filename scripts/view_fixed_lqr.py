"""Interactive MuJoCo viewer for manual fixed-LQR acceptance."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from control import FixedLQR, design_from_files
from sim import MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial-pitch-deg", type=float, default=5.0, help="pitch error relative to theta_eq")
    parser.add_argument("--duration", type=float, default=20.0)
    args = parser.parse_args()

    reduced_path = MODEL_DIR / "reduced_twip.json"
    config_path = MODEL_DIR / "lqr_baseline_config.json"
    parameter_path = MODEL_DIR / "plant_parameters.json"
    reduced = json.loads(reduced_path.read_text(encoding="utf-8"))
    config = json.loads(config_path.read_text(encoding="utf-8"))
    parameters = json.loads(parameter_path.read_text(encoding="utf-8"))
    design = design_from_files(reduced_path, config_path)
    controller = FixedLQR(design, parameters["known"]["wheel_torque_hard_peak_nm"])
    sim = MiniSegwaySim()
    theta_eq = reduced["parameters"]["theta_eq_rad"]
    sim.reset(theta_eq + math.radians(args.initial_pitch_deg))

    held = controller.command(sim.longitudinal_state(theta_eq))
    print(
        f"physics={1/sim.physics_dt:.0f} Hz, controller={config['controller_frequency_hz']:.0f} Hz, "
        f"theta_eq={math.degrees(theta_eq):.4f} deg, initial_error={args.initial_pitch_deg:+.2f} deg"
    )
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running() and sim.data.time < args.duration:
            wall_start = time.perf_counter()
            if sim.step_index % config["physics_steps_per_update"] == 0:
                held = controller.command(sim.longitudinal_state(theta_eq))
            sim.step(held.left_torque_nm, held.right_torque_nm)
            viewer.sync()
            # Wall time only paces rendering; control updates remain step-count based.
            remaining = sim.physics_dt - (time.perf_counter() - wall_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
