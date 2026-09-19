"""Launch the existing Stage 3C moving-payload controller in MuJoCo viewer."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from control import DiscreteStateSpaceModel
import run_stage3b_yaw_control as stage3b
from run_stage3c_dynamic_payload_q import SCENARIO, Stage3CQAdapter
from run_stage3c_mechanical_payload_q_revalidation import configure_mechanical_payload


MODEL_DIR = ROOT / "models" / "minisegway"
RESULT_PATH = MODEL_DIR / "stage3" / "results" / "stage3c_mechanical_payload_q_revalidation_results.json"


class RealtimeViewer:
    def __init__(self, realtime_factor: float) -> None:
        self.realtime_factor = realtime_factor
        self.context = None
        self.viewer = None
        self.next_deadline = None

    def __call__(self, sim) -> None:
        if self.viewer is None:
            for name in (
                "basket_front_wall", "basket_rear_wall",
                "basket_left_wall", "basket_right_wall",
            ):
                sim.model.geom(name).rgba[:] = [0.15, 0.65, 1.0, 0.18]
            self.context = mujoco.viewer.launch_passive(sim.model, sim.data)
            self.viewer = self.context.__enter__()
            self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            self.next_deadline = time.perf_counter()
        if not self.viewer.is_running():
            raise SystemExit(0)
        self.viewer.sync()
        self.next_deadline += sim.physics_dt / self.realtime_factor
        remaining = self.next_deadline - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)

    def close(self) -> None:
        if self.context is not None:
            self.context.__exit__(None, None, None)
            self.context = None
            self.viewer = None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", choices=("q-on", "q-off"), default="q-on")
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    args = parser.parse_args()
    if args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be positive")

    saved = json.loads(RESULT_PATH.read_text(encoding="utf-8"))
    common = stage3b.load_common()
    _, _, _, motion, offline, _, dr_raw, _, plant = common
    model = DiscreteStateSpaceModel(
        np.asarray(offline["fit"]["A_identified"], dtype=float),
        np.asarray(offline["fit"]["B_identified"], dtype=float),
    )
    threshold = saved["frozen_controller"]["gate_thresholds_nm"]
    adapter = Stage3CQAdapter(
        model,
        np.asarray(offline["fit"]["state_scales"], dtype=float),
        float(offline["fit"]["input_scale_nm"]),
        dr_raw,
        2.0 * float(plant["known"]["wheel_torque_hard_peak_nm"]),
        thresholds=(float(threshold["d_on"]), float(threshold["d_off"])),
        actuator_enabled=args.arm == "q-on",
    )
    yaw = saved["frozen_controller"]["yaw_PD"]
    viewer = RealtimeViewer(args.realtime_factor)
    print(
        f"Stage 3C live demo: {args.arm}, {SCENARIO['duration_s']:.0f} s, "
        f"IMU seed={motion['imu_rng_seed']}. Close the viewer to stop."
    )
    try:
        stage3b.run_case(
            SCENARIO,
            float(yaw["K_psi_nm_per_rad"]),
            float(yaw["K_r_nm_per_rad_s"]),
            yaw_enabled=True,
            motor_mismatch_enabled=False,
            common=common,
            keep_history=False,
            payload_mode="free",
            use_accepted_free_payload_initial_state=True,
            common_mode_augmentation=adapter,
            physics_step_callback=viewer,
            simulation_setup_callback=configure_mechanical_payload,
        )
    finally:
        viewer.close()


if __name__ == "__main__":
    main()
