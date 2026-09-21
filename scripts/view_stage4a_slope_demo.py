"""Watch the frozen Stage 3 controller drive four Stage 4A slopes with Q ON."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b
import run_stage4a_slope_robustness as stage4a


SCENES = {
    "up8": 8.0,
    "down8": -8.0,
    "up15": 15.0,
    "down15": -15.0,
}


class ViewerClosed(RuntimeError):
    pass


class RealtimeTrackingViewer:
    """Passive MuJoCo viewer paced in wall time with a chassis-tracking camera."""

    def __init__(self, realtime_factor: float) -> None:
        self.realtime_factor = float(realtime_factor)
        self.context = None
        self.viewer = None
        self.start_wall_s: float | None = None
        self.last_sync_sim_s = -math.inf

    def __call__(self, sim) -> None:
        if self.viewer is None:
            self.context = mujoco.viewer.launch_passive(sim.model, sim.data)
            self.viewer = self.context.__enter__()
            self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            self.viewer.cam.trackbodyid = sim.model.body("chassis").id
            self.viewer.cam.distance = 1.6
            self.viewer.cam.azimuth = 135.0
            self.viewer.cam.elevation = -18.0
            self.start_wall_s = time.perf_counter() - (
                float(sim.data.time) / self.realtime_factor
            )
        if not self.viewer.is_running():
            raise ViewerClosed

        assert self.start_wall_s is not None
        target_wall_s = self.start_wall_s + (
            float(sim.data.time) / self.realtime_factor
        )
        remaining_s = target_wall_s - time.perf_counter()
        if remaining_s > 0.0:
            time.sleep(remaining_s)
        if float(sim.data.time) - self.last_sync_sim_s >= 1.0 / 60.0:
            self.viewer.sync()
            self.last_sync_sim_s = float(sim.data.time)

    def close(self) -> None:
        if self.context is not None:
            self.context.__exit__(None, None, None)
        self.context = None
        self.viewer = None


def run_scene(
    scene: str,
    angle_deg: float,
    realtime_factor: float,
    config: dict,
    common: tuple,
) -> None:
    manifest = common[0]
    reduced = common[7]
    world_path = stage4a.build_slope_world(angle_deg, config)
    adapter = stage4a.make_q_adapter(common, actuator_enabled=True)
    viewer = RealtimeTrackingViewer(realtime_factor)
    holder: dict[str, stage4a.SlopeGroundTruthRecorder] = {}

    def setup(sim):
        holder["recorder"] = stage4a.SlopeGroundTruthRecorder(
            sim,
            angle_deg,
            config,
            float(reduced["parameters"]["theta_eq_rad"]),
        )
        return {"demo_scene": scene, "terrain_angle_deg": angle_deg}

    def physics_step(sim):
        holder["recorder"].physics_step(sim)
        viewer(sim)

    scenario = {
        "name": f"stage4a_view_{scene}_q_on",
        "duration_s": float(config["command"]["duration_s"]),
        "linear_velocity_schedule": config["command"]["linear_velocity_schedule"],
        "yaw_rate_schedule": config["command"]["yaw_rate_schedule"],
    }
    print(
        f"[{scene}] {angle_deg:+g} degree slope, Q ON, "
        f"v_cmd=+0.4 m/s, duration={scenario['duration_s']:.0f} s"
    )
    try:
        stage3b.run_case(
            scenario,
            float(manifest["yaw"]["K_psi_nm_per_rad"]),
            float(manifest["yaw"]["K_r_nm_per_rad_s"]),
            yaw_enabled=True,
            motor_mismatch_enabled=False,
            common=common,
            keep_history=False,
            payload_mode="empty",
            common_mode_augmentation=adapter,
            physics_step_callback=physics_step,
            simulation_setup_callback=setup,
            empty_model_path=world_path,
            termination_guard=lambda sim: holder["recorder"].boundary_guard(sim),
        )
    finally:
        viewer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        choices=(*SCENES, "all"),
        default="all",
        help="run one slope or all four in order",
    )
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.realtime_factor) or args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be positive and finite")

    config = stage4a.load_json(stage4a.CONFIG_PATH)
    common = stage3b.load_common()
    stage4a.validate_frozen_inputs(config, common)
    selected = SCENES.items() if args.scene == "all" else [(args.scene, SCENES[args.scene])]

    print("Stage 4A minimal slope viewer: frozen Stage 3 controller, Q actuator ON.")
    print("The camera follows the chassis. Close the MuJoCo window to stop the demo.")
    try:
        for scene, angle_deg in selected:
            run_scene(scene, angle_deg, args.realtime_factor, config, common)
    except ViewerClosed:
        print("Viewer closed; demo stopped.")


if __name__ == "__main__":
    main()
