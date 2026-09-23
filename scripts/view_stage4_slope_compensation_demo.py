"""Run the frozen Stage 4 slope-estimation/compensation baseline in MuJoCo."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage4_final_closeout as stage4final  # noqa: E402
import run_stage4a_slope_robustness as stage4a  # noqa: E402
from mujoco_realtime_viewer import (  # noqa: E402
    RealtimeTrackingViewer,
    ViewerClosed,
)


def run_scene(
    angle_deg: float,
    realtime_factor: float,
    slope_config: dict,
    stage4_config: dict,
    stage4c_config: dict,
    common: tuple,
) -> None:
    manifest = common[0]
    world_path = stage4a.build_slope_world(angle_deg, slope_config)
    q_adapter = stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage4final.FinalRuntime(
        reduced=common[7],
        config=stage4_config,
        stage4c_config=stage4c_config,
        q_adapter=q_adapter,
    )
    viewer = RealtimeTrackingViewer(realtime_factor)
    holder: dict[str, stage4a.SlopeGroundTruthRecorder] = {}
    state = {"next_status_s": 0.0, "transition_count": 0}

    def setup(sim) -> dict:
        holder["recorder"] = stage4a.SlopeGroundTruthRecorder(
            sim,
            angle_deg,
            slope_config,
            float(common[7]["parameters"]["theta_eq_rad"]),
        )
        return {
            "demo_scene": f"slope_{angle_deg:+g}_deg",
            "terrain_angle_deg_posthoc": angle_deg,
        }

    def physics_step(sim) -> None:
        holder["recorder"].physics_step(sim)
        viewer(sim)

        transitions = runtime.transition_log
        for event in transitions[state["transition_count"]:]:
            print(
                f"[supervisor] t={event['time_s']:.2f}s "
                f"{event['from']} -> {event['to']}: {event['reason']}",
                flush=True,
            )
        state["transition_count"] = len(transitions)

        if float(sim.data.time) + 1e-12 >= state["next_status_s"]:
            fields = runtime.output({})
            theta_eq = stage4final.analytic_theta_eq(
                runtime.alpha_control_rad, runtime.plant
            )
            print(
                f"[{angle_deg:+g} deg] t={float(sim.data.time):5.1f}s "
                f"mode={fields['environment_mode']:<5} "
                f"alpha_hat={fields['alpha_hat_ekf_deg']:+6.2f} deg "
                f"alpha_control={fields['alpha_control_deg']:+6.2f} deg "
                f"theta_eq={math.degrees(theta_eq):+6.2f} deg",
                flush=True,
            )
            state["next_status_s"] += 1.0

    scenario = {
        "name": f"stage4_slope_compensation_{angle_deg:+g}deg",
        "duration_s": float(slope_config["command"]["duration_s"]),
        "linear_velocity_schedule": slope_config["command"][
            "linear_velocity_schedule"
        ],
        "yaw_rate_schedule": slope_config["command"]["yaw_rate_schedule"],
    }
    print(
        f"\n=== {angle_deg:+g} deg: flat lead-in → grade; "
        f"scheduled forward command, "
        f"{scenario['duration_s']:.0f} s ===",
        flush=True,
    )

    try:
        run = stage3b.run_case(
            scenario,
            float(manifest["yaw"]["K_psi_nm_per_rad"]),
            float(manifest["yaw"]["K_r_nm_per_rad_s"]),
            yaw_enabled=True,
            motor_mismatch_enabled=False,
            common=common,
            keep_history=False,
            payload_mode="empty",
            common_mode_augmentation=q_adapter,
            control_observer=runtime,
            simulation_setup_callback=setup,
            physics_step_callback=physics_step,
            empty_model_path=world_path,
            termination_guard=lambda sim: holder["recorder"].boundary_guard(sim),
            equilibrium_reference_callback=lambda _context: (
                stage4final.analytic_theta_eq(runtime.alpha_control_rad, runtime.plant)
            ),
            equilibrium_input_callback=lambda context: (
                stage4final.equilibrium_sum_torque_nm(
                    runtime.alpha_control_rad,
                    float(context["estimate"].velocity_hat_m_s),
                    float(context["estimate"].theta_dot_hat_rad_s),
                    runtime.plant,
                )
            ),
        )
        termination = run["simulation_termination"]
        print(
            f"[{angle_deg:+g} deg] finished: "
            f"reason={termination['reason']}, "
            f"fall={run['longitudinal']['fell']}",
            flush=True,
        )
    finally:
        viewer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.realtime_factor) or args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be finite and positive")

    slope_config = stage4a.load_json(stage4a.CONFIG_PATH)
    stage4_config = stage4final.load_json(stage4final.CONFIG_PATH)
    stage4c_config = stage4final.load_json(stage4final.STAGE4C_CONFIG_PATH)
    stage4final.validate_frozen_baseline(stage4_config, stage4c_config)
    common = stage4final.make_common(stage4_config, int(stage4_config["seed"]))
    stage4a.validate_frozen_inputs(slope_config, common)

    angles = [float(angle) for angle in slope_config["angles_deg"]]
    print(
        "Frozen Stage 4 slope compensation demo. "
        "Q actuator OFF; slope estimate/control angle/theta_eq are logged."
    )
    print("Close the MuJoCo window to stop early; otherwise all four scenes run.")
    try:
        for angle_deg in angles:
            run_scene(
                angle_deg,
                args.realtime_factor,
                slope_config,
                stage4_config,
                stage4c_config,
                common,
            )
    except ViewerClosed:
        print("Viewer closed; remaining scenes skipped.")
    print("Slope demo complete.")


if __name__ == "__main__":
    main()
