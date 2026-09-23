"""Interactively drive the frozen Stage 5 V1.5 command chain in MuJoCo."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage5_v1_5_pi_mcu_stream as stage5_v15  # noqa: E402
from control.rolling_reference import (  # noqa: E402
    DeterministicReferenceBlockStream,
    MotionIntent,
    TargetObservation,
)
from control.trajectory_feedforward import SparseProjectionFactorizationCache  # noqa: E402
from mujoco_realtime_viewer import (  # noqa: E402
    RealtimeTrackingViewer,
    ViewerClosed,
)
from sim.slope_estimation import (  # noqa: E402
    analytic_theta_eq,
    equilibrium_sum_torque_nm,
)


V_CMD_LIMIT_M_S = 0.6
UI_REFRESH_S = 1.0 / 60.0


class ManualCommandPanel:
    """Small Tk control panel; raw input remains a 20 Hz observation source."""

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Stage 5 V1.5 — manual v_cmd")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.closed = False

        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        ttk.Label(frame, text="raw v_cmd [m/s]").grid(
            row=0, column=0, sticky="w"
        )
        self.raw_text = tk.StringVar(value="+0.000")
        ttk.Label(frame, textvariable=self.raw_text, width=9, anchor="e").grid(
            row=0, column=1, sticky="e"
        )

        self.raw_value = tk.DoubleVar(value=0.0)
        self.scale = tk.Scale(
            frame,
            variable=self.raw_value,
            from_=-V_CMD_LIMIT_M_S,
            to=V_CMD_LIMIT_M_S,
            resolution=0.01,
            orient=tk.HORIZONTAL,
            length=390,
            showvalue=False,
            command=self._on_slider,
        )
        self.scale.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(2, 8))

        self.accepted_text = tk.StringVar(value="accepted_v_cmd: +0.000 m/s")
        ttk.Label(frame, textvariable=self.accepted_text).grid(
            row=2, column=0, columnspan=2, sticky="w"
        )
        self.mode_text = tk.StringVar(value="mode: HOLD")
        ttk.Label(frame, textvariable=self.mode_text).grid(
            row=3, column=0, columnspan=2, sticky="w"
        )
        self.candidate_text = tk.StringVar(value="candidate: —")
        ttk.Label(frame, textvariable=self.candidate_text).grid(
            row=4, column=0, columnspan=2, sticky="w", pady=(0, 8)
        )
        ttk.Button(frame, text="v_cmd = 0", command=self.zero).grid(
            row=5, column=0, columnspan=2, sticky="ew"
        )
        ttk.Label(
            frame,
            text="Input is sampled at the normal 20 Hz observation boundary.",
        ).grid(row=6, column=0, columnspan=2, sticky="w", pady=(8, 0))

    def _on_slider(self, value: str) -> None:
        self.raw_text.set(f"{float(value):+.3f}")

    def _on_close(self) -> None:
        self.closed = True

    def zero(self) -> None:
        self.raw_value.set(0.0)
        self.raw_text.set("+0.000")

    @property
    def raw_v_cmd_m_s(self) -> float:
        return float(self.raw_value.get())

    def pump(self, source) -> None:
        self.root.update_idletasks()
        self.root.update()
        scheduler = source.scheduler
        self.accepted_text.set(
            f"accepted_v_cmd: {scheduler.accepted_velocity_m_s:+.3f} m/s"
        )
        if source._full_locked:
            mode = "FULL"
        elif scheduler.candidate_active:
            mode = "CANDIDATE"
        elif (
            source._active_mode == "LIGHTWEIGHT"
            and source.lifecycle.phase.value == "VELOCITY_TRANSIENT"
        ):
            mode = "LIGHTWEIGHT"
        else:
            mode = "HOLD"
        self.mode_text.set(f"mode: {mode}")
        candidate = scheduler.candidate_target_m_s
        self.candidate_text.set(
            "candidate: —" if candidate is None
            else f"candidate: {candidate:+.3f} m/s"
        )
        self.raw_text.set(f"{self.raw_v_cmd_m_s:+.3f}")

    def close(self) -> None:
        try:
            self.root.destroy()
        except tk.TclError:
            pass


class ManualCommandProvider:
    """Expose only the slider command through the existing observation API."""

    def __init__(self, panel: ManualCommandPanel, observation_frequency_hz: float):
        self.panel = panel
        self.period_s = 1.0 / float(observation_frequency_hz)
        self.index = 0
        self.latest: TargetObservation | None = None
        self.latest_raw_v_cmd_m_s = 0.0
        self.last_logged_raw: float | None = None

    def setup(self, sim) -> dict:
        del sim
        self.index = 0
        self.latest = None
        self.latest_raw_v_cmd_m_s = self.panel.raw_v_cmd_m_s
        self.last_logged_raw = None
        return {
            "provider": "ManualSliderObservationProvider",
            "frequency_hz": 1.0 / self.period_s,
            "reads_sim_truth": False,
        }

    def read(self) -> TargetObservation:
        capture_time_s = self.index * self.period_s
        self.index += 1
        raw = min(V_CMD_LIMIT_M_S, max(-V_CMD_LIMIT_M_S, self.panel.raw_v_cmd_m_s))
        self.latest_raw_v_cmd_m_s = raw
        self.latest = TargetObservation(
            capture_time_s=capture_time_s,
            x_forward_m=0.0,
            y_left_m=0.0,
        )
        if self.last_logged_raw is None or abs(raw - self.last_logged_raw) >= 0.005:
            print(
                f"[input] t={capture_time_s:7.2f}s raw_v_cmd={raw:+.3f} m/s",
                flush=True,
            )
            self.last_logged_raw = raw
        return self.latest

    def diagnostic(self, sim) -> dict:
        del sim
        if self.latest is None:
            return {}
        return {
            "target_observation_capture_time_s": self.latest.capture_time_s,
            "manual_raw_v_cmd_m_s": self.latest_raw_v_cmd_m_s,
        }


def manual_follower(
    observation: TargetObservation, provider: ManualCommandProvider
) -> MotionIntent:
    return MotionIntent(
        source_time_s=float(observation.capture_time_s),
        linear_velocity_target_m_s=float(provider.latest_raw_v_cmd_m_s),
        yaw_rate_target_rad_s=0.0,
    )


class ConsoleEventLogger:
    """Print scheduler/planning lifecycle events without storing extra history."""

    def __init__(self) -> None:
        self.scheduler_count = 0
        self.plan_count = 0
        self.quiet_count = 0
        self.fade_count = 0
        self.exit_count = 0

    def poll(self, source) -> None:
        events = source.scheduler.events
        for event in events[self.scheduler_count:]:
            name = event["event"]
            time_s = float(event["time_s"])
            if name == "candidate_started":
                print(
                    f"[scheduler] t={time_s:.2f}s CANDIDATE start; "
                    f"raw={event['raw_velocity_m_s']:+.3f}, "
                    f"baseline={event['frozen_accepted_baseline_m_s']:+.3f}",
                    flush=True,
                )
            elif name == "candidate_stable":
                print(
                    f"[scheduler] t={time_s:.2f}s candidate stable; "
                    f"target={event['candidate_v_cmd_m_s']:+.3f}, "
                    f"delta={event['candidate_delta_v_m_s']:+.3f} m/s",
                    flush=True,
                )
            elif name in ("candidate_cancelled", "candidate_unstable"):
                print(
                    f"[scheduler] t={time_s:.2f}s {name}; "
                    f"reason={event.get('reason', 'window range changed')}",
                    flush=True,
                )
            elif name == "accepted":
                print(
                    f"[scheduler] t={time_s:.2f}s ACCEPT "
                    f"{event['mode']}; accepted_v_cmd="
                    f"{event['accepted_velocity_m_s']:+.3f} m/s",
                    flush=True,
                )
        self.scheduler_count = len(events)

        plans = source.replan_events
        for event in plans[self.plan_count:]:
            print(
                f"[planner] {event['planning_path']} target="
                f"{event['target_velocity_m_s']:+.3f} m/s; "
                f"plan={1000.0 * event['planning_wall_time_s']:.1f} ms; "
                f"T_ruckig={event['ruckig_duration_s']:.2f}s; "
                f"H={event['horizon_s']:.2f}s",
                flush=True,
            )
        self.plan_count = len(plans)

        quiet = source.quiet_events
        for event in quiet[self.quiet_count:]:
            print(f"[lifecycle] t={event['time_s']:.2f}s quiet snap", flush=True)
        self.quiet_count = len(quiet)

        fades = source.fade_events
        for event in fades[self.fade_count:]:
            print(
                f"[lifecycle] t={event['time_s']:.2f}s {event['event']}",
                flush=True,
            )
        self.fade_count = len(fades)

        exits = source.full_exit_events
        for event in exits[self.exit_count:]:
            print(
                f"[scheduler] t={event['time_s']:.2f}s FULL exit; "
                f"accepted_v_cmd={event['accepted_velocity_m_s']:+.3f} m/s; "
                f"pending={event['latest_pending_raw_v_cmd_m_s']:+.3f} m/s",
                flush=True,
            )
        self.exit_count = len(exits)


def make_source(config: dict, common: tuple):
    panel = ManualCommandPanel()
    provider = ManualCommandProvider(
        panel, float(config["observation_frequency_hz"])
    )
    cache = SparseProjectionFactorizationCache()
    full = stage5_v15.make_full_planner(
        config,
        common,
        planning_dt_s=float(config["full_dynamic"]["planning_dt_s"]),
        backend="cached_kkt",
        cache=cache,
    )
    source = stage5_v15.make_reference_source(
        config,
        common,
        provider,
        follower=lambda observation: manual_follower(observation, provider),
        full_planner_override=full,
    )
    stream = DeterministicReferenceBlockStream(
        source, link_latency_s=float(config["stream"]["main_link_latency_s"])
    )
    return panel, provider, source, stream


def run_demo(duration_s: float, realtime_factor: float) -> None:
    config = stage5_v15.load_json(stage5_v15.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen_checks = stage5_v15.load_common(
        config
    )
    if not all(frozen_checks.values()):
        raise RuntimeError("frozen Stage 4 baseline validation failed")

    panel, provider, source, stream = make_source(config, common)
    q_adapter, runtime = stage5_v15.make_runtime(
        common, config, stage4_config, stage4c_config
    )
    viewer = RealtimeTrackingViewer(realtime_factor)
    logger = ConsoleEventLogger()
    last_ui_time_s = -math.inf

    def physics_step(sim) -> None:
        nonlocal last_ui_time_s
        viewer(sim)
        if float(sim.data.time) - last_ui_time_s >= UI_REFRESH_S:
            panel.pump(source)
            logger.poll(source)
            last_ui_time_s = float(sim.data.time)
        if panel.closed:
            raise ViewerClosed

    scenario = {
        "name": "stage5_v1_5_manual_viewer_demo",
        "duration_s": float(duration_s),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    manifest = common[0]
    print("Stage 5 V1.5 manual viewer: frozen checks passed.")
    print(
        f"raw_v_cmd range: [{-V_CMD_LIMIT_M_S:+.1f}, "
        f"{V_CMD_LIMIT_M_S:+.1f}] m/s; observation 20 Hz; "
        f"reference blocks {config['reference_block']['sample_count']} × "
        f"{1000 * float(config['reference_block']['dt_s']):.0f} ms."
    )
    print("Close either window to stop the demo; the camera follows the chassis.")
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
            common_mode_augmentation=q_adapter,
            simulation_setup_callback=provider.setup,
            history_diagnostic_callback=provider.diagnostic,
            equilibrium_reference_callback=lambda context: analytic_theta_eq(
                runtime.alpha_control_rad, runtime.plant
            ),
            equilibrium_input_callback=lambda context: equilibrium_sum_torque_nm(
                runtime.alpha_control_rad,
                float(context["estimate"].velocity_hat_m_s),
                float(context["estimate"].theta_dot_hat_rad_s),
                runtime.plant,
            ),
            control_observer=runtime,
            reference_source=stream,
            physics_step_callback=physics_step,
        )
    except ViewerClosed:
        print("Viewer/control panel closed; demo stopped.")
    finally:
        viewer.close()
        panel.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration", type=float, default=180.0,
        help="maximum simulation time [s]; close a window to stop sooner",
    )
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and positive")
    if not math.isfinite(args.realtime_factor) or args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be finite and positive")
    run_demo(args.duration, args.realtime_factor)


if __name__ == "__main__":
    main()
