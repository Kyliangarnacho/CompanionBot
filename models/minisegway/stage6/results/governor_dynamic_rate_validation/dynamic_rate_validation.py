"""No-noise, zero-camera-delay speed-step checks for the Stage 6 viewer."""

from __future__ import annotations

import json
from pathlib import Path
import sys


REPO = Path.cwd()
sys.path.insert(0, str(REPO / "scripts"))
import view_stage6_5_manual_2d_follow as demo  # noqa: E402


def run_case(name: str, initial_speed: float, final_speed: float | None,
             step_time_s: float | None, duration_s: float) -> dict:
    captured: dict = {}
    original_run_case = demo.stage3b.run_case
    original_smoke_command = demo.ManualMasterDriver._smoke_command
    original_sensor_init = demo.ManualMasterTargetSensor.__init__
    original_sensor_setup = demo.ManualMasterTargetSensor.setup
    original_link_init = demo.stage6_5.KFPiLink.__init__

    def sensor_init(self, *args, **kwargs):
        original_sensor_init(self, *args, **kwargs)
        self.noise_std_m = 0.0

    def link_init(self, *args, **kwargs):
        kwargs["processing_delay_s"] = 0.0
        kwargs["jitter_s"] = 0.0
        original_link_init(self, *args, **kwargs)

    def phase_locked_sensor_setup(self, sim):
        result = original_sensor_setup(self, sim)
        # The physics callback samples at t = 0.001 mod 0.002, while MCU
        # RobotState packets are sent every 50 ms at t = 0 mod 0.050. Shift
        # only the 20 Hz capture phase by 1 ms so zero-delay observations still
        # have a future state sample available for the required interpolation.
        self.next_capture_time_s -= 0.001
        return result

    def command(time_s: float) -> tuple[float, float]:
        speed = (initial_speed if step_time_s is None
                 or time_s < step_time_s else float(final_speed))
        return speed, 0.0

    def capture_run_case(*args, **kwargs):
        link = kwargs["pre_reference_tick_callback"].__self__
        run = original_run_case(*args, **kwargs)
        captured.update(link=link, run=run)
        return run

    demo.ManualMasterTargetSensor.__init__ = sensor_init
    demo.ManualMasterTargetSensor.setup = phase_locked_sensor_setup
    demo.stage6_5.KFPiLink.__init__ = link_init
    demo.stage3b.run_case = capture_run_case
    demo.ManualMasterDriver._smoke_command = staticmethod(command)
    try:
        summary = demo.run_demo(duration_s, 100.0, headless_smoke=True)
    finally:
        demo.ManualMasterTargetSensor.__init__ = original_sensor_init
        demo.ManualMasterTargetSensor.setup = original_sensor_setup
        demo.stage6_5.KFPiLink.__init__ = original_link_init
        demo.stage3b.run_case = original_run_case
        demo.ManualMasterDriver._smoke_command = staticmethod(
            original_smoke_command
        )

    link = captured["link"]
    run = captured["run"]
    governor = link.governor
    source = link.producer.source
    transitions = []
    for event in governor.events:
        if event.get("event") != "cruise_resumed":
            continue
        transition = {
            key: event.get(key) for key in (
                "time_s", "sequence_id",
                "single_sample_kf_master_radial_velocity_m_s",
                "single_sample_shadow_latched_velocity_m_s",
                "cruise_velocity_median_m_s",
                "cruise_velocity_long_median_m_s",
                "cruise_velocity_window_samples",
                "cruise_velocity_long_window_samples",
                "latched_velocity_m_s", "distance_m", "reason",
            )
        }
        transition["master_speed_at_transition_m_s"] = (
            initial_speed if step_time_s is None or event["time_s"] < step_time_s
            else float(final_speed)
        )
        transition["step_to_transition_s"] = (
            None if step_time_s is None or event["time_s"] < step_time_s
            else event["time_s"] - step_time_s
        )
        transitions.append(transition)

    post_step_events = [
        {key: event.get(key) for key in (
            "time_s", "event", "reason", "master_velocity_hat_m_s",
            "latched_velocity_m_s", "distance_m",
        )}
        for event in governor.events
        if step_time_s is not None and event["time_s"] >= step_time_s
    ]
    alignments = {}
    for row in link.pipeline.rows:
        key = row["alignment"]
        alignments[key] = alignments.get(key, 0) + 1
    result = {
        "case": name,
        "initial_speed_m_s": initial_speed,
        "final_speed_m_s": final_speed,
        "step_time_s": step_time_s,
        "duration_s": duration_s,
        "noise_std_m": 0.0,
        "processing_delay_s": 0.0,
        "capture_phase_adjustment_s": -0.001,
        "alignment_counts": alignments,
        "jitter_s": 0.0,
        "summary": summary,
        "cruise_transitions": transitions,
        "post_step_governor_events": post_step_events,
        "post_step_full_plans": [
            event for event in source.replan_events
            if step_time_s is not None
            and event["time_s"] >= step_time_s
            and event["planning_path"] == "FULL_DYNAMIC"
        ],
        "reference_underruns": link.arbiter.receiver.unhandled_underrun_count,
        "wheel_saturation_fraction": run["torque_allocation"][
            "per_wheel_saturation_fraction"
        ],
        "sum_command_saturated_samples": sum(
            bool(row["sum_command_saturated"]) for row in run["history_50hz"]
        ),
        "allocator_guard_clipped_samples": sum(
            bool(row["allocator_guard_clipped"]) for row in run["history_50hz"]
        ),
        "max_abs_yaw_command_rad_s": summary["max_abs_yaw_command_rad_s"],
        "falls": summary["fell"],
        "pipeline_rows": link.pipeline.rows,
    }
    output = Path(__file__).with_name(f"dynamic_rate_{name}.json")
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps({
        "case": name,
        "first_cruise_transition": transitions[0] if transitions else None,
        "post_step_governor_event_count": len(post_step_events),
        "alignment_counts": alignments,
        "full_plans_after_step": len(result["post_step_full_plans"]),
        "fell": summary["fell"],
        "output": str(output),
    }), flush=True)
    return result


def main() -> None:
    # First measure transition time with each steady speed. The six step times
    # are then placed 0.2, 0.5, and 0.9 seconds before those reference events.
    accel_base = run_case("accel_constant_020", 0.20, None, None, 45.0)
    decel_base = run_case("decel_constant_040", 0.40, None, None, 15.0)
    if not accel_base["cruise_transitions"] or not decel_base["cruise_transitions"]:
        raise RuntimeError("steady-speed reference did not reach CRUISE")
    accel_t = accel_base["cruise_transitions"][0]["time_s"]
    decel_t = decel_base["cruise_transitions"][0]["time_s"]
    for phase in (0.20, 0.50, 0.90):
        run_case(f"accel_tail_phase_{phase:.2f}s", 0.20, 0.40,
                 max(0.0, accel_t - phase), accel_t + 17.0)
        run_case(f"decel_phase_{phase:.2f}s", 0.40, 0.20,
                 max(0.0, decel_t - phase), decel_t + 7.0)


if __name__ == "__main__":
    main()
