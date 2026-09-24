"""Headless, equal-speed Stage 6 viewer comparison; diagnostics only."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


REPO = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(REPO / "scripts"))
import view_stage6_5_manual_2d_follow as demo  # noqa: E402
import control.follow_governor as governor_module  # noqa: E402


def run_case(label: str, name: str, command: tuple[float, float]) -> None:
    captured: dict = {}
    original_run_case = demo.stage3b.run_case
    original_smoke_command = demo.ManualMasterDriver._smoke_command
    original_median = governor_module.median

    def capture_run_case(*args, **kwargs):
        link = kwargs["pre_reference_tick_callback"].__self__
        run = original_run_case(*args, **kwargs)
        captured.update(link=link, run=run)
        return run

    demo.stage3b.run_case = capture_run_case
    demo.ManualMasterDriver._smoke_command = staticmethod(
        lambda time_s: command
    )
    if label == "baseline":
        # Diagnostic ablation: replay the former single-sample transition.
        governor_module.median = lambda samples: samples[-1]
    try:
        summary = demo.run_demo(60.0, 100.0, headless_smoke=True)
    finally:
        demo.stage3b.run_case = original_run_case
        demo.ManualMasterDriver._smoke_command = staticmethod(
            original_smoke_command
        )
        governor_module.median = original_median

    link = captured["link"]
    run = captured["run"]
    source = link.producer.source
    history = run["history_50hz"]
    result = {
        "label": label,
        "transition_estimator": (
            "single_sample" if label == "baseline" else "short_long_robust"
        ),
        "case": name,
        "master_command_forward_lateral_m_s": command,
        "duration_s": 60.0,
        "summary": summary,
        "pipeline_rows": link.pipeline.rows,
        "governor_observations": link.governor.observation_history,
        "governor_events": link.governor.events,
        "full_replan_events": [
            event for event in source.replan_events
            if event["planning_path"] == "FULL_DYNAMIC"
        ],
        "full_exit_events": source.full_exit_events,
        "scheduler_events": source.scheduler.events,
        "runtime_20hz": [
            {key: row.get(key) for key in (
                "t", "v_ref_m_s", "v_hat_m_s", "v_GT_m_s",
                "pending_command_active", "latest_pending_raw_v_cmd_m_s",
                "yaw_rate_cmd_rad_s", "mcu_safety_state",
            )}
            for row in history[::25]
        ],
        "sum_command_saturated_samples": sum(
            bool(row["sum_command_saturated"]) for row in history
        ),
        "allocator_guard_clipped_samples": sum(
            bool(row["allocator_guard_clipped"]) for row in history
        ),
        "allocator_differential_clipped_samples": sum(
            bool(row["allocator_differential_clipped"]) for row in history
        ),
        "reference_underruns": link.arbiter.receiver.unhandled_underrun_count,
        "wheel_saturation_fraction": run["torque_allocation"][
            "per_wheel_saturation_fraction"
        ],
    }
    output = Path(__file__).with_name(f"{label}_{name}.json")
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"{name}: {summary}; wrote {output}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("label", choices=("baseline", "robust"))
    args = parser.parse_args()
    run_case(args.label, "vx_040", (0.40, 0.0))
    run_case(args.label, "vy_040", (0.0, 0.40))
