"""Diagnostic: replay the eight latest, unchanged Stage 6 dynamic-rate cases.

Reuses the archived runner, including its documented -1 ms sensor phase.
Redirects only __file__ (that runner's output directory); never opens a viewer.
"""
from companionbot_contracts.paths import repository_root

import importlib.util
import importlib
import json
from pathlib import Path
import sys

ROOT = repository_root()

ORIGINAL = ROOT / "models/minisegway/stage6/results/governor_dynamic_rate_validation"
OUTPUT = ROOT / "models/minisegway/stage9/results/architecture/dynamic_control"


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    # The frozen evidence runner imports this historical bare module name.
    # Bind just that dependency to its sole relocated implementation; the
    # original archived runner and its -1 ms sensor phase remain byte-identical.
    sys.modules["view_stage6_5_manual_2d_follow"] = importlib.import_module(
        "companionbot_simulation.scenarios.view_stage6_5_manual_2d_follow")
    spec = importlib.util.spec_from_file_location("frozen_dynamic_validation", ORIGINAL / "dynamic_rate_validation.py")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.__file__ = str(OUTPUT / "dynamic_rate_validation.py")
    names = ["accel_constant_020", "decel_constant_040"]
    names += [f"{kind}_phase_{phase:.2f}s" for phase in (.2, .5, .9)
              for kind in ("accel_tail", "decel")]
    comparisons = []
    for name in names:
        old = json.loads((ORIGINAL / f"dynamic_rate_{name}.json").read_text())
        new = runner.run_case(name, old["initial_speed_m_s"], old["final_speed_m_s"],
                              old["step_time_s"], old["duration_s"])
        old_events = [(r["event"], round(r["time_s"], 6)) for r in old["post_step_governor_events"]]
        new_events = [(r["event"], round(r["time_s"], 6)) for r in new["post_step_governor_events"]]
        old_transitions, new_transitions = old["cruise_transitions"], new["cruise_transitions"]
        # Numerical comparison tolerance only; frozen controller/acceptance
        # limits and the original scenarios are not changed.
        latch_delta = max((abs(a["latched_velocity_m_s"] - b["latched_velocity_m_s"])
                           for a, b in zip(old_transitions, new_transitions)), default=0.)
        checks = {
            "finite": new["summary"]["finite"], "no_fall": not new["falls"],
            "no_wheel_saturation": new["wheel_saturation_fraction"] == 0,
            "no_sum_saturation": new["sum_command_saturated_samples"] == 0,
            "no_allocator_guard_clipping": new["allocator_guard_clipped_samples"] == 0,
            "no_underrun": new["reference_underruns"] == 0,
            "alignment_counts_equal": old["alignment_counts"] == new["alignment_counts"],
            "post_step_event_times_equal_to_1us": old_events == new_events,
            "full_plan_count_equal": len(old["post_step_full_plans"]) == len(new["post_step_full_plans"]),
            "cruise_sequence_equal": [r["sequence_id"] for r in old_transitions] == [r["sequence_id"] for r in new_transitions],
            "latch_difference_below_5e_5_m_s": latch_delta <= 5e-5,
        }
        comparisons.append({"case": name, "checks": checks, "max_latch_delta_m_s": latch_delta,
                            "post_step_events": new_events})
        (OUTPUT / "comparison.json").write_text(json.dumps(comparisons, indent=2))
        print(json.dumps(comparisons[-1]), flush=True)
    assert all(all(r["checks"].values()) for r in comparisons), "Inspect evidence; do not retune"


if __name__ == "__main__":
    main()
