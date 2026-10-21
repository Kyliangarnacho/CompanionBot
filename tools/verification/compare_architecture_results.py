"""Diagnostic: exact semantic comparison with the pre-reorganization Linux run.

This compares existing complete results, never changes scenarios or tolerance.
File paths and measured planner wall times are not controller outputs.
"""
import json

from companionbot_contracts.paths import repository_root


def main():
    base = repository_root() / "models/minisegway/stage9/results"
    output = base / "architecture"
    comparisons = {}
    for name in ("stage3_moving_q_off.json", "stage3_moving_history.json",
                 "stage4_final_baseline_metrics.json", "stage4_final_baseline_history.json"):
        before = json.loads((base / "extended_control" / name).read_text())
        after = json.loads((output / "extended_control" / name).read_text())
        comparisons[f"complete_json:{name}"] = before == after
    before = json.loads((base / "control/stage9_control.json").read_text())
    after = json.loads((output / "control/stage9_control.json").read_text())
    comparisons["stage6_frozen_checks"] = before["frozen_checks"] == after["frozen_checks"]
    comparisons["stage6_all_case_names"] = [r["case"] for r in before["cases"]] == [r["case"] for r in after["cases"]]
    for old, new in zip(before["cases"], after["cases"]):
        # Complete numerical results, events, counters and alignment; no
        # machine-dependent planning wall times or output figure paths.
        for key in ("kf_velocity", "raw_velocity", "stop_response_delay_s",
                    "governor_event_count", "raw_shadow_governor_event_count",
                    "governor_events", "raw_shadow_governor_events", "alignment",
                    "simulation", "turning_window", "state_packets_sent",
                    "observation_packets_sent", "yaw_packets_sent", "reference_blocks_sent"):
            comparisons[f"stage6:{old['case']}:{key}"] = old[key] == new[key]
    report = {"baseline": "pre-reorganization Linux Stage 9 results (not older single-window Windows results)",
              "comparison": "exact decoded JSON equality, no numerical tolerance",
              "checks": comparisons, "passed": all(comparisons.values())}
    (output / "behavior_comparison.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    assert report["passed"], "Inspect the full evidence; do not retune"


if __name__ == "__main__":
    main()
