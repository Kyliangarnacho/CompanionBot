"""Diagnostic: original Stage 5 V1.5 benchmark/replay/stream, outputs in Stage 9.

All phases call the existing runner. Config, controller, seeds, solver choices
and the 4 ms delay case are unchanged. No viewer or real transport is used.
"""
import json
import argparse
from pathlib import Path
import sys

from companionbot_contracts.paths import repository_root
from companionbot_simulation.scenarios import run_stage5_v1_5_pi_mcu_stream as stage5


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir", type=Path, required=True)
    output = cli.parse_args().output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    stage5.RESULT_DIR = output
    stage5.PLOT_DIR = output / "plots"
    # PLOT_PATHS is materialized when the original runner is imported; changing
    # PLOT_DIR alone does not redirect its four frozen-evidence output paths.
    stage5.PLOT_PATHS = {name: stage5.PLOT_DIR / path.name
                         for name, path in stage5.PLOT_PATHS.items()}
    stage5.RESULT_PATH = output / "stage5_v1_5_pi_mcu_stream_results.json"
    stage5.BENCHMARK_PATH = output / "stage5_v1_5_planner_benchmark.json"
    stage5.REPLAY_PATH = output / "stage5_v1_5_closed_loop_comparison.json"
    stage5.SUMMARY_PATH = output / "STAGE5_REPLAY.md"
    saved = sys.argv
    try:
        for phase in ("benchmark", "stream", "finalize"):
            sys.argv = ["stage9-stream-diagnostic", "--phase", phase]
            stage5.main()
    finally:
        sys.argv = saved
    result = json.loads(stage5.RESULT_PATH.read_text())
    metrics = result["final_stream"]["metrics"]
    delay = result["final_stream"]["synthetic_delay_case"]
    checks = {
        "frozen_baseline": all(result["frozen_baseline_checks"].values()),
        "completed": result["status"] == "COMPLETED",
        "cached_kkt_250hz_unchanged": result["selected_planner"] == "cached_kkt_250hz",
        "no_reference_underrun": metrics["reference_block_underrun_count"] == 0,
        "no_stale_or_gap": metrics["stale_block_count"] == metrics["sequence_gap_count"] == 0,
        "no_fall_or_saturation": not metrics["fall"] and metrics["saturation_count"] == 0,
        "no_false_slope": metrics["false_slope_count"] == 0,
        "original_delay_4ms": delay["link_latency_s"] == .004,
        "delay_no_underrun": delay["underrun_count"] == 0,
    }
    summary = {"checks": checks, "passed": all(checks.values())}
    (output / "verification.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)
    assert summary["passed"], "Inspect original run evidence; do not retune"


if __name__ == "__main__":
    main()
