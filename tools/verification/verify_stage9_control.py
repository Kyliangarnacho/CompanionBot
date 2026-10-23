"""Stage 9 migration regression; reuses frozen Stage 4/6 runners and scenarios.

Only output destinations are redirected into Stage 9. No parameters, truth,
noise, duration or acceptance thresholds are changed. No viewer is opened.
"""
from companionbot_contracts.paths import repository_root

import json
import argparse
from pathlib import Path
import sys

ROOT = repository_root()



def extended_baseline(output):
    """Original Stage 4 closeout and frozen Stage 3 moving-payload Q-OFF arm."""
    import numpy as np
    import companionbot_simulation.scenarios.run_stage4_final_closeout as stage4
    import companionbot_simulation.scenarios.run_stage4c_slope_estimation as resolver
    import companionbot_simulation.scenarios.run_stage3c_mechanical_payload_q_revalidation as moving
    from companionbot_control.core.full_state_identification import DiscreteStateSpaceModel
    output.mkdir(parents=True, exist_ok=True)
    stage4.RESULT_DIR = output
    stage4.METRICS_PATH = output / "stage4_final_baseline_metrics.json"
    stage4.HISTORY_PATH = output / "stage4_final_baseline_history.json"
    stage4.REPORT_PATH = output / "STAGE4_REPLAY.md"
    stage4.main()  # Only output globals differ; all six scenarios/configs intact.
    metrics = json.loads(stage4.METRICS_PATH.read_text())
    assert all(metrics["frozen_baseline_checks"].values())
    assert metrics["Q_actuator_enabled"] is False and metrics["GT_runtime_dependency"] is False
    assert metrics["payload_id"]["result"]["accepted"]
    assert all(not case["fell"] and case["wheel_saturation_fraction"] == 0
               for group in ("lateral_payload", "single_wheel_impulse") for case in metrics[group].values())
    # Resolve the original Stage 3 motion seed/history frequency, rather than
    # importing Stage 5's seed overrides into the historical moving-payload arm.
    config_dir = ROOT / "models/minisegway/stage3/results/config"
    manifest = json.loads((config_dir / "baseline.json").read_text())
    motion = json.loads((config_dir / Path(manifest["motion_config"]).name).read_text())
    common = resolver.make_common({"history_frequency_hz": motion["history_frequency_hz"]}, motion["imu_rng_seed"])
    offline, plant = common[4], common[8]
    fit = offline["fit"]
    run, adapter = moving.run_arm(common, DiscreteStateSpaceModel(np.asarray(fit["A_identified"]), np.asarray(fit["B_identified"])),
        np.asarray(fit["state_scales"]), float(fit["input_scale_nm"]),
        2 * float(plant["known"]["wheel_torque_hard_peak_nm"]),
        float(manifest["yaw"]["K_psi_nm_per_rad"]), float(manifest["yaw"]["K_r_nm_per_rad_s"]), q_enabled=False)
    summary = moving.summarize_arm(run, adapter, common[3])
    gate = moving.mechanical_health_gate(summary)
    (output / "stage3_moving_q_off.json").write_text(json.dumps({"summary": summary, "original_health_gate": gate}, indent=2))
    (output / "stage3_moving_history.json").write_text(json.dumps(run["history_50hz"]))
    print(json.dumps({"moving_payload_original_gate": gate}), flush=True)
    assert gate["passed"]


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--reuse-recorded", action="store_true", help="Re-audit this stage's recorded runs without repeating physics")
    cli.add_argument("--extended-baseline", action="store_true", help="Replay Stage 4 closeout and Stage 3 moving-payload Q-OFF instead of radial cases")
    cli.add_argument("--output-dir", type=Path, required=True, help="This engineering session's results, never frozen evidence")
    args = cli.parse_args()
    if args.extended_baseline:
        extended_baseline(args.output_dir.resolve())
        return
    import companionbot_simulation.scenarios.run_stage6_5_radial_kf as stage6
    stage5 = stage6.stage5
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = stage5.load_json(stage6.CONFIG_PATH)
    stage6_config = stage5.load_json(stage6.stage6_4.CONFIG_PATH)
    stage5_config = stage5.load_json(stage5.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen = stage5.load_common(stage5_config)
    if not all(frozen.values()):
        raise RuntimeError("frozen baseline check failed")
    prior = json.loads((stage6.RESULT_DIR / "stage6_5_results.json").read_text())
    prior_by_name = {r["case"]["name"]: r for r in prior["cases"]}
    stage6.RESULT_DIR = output  # results only; original configuration/scenarios stay intact
    results, comparisons = [], []
    recorded = json.loads((output / "stage9_control.json").read_text())["cases"] if args.reuse_recorded else []
    recorded_by_name = {r["case"]["name"]: r for r in recorded}
    for case in config["cases"]:
        print(("AUDIT " if args.reuse_recorded else "RUN ") + case["name"], flush=True)
        result = (recorded_by_name[case["name"]] if args.reuse_recorded else
                  stage6.run_case(config, stage6_config, stage5_config, common, stage4_config, stage4c_config, case))
        assert result["case"] == case
        results.append(result)
        old = prior_by_name[case["name"]]
        checks = {"finite": result["simulation"]["finite"],
                  "no_fall": not result["simulation"]["fell"],
                  "zero_wheel_saturation": result["simulation"]["wheel_saturation_fraction"] == 0,
                  "zero_sum_saturation": result["simulation"]["sum_command_saturated_control_samples"] == 0,
                  "no_underrun": result["simulation"]["reference_underrun_count"] == 0,
                  "GT_runtime_dependency_false": result["simulation"]["GT_runtime_dependency"] is False,
                  "no_alignment_miss": result["alignment"]["history_miss"] == 0}
        # These early Stage 6.5 artifacts predate the current 5/20-frame latch.
        # Record the differences, but do not confuse them with same-code replay.
        # verify_stage9_dynamic.py compares the latest matching Windows evidence.
        old_dual_window = any("cruise_velocity_long_window_samples" in e for e in old["governor_events"])
        current_dual_window = any("cruise_velocity_long_window_samples" in e for e in result["governor_events"])
        comparisons.append({"case": case["name"], "checks": checks,
            "old_dual_window": old_dual_window, "current_dual_window": current_dual_window,
            "same_behavior_version": old_dual_window == current_dual_window,
            "original_governor_event_count": old["governor_event_count"],
            "current_governor_event_count": result["governor_event_count"],
            "kf_velocity_rmse_original": old["kf_velocity"]["rmse_m_s"],
            "kf_velocity_rmse_linux": result["kf_velocity"]["rmse_m_s"],
            "rmse_delta": result["kf_velocity"]["rmse_m_s"] - old["kf_velocity"]["rmse_m_s"]})
        (output / "stage9_control.json").write_text(json.dumps(
            {"frozen_checks": frozen, "cases": results, "original_comparison": comparisons}, indent=2))
        print(comparisons[-1], flush=True)
    if not all(all(r["checks"].values()) for r in comparisons):
        raise RuntimeError("migration invariant failed; inspect evidence, do not retune")


if __name__ == "__main__":
    main()
