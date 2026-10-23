"""Diagnostic acceptance of shared C against unchanged frozen MuJoCo suites.

Every output is redirected to the caller's engineering session. Never opens a
viewer or connects the Agent/perception to motion. No acceptance threshold tuning.
"""
from contextlib import nullcontext
from pathlib import Path
import argparse
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))  # existing tools are repository-owned diagnostics

def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--backend", choices=("python", "native"), required=True)
    cli.add_argument("--suite", choices=("baseline", "radial", "dynamic", "safety", "stream"), required=True)
    cli.add_argument("--output-dir", type=Path, required=True)
    args = cli.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    from companionbot_simulation.runtime.native_backend import native_backend, parity_summary
    context = native_backend() if args.backend == "native" else nullcontext()
    started = time.perf_counter()
    summary = {"backend": args.backend, "suite": args.suite, "passed": False}
    try:
        with context:
            if args.suite == "baseline":
                from tools.verification.verify_stage9_control import extended_baseline
                extended_baseline(output)
            elif args.suite == "safety":
                from companionbot_simulation.scenarios import run_stage6_4_2d_follow as runner
                config = runner.stage5.load_json(runner.CONFIG_PATH)
                stage5_config = runner.stage5.load_json(runner.stage5.CONFIG_PATH)
                common, stage4, stage4c, frozen = runner.stage5.load_common(stage5_config)
                assert all(frozen.values())
                runner.RESULT_DIR = output
                results = []
                # Full frozen scenario list, including pass-by / safety / dropouts.
                for scenario in config["scenarios"]:
                    print("RUN " + scenario["name"], flush=True)
                    result = runner.run_scenario(config, stage5_config, common, stage4, stage4c, scenario)
                    results.append(result)
                    (output / "safety_cases.json").write_text(json.dumps(results, indent=2))
                    assert result["simulation"]["finite"] and not result["simulation"]["fell"]
                    assert result["split_authority"]["reference_underrun_count"] == 0
                    assert result["baseline"]["GT_runtime_dependency"] is False
            else:
                from tools.verification import verify_stage9_control, verify_stage9_dynamic, verify_stage9_stream
                runner = {"radial": verify_stage9_control, "dynamic": verify_stage9_dynamic,
                          "stream": verify_stage9_stream}[args.suite]
                saved = sys.argv
                try:
                    sys.argv = [str(__file__), "--output-dir", str(output)]
                    runner.main()
                finally:
                    sys.argv = saved
            summary["passed"] = True
    except BaseException as exc:
        summary["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        summary["duration_s"] = time.perf_counter() - started
        if args.backend == "native": summary["parity"] = parity_summary()
        (output / "cross_language_verification.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary), flush=True)

if __name__ == "__main__": main()
