"""Single-variable ablation of the observed frozen noise-free event shift.

Diagnostic only. Uses the full original failing case and its exact configuration.
No disturbance, plant, controller, governor threshold or truth tolerance changes.
"""
from pathlib import Path
import argparse
import json

def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--part",choices=("estimator","feedback","yaw","arbitration","all"),required=True)
    cli.add_argument("--output-dir",type=Path,required=True)
    args=cli.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    from companionbot_simulation.runtime.native_backend import native_backend
    from companionbot_simulation.scenarios import run_stage6_5_radial_kf as runner
    parts=("estimator","feedback","yaw","arbitration") if args.part=="all" else (args.part,)
    config=runner.stage5.load_json(runner.CONFIG_PATH)
    stage6_config=runner.stage5.load_json(runner.stage6_4.CONFIG_PATH)
    stage5_config=runner.stage5.load_json(runner.stage5.CONFIG_PATH)
    common,stage4,stage4c,frozen=runner.stage5.load_common(stage5_config)
    assert all(frozen.values());runner.RESULT_DIR=out
    case=next(case for case in config["cases"] if case["name"]=="noise_free_regression")
    with native_backend(parts):result=runner.run_case(config,stage6_config,stage5_config,common,stage4,stage4c,case)
    (out/"case.json").write_text(json.dumps({"parts":parts,"case":result},indent=2))
    print(json.dumps({"parts":parts,"events":result["governor_events"],"simulation":result["simulation"]}),flush=True)

if __name__=="__main__":main()
