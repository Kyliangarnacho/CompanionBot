"""One extended walk-stop-walk run using the Stage 6.2 packet boundary."""

from __future__ import annotations

import json
from pathlib import Path

import run_stage5_v1_5_pi_mcu_stream as stage5
import run_stage6_2_safety_decoupling as stage6_2


ROOT = Path(__file__).resolve().parents[1]
STAGE_DIR = ROOT / "models" / "minisegway" / "stage6"
CONFIG_PATH = STAGE_DIR / "config" / "stage6_3_wait_resume_config.json"
RESULT_DIR = STAGE_DIR / "results" / "wait_resume_extended_walk"


def main() -> None:
    config = stage5.load_json(CONFIG_PATH)
    stage5_config = stage5.load_json(stage5.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen_checks = stage5.load_common(
        stage5_config
    )
    if not all(frozen_checks.values()):
        raise RuntimeError(f"frozen baseline check failed: {frozen_checks}")
    scenarios = config["scenarios"]
    if len(scenarios) != 1:
        raise ValueError("Stage 6.3 permits exactly one extended scenario")
    scenario = scenarios[0]
    if any(float(row["value_m_s"]) < 0.0
           for row in scenario["master_speed_schedule"]):
        raise ValueError("Master schedule must be nonnegative")
    result = stage6_2.run_scenario(
        config, stage5_config, common, stage4_config, stage4c_config,
        scenario, result_dir=RESULT_DIR, show_master_gt=True,
    )
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    output = {"configuration": config, "frozen_checks": frozen_checks,
              "experiment": result}
    path = RESULT_DIR / "stage6_3_extended_results.json"
    path.write_text(json.dumps(output, indent=2, ensure_ascii=False),
                    encoding="utf-8")
    print(f"RESULT {path}", flush=True)


if __name__ == "__main__":
    main()
