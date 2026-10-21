"""Migration diagnostic: compare against the user-owned pre-migration tree."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "models/minisegway/stage9/results"


def main():
    original = json.loads((OUTPUT / "pre_migration_sha256.json").read_text())
    expected = {"AGENTS.md", ".gitignore", "README.md", "CURRENT_STATE.md",
                "embodied_agent/audio.py", "embodied_agent/devices.py", "embodied_agent/ui.py",
                "scripts/demo_stage8_interaction.py", "scripts/run_stage8_agent.py",
                "scripts/demo_yolo26n_depth.py", "scripts/smoke_test_minisegway.py",
                "models/minisegway/stage7/RUNNING.md", "models/minisegway/stage8/RUNNING.md"}
    changed, frozen = [], []
    for name, digest in original.items():
        path = ROOT / name
        same = path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest
        if not same:
            changed.append(name)
        if (name.startswith(("control/", "sim/", "learning/", "perception/", "tests/"))
                or any(name.startswith(f"models/minisegway/stage{stage}/") for stage in range(3, 7))
                or any(name.startswith(f"scripts/{kind}_stage{stage}") for kind in ("run", "view") for stage in range(3, 7))
                or name in ("models/minisegway/mini_segway.xml", "models/minisegway/plant_report.json", "models/minisegway/reduced_twip.json")):
            frozen.append({"path": name, "same_sha256": same})
    report = {"original_files": len(original), "changed_existing_files": sorted(changed),
              "unexpected_changed_existing_files": sorted(set(changed) - expected),
              "frozen_files_checked": len(frozen), "frozen_sha256_unchanged": all(r["same_sha256"] for r in frozen),
              "frozen_differences": [r for r in frozen if not r["same_sha256"]]}
    (OUTPUT / "boundary_audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    assert report["frozen_sha256_unchanged"] and not report["unexpected_changed_existing_files"]


if __name__ == "__main__":
    main()
