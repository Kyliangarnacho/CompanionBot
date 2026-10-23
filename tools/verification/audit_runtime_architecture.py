"""Read-only audit of frozen assets/algorithms and current documentation links."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
from urllib.parse import unquote


ROOT = Path(__file__).resolve().parents[2]


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--before", type=Path, required=True, help="Pre-engineering before_hashes.json")
    cli.add_argument("--output-dir", type=Path, required=True)
    args = cli.parse_args()
    before = json.loads(args.before.read_text())
    def mismatches(entries):
        return [name for name, digest in entries.items() if not (ROOT / name).is_file() or
                hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != digest]
    # Lifecycle / ROS wrappers may change; numerical algorithms and the full
    # frozen scenario/transport/control runtime implementation must not.
    algorithms = {name: digest for name, digest in before["source"].items()
                  if name.endswith(".py") and ("/core/" in name or "/estimation/" in name or
                  "/sensors/" in name or "/scenarios/" in name or
                  any(f"/{owner}/runtime/" in name for owner in
                      ("companionbot_control", "companionbot_follow", "companionbot_simulation")))}
    shared_assets = [name for name in subprocess.check_output(
        ["git", "ls-files", "-z", "models/minisegway"], cwd=ROOT).decode().split("\0")
        if name and not Path(name).relative_to("models/minisegway").parts[0].startswith("stage")]
    shared_changed = [name for name in shared_assets if not (ROOT / name).is_file() or
                      (ROOT / name).read_bytes() != subprocess.check_output(["git", "show", "HEAD:" + name], cwd=ROOT)]
    documents = [ROOT / "README.md", ROOT / "CURRENT_STATE.md", ROOT / "ros2_ws/README.md", ROOT / "tools/README.md"]
    documents += list((ROOT / "docs").glob("*.md"))
    documents += list((ROOT / "ros2_ws/src").glob("*/README.md"))
    documents += list((ROOT / "models/minisegway/runtime_architecture").glob("*.md"))
    broken = []
    for path in documents:
        for target in re.findall(r"\[[^\]]*\]\(([^)]+)\)", path.read_text()):
            target = target.strip().strip("<>")
            if target.startswith(("https://", "http://", "#", "mailto:")):
                continue
            target = unquote(target.split("#", 1)[0])
            if target and not (path.parent / target).exists():
                broken.append({"document": str(path.relative_to(ROOT)), "target": target})
    result = {"historical_files_checked": len(before["historical"]),
              "historical_changed": mismatches(before["historical"]),
              "algorithm_files_checked": len(algorithms), "algorithm_changed": mismatches(algorithms),
              "shared_tracked_model_files_checked": len(shared_assets), "shared_model_changed": shared_changed,
              "current_documents_checked": len(documents), "broken_local_links": broken}
    result["passed"] = not any(result[k] for k in ("historical_changed", "algorithm_changed", "shared_model_changed", "broken_local_links"))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "source_audit.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(result, indent=2, ensure_ascii=False))
    assert result["passed"], "Inspect the exact changed paths; never retune or rewrite frozen evidence"


if __name__ == "__main__":
    main()
