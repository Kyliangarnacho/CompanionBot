"""Stage 9 diagnostic: snapshot and verify an authorized source relocation.

Evidence lives in Stage 9. Original experiment/config/provenance files are
hashed, never rewritten by this tool. No GUI or hardware access.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "models/minisegway/stage9/results/architecture"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def snapshot():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    manifest = OUTPUT / "before.json"
    if manifest.exists():
        raise RuntimeError("Refusing to replace the pre-reorganization snapshot")
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode().split("\0")
    paths = {name for name in tracked if name and (ROOT / name).is_file()}
    paths.update(str(p.relative_to(ROOT)) for p in (ROOT / "ros2_ws/src").rglob("*")
                 if p.is_file() and "__pycache__" not in p.parts)
    paths.update(str(p.relative_to(ROOT)) for p in (ROOT / "scripts").glob("*.*") if p.is_file())
    paths.update(["pyproject.toml", "requirements.txt", "requirements-linux.txt"])
    paths.update(str(p.relative_to(ROOT)) for p in (ROOT / "models/minisegway/stage9").glob("*.md"))
    data = {name: digest(ROOT / name) for name in sorted(paths)}
    manifest.write_text(json.dumps(data, indent=2), encoding="utf-8")
    protected = {str(p.relative_to(ROOT)): digest(p) for p in (ROOT / "models").rglob("*")
                 if p.is_file() and not p.is_relative_to(OUTPUT)
                 and not (p.parent == ROOT / "models/minisegway/stage9" and p.suffix == ".md")}
    (OUTPUT / "protected_models.json").write_text(json.dumps(protected, indent=2))
    (OUTPUT / "git_status_before.txt").write_text(subprocess.check_output(
        ["git", "status", "--short"], cwd=ROOT).decode(), encoding="utf-8")
    (OUTPUT / "history_review.txt").write_text(subprocess.check_output(
        ["git", "log", "-12", "--format=%h %s"], cwd=ROOT).decode(), encoding="utf-8")
    with zipfile.ZipFile(OUTPUT / "before_sources.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for name in sorted(paths):
            if name.endswith((".py", ".md", ".toml", ".ini", ".xml", ".sh", ".cfg", ".msg", ".srv")):
                archive.write(ROOT / name, name)
    print(f"Snapshot: {len(data)} files; originals retained in Stage 9")


def definitions(source):
    """Compare complete executable bodies; imports/module names are packaging."""
    tree = ast.parse(source)
    return [(node.name, ast.dump(node, include_attributes=False))
            for node in tree.body if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))]


def project_bindings(source, prefixes):
    """Check local names too: a moved export must retain its original alias."""
    result = set()
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ImportFrom) and (node.level or (node.module or "").startswith(prefixes)):
            result.update(alias.asname or alias.name for alias in node.names)
        elif isinstance(node, ast.Import):
            result.update(alias.asname or alias.name.split(".")[0] for alias in node.names
                          if alias.name.startswith(prefixes))
    return result


def audit():
    before = json.loads((OUTPUT / "before.json").read_text())
    plan = json.loads((OUTPUT / "relocation.json").read_text())
    changes = []
    protected = json.loads((OUTPUT / "protected_models.json").read_text())
    for name, original_digest in protected.items():
        if not (ROOT / name).is_file() or digest(ROOT / name) != original_digest:
            changes.append(name)
    with zipfile.ZipFile(OUTPUT / "before_sources.zip") as archive:
        body_mismatches, binding_mismatches, checked = [], [], 0
        for old, destinations in plan["algorithm_definitions"].items():
            original = definitions(archive.read(old).decode("utf-8"))
            actual = []
            for new in destinations:
                actual.extend(definitions((ROOT / new).read_text()))
            for name, body in original:
                # All original definitions, including the intentional two-stage
                # SparseTwoPathReferenceSource implementation, must be retained.
                if (name, body) not in actual:
                    body_mismatches.append(f"{old}:{name}")
                checked += 1
        for old, new in plan["moves"].items():
            if old.startswith("ros2_ws/") or not old.endswith(".py"):
                continue  # The former generic adapter was explicitly decomposed.
            required = project_bindings(archive.read(old).decode("utf-8"),
                                        ("control", "sim", "perception", "embodied_agent", "scripts", "learning"))
            actual = project_bindings((ROOT / new).read_text(), ("companionbot_",))
            if required - actual:
                binding_mismatches.append({"source": old, "missing_local_names": sorted(required - actual)})
    report = {"protected_files": len(protected), "protected_changed": changes,
              "algorithm_definitions_checked": checked, "body_mismatches": body_mismatches,
              "import_binding_mismatches": binding_mismatches,
              "passed": not changes and not body_mismatches and not binding_mismatches}
    (OUTPUT / "source_audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise RuntimeError("Architecture relocation changed protected evidence or executable bodies")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", action="store_true")
    args = parser.parse_args()
    snapshot() if args.snapshot else audit()
