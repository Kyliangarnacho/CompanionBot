"""Place the pinned official OSNet implementation in the active venv cache."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import sysconfig

UPSTREAM_URL = "https://github.com/KaiyangZhou/deep-person-reid.git"
UPSTREAM_COMMIT = "f8cd150fdf77e8d9e1ed143b7f308c2c609ded50"
DESTINATION = (
    Path(sysconfig.get_paths()["purelib"]) / "_companionbot_torchreid_source"
)


def main() -> int:
    print(f"python: {sys.executable}")
    print(f"version: {sys.version.split()[0]}")
    import pip
    print(f"pip: {pip.__version__} {pip.__file__}")
    if sys.prefix == sys.base_prefix:
        raise RuntimeError("run this installer from the CompanionBot .venv")

    if DESTINATION.exists():
        revision = subprocess.run(
            ["git", "-C", str(DESTINATION), "rev-parse", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
        )
        if revision.returncode == 0 and revision.stdout.strip() == UPSTREAM_COMMIT:
            (DESTINATION / "COMPANIONBOT_SOURCE_COMMIT").write_text(
                UPSTREAM_COMMIT + "\n", encoding="ascii"
            )
            print(f"official TorchReID source already present at {DESTINATION}")
            return 0
        raise RuntimeError(
            f"destination already exists and is not the pinned source: {DESTINATION}"
        )

    subprocess.run(
        ["git", "clone", "--filter=blob:none", "--no-checkout", UPSTREAM_URL,
         str(DESTINATION)],
        check=True,
    )
    subprocess.run(
        ["git", "-C", str(DESTINATION), "checkout", "--detach", UPSTREAM_COMMIT],
        check=True,
    )
    source_file = DESTINATION / "torchreid" / "models" / "osnet.py"
    if not source_file.is_file():
        raise RuntimeError(f"pinned checkout does not contain {source_file}")
    (DESTINATION / "COMPANIONBOT_SOURCE_COMMIT").write_text(
        UPSTREAM_COMMIT + "\n", encoding="ascii"
    )
    print(f"installed OSNet source from {UPSTREAM_COMMIT}: {source_file}")
    print("No TorchReID training requirements were installed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
