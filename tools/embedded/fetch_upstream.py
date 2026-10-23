"""Fetch/verify ST's pinned sources into an ignored local cache, never vendor them."""
from pathlib import Path
import argparse
import json
import subprocess

ROOT = Path(__file__).resolve().parents[2]

def run(*args, cwd=None):
    subprocess.run(args, cwd=cwd, check=True)

def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--cache", type=Path, default=ROOT / ".venv/embedded/upstream")
    cli.add_argument("--verify-only", action="store_true")
    args = cli.parse_args()
    lock = json.loads((ROOT / "firmware/upstream.lock.json").read_text())
    package = lock["STM32CubeG4"]
    path = args.cache.resolve() / "STM32CubeG4"
    if not (path / ".git").exists():
        if args.verify_only:
            raise SystemExit(f"Missing {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        run("git", "clone", "--depth", "1", "--branch", package["tag"],
            "--filter=blob:none", "--no-checkout", package["url"], str(path))
        run("git", "sparse-checkout", "init", "--cone", cwd=path)
        run("git", "sparse-checkout", "set", *package["sparse_paths"], cwd=path)
        run("git", "checkout", "--detach", package["commit"], cwd=path)
    actual = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=path, text=True).strip()
    if actual != package["commit"]:
        raise SystemExit(f"Refusing to replace existing upstream checkout {actual}: {path}")
    if not args.verify_only:
        run("git", "submodule", "update", "--init", "--depth", "1", *lock["submodules"], cwd=path)
    for subpath, expected in lock["submodules"].items():
        actual = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=path / subpath, text=True).strip()
        if actual != expected:
            raise SystemExit(f"Submodule mismatch: {subpath}: {actual}")
    required = ["Drivers/CMSIS/Include/core_cm4.h", "Drivers/STM32G4xx_HAL_Driver/LICENSE.md",
                "Drivers/CMSIS/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g474xx.s",
                "Projects/NUCLEO-G474RE/Templates/MDK-ARM/Project.uvprojx"]
    for filename in required:
        if not (path / filename).is_file():
            raise SystemExit(f"Incomplete upstream: {path / filename}")
    print(f"Verified pinned ST upstream: {path}")

if __name__ == "__main__":
    main()
