"""Locate this project's unchanged model/config assets; no ROS dependency."""
import os
from pathlib import Path


def repository_root():
    configured = os.environ.get("COMPANIONBOT_ROOT")
    if configured:
        root = Path(configured).resolve()
        if not (root / "models/minisegway").is_dir():
            raise RuntimeError("COMPANIONBOT_ROOT has no CompanionBot model/config assets")
        return root
    # Supports editable installs, colcon symlinks and Windows maintenance.
    for root in Path(__file__).resolve().parents:
        if (root / "AGENTS.md").is_file() and (root / "models/minisegway").is_dir():
            return root
    raise RuntimeError("Set COMPANIONBOT_ROOT to the project containing models/minisegway")
