"""CompanionBot follow algorithms and installed native runtime.

The editable workspace and ament install share Python sources. Native modules
are built/installed by ament, never compiled or replaced with Python on import.
"""
from pathlib import Path
import sys

# Keep editable-venv imports able to find the same installed ELF even when the
# caller doesn't need a ROS overlay. This adds a module search path only.
_root = Path(__file__).resolve().parents[4]
_installed = (_root / "ros2_ws/install/companionbot_follow/lib" /
              f"python{sys.version_info.major}.{sys.version_info.minor}" /
              "site-packages/companionbot_follow")
if _installed.is_dir() and str(_installed) not in __path__:
    __path__.append(str(_installed))
del _root, _installed
