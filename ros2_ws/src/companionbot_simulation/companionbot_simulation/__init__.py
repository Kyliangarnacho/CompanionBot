"""CompanionBot simulation, with installed native scheduling bindings."""
from pathlib import Path
import sys

_root = Path(__file__).resolve().parents[4]
_installed = (_root / "ros2_ws/install/companionbot_simulation/lib" /
              f"python{sys.version_info.major}.{sys.version_info.minor}" /
              "site-packages/companionbot_simulation")
if _installed.is_dir() and str(_installed) not in __path__:
    __path__.append(str(_installed))
del _root, _installed
