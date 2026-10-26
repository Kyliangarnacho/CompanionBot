"""AI perception and its explicitly installed native stereo image boundary."""
from pathlib import Path
import sys

# The editable workspace shares these sources with ament. Native modules are
# built by ament, with no compile-on-import or Python implementation fallback.
_root = Path(__file__).resolve().parents[4]
_installed = (_root / "ros2_ws/install/companionbot_perception/lib" /
              f"python{sys.version_info.major}.{sys.version_info.minor}" /
              "site-packages/companionbot_perception")
if _installed.is_dir() and str(_installed) not in __path__:
    __path__.append(str(_installed))
del _root, _installed
