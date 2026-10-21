"""Repair the Ubuntu Matplotlib namespace collision inside this venv only.

Ubuntu's legacy matplotlib nspkg.pth can prepopulate mpl_toolkits from the
system-only path, hiding the matching pip toolkit. Import the normal namespace
while the venv site directory is processed first; Python then extends it as
system ROS paths are added. No system files are changed.
"""
from companionbot_contracts.paths import repository_root

from pathlib import Path
import sys
import sysconfig

ROOT = repository_root()
if sys.platform != "linux" or Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
    raise SystemExit("Run with CompanionBot's Linux .venv/bin/python")
path = Path(sysconfig.get_path("purelib")) / "companionbot_mpl_toolkits.pth"
path.write_text("import mpl_toolkits\n", encoding="utf-8")
print(path)
