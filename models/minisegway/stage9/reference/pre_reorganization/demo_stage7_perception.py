"""Final Stage 7 RGB demo; uses the validated depth runner and source workers."""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.demo_yolo26n_depth import main


if __name__ == "__main__":
    raise SystemExit(main(["--mode", "full", *sys.argv[1:]]))
