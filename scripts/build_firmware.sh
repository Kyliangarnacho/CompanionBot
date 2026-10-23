#!/usr/bin/env bash
set -euo pipefail
companionbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$companionbot_root"
.venv/bin/python -c 'import sys; print(sys.executable); print(sys.version)'
.venv/bin/python -m pip --version
.venv/bin/python tools/embedded/fetch_upstream.py --verify-only
.venv/bin/python tools/embedded/export_frozen_config.py --check
.venv/bin/python tools/embedded/generate_board_projects.py --check
companionbot_firmware_build="${1:-$companionbot_root/models/minisegway/cross_language_embedded/results/local_firmware_build}"
cmake -S firmware -B "$companionbot_firmware_build" \
  -DCMAKE_TOOLCHAIN_FILE="$companionbot_root/firmware/cmake/arm-none-eabi.cmake" -DCMAKE_BUILD_TYPE=Release
cmake --build "$companionbot_firmware_build" -j 4
