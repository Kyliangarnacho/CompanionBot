"""Diagnostic/setup only: restore the EXISTING speech assets into Linux .venv.

Downloads occur only when explicitly invoked, never during sleep/runtime. The
Stage 8 pinned source/revision/size/hash evidence remains authoritative.
"""
from companionbot_contracts.paths import repository_root

import hashlib
import json
from pathlib import Path
import sys
from urllib.request import urlopen
import zipfile

ROOT = repository_root()


def download(url, path, expected=None):
    if path.is_file() and (expected is None or hashlib.sha256(path.read_bytes()).hexdigest() == expected):
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".download")
    with urlopen(url, timeout=60) as response, temporary.open("wb") as output:
        while block := response.read(1024 * 1024):
            output.write(block)
    if expected and hashlib.sha256(temporary.read_bytes()).hexdigest() != expected:
        raise ValueError("upstream_asset_sha256_mismatch: " + path.name)
    temporary.replace(path)


def main():
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError("Use this repository's Linux .venv")
    manifest = json.loads((ROOT / "models/minisegway/stage8/results/speech_assets_dfcbbc9df68f4395b08363fe80ad3882.json").read_text())
    restored = []
    for entry in manifest["assets"]:
        path = ROOT / ".venv/models/sensevoice" / entry["asset"]
        download(entry["source"], path, entry["sha256"])
        restored.append({"asset": entry["asset"], "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                         "source": entry["source"], "bytes": path.stat().st_size})
    vosk_dir = ROOT / ".venv/models/vosk-model-small-cn-0.22"
    if not vosk_dir.is_dir():
        archive = ROOT / ".venv/models/vosk-model-small-cn-0.22.zip"
        url = "https://alphacephei.com/vosk/models/vosk-model-small-cn-0.22.zip"
        download(url, archive)
        with zipfile.ZipFile(archive) as bundle:
            # Reject paths outside the documented model tree.
            for name in bundle.namelist():
                path = Path(name)
                if path.is_absolute() or ".." in path.parts or path.parts[0] != vosk_dir.name:
                    raise ValueError("invalid_vosk_archive_path")
            bundle.extractall(vosk_dir.parent)
        restored.append({"asset": archive.name, "source": url,
                         "sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                         "license": "Apache-2.0"})
    output = ROOT / "models/minisegway/stage9/results/restored_speech_assets.json"
    output.write_text(json.dumps({"source_manifest": str(manifest["revision"]), "assets": restored}, indent=2))
    print(output)


if __name__ == "__main__":
    main()
