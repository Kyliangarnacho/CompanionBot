# Stage 7 running and output guide

All commands run from the repository root in the project `.venv`.

## Environment

Before installing or testing:

```powershell
.\.venv\Scripts\python.exe -c "import sys; print(sys.executable); print(sys.version)"
.\.venv\Scripts\python.exe -m pip --version
```

For a new project environment:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-yolo.txt
.\.venv\Scripts\python.exe scripts\install_stage7_5_osnet.py
.\.venv\Scripts\python.exe -c "from perception.person_reid import download_default_osnet_weights; print(download_default_osnet_weights())"
```

OSNet source/license and weights stay in `.venv`; its training dependencies are
not installed. OSNet currently uses CPU. Detector/depth accept `--device`; choose
an appropriate official PyTorch wheel for a new host. Desktop timing is not Pi timing.

## Final RGB demo

```powershell
.\.venv\Scripts\python.exe scripts\demo_stage7_perception.py --camera-device 1 --preview
```

- Click a person: PENDING_LOCK → LOCKED after the Stage 7.6 reference gate.
  `c` clears; `q`/Escape exits. RGB shows source-pixel bbox/ROI/anchor, Master state,
  depth, camera XYZ, provisional robot XY and Hz/infer/age.
- Default K/D requires **1280×720** capture; neural-network imgsz stays independent.
- Camera device is explicit; OpenCV selects the backend unless `--opencv-backend`
  is supplied. Index 1/MSMF are this host's choices, not system-wide identities.
- `--config PATH` changes geometry. Asset paths inside the config are relative to
  the config file. Replace simulated R/t with measured mounting values. A measured
  `PitchProvider` can be injected with `main(..., pitch_provider_override=provider)`
  without changing business geometry. It owns clock mapping/history/sign conversion.
- `--select-track-id N` explicitly selects that ID once it appears, including in
  headless mode. It still uses pending reference confirmation.
- Full-mode output defaults to `models/minisegway/stage7/results/`; use distinct
  `--run-name` / `--output-dir` for separate trials.

Actual final smoke command:

```powershell
.\.venv\Scripts\python.exe scripts\demo_stage7_perception.py --camera-device 1 --select-track-id 1 --max-source-frames 300 --run-name final_live_smoke --output-dir models\minisegway\stage7\results\final_live_smoke
```

## Tests and diagnostic entrypoints

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

`pytest.ini` selects the active `tests/` suite, including all Stage 7 tests. The
Stage 5.1 test snapshot under `models/.../reference/` describes an archived API.

| Script | Use |
| --- | --- |
| `probe_rgb_camera.py` | Source/backend/profile/timestamp/calibration bring-up |
| `demo_yolo26n_image.py` | Static-image detection |
| `demo_yolo26n_stream.py` | Camera/video detection throughput |
| `demo_yolo26n_tracking.py` | ByteTrack diagnostics |
| `demo_yolo26n_master_lock.py` | Manual Master; `--reid` enables Stage 7.6 |
| `demo_yolo26n_depth.py` | `depth`, `detector`, `concurrent` benchmark modes; `full` final integration |

Old script filenames remain valid; output directories were relocated. The depth
runner now previews RGB only. `concurrent` retains manual-only Master behavior;
the final launcher selects `full`. `--video PATH --realtime-playback` can replace
camera input; final mode still requires the chosen calibration's resolution.
Video timestamps describe playback read-complete, not exposure.

## Output semantics

- `*_depth_frames.csv`: depth adapter elapsed time and ready-minus-source age.
- `*_detector_frames.csv`: detector/tracker timing and worker result age. In full
  mode, readiness follows Master/ReID; detector call timing excludes OSNet.
- `*_master_camera_points.csv`: exact-frame visible-Master ROI depth and camera XYZ;
  unavailable numeric values remain empty.
- `*_robot_observations.csv`: source metadata, forward/left/up, pitch/provider,
  provisional flags and transform-result-ready time. Only available geometry is
  emitted; this is not a Stage 6 packet stream.
- `*_master_reid_events.jsonl`: lifecycle/reference/score/rejection/reacquire events,
  with no full embedding vectors.
- `*_resolved_config.json`, `*_metrics.json`, `*_report.md`: provenance and summary.

See [the final report](STAGE7_FINAL_REPORT.md) for current geometry/identity limits
and hardware work still required.
