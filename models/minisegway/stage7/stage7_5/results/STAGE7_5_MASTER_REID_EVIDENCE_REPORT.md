# Stage 7.5 Master ReID Evidence Baseline

> Historical Stage 7.5 policy and measurements. Current `--reid` runs Stage 7.6
> pending-reference/automatic reacquire, so the old evidence-only behavior described
> below is not the current demo policy. See [Stage 7 closeout](../../STAGE7_FINAL_REPORT.md).

## Scope and decision

Stage 7.5 adds one OSNet appearance reference and candidate similarity diagnostics
to the Stage 7.4 manual Master selection flow. ReID has no authority to modify
`MasterManager`; only a user click can change `bound_track_id`. There is no
automatic reacquisition, threshold, gallery, EMA, tracker association change,
or Stage 6/controller connection.

The selected model is TorchReID's OSNet x0.25 with its MSMT17 ReID checkpoint.
It is a compact person-ReID model rather than a generic classification feature
extractor. The checkpoint is published by the upstream author in the MIT-licensed
[`kaiyangzhou/osnet` model collection](https://huggingface.co/kaiyangzhou/osnet).
The model implementation is [TorchReID](https://github.com/KaiyangZhou/deep-person-reid),
whose standard feature extractor uses 256×128 input, RGB ImageNet normalization,
and OSNet model outputs. This adapter uses the model on CPU and returns a
512-dimensional L2-normalized NumPy vector.

The full TorchReID training requirements were not copied into CompanionBot.
The upstream setup script imports its complete data/training package merely to
read package metadata, which pulled in unrelated dataset-download dependencies.
Instead, the installer checks out the upstream repository at commit
`f8cd150fdf77e8d9e1ed143b7f308c2c609ded50` under the active `.venv` and the
adapter loads only its self-contained `torchreid/models/osnet.py`. No upstream
source is stored in the CompanionBot repository. Runtime inference needs only
the existing PyTorch and torchvision packages. No Torch, torchvision, NumPy,
OpenCV, or Ultralytics version was changed.

The verified project environment was Python 3.11.9, PyTorch 2.14.0+cpu,
torchvision 0.29.0+cpu, NumPy 2.2.6, OpenCV 5.0.0.93, and Ultralytics 8.4.163.
The official checkpoint is pinned to Hugging Face revision
`01af85e82a9db4f3a4f6ed3a72ed9150bd416d04`, SHA256
`cf55163d78fc44c62c82f85ab62d39f10438679b5abe8c698ae08cfa84aa6e18` (9.34 MB),
and loads with `torch.load(weights_only=True)`. Its classifier head is discarded
while the OSNet feature tensors are loaded.

## Data and timing contract

The Stage 7.4 source loop keeps the dequeued `ColorFrame` while running detector
and tracker sequentially. Stage 7.5 passes that exact frame alongside its
`PersonTrackingFrame`; `PersonReIdentifier.embed_track()` rejects sequence,
source, or dimensions mismatch before cropping. It uses the track's original
source-frame `bbox_xyxy_px`, clips it to frame bounds, and rejects an empty crop.
It does not expand boxes or synthesize pixels from tracker prediction.

One embedding is created from the explicitly clicked track on that same frame.
The reference stays fixed while locked and is replaced only by an explicit
manual selection. On entering `LOST`, each active track ID is scored at most
once in that LOST episode. This bounds duplicate inference on persistent tracks;
new IDs that appear later in the same LOST episode receive one score when first
seen. Ranking and the top1/top2 margin are computed for the candidates scored
together on one frame. Scores are cosine similarities, not probabilities.

The result/event log records host read-complete `source_time_s`, track ID, source
sequence, bbox, score, cosine distance, rank, margin, embedding timing, and Master
state. Complete feature vectors are not logged. `embedding_wall_time_s` includes
crop plus adapter work; preprocessing and inference are also reported separately.

## Runtime boundary

The selected inference adapter uses CPU PyTorch and the official OSNet source;
the public ReID contracts expose NumPy only. Its checkpoint hash is verified
before loading. PyTorch versions with `weights_only=True` use that safe loader;
older builds can load only the exact SHA-verified upstream checkpoint. The
adapter boundary can later host an ONNX Runtime implementation without changing
the Master/reference/scoring contracts. This stage does not add or validate an
ONNX runtime, and the desktop timing is not a Pi 5 performance estimate.

## Validation and measurements

Hardware-free contract tests use a fake NumPy backend; they cover exact frame
pairing, track membership, clipping, invalid crops, feature validation, reference
replacement/clear, LOST-only ranking, margin, empty candidates, one score per
candidate per LOST episode, backend-object isolation, and the invariant that
Master state and binding remain unchanged during scoring.

The OSNet CPU benchmark uses a deterministic synthetic BGR crop. Its measurements
are performance-only and are not identity-accuracy samples. Run it with the
project virtual environment and the downloaded official checkpoint:

```powershell
.\.venv\Scripts\python.exe scripts\benchmark_stage7_5_reid.py --iterations 50
```

On the Windows desktop used for this run (32 logical CPUs, PyTorch configured
for 16 CPU threads), one 5-warm-up/50-crop run measured:

| Stage | Median | p95 |
|---|---:|---:|
| Preprocess | 0.53 ms | 0.72 ms |
| OSNet CPU inference | 16.96 ms | 19.34 ms |
| Total crop embedding | 17.61 ms | 19.94 ms |

Across four 50-crop runs with the same model, total median ranged from 17.38 to
19.21 ms and total p95 ranged from 19.94 to 40.87 ms. This host showed noticeable
tail-latency variability, so the lower p95 from the last run should not be
treated as a stable guarantee.

These are host-specific timing measurements, not Raspberry Pi estimates. The
synthetic random crop carries no identity information. The detailed local output
is written to `results/reid_cpu_benchmark.json` (ignored by Git). Same-person
and different-person score distributions, overlap, and the manual C920 scenes
are intentionally left for human acceptance; no threshold should be selected
until those samples exist.

## Human C920 acceptance

The pinned upstream model source and checkpoint are already in the active
project `.venv`. Start the demo with:

```powershell
.\.venv\Scripts\python.exe scripts\install_stage7_5_osnet.py
.\.venv\Scripts\python.exe scripts\demo_yolo26n_master_lock.py --camera-device 1 --width 640 --height 480 --fps 30 --preview --reid --download-reid-weights
```

Click the Master box to create the reference. Wait until the tracker reports
Master `LOST`, then bring the person and distractors into view. Candidate IDs,
cosine scores, rank, and margin are diagnostics only. `q` exits; `c` or `r`
clears Master and its reference. Use the Stage 7.5 scenarios A–F from the task
specification and retain the JSONL evidence file for the score-distribution
report. Duplicate tracks remain candidates; they never change the Master ID.

## Current acceptance status

- ReID API and Stage 7.1–7.5 contract tests: `72 passed`; all root `tests/`
  regression tests: `111 passed`, both in the project `.venv`.
- A repository-wide `pytest -q` also collects a historical nested Stage 5.1
  test that imports the removed `SimpleFollower` symbol; this pre-existing
  collection error is outside the Stage 7 scope. The full run does not reach
  execution after collection fails.
- Real OSNet CPU loading and timing passed using the pinned official weights;
  these timings are not identity-accuracy results.
- Official OSNet source is pinned in the `.venv`; checkpoint is cached under
  `.venv/reid_models`. Full feature vectors are kept out of event logs.
- Real C920 person-identity scenes: not claimed. They require operator-run
  same-person and different-person observations; no identity accuracy or
  reacquisition threshold is inferred from synthetic tests.
- Stage 7.3 ByteTrack, Stage 7.4 Master lifecycle behavior, and Stage 3–6
  controller/planner/safety baselines remain unchanged.
