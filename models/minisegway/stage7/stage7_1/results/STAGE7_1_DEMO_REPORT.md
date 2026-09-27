# Stage 7.1 — YOLO26n person detection smoke

> Historical report. The system-Python test run below was an environment mistake;
> the user subsequently reran the tests in the project `.venv`. Current validation
> and the final entrypoint are in [Stage 7 closeout](../../STAGE7_FINAL_REPORT.md).

Date: 2026-09-26

## Input and model

- Input: Ultralytics package's bundled `ultralytics/assets/bus.jpg`; no source
  photo or upstream code was copied into the repository.
- Original image: `(1080, 810, 3)`, `uint8`, OpenCV BGR.
- Checkpoint: official pretrained `yolo26n.pt`, downloaded from the Ultralytics
  `v8.4.0` assets release. The local checkpoint remains under ignored `.venv`.
- Python: 3.11.9; Ultralytics: 8.4.163; PyTorch: 2.14.0+cpu;
  torchvision: 0.29.0+cpu; NumPy: 2.2.6; OpenCV: 5.0.0.93;
  SciPy: 1.14.1.
- Runtime selected by Ultralytics: CPU. `torch.cuda.is_available()` was false
  in this project virtual environment. The detector code leaves device selection
  configurable and does not bind to a particular GPU.
- Dependency health: `pip check` passed.

## Inference

The learning demo called `model.predict(source=image, imgsz=640, conf=0.25,
verbose=False, stream=False)`. Ultralytics received the original BGR ndarray
and performed its own preprocessing. `Results.orig_shape` was `(1080, 810)`.

Ultralytics returned five boxes: one bus and four people. `person` was resolved
from `model.names` as class ID 0 for this checkpoint; the implementation does
not assume that ID in the detector contract.

| Detection | Class | Confidence | Original-image `xyxy` pixels |
|---|---|---:|---|
| 1 | person | 0.8720 | `(48.6143, 397.6982, 240.3370, 902.4067)` |
| 2 | person | 0.8608 | `(222.9416, 404.8176, 345.4665, 861.0154)` |
| 3 | person | 0.8462 | `(669.6431, 394.4845, 809.8669, 879.4365)` |
| 4 | person | 0.6562 | `(0.2079, 552.9568, 59.7275, 871.2654)` |

The saved preview draws only these four person boxes on a copy of the original
image: `person_detection_preview.jpg`.

Ultralytics reported per-image timing:

- preprocess: 6.514 ms
- inference: 57.620 ms
- postprocess: 1.585 ms
- whole `predict()` call: 2686.72 ms, including first-call CPU initialization /
  warm-up overhead; this is not the steady-state model timing.
- model load: 3507.72 ms

The static image has no source frame timestamp, so result-ready age is not
defined. The demo prints host monotonic inference start/end without replacing
or inventing source time.

## Adapter and tests

- `PersonDetector` real-model smoke returned a tuple of four `PersonDetection`
  records. For ndarray input, source sequence/time are `None`.
- Hardware-free camera and detector tests: **28 passed** under system Python
  3.8.5. Tests use fake model outputs and do not download weights.
- C920 live detection was not run in this stage; the required offline detection
  passed. No tracking, geometry, depth, or Stage 6 integration was added.
