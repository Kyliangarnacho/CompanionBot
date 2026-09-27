# Stage 7.7 Depth Foundation / YOLO26 Depth Baseline

> Historical depth/geometry baseline and performance evidence. Final preview is
> now RGB only; full mode adds existing Stage 7.6 ReID and provisional robot geometry.
> Current entrypoint/config/validation: [Stage 7 closeout](../../STAGE7_FINAL_REPORT.md).

## Calibration heritage and capture

- MonoTeach `data/calibrations/` contains workspace homography JSON files, not a K/D NPZ or YAML. Their `camera_calibration_reference` points to `D:\project\C920e_Calibration\output\camera_params.npz`.
- The original NPZ and calibration report identify a Logitech C920e calibration at **1280×720**, with 20 valid images and 0.565730 px RMS. The NPZ K/D load through the existing CompanionBot `perception.camera_calibration` loader. Its SHA-256 is `253DBA67A99F9DD6B2BC8A2822C6483C654C8726ACA7E4A38935F1E30EF0B57C`. A byte-identical copy is in `../config/c920e_camera_params_1280x720.npz`.
- CompanionBot previously had the K/D loader but no intrinsic asset. This stage copies the original asset with provenance; no recalibration or K scaling occurred. Physical identity between the C920e calibration unit and the currently enumerated camera index 1 has not been independently verified.
- Headless C920 probe: OpenCV MSMF, requested 1280×720@30, reported 1280×720@30, actual BGR uint8 frame `(720, 1280, 3)`, K/D exact-resolution match, 60 continuous source sequence IDs, strictly increasing host read-complete timestamps. Measured read-complete rate: 24.87 Hz. The three 240-frame benchmark runs measured source rates of 20.48, 22.76 and 22.32 Hz, respectively; the requested/reported 30 FPS is not the measured rate.

## Contract and alignment

- `DepthFrame` contains source ID, source sequence, host read-complete source time, actual width/height, `float32` H×W depth in **metres**, backend and model name. Invalid/nonpositive model values become NaN. There are no torch or Ultralytics objects in the public result.
- The adapter sends the original BGR `ColorFrame` to official `yolo26n-depth.pt` with independent `imgsz=768`. The installed Ultralytics 8.4.163 `DepthPredictor.postprocess` scales model output back through the original image shape. The adapter rejects a mismatched `Results.orig_shape` or depth map shape and copies sequence/time/size from the precise input frame. The live 1280×720 inference smoke test returned `(720, 1280)` float32 data. Pixelwise geometric accuracy has not been checked against ground truth.
- Detector `imgsz=640` remains independent of source resolution. A 1280×720 hardware detector/tracker run produced 197 results with a person and track in every result, and all logged frames were 1280×720. A hardware-free 1280×720 regression test checks detector bbox coordinates near the frame edge, Master click selection, ReID source crop dimensions and preview output size. Existing Stage 7.1–7.6 runtime code was unchanged.
- One Stage 7.2 camera source feeds independent capacity-one latest-frame slots for depth and detector/tracker. No channel waits for the other's inference. No FIFO of stale frames or second VideoCapture is introduced.

## Headless performance on this Windows CPU environment

All runs used camera index 1, MSMF, requested 1280×720@30, exact-resolution K/D, 240 source frames, and excluded 2 result warmups. Depth model: `yolo26n-depth.pt`, `imgsz=768`; detector: `yolo26n.pt`, `imgsz=640`, confidence 0.1, existing ByteTrack. Python 3.11.9, Ultralytics 8.4.163, PyTorch 2.14.0+cpu. Wall time includes each adapter call and conversion. Result age is host read-complete to result-ready, not exposure latency.

| Run | Depth results / Hz | Depth wall median / p95 | Depth age median / p95 | Detector results / Hz | Detector call median / p95 | Tracking result age median / p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Depth standalone | 125 / 13.11 | 75.62 / 79.48 ms | 96.67 / 126.52 ms | — | — | — |
| Detector/tracker only | — | — | — | 197 / 23.24 | 24.63 / 26.88 ms | 25.54 / 29.74 ms |
| Depth + detector/tracker | 108 / 12.64 | 79.10 / 84.92 ms | 102.80 / 136.83 ms | 185 / 21.82 | 30.94 / 35.48 ms | 32.94 / 54.19 ms |

Against the same-resolution detector-only run, concurrent depth reduced detector processed rate by **6.12%**, increased median detector call time by **6.32 ms**, and increased median tracking result age by **7.40 ms**. These are single short runs under an uncontrolled live scene, so they are baseline observations rather than a hardware-wide guarantee. Per-frame CSV and metrics JSON are saved beside this report.

## Verification and limits

- The project's `.venv` ran `python -m pytest tests -q`: **141 passed** after the camera-frame extension. No dependency was installed. The official depth checkpoint is cached in `.venv/models`, outside the repository's stage results.
- Human GUI preview has not been run by Codex, per project AGENTS.md. For manual acceptance, from the repository root run:

  ```powershell
  .\.venv\Scripts\python.exe scripts\demo_yolo26n_depth.py --camera-device 1 --mode concurrent --calibration models\minisegway\stage7\stage7_7\config\c920e_camera_params_1280x720.npz --fps 30 --preview --run-name manual_preview_1280
  ```

  Inspect the side-by-side RGB and depth panels, then press `q` or Escape. Both panels use the exact ColorFrame associated with the depth result, with no source-pixel resize before display. The `infer` label shows the fixed depth adapter elapsed time; `age` shows result-ready time minus host read-complete source time. The prior growing display staleness is no longer shown. CSV fields and timing definitions are unchanged. Camera opening on this host took tens of seconds during headless runs.
- Monocular metre values are model estimates; metric accuracy and person representative depth have not been calibrated against a range instrument. This stage does not interpret depth for robot-relative XY or Stage 6. The source time remains host read-complete time. The current C920e K/D file's physical camera identity must be confirmed before using its rays for 3D geometry.

## Visible Master → camera-frame 3D extension

- The concurrent preview now permits a click on an ID box in the **left RGB panel**. Selection is hit-tested on the displayed source-frame pixels and bound only if that ID is still visible in the newest tracking update. The existing tracker and Master modules are unchanged. A new exact-frame pair is required before showing a Master depth after selection. This preview uses only manual Master selection; it does not add Stage 7.6 ReID acquisition to the depth demo.
- Completed depth and tracking results are retained in bounded, 16-result views. Pairing requires identical source ID, sequence ID, host read-complete time, width and height. This is result-side retention only; each worker still receives its own capacity-one latest-frame input. An unmatched depth map cannot borrow a newer bbox. The paired RGB image and heatmap represent the same ColorFrame and show its source sequence ID.
- A **visible, LOCKED** Master supplies the source-pixel bbox. The central 40% of its width and height is the fixed torso ROI; the median of finite positive `DepthFrame.depth_m` values is `master_depth_m`. Fewer than 10 valid pixels, an empty clipped ROI, or an invisible Master makes depth unavailable. No segmentation, temporal smoothing, foot point or single-center-pixel depth is used. The chosen anchor for projection is the source-pixel bbox center, marked in the RGB preview along with the Master bbox and torso ROI.
- `perception.master_depth.back_project_camera_point` uses the inherited C920e **1280×720** K/D. It calls the existing `undistort_image_points` and then solves `K·ray = [u_undistorted, v_undistorted, 1]`. For the demo's **provisional axial-Z interpretation**, `P_C = ray · (master_depth_m / ray_z)`. The helper also implements the distinct Euclidean-range formula `P_C = ray · (range_m / ||ray||)` so the convention stays explicit. The output axes are OpenCV camera frame **+X right, +Y down, +Z forward**, in metres. There is no camera→robot transform or Stage 6 observation.
- **Depth convention remains an open risk.** [Ultralytics' depth task documentation](https://docs.ultralytics.com/tasks/depth/) states metres and “distance from the camera to that surface point” but does not explicitly distinguish optical-axis Z from Euclidean ray range for the released `yolo26n-depth.pt` checkpoint. Its [Hypersim conversion documentation](https://docs.ultralytics.com/datasets/depth/hypersim/) explicitly converts ray distance to planar depth before using that training source, which supports an axial-Z interpretation for that dataset. The released model uses [multiple datasets](https://docs.ultralytics.com/datasets/depth/), and the documentation inspected does not establish a single global checkpoint convention. Consequently the preview labels XYZ **“axial Z assumed”**; it is a diagnostic estimate, not confirmed metric 3D geometry. This convention and the physical camera/calibration identity need independent evidence before robot-frame use.
- With `--mode concurrent --preview` and the calibration argument shown above, click the person ID box in the left panel. The preview shows `Master ID`, `Depth = ... m`, `Camera XYZ = (...) m`, plus depth `Hz`, `infer`, and result `age`. Press `q` or Escape to exit. Successful matched visible-Master samples are recorded in the run's `*_master_camera_points.csv`; unavailable samples have empty numeric fields. This GUI check is for the user to perform under project AGENTS.md.
- A new headless C920 concurrent smoke run requested 1280×720@30, reported and received 1280×720, loaded the exact K/D, and produced **2 exact same-frame depth/tracking pairs** from 60 source frames. First-call model warmup consumed most of the short run, so this smoke run is evidence of runtime pairing and clean shutdown only; it is not a new throughput measurement. Its metrics and per-worker CSV are saved as `master_pairing_headless_smoke_*` beside this report. The GUI click, depth-versus-real-distance check and lateral sign check still require the user's manual acceptance.
