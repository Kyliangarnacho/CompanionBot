# Native stereo RGB depth contract

This is the explicitly authorized additional image ranging source. It does not
repair, rescale or certify YOLO monocular depth. Original detector, ByteTrack,
Master/ReID and independent YOLODepth still consume the unchanged left RGB.
The stereo processor accepts no renderer depth, actor pose, skin mask, hit ID,
tracking ID, reference command or actuator state. `hardware_execution_ready`
remains false.

## Ownership and processing

| API | Input | Processing | Output and consumer |
| --- | --- | --- | --- |
| `StereoDepthProcessor(StereoConfig)` | Explicit image dimensions, left K, 150 mm baseline, rectified right-from-left transform, calibration ID and frozen SGBM parameters | Validate finite/bounded configuration, enforce parallel axes and `t=[-baseline,0,0]`; native OpenCV `stereoRectify` audits the already rectified geometry; construct two standard `StereoSGBM` instances and reusable grayscale/matching/texture scratch | One processor owns one source epoch and clock domain; no guessed rectification or online scale estimation |
| `process(left_bgr,right_bgr,left_stamp,right_stamp)` | Actual BGR8 image pair, exact common source epoch/sequence/time/clock/calibration ID | Validate exact pairing and strictly increasing sequence **and** time; BGR→gray; official left SGBM and independent reverse SGBM; positive disparity, search-border, bilateral horizontal texture and reverse-consistency checks; `Z=fx*baseline/disparity` | Independent float32 axial-Z/disparity/LR-error arrays, uint8 masks, exact source stamp and measured diagnostics; metric pairing/geometry adapter consumes this result only after external dataset acceptance |
| `snapshot()` | No observation or authority input | Read counters under the same mutex | Accepted/rejected input counts and source watermark for diagnostics |
| `calibration_audit()` | None after construction | Return immutable construction result | R1/R2/P1/P2/Q and valid image ROIs for reproducible calibration checks |

Native matching and `snapshot` release the Python GIL **before** waiting for the
processor mutex. Calls on one processor serialize; an out-of-order concurrent
call is rejected rather than replayed. Changing epoch/clock requires an explicit
new processor; it cannot silently reset the watermark. Invalid input does not
advance the accepted watermark. There is no previous-frame or monocular fallback.

At the Python boundary, positive row padding is supported; wrong dtype, channel
count, negative/pixel-strided views or oversized dimensions are rejected. The
binding makes bounded native image snapshots before releasing the GIL. Callers
must keep source buffers stable during this entry copy, including against native
NumPy writers. Production transfers read-only owned image pairs. Matching never
borrows a writable Python buffer. Each returned NumPy array owns its independent,
initially read-only storage; later calls cannot overwrite old results.

Invalid metric pixels are NaN. Disparity can remain diagnostic on pixels rejected
for texture, occlusion or inconsistency; it must not be interpreted as valid Z.
Masks use 0/255, while `rejection_mask` is a bit field. Existing torso ROI and its
minimum ten valid pixels remain the observation extraction contract. Accuracy
scoring must use the original visible-skin ROI target, never a GT subset selected
by the stereo valid mask.

## Borrow / Adapt / Reject

- **Borrow:** exact installed native OpenCV 4.6.0 `StereoSGBM`, including its own
  subpixel interpolation, median filter and speckle filtering, without modifying
  upstream. Native C++ test compares every positive left disparity to a separately
  constructed same-version official matcher. Python cv2 5.x is not a bit-exact
  oracle and no `cv::Mat` ABI crosses into its extension.
- **Borrow:** official `stereoRectify`/`getValidDisparityROI`; explicit rectified
  pinholes require no per-frame remap. `stereoRectify` internally uses float32
  image corners, so its audit permits `4*float32_epsilon*max(1,fx,fy,width,height)`
  projection representation rounding. Native processing uses the original K,
  without changing output scale. The first synthetic audit failed with an
  unnecessarily exact 1e-7-pixel comparison; the original failure and rounding
  probe are retained under `stereo/native_review/`.
- **Adapt:** exact source/calibration/watermark gates, explicit reverse matching
  with interpolated `|d_left+d_right|<=1 px`, and original predeclared texture
  observability floor. These gates reject known failures; they are not a
  statistical accuracy certificate.
- **Reject:** a custom stereo algorithm, hole filling, temporal depth reuse,
  actor-GT calibration/scale fitting, hidden raw-YOLO fallback and any relaxed
  follow safety deadline. A swapped/ambiguous/repetitive-texture image pair can
  produce false matches despite consistency checks; identical stamps cannot
  prove pixel provenance. The image acquisition owner must preserve camera
  identity, and actual held-out error remains a separate mandatory gate.

Primary sources inspected: [OpenCV 4.6 SGBM implementation](https://github.com/opencv/opencv/blob/4.6.0/modules/calib3d/src/stereosgbm.cpp),
[calibration implementation](https://github.com/opencv/opencv/blob/4.6.0/modules/calib3d/src/calibration.cpp),
and [official parameter documentation](https://docs.opencv.org/4.6.0/d2/d85/classcv_1_1StereoSGBM.html).
`preFilterCap=0` has effective clipping 15 in this source version; disparity
output is scaled by 16. No third-party source is vendored.

## Timing and verification scope

Diagnostics expose native lock-wait, grayscale, both-matchers, quality, total
native wall time and calling-thread CPU. Binding diagnostics additionally expose
input snapshot bytes/time, output-copy time and total binding wall time, including
GIL reacquisition. Calling-thread CPU excludes OpenCV helper thread CPU; separate
process/resource measurements are needed for total CPU. Native scratch is reused;
returned results and boundary snapshots are bounded per-call allocations.

`native/test/stereo_depth_test.cpp` and `tests/test_native_stereo.py` establish
geometry, upstream matcher parity, invalid pairing, replay rejection, output
ownership, flat-image failure and concurrent-call behavior using software RGB
fixtures. Passing these tests does **not** establish rendered scene accuracy,
end-to-end age, visual motion closure, Pi performance or hardware readiness.
The fixed 16/32 pose protocol, original RMSE 0.03 m/P95 0.09 m and 0.2 s full
acquisition-to-authority deadline remain separate unchanged requirements.

Verified review evidence: `results/session_20261023/stereo/native_review/summary.json`
records **45 pytest, one native CTest and standalone ASan/UBSan PASS**. Leak
Sanitizer was disabled and installed upstream OpenCV libraries were not sanitizer
instrumented. The native test checks the borrowed matcher numerically; the Python
tests additionally cover uint64 sequence preservation, source/replay rejection,
float32 metric underflow, padded-row ownership and concurrent duplicate/snapshot
calls. These are software contract checks, not measurements of camera accuracy.
