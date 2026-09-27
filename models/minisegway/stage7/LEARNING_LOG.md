# Stage 7 learning and debugging log

This log summarizes the task history and preserved local artifacts. It distinguishes
implemented behavior, automated evidence and human observations; it does not invent
acceptance results for scenes that were not recorded.

## Camera foundation and Stage 7.1 — detector boundary

- Borrowed MonoTeach's camera ownership and K/D loader/undistortion logic. Adapted
  the source/backend selection to support OpenCV on Windows/Linux without hardcoding
  C920, index 1 or DirectShow. Workspace homography, fingertips and MediaPipe were excluded.
- `ColorFrame` became the shared BGR uint8 source boundary. The time sampled after
  `capture.read()` was correctly named host receive/read-complete, not exposure.
- The user's 1280×720@30 MSMF bring-up produced 120 continuous sequence IDs and
  29.87 read-complete Hz, but timestamp strict monotonicity was false and minimum
  interval was zero. The camera now uses `perf_counter_ns`, advancing ties by 1 ns.
  That fixes ordering only; it does not measure sensor exposure or remove buffering.
- The first Stage 7.1 tests ran under system Python 3.8.5, an environment mismatch.
  The user installed pytest and reran them successfully in the project `.venv`.
  All later checks use the explicit project interpreter for pip, pytest and scripts.
- YOLO26n remains behind `PersonDetector`; detections are restored to original
  source pixels, separate from neural-network imgsz. The original image smoke and
  model/timing evidence remain in [Stage 7.1](stage7_1/results/STAGE7_1_DEMO_REPORT.md).

## Stage 7.2 — continuous observation stream

- One capture owner feeds a capacity-one latest-frame slot. Slow inference skips
  source sequences rather than accumulating an input FIFO. A source sequence gap
  is logged instead of fabricated as a detector/tracker update.
- The 640×480 C920 run measured 27.64 processed Hz against 30.00 source Hz; detector
  wall median/p95 was 24.95/30.56 ms. This is a Windows host result, not Pi performance.
- [Stage 7.2 evidence](stage7_2/results/STAGE7_2_STREAM_REPORT.md).

## Stage 7.3 — tracking and the retracted camera diagnosis

- Borrowed official Ultralytics ByteTrack with existing thresholds/track_buffer.
  IDs are tracklets, not person identity. Its dt=1 transition and removal buffer
  count tracker API calls; 31 calls take 3.1 s at 10 Hz and about 1.03 s at 30 Hz.
- A user-reported source-initialization timeout prompted debugging. The user later
  confirmed the original baseline opened C920 with the default tracking command
  and attributed the report to an operating mistake. The requested rollback scope
  was only the later timeout debugging, preserving the uncommitted tracking baseline.
  The insurance patch is retained under `stage7_3/reference/`; it is historical
  evidence, not a source of runtime settings.
- Final closeout reuses the existing Stage 7.2 source thread. It adds no preflight
  camera open or alternate backend/thread workaround. Native opening can still take
  tens of seconds on this host.
- [Tracking evidence](stage7_3/results/STAGE7_3_TRACKING_REPORT.md) and
  [cadence/gap audit](stage7_3/results/STAGE7_3_SYNTHETIC_TRACKER_BENCHMARK.md).

## Stage 7.4–7.5 — explicit Master and appearance evidence

- Manual selection was added above the tracker. Master lifecycle distinguishes
  visible LOCKED, TEMPORARILY_LOST and removed/LOST; another visible ID is not
  automatically the same person.
- OSNet x0.25 MSMT17 provides 512D normalized appearance embeddings from exact-frame
  original RGB crops. Only its pinned implementation/checkpoint is loaded under
  `.venv`; no full TorchReID training stack or copied upstream source enters the repo.
- Stage 7.5 initially stored a single clicked-frame reference and displayed candidate
  evidence without rebinding. This historical policy was superseded by Stage 7.6.
- CPU crop embedding median 17.61 ms/p95 19.94 ms in one benchmark; other runs had
  p95 up to 40.87 ms. Synthetic crops test cost/contracts, not identity accuracy.
- [Master baseline](stage7_4/results/STAGE7_4_MASTER_LOCK_BASELINE.md) and
  [ReID evidence report](stage7_5/results/STAGE7_5_MASTER_REID_EVIDENCE_REPORT.md).

## Stage 7.6 — reference stability, reacquire and retry

- Clicking enters PENDING_LOCK. Three aligned embeddings span at least 0.30 s;
  their mean is normalized into one fixed reference. Disappearance cancels pending.
- LOST reacquire requires ≥3 tracker updates and cosine ≥0.68. A real weakness of
  once-per-ID scoring was reported: a first side/back/poor crop could block an ID
  forever. The small dual-threshold patch permits 0.20 s retry only for scores in
  [0.50,0.68); <0.50 stops retries until disappearance. No gallery or quality model.
- Both thresholds remain provisional without negative-person calibration. A score
  is not a probability, and successful code execution does not establish identity.
- An exit summary still referenced deleted `REID_REACQUIRE_THRESHOLD` after the
  dual-threshold change. That stale variable was replaced with accept/retry values
  and a summary regression test. Runtime behavior was unchanged by that repair.
- [Stage 7.6 report](stage7_6/results/STAGE7_6_REID_AUTO_REACQUIRE_REPORT.md).

## Stage 7.7 — depth, alignment and geometry

- The initial camera work found no versioned MonoTeach intrinsic. Heritage inspection
  later found a workspace JSON reference to the independent C920e calibration
  project. Its actual NPZ is 1280×720; it was copied byte-for-byte with provenance.
  The workspace homography itself was never used as K/D.
- Source capture returned to 1280×720. Detector imgsz=640 and depth imgsz=768 stay
  internal. Depth outputs float32 metres/NaN and checks original source dimensions.
- Depth runs as an independent latest-frame worker. The fair 1280×720 ablation
  measured detector throughput -6.12% with depth, and median detector latency +6.32 ms.
- Preview naming was corrected: `infer` is adapter elapsed time, `age` is result-ready
  minus host read-complete. Growing screen staleness is not labeled result age.
- Representative depth is central 40% torso ROI median, not one pixel/foot point.
  Bbox and depth are paired by source ID/seq/time/resolution with bounded retention.
  Sparse anchor undistortion and K inverse yield camera XYZ.
- Axial Z versus ray range remains a real semantic uncertainty. Hypersim's upstream
  conversion supports planar Z, but the released mixed-dataset checkpoint has no
  definitive convention statement in the inspected docs. The implementation labels
  axial Z as provisional and supports separate formulas; no physical accuracy claim.
- The previous operator-generated camera-XYZ log contains 788 visible-Master depth
  samples. It confirms recorded data, not a metrology or identity acceptance result.

## Final closeout

- Audit found that Stage 7.7's manual-only demo had not actually included Stage 7.6
  ReID. Final full mode now calls the existing pending/retry/reacquire APIs for every
  detector/tracker update in one worker. Depth remains independent.
- Final display is RGB only. A thin configurable rigid transform and pitch provider
  produce leveled robot-relative observations. Default mounting and pitch are
  simulated; source time remains host read-complete; Stage 6 is disconnected.
- Seven result/config directories moved under `stage7/`; all 56 original files and
  the rollback patch remain. Defaults and textual references were relocated. Current
  tests are explicitly rooted at `tests/`, leaving archived Stage 5 tests as history.
- Final validation: 150 tests passed; real 300-frame C920 smoke generated 124 exact
  pairs and 104 robot observations, and exercised one new-ID ReID reacquire at 0.8963.
  These are execution results, not identity ground truth or calibrated robot geometry.
- A final 180-frame run after the explicit pitch-time mapping boundary produced
  56 pairs and 54 observations with source time and pitch alignment time separately
  recorded. Depth 12.04 Hz was slower than the first run's 14.03 Hz; both runs remain
  available, with no tuning or selective removal of the slower result.
- See [final report](STAGE7_FINAL_REPORT.md) for provisional parameters, evidence,
  architecture, the final GUI command and the remaining hardware work.
