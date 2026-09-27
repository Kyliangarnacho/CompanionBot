# Stage 7.3 Synthetic ByteTrack Timing and Gap Audit

- Upstream: Ultralytics 8.4.163 `BYTETracker`.
- Installed upstream defaults: `{"fuse_score": true, "match_thresh": 0.8, "new_track_thresh": 0.25, "track_buffer": 30, "track_high_thresh": 0.25, "track_low_thresh": 0.1, "tracker_type": "bytetrack"}`.
- Synthetic boxes only: this establishes API, update-count, ID continuity, and lifecycle semantics; it is not a human-scene tracking result.
- Simulated timestamps do not drive ByteTrack's Kalman filter; each real API call increments its internal frame counter exactly once.

## Constant detections at simulated source cadence

| Cadence | Source frames | Tracker updates | IDs stable | Median update | p95 update |
|---:|---:|---:|:---:|---:|---:|
| 10 Hz | 120 | 120 | True | 0.188 ms | 0.238 ms |
| 15 Hz | 120 | 120 | True | 0.186 ms | 0.202 ms |
| 20 Hz | 120 | 120 | True | 0.185 ms | 0.214 ms |
| 30 Hz | 120 | 120 | True | 0.184 ms | 0.204 ms |

## Latest-frame style sequence gaps

- 120 source-frame indices at 30 Hz; 40 calls to the tracker.
- Between-update gap sum/max/mean: 78 / 2 / 2.00 frames.
- Trailing unprocessed frames: 2; total unprocessed source frames: 80.
- Stable single temporary tracklet ID: True; no skipped-frame updates were synthesized.

## `track_buffer` wall-clock interpretation

| Simulated cadence | Buffer | Updates until removed after last detection | Simulated seconds |
|---:|---:|---:|---:|
| 10 Hz | 30 | 31 | 3.100 s |
| 15 Hz | 30 | 31 | 2.067 s |
| 20 Hz | 30 | 31 | 1.550 s |
| 30 Hz | 30 | 31 | 1.033 s |

Synthetic schedules advance source timestamps without sleeping. The seconds column is the simulated elapsed time represented by the same number of tracker updates; it demonstrates that the upstream buffer is frame-count based.
- Machine-readable results: `D:\project\CompanionBot\models\minisegway\stage7\stage7_3\results\stage7_3_synthetic_tracker_benchmark.json`
