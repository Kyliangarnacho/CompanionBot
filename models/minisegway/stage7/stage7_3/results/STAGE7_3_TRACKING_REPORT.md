# Stage 7.3 Person Tracking Baseline

- Source: `camera:1`; frames 1281 at 30.00 FPS
- Tracker: Ultralytics 8.4.163 ByteTrack; config SHA-256 `395701d947a749179dee3e327b1181b730c4ca98e7cac5a2ab05280aae573b8b`
- Processed / steady-state frames: 1092 / 1089; processed rate 27.69206585661233 FPS
- Detection boxes: 987; tracklets first observed: 14; active at end: 1; unique lost tracklets: 14; lost events: 16; removed tracklets/events: 13/23
- Unambiguous single-detection/single-track observations: 875; track ID changes across those observations: 5
- Sequence gaps between tracker updates: total 187, max 95, mean 0.171; trailing skipped frames 2; slot overwrites 188
- Tracker update wall mean / median / p95: 0.482 / 0.502 / 0.601 ms
- Read-complete-to-tracking-result age mean / median / p95: 39.528 / 35.712 / 60.586 ms
- Timing input to ByteTrack: none. Its Kalman transition uses fixed dt=1 and its track_buffer is measured in actual tracker update calls, not source sequence or seconds.
- No FOV/lost-person decision or camera-motion compensation is present. Leaving and re-entering the view may create a new tracklet; rapid yaw, pitch, or gimbal motion may degrade association.
- Track IDs are temporary tracklets. This run does not perform person identity, Master Lock, or ReID.
- Human scenes A-F (stationary, walking, fast lateral motion, occlusion, leaving/re-entering FOV, optional multiple people) require operator preview and are not claimed as passed by this run.
- Frame log: `D:\project\CompanionBot\models\minisegway\stage7\stage7_3\results\person_tracking_frames.csv`
- Tracklet lifetime summary: `D:\project\CompanionBot\models\minisegway\stage7\stage7_3\results\person_track_lifetimes.csv`
- Run config / metrics JSON: `D:\project\CompanionBot\models\minisegway\stage7\stage7_3\results\stage7_3_run_config.json` / `D:\project\CompanionBot\models\minisegway\stage7\stage7_3\results\stage7_3_tracking_metrics.json`
