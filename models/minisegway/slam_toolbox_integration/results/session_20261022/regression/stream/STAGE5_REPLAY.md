# Stage 5 V1.5 — Pi/MCU reference stream and FULL planner acceleration

- Selected path: `cached_kkt_250hz`; final status: **COMPLETED**.
- Stored V1.4 FULL planning time: 108.92 ms; final stream warm plan: 2.90 ms (37.51× faster).
- Selected warm benchmark: 3.04 ms; one-time matrix build + LU factorization: 4.45 ms.
- Horizon: bucket 1.700 s; actual projection 1.700 s; post-interpolation 500 Hz residual 0.01776 (V1.4 was 0.01750).
- Stream: underruns 0, stale 0, sequence gaps 0; saturation 0; false SLOPE 0; fall false.
- 4 ms synthetic-delay check: 0.004 s; underruns 0, stale 0, sequence gaps 0.

Closed-loop replay (same V1.4 episode):
| Path | Warm planning | 500 Hz residual | v error RMS / peak | theta tracking RMS | sat / fall |
|---|---:|---:|---:|---:|---:|
| LSQR 500 Hz | 94.33 ms | 0.01750 | 0.05545 / 0.16734 m/s | 0.06059 rad | 0 / false |
| cached KKT 500 Hz | 5.39 ms | 0.01750 | 0.05546 / 0.16734 m/s | 0.06058 rad | 0 / false |
| cached KKT 250 Hz | 3.04 ms | 0.01776 | 0.05547 / 0.16734 m/s | 0.06057 rad | 0 / false |
- Replay quiet snaps / fade completions: LSQR 500 Hz 1 / 1 (overshoot 0.01254 m/s); cached KKT 500 Hz 1 / 1 (overshoot 0.01254 m/s); cached KKT 250 Hz 1 / 1 (overshoot 0.01239 m/s).

- Benchmark JSON: `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/stage5_v1_5_planner_benchmark.json`.
- Replay comparison JSON: `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/stage5_v1_5_closed_loop_comparison.json`.
- Final JSON: `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/stage5_v1_5_pi_mcu_stream_results.json`.
- Summary: `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/STAGE5_REPLAY.md`.
- Raw histories: `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/stage5_v1_5_stream_final_history.csv`, `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/stage5_v1_5_stream_consumption.csv`, `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/stage5_v1_5_stream_blocks.csv`.
- Plots: `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/plots/stage5_v1_5_velocity.png`, `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/plots/stage5_v1_5_dynamic_ff.png`, `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/plots/stage5_v1_5_block_stream.png`, `models/minisegway/slam_toolbox_integration/results/session_20261022/regression/stream/plots/stage5_v1_5_planning_timeline.png`.
