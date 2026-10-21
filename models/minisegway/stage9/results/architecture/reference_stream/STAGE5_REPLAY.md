# Stage 5 V1.5 — Pi/MCU reference stream and FULL planner acceleration

- Selected path: `cached_kkt_250hz`; final status: **COMPLETED**.
- Stored V1.4 FULL planning time: 108.92 ms; final stream warm plan: 3.11 ms (35.07× faster).
- Selected warm benchmark: 2.89 ms; one-time matrix build + LU factorization: 4.48 ms.
- Horizon: bucket 1.700 s; actual projection 1.700 s; post-interpolation 500 Hz residual 0.01776 (V1.4 was 0.01750).
- Stream: underruns 0, stale 0, sequence gaps 0; saturation 0; false SLOPE 0; fall false.
- 4 ms synthetic-delay check: 0.004 s; underruns 0, stale 0, sequence gaps 0.

Closed-loop replay (same V1.4 episode):
| Path | Warm planning | 500 Hz residual | v error RMS / peak | theta tracking RMS | sat / fall |
|---|---:|---:|---:|---:|---:|
| LSQR 500 Hz | 90.66 ms | 0.01750 | 0.05543 / 0.16734 m/s | 0.06059 rad | 0 / false |
| cached KKT 500 Hz | 5.29 ms | 0.01750 | 0.05545 / 0.16734 m/s | 0.06059 rad | 0 / false |
| cached KKT 250 Hz | 2.89 ms | 0.01776 | 0.05545 / 0.16734 m/s | 0.06057 rad | 0 / false |
- Replay quiet snaps / fade completions: LSQR 500 Hz 1 / 1 (overshoot 0.01254 m/s); cached KKT 500 Hz 1 / 1 (overshoot 0.01254 m/s); cached KKT 250 Hz 1 / 1 (overshoot 0.01194 m/s).

- Benchmark JSON: `models/minisegway/stage9/results/architecture/reference_stream/stage5_v1_5_planner_benchmark.json`.
- Replay comparison JSON: `models/minisegway/stage9/results/architecture/reference_stream/stage5_v1_5_closed_loop_comparison.json`.
- Final JSON: `models/minisegway/stage9/results/architecture/reference_stream/stage5_v1_5_pi_mcu_stream_results.json`.
- Summary: `models/minisegway/stage9/results/architecture/reference_stream/STAGE5_REPLAY.md`.
- Raw histories: `models/minisegway/stage9/results/architecture/reference_stream/stage5_v1_5_stream_final_history.csv`, `models/minisegway/stage9/results/architecture/reference_stream/stage5_v1_5_stream_consumption.csv`, `models/minisegway/stage9/results/architecture/reference_stream/stage5_v1_5_stream_blocks.csv`.
- Plots: `models/minisegway/stage9/results/architecture/reference_stream/plots/stage5_v1_5_velocity.png`, `models/minisegway/stage9/results/architecture/reference_stream/plots/stage5_v1_5_dynamic_ff.png`, `models/minisegway/stage9/results/architecture/reference_stream/plots/stage5_v1_5_block_stream.png`, `models/minisegway/stage9/results/architecture/reference_stream/plots/stage5_v1_5_planning_timeline.png`.
