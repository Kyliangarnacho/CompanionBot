# Stage 5 V1.5 — Pi/MCU reference stream and FULL planner acceleration

- Selected path: `cached_kkt_250hz`; final status: **COMPLETED**.
- Stored V1.4 FULL planning time: 108.92 ms; final stream warm plan: 3.94 ms (27.63× faster).
- Selected warm benchmark: 3.75 ms; one-time matrix build + LU factorization: 4.16 ms.
- Horizon: bucket 1.700 s; actual projection 1.700 s; post-interpolation 500 Hz residual 0.01776 (V1.4 was 0.01750).
- Stream: underruns 0, stale 0, sequence gaps 0; saturation 0; false SLOPE 0; fall false.
- 4 ms synthetic-delay check: 0.004 s; underruns 0, stale 0, sequence gaps 0.

Closed-loop replay (same V1.4 episode):
| Path | Warm planning | 500 Hz residual | v error RMS / peak | theta tracking RMS | sat / fall |
|---|---:|---:|---:|---:|---:|
| LSQR 500 Hz | 104.03 ms | 0.01750 | 0.05543 / 0.16734 m/s | 0.06058 rad | 0 / false |
| cached KKT 500 Hz | 6.75 ms | 0.01750 | 0.05543 / 0.16734 m/s | 0.06058 rad | 0 / false |
| cached KKT 250 Hz | 3.75 ms | 0.01776 | 0.05540 / 0.16734 m/s | 0.06060 rad | 0 / false |
- Replay quiet snaps / fade completions: LSQR 500 Hz 1 / 1 (overshoot 0.01254 m/s); cached KKT 500 Hz 1 / 1 (overshoot 0.01254 m/s); cached KKT 250 Hz 1 / 1 (overshoot 0.01194 m/s).

- Benchmark JSON: `models/minisegway/stage5/results/stage5_v1_5_planner_benchmark.json`.
- Replay comparison JSON: `models/minisegway/stage5/results/stage5_v1_5_closed_loop_comparison.json`.
- Final JSON: `models/minisegway/stage5/results/stage5_v1_5_pi_mcu_stream_results.json`.
- Summary: `models/minisegway/stage5/results/STAGE5_V1_5_SUMMARY.md`.
- Raw histories: `models/minisegway/stage5/results/stage5_v1_5_stream_final_history.csv`, `models/minisegway/stage5/results/stage5_v1_5_stream_consumption.csv`, `models/minisegway/stage5/results/stage5_v1_5_stream_blocks.csv`.
- Plots: `models/minisegway/stage5/results/plots/stage5_v1_5_velocity.png`, `models/minisegway/stage5/results/plots/stage5_v1_5_dynamic_ff.png`, `models/minisegway/stage5/results/plots/stage5_v1_5_block_stream.png`, `models/minisegway/stage5/results/plots/stage5_v1_5_planning_timeline.png`.
