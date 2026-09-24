# Stage 6 历史参考

本目录保存已被当前 Stage 6.5 运行链取代的 follower 版本、门限试验、诊断数据和旧配置。归档保留失败与中间结果，供追溯调试过程；这些材料不属于当前 quick-start baseline。

## 目录索引

- `stage6_1/`：初始纵向 follower、动态制动 all-FULL/mixed 对照、早期 FOLLOW gate 失败、旧配置与对应报告。历史 runner 已移入 `stage6_1/run_stage6_1_longitudinal_follow.py`；需要重放时可运行 `.\.venv\Scripts\python.exe models\minisegway\stage6\reference\stage6_1\run_stage6_1_longitudinal_follow.py --help`，默认结果写回本目录。当前 Stage 6.2–6.5 共用的合成传感器和 CSV/JSON 工具位于 `scripts/stage6_synthetic_support.py`，不包含 follower 控制策略。
- `stage6_3_deadband_trial/`：最初 0.03 m/s 停车 deadband 试跑；当前 Stage 6.3/6.4 配置使用 0.05 m/s。
- `viewer_axis_hunting_diagnosis/`：Vy-only 与 Vx-only 稳态跟随 hunting 诊断数据，未改动或重放控制器。
- `viewer_cruise_median_diagnosis/`：单帧、5/10/20 帧、adaptive/candidate 等 Governor 锁存候选比较，以及被拒绝方案、早期对齐试验和复现脚本。最终双窗口方案的历史对照报告在该目录 `REPORT.md`。

## 当前生效位置

- 当前 2D follower 参数：`../config/stage6_4_2d_follow_config.json`
- 当前 radial KF/Pi 对齐参数：`../config/stage6_5_radial_kf_config.json`
- 当前 KF 与 dual-window Governor：`../../../../control/radial_velocity_kf.py`、`../../../../control/follow_governor.py`
- 当前交互 viewer：`../../../../scripts/view_stage6_5_manual_2d_follow.py`
- 本阶段动态变速验证：`../results/governor_dynamic_rate_validation/`

归档 JSON 中记录的原始输出路径和日志字段保留了实验写入时的历史位置；移动归档文件时没有改写实验数据。
