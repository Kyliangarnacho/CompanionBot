# Stage 4A — Slope / Grade Robustness

## Repository audit

- 权威 Stage 3 manifest 中的 A/B、K、acceleration-compensated estimator、FF lifecycle、yaw PD、1 ms / 2 ms 时序和 ±0.63 N·m 单轮峰值均保持冻结。
- production baseline 的 Q observer 仅用于 diagnostic，actuator augmentation 为 OFF；Stage 4A 只在显式 Q ON 实验臂接入 actuator。
- Stage 4A 复用 Stage 3 commanded-motion runner、reference lifecycle、estimator、allocator 和现有 2 Hz matched-disturbance observer。坡度/法向/world pose/along-track GT 只进入 logger、evaluator 和仿真 boundary guard。

## 结果

| Angle | Q | v RMSE steady / transition-inclusive (m/s) | Steady bias (m/s) | Pitch RMS / peak (deg) | Torque RMS (N m/wheel) | Saturation | Q mean / peak (N m) | Status |
|---:|:---:|---:|---:|---:|---:|---:|---:|:---:|
| -15° | OFF | 0.1025 / 0.2077 | +0.0555 | 26.31 / 29.82 | 0.031 | 0.00% | +0.000 / 0.000 | FAIL |
| -15° | ON | 0.1045 / 0.2104 | +0.0566 | 26.32 / 29.87 | 0.031 | 0.00% | -0.051 / 0.059 | FAIL |
| -8° | OFF | 0.0469 / 0.1083 | +0.0255 | 14.82 / 16.59 | 0.013 | 0.00% | +0.000 / 0.000 | PASS |
| -8° | ON | 0.0482 / 0.1096 | +0.0263 | 14.82 / 16.59 | 0.013 | 0.00% | -0.028 / 0.032 | PASS |
| +8° | OFF | 0.0148 / 0.1118 | -0.0086 | 10.70 / 12.31 | 0.050 | 0.00% | +0.000 / 0.000 | PASS |
| +8° | ON | 0.0148 / 0.1134 | -0.0086 | 10.70 / 12.32 | 0.050 | 0.00% | +0.028 / 0.031 | PASS |
| +15° | OFF | 0.0131 / 0.1898 | -0.0100 | 21.79 / 24.37 | 0.071 | 0.00% | +0.000 / 0.000 | FAIL |
| +15° | ON | 0.0128 / 0.1919 | -0.0098 | 21.78 / 24.42 | 0.071 | 0.00% | +0.050 / 0.055 | FAIL |

冻结 Q-OFF baseline 通过：-8°, +8°。
冻结 Q-OFF baseline 未通过 / operating envelope：-15°, +15°。

Q ON 没有在任一角度同时把恒坡速度 RMSE 和稳态 bias 降低至少 10%，因此没有显示实质增量价值，也没有调 Q 参数的依据。

Q ON 的 transition / steady-grade augmentation RMS（N·m）分别为：-15° 0.045/0.052, -8° 0.027/0.028, +8° 0.028/0.028, +15° 0.050/0.051。两者同量级，说明 Q 主要在估计并补偿持续 grade disturbance，不是只在 transition 瞬间“瞎忙”；但该持续输出没有转化为 tracking 改善。

8 个实验臂中出现 torque saturation 的数量为 0；所有实验均无 fall、无 chassis-terrain contact、无 boundary termination。正常 acceptance arm 若触发 fall 或 boundary guard 均不得通过。

## 结论

- ±8°：冻结 Stage 3 baseline 能处理恒坡；入坡 transition 的误差显著大于恒坡稳态误差，但之后恢复。
- +15°：恒坡速度仍可跟踪，但 world-pitch 超过复用的 Stage 3 20°门槛；归入压力测试 envelope。
- -15°：恒坡速度 RMSE / bias 和 pitch 均越过门槛；归入压力测试 envelope。
- Q：四个角度均持续输出，matched residual fraction 也较高，但 OFF→ON 的 RMSE/bias 变化约为 -2.5% 到 +3.1%，没有工程上有意义的改善。
- saturation / authority：无 wheel saturation、无 projection clipping、无 Q authority limiting；失败不是 actuator authority 耗尽造成的。
- 没有证据要求修改 Stage 3 controller。尤其不应因 ±15° 压力测试或 entry transient 自动重算 LQR、修改 estimator/FF，或调 Q cutoff/authority。

## 解释纪律

- ±15° 的失败被保留为 operating-envelope 证据，不是必须修复的产品 requirement。
- Q estimate 大但 tracking 未改善，不构成调 cutoff 或 authority 的理由；后续若继续研究，应先检查 equilibrium/reference 与 estimator semantics。
- 本阶段没有修改 controller，也没有把 Q 接入 production；保存的 OFF/ON ablation 是后续决策输入。
