# Stage 6 双窗口 Governor 动态变速验证

## 范围与有效性

- 只改动 Stage 6 results 目录下的诊断运行脚本与结果；未改生产控制逻辑、配置参数、KF、FULL lifecycle、safety 或 Stage 3/4/5 baseline。
- Master 仅沿直线运动；测量噪声、processing delay、jitter 均为 0。yaw 没有外部扰动，最大观测 |beta| 为 0.0102 rad（0.58°）。
- 为保留零延迟并满足 capture-time 插值，测试脚本把 20 Hz observation 采样相位平移 -1 ms：捕获时间落在 MCU 50 ms RobotState 样本之间，由已到达的相邻状态插值。所有纳入报告的样本均对齐：稳速基准 900/900、300/300；加速阶段短跑 669/669、延长观察 869/869；减速三组各 295/295（其中首帧 EXACT，其余 INTERPOLATED）。原始零延迟相位会在 11.601 s 起出现 STATE_HISTORY_MISS，因此没有用该批失配结果。
- 三个阶跃时刻按无阶跃稳速基准的首次 CRUISE 时刻分别提前 0.2/0.5/0.9 s。速度改变本身会移动实际 transition；下表另列真实的 step-to-transition 时间，不能把计划相位当成实际相位。

## 每次 CRUISE transition

速度单位均为 m/s；目标变速后速度分别为 0.40（加速）与 0.20（减速）。

| 场景 | 相对稳速基准提前 | 变速到 transition | 单帧 KF Vm | 5 帧 median | 20 帧 median | CRUISE 锁存 |
|---|---:|---:|---:|---:|---:|---:|
| 加速 0.20→0.40 | 0.20 s | 6.20 s | 0.4013 | 0.4013 | 0.4009 | 0.4009 |
| 加速 0.20→0.40 | 0.50 s | 6.15 s | 0.4015 | 0.4011 | 0.4005 | 0.4005 |
| 加速 0.20→0.40 | 0.90 s | 6.10 s | 0.4002 | 0.3998 | 0.4002 | 0.4002 |
| 减速 0.40→0.20 | 0.20 s | 0.10 s | 0.3716 | 0.4007 | 0.4012 | 0.4012 |
| 减速 0.40→0.20 | 0.50 s | 0.15 s | 0.3493 | 0.3898 | 0.4013 | 0.4013 |
| 减速 0.40→0.20 | 0.90 s | 0.25 s | 0.3031 | 0.3502 | 0.4012 | 0.3502 |

## 结论

- **加速：未见 long-window 导致低锁存。** 0.20→0.40 后，Governor 因距离重新拉大而先在约 1.75–1.85 s 触发一次 `catch_up_retriggered`，命令约 0.506–0.508；直到变速后 6.10–6.20 s 才进入 CRUISE。此时两种 median 均已到约 0.40，锁存误差不超过 0.0009。三个延长运行在 CRUISE 后又观察了 11.0–11.8 s，没有后续 Governor 事件或周期性 hunting。这里实际 transition 都被加速阶跃推迟，故不能把它解读成“阶跃发生在实际 transition 前 0.2–0.9 s”。
- **减速：20 帧在即时 transition 明显滞后。** 变速后 0.10–0.25 s 就 transition，20 帧仍约 0.401，前两组因此锁在约 0.401；第三组 5 帧 median 降至 0.350，触发快速通道后锁在 0.350，但仍比新速度高 0.150。也就是说，5 帧通道只在最晚一组部分覆盖 long-window，不能在 transition 当下把锁存拉到 0.20。
- 减速后 Governor 都通过 `slowdown_wait` 单调降低锁存，没有 `catch_up_retriggered`，没有反复加速/减速循环。按“锁存进入新速度 ±0.05 m/s”计，三组分别要 **0.85 s、0.90 s、3.65 s**。最后一组在 step 后 0.85 s 锁到 0.252，直到 3.65 s 才到 0.201；其间观测距离从 1.021 m 降至 0.759 m，表现为明显靠近超程风险，虽未发生周期性 hunting。

| 场景 | step 后 Governor event 数 | FULL_DYNAMIC plan 数 | 后续 catch retrigger | 结果 |
|---|---:|---:|---:|---|
| 加速三组（每组） | 2（1 retrigger + 1 cruise） | 2 | transition 后 0 | 11.0–11.8 s 稳定观察期无新事件 |
| 减速提前 0.20 s | 3（1 cruise + 2 slowdown_wait） | 2 | 0 | 锁存约 0.85 s 到新速度 ±0.05 |
| 减速提前 0.50 s | 3（1 cruise + 2 slowdown_wait） | 2 | 0 | 锁存约 0.90 s 到新速度 ±0.05 |
| 减速提前 0.90 s | 3（1 cruise + 2 slowdown_wait） | 3 | 0 | 锁存约 3.65 s 到新速度 ±0.05；期间距离降至 0.759 m |

所有运行均无 fall、轮饱和、sum-command saturation、allocator guard clipping 或 reference underrun。

## 判定

20 帧窗口对**加速**没有观察到低估问题，因为 CATCH_UP 会保持到约 6.1 s 后才 CRUISE；对**减速**则有即时锁存偏高风险。现有 5 帧快速通道在本组阶跃数据中只部分缓解，后续 Governor slowdown 更新最终纠正了速度，且没有形成周期性 hunting。不过最后一组纠正耗时 3.65 s 并伴随距离降到 0.759 m，说明“无 hunting”不等于减速瞬态足够快。

详细逐帧 JSON 在同目录 `dynamic_rate_*.json`；运行脚本为 `dynamic_rate_validation.py`。
