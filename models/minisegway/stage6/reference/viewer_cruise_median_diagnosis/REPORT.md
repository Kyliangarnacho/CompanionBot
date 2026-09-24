# Stage 6.x 巡航速度锁存诊断与修复

## 原因与本轮改动

先按要求试了仅在 CATCH_UP → CRUISE 时锁存最近 5 个有效 KF Master 径向速度的 median。它改善了个别切换帧的低估，但 20–60 秒的 FULL 次数由 Vx/Vy 的 9/10 次变为 11/12 次。原因有两部分：KF 相邻 5 帧误差仍相关，median 仍可能偏低；较高的巡航锁存值又让原有 CRUISE → CRUISE 的 `slowdown_wait` 在匀速 Master 的短时 KF 下探中触发，并重新锁存单帧低值。

最终只在 Follow Governor 内保留 20 个有效 KF 速度值：切换时同时计算最近 5 帧 median 和最近 20 帧 median。通常用 20 帧值锁存；当 5 帧值比 20 帧值低超过既有 `slowdown_velocity_deadband_m_s`，用 5 帧值及时响应真实减速。`slowdown_wait` 对非停车减速要求 20 帧值也比当前锁存值低超过同一 deadband；5 帧 median 已降至 deadband 以下时仍走快速零速路径。没有改 KF、FULL、d1/d2、safety、Stage 3/4/5 参数，也没有新增配置或连续速度伺服。

被拒绝的短时 slowdown 候选在 Governor observation history 中记录 `slowdown_rejection_reason`。CATCH_UP → CRUISE 事件同时记录单帧 KF、单帧旧逻辑 shadow、5 帧 median、20 帧 median 和最终锁存值。

## 两组 60 秒对照

Master 分别固定为 `(Vx,Vy)=(0.40,0)` 和 `(0,0.40)` m/s。使用 viewer 原配置、同一随机种子、20 Hz observation、完整 packet/KF/Governor/FULL/yaw/MCU/MuJoCo 链路。以下只统计 20–60 秒；`d` 是 Governor 使用的 KF 距离，数值为均值 ± 标准差（最小值–最大值）。

| 方案 | 方向 | Governor 事件 | FULL 计划 | d (m) | 锁存 v_cmd (m/s) |
|---|---|---:|---:|---|---|
| 原单帧 | Vx | 9 | 9 | 1.253 ± 0.110 (1.019–1.472) | 0.401 ± 0.092 (0.291–0.566) |
| 原单帧 | Vy | 10 | 10 | 1.231 ± 0.109 (1.029–1.449) | 0.396 ± 0.088 (0.304–0.572) |
| 仅 5 帧 median | Vx | 11 | 11 | 1.263 ± 0.108 (1.029–1.457) | 0.405 ± 0.097 (0.302–0.582) |
| 仅 5 帧 median | Vy | 12 | 12 | 1.216 ± 0.126 (0.999–1.476) | 0.402 ± 0.100 (0.274–0.570) |
| 本轮最终方案 | Vx | **0** | **0** | **1.144 ± 0.029 (1.059–1.231)** | **0.398，保持不变** |
| 本轮最终方案 | Vy | **0** | **0** | **1.113 ± 0.014 (1.066–1.161)** | **0.400，保持不变** |

最终方案两组各有 3 次启动阶段 FULL 计划；20–60 秒没有周期性追赶/减速。两组均未跌倒，轮扭矩饱和率为 0，sum command 饱和、allocator guard 裁剪和 reference underrun 均为 0。

| 方向 | 切换时间 (s) | 当帧 KF Vm | 5 帧 median | 20 帧 median | 最终锁存 v_cmd | 窗口样本数 |
|---|---:|---:|---:|---:|---:|---:|
| Vx | 11.05 | 0.384 | 0.409 | 0.398 | 0.398 | 5 / 20 |
| Vy | 9.75 | 0.349 | 0.367 | 0.400 | 0.400 | 5 / 20 |

`single_sample_shadow_latched_velocity_m_s` 在这两次切换分别为 0.384、0.349 m/s，只用于诊断。

## 减速响应与限制

现有 `stop_restart_noisy` 场景中，本方案与仅 5 帧版本均在 Master 停车后 **0.45 s** 发出零速命令；停车后 8–12 秒的最小 KF 距离均为 **0.686 m**。均未跌倒、未饱和、未发生 reference underrun。曾试验仅用 20 帧 median，虽然匀速稳定，但零速命令延至 1.05 s、最近距离缩至 0.621 m，因此没有采用。

结论限于上述两组 0.40 m/s 匀速方向和现有停车/重启场景；尚未通过交互式 GUI 人工验收，也不能据此推断任意 Master 轨迹均无 hunting。

## 复现

在仓库根目录使用 `.venv`：

```powershell
.\.venv\Scripts\python.exe models\minisegway\stage6\reference\viewer_cruise_median_diagnosis\reproduce.py baseline
.\.venv\Scripts\python.exe models\minisegway\stage6\reference\viewer_cruise_median_diagnosis\reproduce.py robust
```

`baseline` 是诊断进程内的旧单帧 transition ablation，不更改生产文件。结果分别保存在 `baseline_*.json` 和 `robust_*.json`；失败的 5 帧 median 结果保留在 `median_*.json`。
