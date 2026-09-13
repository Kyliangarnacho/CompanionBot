# CompanionBot V1 当前状态

冻结日期：2026-09-13。本页只描述已经验证的能力，不把计划中的功能写成已完成。

## Plant 与执行约束

- MuJoCo physics：1 kHz；controller：500 Hz；控制周期固定为 2 个 physics step。
- Nominal 整机质量：1.02757 kg；整机 CoM：`[0.000127, -0.001109, 0.026795] m`。
- Reduced body mass：0.81737 kg；单轮旋转总成：0.1051 kg；轮半径：0.042 m；轮距：0.177 m。
- Nominal 平衡角：-2.3704°；单轮 hard peak torque：±0.63 N·m。
- Frame/cover 的真实材料与 infill 尚未知，相关质量仍是集中配置的 provisional 值。

## Nominal baseline

Offline full-state identification 使用独立 PRBS train/validation 数据拟合完整 `Ad(4×4), Bd(4×1)`。归一化 regressor condition number 为 3.80，validation one-step scaled RMS 为 0.0593；同一 Q/R 产生：

```text
K_id = [-2.96019, -4.94019, -8.66198, -0.492105]
```

ID-LQR 在 ±2°、±5° 全部未翻倒、无饱和；±5° settling 约 0.154 s，pitch RMS 约 0.258°。

Cascade PID 同样在四个工况全部通过、无饱和；±5° settling 约 0.303 s，pitch RMS 约 0.485°。PID 和 LQR 参数均已冻结。

## Payload 验证摘要

### Static rigid payload

运行时 rigid payload 机制已验证可真实修改 MuJoCo body mass、CoM 和 inertia。早期中心 0.25 kg 静态载荷实验中 Frozen/Probe/Adaptive 均未翻倒且无饱和；该实验用于验证机制与辨识链，旧在线 `A/B/c/theta_eq` 控制路线已经退役。最终 slow/fast 主链没有在冻结后重新针对静态 offset payload 调参或刷分。

### Free-sliding payload，μ=0.024

0.20 kg payload 可自由滑动，控制器不读取 payload GT。该次确定性场景没有撞壁，载荷相对运动自然衰减：

| 控制 | Fall | Pitch RMS | Peak | Position drift | Max wheel torque | Saturation |
|---|---:|---:|---:|---:|---:|---:|
| A Frozen ID-LQR | 否 | 3.066° | 4.734° | -0.1599 m | 0.0321 N·m | 0% |
| C ID-LQR + slow + fast | 否 | 2.151° | 4.423° | -0.1066 m | 0.0369 N·m | 0% |

### Limited-impact acceptance，μ=0.040

Payload 为 0.20 kg、64×32×32 mm，距纵向墙约 2 mm，初始相对速度 0.10 m/s。有效碰撞定义为连续 wall contact episode 中峰值法向力至少 0.01 N。

| 控制 | Fall | 留在篮内 | 有效碰撞 | Pitch RMS / Peak | Drift | Max wheel torque | Saturation |
|---|---:|---:|---:|---:|---:|---:|---:|
| A Frozen ID-LQR | 否 | 是 | 59 | 10.13° / 19.90° | -47.7 mm | 0.630 N·m | 11.30% |
| B ID-LQR + slow | 否 | **否** | 30 | 6.61° / 27.62° | +12.6 mm | 0.630 N·m | 5.38% |
| C ID-LQR + slow + fast | 否 | **是** | **2** | **1.77° / 5.72°** | -21.4 mm | **0.169 N·m** | **0%** |

C 的两次有效接触发生在约 0.015 s 和 0.026 s，之后 terminal/early payload speed RMS ratio 为 0.00043；确定性重复结果一致。

## V1 能力边界

- 当前模型只有 longitudinal 4-state，不含 yaw、payload state、传感器噪声/延迟或电机电气动态。
- Slow/fast 补偿只处理能投影到 wheel-torque input direction 的 matched disturbance；强烈的 hidden/unmatched contact dynamics 不保证可拒绝。
- 持续高能量“乒乓”撞壁会造成大姿态误差、饱和甚至载荷逃出，**不属于 V1 设计域**。
- Auto Probe/RLS 可用于诊断慢模型变化，但 moving-payload 主链不在线更新 A/B/K，也不依据瞬时 affine `c` 改写长期平衡角。
- 所有结论目前都是 MuJoCo simulation evidence，不等同于实机安全认证。
