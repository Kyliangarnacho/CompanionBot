# CompanionBot Sensorization V1 最终状态

冻结日期：2026-09-14。本页只描述当前 production baseline；历史选型与失败数据见 `LEARNING_LOG.md`。

## Plant 与时序

- MuJoCo physics 1 kHz；estimator/controller 500 Hz，控制周期固定 2 ms。
- Nominal 总质量 1.02757 kg，CoM `[0.000127, -0.001109, 0.026795] m`。
- Reduced body 0.81737 kg；单轮总成 0.1051 kg；轮半径 0.042 m；轮距 0.177 m。
- Nominal 平衡角 -2.3704°；单轮 hard torque limit ±0.63 N·m。
- Frame/cover 材料与 infill 尚未实测，相关质量仍是集中配置的 provisional 值。

## 最终 sensor 数据流

Nominal 与 moving-payload MJCF 使用相同安装位姿和 profile：IMU site 位于 chassis local
`[0, 0, 0.020] m`、与 chassis frame 对齐；左右 wheel joint angle 经同一 2248.8576
counts/output-rev virtual quadrature profile 量化。

```text
MuJoCo ideal IMU
  -> RotorS ADIS16448 noise/bias/full-scale hardware model @ 1 kHz
  -> timestamped packet + 1 ms availability latency + startup gyro calibration
  -> complementary pitch estimator at packet time
  -> actual-age constant-rate theta extrapolation / theta-dot ZOH

integer wheel counts @ 500 Hz
  -> per-wheel ODrive-style PLL (80 rad/s)
  -> continuous phi_hat / omega_hat

aligned IMU + wheel PLL
  -> p_hat / v_hat
  -> x_hat_control_time
```

Controller state 为：

```text
[p_hat, v_hat, wrap(theta_hat - theta_eq), theta_dot_hat]
```

LQR 和 matched-disturbance residual 都只读取同一套 control-time estimate。MuJoCo chassis
GT、payload pose/velocity/contact 和 instantaneous equilibrium 只进入 evaluator/logger。
Reset 时 `p_hat=0`，PLL position 初始化为当前 integer count、PLL velocity 为 0，pitch
从当前 accelerometer 初始化。Invalid/stale IMU 复用上一有效 measurement，且禁止继续外推。

## 最终控制器

```text
K_id = [-2.96019, -4.94019, -8.66198, -0.492105]
u = -K_id*x_hat_control_time + u_dr
```

`u_dr` 是 bounded matched innovation 经单一 first-order Q-filter（2 Hz）与 ±0.18 N·m
augmentation authority 后的补偿；final slew 当前关闭。旧 slow/fast actuator 分路、shared
slew arbitration 和 boxcar velocity 已删除。Auto Probe/RLS 仅保留为离线辨识/诊断资产，
不进入最终 actuator path。

## 最终 moving-payload acceptance

场景：0.20 kg、64×32×32 mm payload，basket friction 0.040，固定 IMU seed 16448。

| Fell | Contained | Pitch RMS / peak | Terminal RMS | Real drift | Collisions | Decay | Wheel torque / saturation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 否 | 是 | 4.377° / 9.369° | 0.550° | -113.2 mm | 5 | 0.01773 | 0.2656 N·m / 0% |

`u_dr` RMS/peak 为 0.1327/0.180 N·m，authority hit 33.53%，innovation scaled RMS 6.312。
Estimator RMS/peak error：position 0.978/1.938 mm，velocity 0.01245/0.07113 m/s，pitch
1.324/2.485°，pitch-rate 1.703/24.626°/s。IMU typical/max age 约 1 ms，无 invalid、stale
或 saturation。结果保存在 `moving_payload_timestamp_aligned_collision_results.json`。

## 能力边界

- 当前只有 longitudinal 4-state，不包含 yaw、terrain/slope、motor electrical dynamics、IMU temperature/vibration、通信 dropout 或 encoder fault。
- Encoder odometry 没有 wheel-slip compensation；真实 drift 不应被解释为 estimator 纯数值误差。
- 单 Q DOB 只处理可投影到 wheel-torque input direction 的 matched component；强 hidden/unmatched contact 不保证可拒绝。
- 当前结论是 MuJoCo simulation evidence，不等同于实机安全认证。
