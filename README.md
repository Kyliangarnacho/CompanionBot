# CompanionBot

CompanionBot 是一个面向个人学习与研究的双轮自平衡机器人项目。当前冻结版本为 **V1 MuJoCo longitudinal control baseline**：重点验证机械 plant、状态空间辨识、经典控制基线，以及自由移动载荷下的扰动抑制能力；尚未进入完整 CompanionBot 外形、感知、导航或 STM32H743 实机阶段。

## V1 控制架构

物理仿真以 1 kHz 运行，控制器以 500 Hz 确定性更新。状态为：

```text
x = [position, velocity, pitch_error, pitch_rate]
u = tau_left + tau_right
tau_left = tau_right = clip(u / 2, -0.63, +0.63) N·m
```

最终 moving-payload 控制律：

```text
u = u_lqr + u_dr
```

- `u_lqr`：由 nominal MuJoCo 数据离线辨识得到的固定 4-state ID-LQR，是基础稳定器。
- `u_dr`：sensorized nominal-model innovation 经 matched projection、单一 2 Hz Q-filter
  和 ±0.18 N·m augmentation authority bound 后得到的补偿。
- Virtual IMU hardware：1 kHz sensor tick 将 MuJoCo ideal accel/gyro 经过 RotorS ADIS16448
  bias/noise 与 full-scale clipping 后形成 packet；固定 1 ms availability latency，启动时用
  0.5 s noisy gyro 均值校准零偏，500 Hz estimator 只读取最新有效 available packet；输出端
  根据 packet 实际 age 将 delayed attitude 外推到当前控制时刻，并与 current encoder PLL
  统一形成 `x_hat_control_time`。
- Auto Probe Manager：仅保留为辨识诊断/实验工具，不属于最终 moving-payload actuator 主链。
- Payload 位置、速度、接触状态和事后真实平衡角只用于验收日志，控制器不可读取。

## 快速运行

在项目根目录使用现有 Python 3.11 虚拟环境：

```powershell
# Plant load/contact smoke test
.\.venv\Scripts\python.exe scripts\smoke_test_minisegway.py

# 保留的 sensor/encoder 几何 sanity checks
.\.venv\Scripts\python.exe scripts\check_raw_sensors.py
.\.venv\Scripts\python.exe scripts\check_virtual_encoders.py
.\.venv\Scripts\python.exe scripts\check_encoder_displacement_geometry.py

# Nominal baselines
.\.venv\Scripts\python.exe scripts\run_fixed_lqr_headless.py
.\.venv\Scripts\python.exe scripts\run_cascade_pid_headless.py

# Offline full-state identification（会重新生成辨识结果）
.\.venv\Scripts\python.exe scripts\run_full_state_identification.py

# 最终 sensorized moving-payload acceptance（单次运行）
.\.venv\Scripts\python.exe scripts\run_moving_payload_collision_acceptance.py
```

`run_moving_payload_collision_acceptance.py` 当前只执行一次冻结的 2 Hz Sensorization V1
timestamp-aligned final case，不做 cutoff sweep 或 deterministic repeat。最近结果通过：
payload decay 0.01773、terminal pitch RMS 0.550°、无 wheel saturation；记录在
`moving_payload_timestamp_aligned_collision_results.json`。阶段性的 cutoff/PSD 诊断已收口为
`LEARNING_LOG.md` 表格，不再留在 production acceptance 入口中。

有限碰撞入口的当前在线主链统一使用 sensorized
`x_hat=[p_hat,v_hat,wrap(theta_hat-theta_eq),theta_dot_hat]`；LQR 和 matched
innovation 均不读取 MuJoCo GT。旧 boxcar、slow/fast 和未对齐 Sensorization V1 执行逻辑
已经删除。

人工查看最终有限碰撞场景：

```powershell
.\.venv\Scripts\python.exe scripts\view_moving_payload_stress.py --case A
.\.venv\Scripts\python.exe scripts\view_moving_payload_stress.py --case Q
```

其中 A 为 Frozen ID-LQR，Q 为 ID-LQR + 当前选定的单通道 Q-filter 补偿。

## 目录

```text
control/                 PID、固定 LQR、Offline-ID、filtered disturbance rejection 与 probe 诊断
sim/                     MuJoCo 确定性仿真和运行时 rigid payload 支持
models/minisegway/       MJCF、集中配置、质量属性和冻结 benchmark 结果
scripts/                 少量可复现 run/smoke/viewer 入口
tools/                   从本地 MiniSegway CAD 构建 plant、提取 reduced TWIP 参数
CURRENT_STATE.md         V1 指标、能力和设计边界
LEARNING_LOG.md          路线演进、失败结论与冻结 benchmark 表格
AGENTS.md                后续开发约束
```

Plant 参数来源与 provisional 项见 [models/minisegway/README.md](models/minisegway/README.md)。
