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
u = u_lqr + u_slow + u_fast
```

- `u_lqr`：由 nominal MuJoCo 数据离线辨识得到的固定 4-state ID-LQR，是基础稳定器。
- `u_slow`：对 nominal model 一步预测残差做低带宽等效输入扰动补偿，处理持续慢变化。
- `u_fast`：对同一残差的快速频段做有界补偿，处理滑动和碰撞瞬态。
- Auto Probe Manager：仅保留为辨识诊断/实验工具，不属于最终 moving-payload actuator 主链。
- Payload 位置、速度、接触状态和事后真实平衡角只用于验收日志，控制器不可读取。

## 快速运行

在项目根目录使用现有 Python 3.11 虚拟环境：

```powershell
# Plant load/contact smoke test
.\.venv\Scripts\python.exe scripts\smoke_test_minisegway.py

# Nominal baselines
.\.venv\Scripts\python.exe scripts\run_fixed_lqr_headless.py
.\.venv\Scripts\python.exe scripts\run_cascade_pid_headless.py

# Offline full-state identification（会重新生成辨识结果）
.\.venv\Scripts\python.exe scripts\run_full_state_identification.py

# 自由滑动与最终有限碰撞 benchmark
.\.venv\Scripts\python.exe scripts\run_two_timescale_disturbance_benchmark.py
.\.venv\Scripts\python.exe scripts\run_moving_payload_collision_acceptance.py
```

人工查看最终有限碰撞场景：

```powershell
.\.venv\Scripts\python.exe scripts\view_moving_payload_stress.py --case A
.\.venv\Scripts\python.exe scripts\view_moving_payload_stress.py --case B
.\.venv\Scripts\python.exe scripts\view_moving_payload_stress.py --case C
```

其中 A 为 Frozen ID-LQR，B 为 ID-LQR + slow，C 为 ID-LQR + slow + fast。

## 目录

```text
control/                 PID、固定 LQR、Offline-ID、slow/fast 补偿与 probe 诊断
sim/                     MuJoCo 确定性仿真和运行时 rigid payload 支持
models/minisegway/       MJCF、集中配置、质量属性和冻结 benchmark 结果
scripts/                 少量可复现 run/smoke/viewer 入口
tools/                   从本地 MiniSegway CAD 构建 plant、提取 reduced TWIP 参数
CURRENT_STATE.md         V1 指标、能力和设计边界
LEARNING_LOG.md          Phase 0/1 的路线演进与失败结论
AGENTS.md                后续开发约束
```

Plant 参数来源与 provisional 项见 [models/minisegway/README.md](models/minisegway/README.md)。
