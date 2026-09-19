# CompanionBot

CompanionBot 是一个双轮自平衡机器人研究项目。Stage 3 已收口为一套可复现的 MuJoCo
simulation baseline，覆盖 longitudinal commanded motion、并联 yaw control，以及机械整形后的
free-moving payload 验证。它仍是仿真研究基线，不代表实机 torque/speed/payload 额定能力。

## 冻结的 Stage 3 baseline

- MuJoCo physics：1 kHz；controller/estimator：500 Hz。
- 控制状态：`[p_hat, v_hat, pitch_error, pitch_rate_hat]`。
- Longitudinal：new fixed A/B 用于 dynamic nominal/feedforward，反馈保留经挑战验证的 K_old。
- Reference：jerk-limited S-curve；velocity transient 使用 dynamic lean reference 和
  `lambda_ff=0.6`，到 `T_full` 后 0.15 s smooth fade，hold 阶段 feedforward 严格为零。
- Estimator：acceleration-compensated complementary pitch filter；GT 只用于 post-hoc evaluator。
- Yaw：并联 relative-heading/yaw-rate PD，`K_psi=0.55`、`K_r=0.20`；优先保护 longitudinal torque。
- Q observer：保留连续 diagnostic，actuator augmentation 关闭。
- Stage 3C-R payload：0.10 kg、64×32×32 mm，滑动摩擦 0.35，使用统一柔顺/阻尼壁面接触。

唯一 baseline manifest：
[`models/minisegway/stage3/config/baseline.json`](models/minisegway/stage3/config/baseline.json)。

## 快速运行

在项目根目录使用现有 Python 3.11 虚拟环境：

```powershell
# Plant smoke / Stage 1 baselines
.\.venv\Scripts\python.exe scripts\smoke_test_minisegway.py
.\.venv\Scripts\python.exe scripts\run_fixed_lqr_headless.py
.\.venv\Scripts\python.exe scripts\run_cascade_pid_headless.py

# 冻结的 Stage 2 acceptance
.\.venv\Scripts\python.exe scripts\run_moving_payload_collision_acceptance.py

# Stage 3 longitudinal / yaw / mechanically conditioned payload
.\.venv\Scripts\python.exe scripts\run_stage3a_velocity_feedforward_handoff.py
.\.venv\Scripts\python.exe scripts\run_stage3b_yaw_control.py --final-only
.\.venv\Scripts\python.exe scripts\run_stage3c_mechanical_payload_q_revalidation.py
```

人工查看 Stage 3C-R 控制过程：

```powershell
.\.venv\Scripts\python.exe scripts\view_stage3c_dynamic_payload_q.py --arm q-off
```

Viewer 由用户亲自启动；关闭窗口即可停止。`q-on` 仅复现实验臂，不是 production baseline。

## 目录

```text
control/                         LQR、reference、feedforward、yaw 与 Q observer primitives
sim/                             MuJoCo 仿真、sensor/estimator 与 payload runtime support
models/minisegway/               共享 plant/sensor config 与 stage 分层 artifacts
  stage1/results/                smoke、equilibrium、PID/LQR 基线
  stage2/results/                full-state ID 与冻结 moving-payload acceptance
  stage3/config/                 Stage 3 manifest 和专用配置
  stage3/results/                Stage 3 最终结果及少量历史摘要
scripts/                         保留的可复现 run/check/viewer 入口
CURRENT_STATE.md                 当前冻结状态和边界
LEARNING_LOG.md                  调试路线、负结果和最终决策
```

Plant 参数与 sensor semantics 见
[`models/minisegway/README.md`](models/minisegway/README.md)，当前冻结结论见
[`CURRENT_STATE.md`](CURRENT_STATE.md)。
