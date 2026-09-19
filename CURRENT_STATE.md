# CompanionBot Stage 3 frozen state

Stage 3 已完成并冻结为 MuJoCo simulation baseline。权威索引是
`models/minisegway/stage3/config/baseline.json`；生产配置未自动替换为调参过程中任何临时候选。

## Plant、时序与数据边界

- MuJoCo physics step：1 ms；controller/estimator update：2 ms。
- 两轮纵向输入语义：`u = tau_left + tau_right`；单轮 hard peak 仍为 ±0.63 N·m。
- 控制状态顺序：`[p_hat, v_hat, pitch_error, pitch_rate_hat]`。
- Virtual IMU 保留 1 kHz packet、1 ms availability latency、启动 gyro bias calibration 和
  control-time age compensation。
- Controller/estimator 不读取 GT payload/contact/pose；GT 只进入 evaluator 与结果 history。

## Longitudinal baseline（Stage 3A）

- Nominal model：batch-refit 的单套 fixed A/B，来源为
  `stage3/results/stage3a_refit_ab_0p60_validation_results.json`。
- Feedback：保留 K_old；new-A/B 原 Q/R gain 和最多六组有限 Q/R challenge 均未形成更好的替代证据。
- Estimator：acceleration-compensated complementary pitch filter。
- Velocity reference：jerk-limited S-curve，baseline limits 为 0.6 m/s、0.6 m/s²、1.0 m/s³。
- Dynamic nominal 只服务于 command transient；`lambda_ff=0.6`。`T_full` 后执行 0.15 s
  smooth fade，`VELOCITY_HOLD` 中 `u_ff_used=0`，position-error memory 不被 preload/reset。
- Finite position goal 保留 `TRACKING -> TERMINAL_HOLD -> GOAL_REACHED` 语义。
- Q/DOB actuator contribution 在正常 longitudinal baseline 中关闭。

## Yaw baseline（Stage 3B）

- Parallel relative-heading/yaw-rate PD：`K_psi=0.55 N·m/rad`、
  `K_r=0.20 N·m/(rad/s)`。
- Yaw estimator 只使用 startup-bias-corrected gyro-z，并按 packet timestamp/age 对齐。
- Wheel allocator 先保护 longitudinal common-mode torque，再分配 differential torque。
- Production baseline 使用 symmetric motors；motor mismatch wrapper 只保留为专门诊断路径。

## Moving payload 与 Q 决策（Stage 3C-R）

早期 0.20 kg、低摩擦、刚性壁面场景出现高能重复碰撞、payload escape、fall 和 saturation；该失败
归类为机械 disturbance shaping/hidden contact dynamics，不作为 Q observer 的成功条件。

一次固定被动机械整形后采用 0.10 kg、64×32×32 mm payload、0.35 滑动摩擦，以及
`solref=[0.02,1.5]`、`solimp=[0.7,0.95,0.005,0.5,2]` 的壁面接触。冻结 controller 下：

- Q-OFF：无 fall、payload contained、wheel saturation 0、无 large collision-correlated velocity drop。
- Pitch GT RMS/peak：6.09° / 13.81°；velocity GT error RMS：0.0394 m/s。
- Q-ON 只产生很小的指标变化，同时增加 feedback burden；因此 actuator augmentation 被拒绝。
- Q observer 保留 diagnostic-only；production `u_Q_used=0`。

完整结果：`models/minisegway/stage3/results/stage3c_mechanical_payload_q_revalidation_results.json`。

## 当前边界

- 这是 nominal flat-ground MuJoCo research baseline，不是硬件安全认证或 payload rating。
- 未解决项包括真实 motor torque-speed envelope、电池/驱动动态、地面变化、视觉/导航输入和实机验证。
- 不应从已删除的 tuning artifacts 恢复临时 A patch、3-state Drive LQR、persistent cruise FF、
  GT-in-controller diagnostic 或 Q actuator augmentation。
- 后续阶段若改变 plant、payload、timing、estimator、K/A/B 或 torque limit，应新建独立公平验证，
  不覆盖 Stage 1/2/3 冻结结果。
