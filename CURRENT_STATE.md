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

## Stage 4 研究结论（不改写 Stage 3 baseline）

- Stage 4A：冻结 baseline 在 ±8° 通过；±15° 属于压力测试 envelope。现有 2 Hz Q actuator
  对坡面 tracking 无独立收益，production 继续 OFF；Q observer 保留。
- Stage 4B：68 个 episode、episode-exclusive split、100 Hz / 0.8 s window 的 proprioception
  pilot 已真实生成和训练。statistics+Ridge 的 test α MAE/p95 为 1.190°/3.348°；tiny raw
  Conv1D 为 2.007°/3.815°，两者均未过 α Gate，payload counterfactual drift 也为 1.070°。
- Stage 4B-R 发现旧 generator 只改 terrain friction、未改 wheel collision friction；修正版以新 ID
  生成 108/9/40 train/val/test episode，旧数据不覆盖。2.4 s Ridge 的 online MAE/p95/max 为
  2.615°/8.187°/21.954°；Ridge+±3° bounded residual 为 2.579°/7.284°/24.744°，仍有
  53 个 window 的绝对误差超过 5°。Slip CNN recall 0.689，rough CNN F1 0.441。
- Stage 4B-R 最终定案：**PERMANENT REJECT proprioceptive slope learning**；slip detector 不进
  production；rough cheap baseline 只保留结果/代码。按 Stop Rule 未把 estimator 接回 controller，
  也未执行条件式 A/B/C/D。解析 α→θ_eq 的 0.237°验证结果保留，但不代表 online sensing 可用。
- Stage 4C 没有恢复学习路线，而是按 Parravicini/Corno/Savaresi 的解耦结构增加独立 physics
  slope EKF。clean empty、smooth、`mu=1.0` 的 held-out ±3/±6/±10/±14°、0.2/0.4/0.6 m/s
  上，TWIP-EKF MAE/p95/steady bias 为 0.206°/0.400°/0.143°，48/48 有效 episode 在恒坡
  入口后最坏 0.08 s 收敛；因此定案 **KEEP TWIP-EKF**。STATIC-INVERSE 因 4 个有效 episode
  未满足 5 s hard convergence 而不保留。
- Stage 4C oracle `x_eq/u_eq` 和 estimated-alpha A/B/C 闭环均无 fall/saturation；estimated 闭环
  steady velocity RMSE 为 0.0031–0.0054 m/s。fixed 0.10 kg payload 则使坡度 MAE/p95 恶化到
  1.888°/2.564°，明确需要 payload-aware model parameters，本轮未做 payload adaptation。
- Stage 4D 的 scalar `[v_body,d_slip]` IMU/encoder observer 在严格截断 45° fall 后的 held-out
  pure-slip test 上 precision/recall/F1 = 0.947/0.419/0.581、FPR=0.0006、median latency=0.10 s；
  recall 未过 0.90 Gate。按 Stop Rule 未接 safe slowdown、未跑 slope HOLD integration，production
  不保留 slip path，并记录 future camera/visual-odometry independent velocity dependency。
- Payload Q 最终 A/B 对 fixed/free 没有一致明确收益，且 free payload 的 pitch/velocity 指标略退化；
  Q actuator 定案 **PERMANENT PRODUCTION OFF**，observer 仅 diagnostic。Frozen LQR 对 rough 与
  12/12 个 0.5/1.0/1.5 N、0.10 s 双向 stationary/moving push 自行恢复；现有 modest bump 使车体
  无 fall/无 saturation 但停滞在障碍前，属于当前 V1 traversable envelope 之外。
- 最终职责：Q observer 继续 diagnostic/physics logging；Q actuator production OFF；Stage4B-R
  的 proprioceptive **learning** 永久拒绝不变。TWIP-EKF 只在 clean empty nominal-traction
  假设内保留；slip coupling 留给 Stage4D，rough/bump/free-payload 和几何传感不在 Stage4C 范围。
- Stage 4 最终 runtime 只保留 `FLAT/SLOPE` 与 `payload_id_pending/payload_id_active`。Q one-step
  innovation 只触发一次短 session；payload ID 以 50 Hz 使用 31 次自然 transient 更新，直接冻结
  两个 sagittal 参数。本次 0.25 kg、前置 15 mm 真值的估计为 0.3476 kg、15.18 mm；质量偏高作为
  baseline limitation 保留，不再增加 acceptance layer 或调参。
- 最终 robustness smoke 中，中心/左右 ±10 mm 固定偏载与左右各 1 N×0.10 s 单轮纵向冲激均无
  fall、无 wheel saturation。左偏载的 yaw drift/peak 最大（0.0627°/s、0.718°）；左右冲激相对
  冲激前稳态带的 settling 分别为 0.062 s / 0.002 s。Stage 4 据 Stop Rule 收口。

最终记录：`models/minisegway/stage4/stage4b/results/STAGE4B-R_REPORT.md`。
Stage 4C 记录：`models/minisegway/stage4/results/STAGE4C_SLOPE_REPORT.md`。
Stage 4 最终记录：`models/minisegway/stage4/results/final_baseline/STAGE4_FINAL_BASELINE.md`。
