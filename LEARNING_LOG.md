# Phase 0/1 学习记录

## 1. Legacy 与 upstream

- 旧 STM32F103 平衡车代码被定位为历史参考：控制数据流、周期组织和安全思路可 Reuse/Adapt；GPIO、Timer、PWM、UART、DMA、NVIC、Clock 和 startup 等板级内容应在未来 STM32H743 工程中 Rewrite。
- `balance-robot-mujoco-sim` 在 Windows + Python 3.11 中真实运行，适合用作最小 MuJoCo 入门参考，但其 velocity actuator、简化模型与硬编码 LQR 不可直接作为 CompanionBot 物理 baseline。
- 发现并复现了 SciPy `xyzw` 到 MuJoCo `wxyz` 的 reset 四元数顺序错误。
- MiniSegway CAD 被整理为机械几何母版；visual 使用 CAD mesh，collision 使用 cylinder/box 等解析几何，内部件只贡献 inertial。

## 2. Analytic LQR baseline

- 从 CAD 与厂家质量提取 reduced TWIP 参数，建立直接 wheel-torque actuator。
- 使用 SciPy ZOH 与 `solve_discrete_are()` 得到首个固定 LQR；±2°、±5° 均通过。
- 结论：解析 reduced model 是有价值的 physics reference，但质量、CoM、轮半径或接触条件改变后，原 K 不能被默认视为仍然正确。

## 3. Cascade PID baseline

- 建立 position/velocity 外环与 pitch 内环，加入限幅和 anti-windup。
- 在与 LQR 相同的 plant、频率、初始角度和 torque limit 下通过。
- 原则：后续方案必须与冻结 PID/LQR 公平比较，不允许为了新算法获胜而修改 baseline。

## 4. Structured RLS 与离散回归

- 最初连续四参数回归无法解释 full MuJoCo 数据；加入速度、角速度和 intercept 后 residual 明显下降。
- 改成与 500 Hz ZOH 严格对齐的 structured discrete regression，并验证 `x[k], u[k], x[k+1]` alignment。
- 加 `u[k-1]` 只解释了部分短期记忆，没有形成可靠的通用模型。
- 结论：不能靠继续堆 lag 或手工 state 让错误结构“看起来能拟合”。

## 5. Offline full-state ID-LQR

- 改用标准完整 `x[k+1] = Ad x[k] + Bd u[k]`，对 state/input scaling，train/validation 分离。
- Offline-ID 在独立 validation 上明显优于 analytic ZOH，并通过 `Ad/Bd → DARE → K_id → MuJoCo` 全链稳定测试。
- 这成为 V1 的强 nominal baseline。

## 6. Online Matrix RLS

- 从 analytic ZOH 初始化时，matrix error 曾从 13.26% 学至最低约 1.07%，证明 estimator 能学习 Offline-ID 模型。
- Probe/PE 消失后仍更新会漂移至约 8.03%；加入 information-aware freeze、hysteresis 和 covariance management 后可在低信息阶段保持。
- 原则：estimator 是否学习，与 candidate K 是否有权接管必须分离；无信息时不能持续 forgetting。

## 7. Affine equilibrium / theta_eq 路线

- 前后静态 offset payload 需要 affine `x[k+1]=Ax[k]+Bu[k]+c`，否则 A/B 会吸收 operating-point shift。
- Estimator 能把 front theta offset 估到约 -8.25°，接近 oracle -8.23°；但 reference deployment 曾被 probe session/gain gate 生命周期错误连带冻结。
- `theta_offset` 与 `u_ff` 强耦合，正确几何平衡点下 steady wheel torque 理应接近零；弱正则比自由解释 `u_ff` 更物理。
- 对自由移动 payload，瞬时 affine `c` 同时混合滑动、碰撞和局部偏置。用它不断改长期 theta reference 会造成错误 operating-point 追逐，因此该路线退出最终 actuator 主链。

## 8. Moving payload 与 Auto Probe

- 自由 payload 引入隐藏自由度和真实接触；4-state 模型无法把所有瞬态解释成一套缓慢变化的 A/B/c。
- Auto Probe Manager 使用 prediction mismatch persistence/hysteresis 自动触发，避免单个碰撞峰值触发，并在 PE/平台或 max-duration 条件下结束。
- 它对系统辨识仍有价值，但最终 single-Q compensation 不依赖 probe 或 RLS，因此降级为诊断/实验工具。

## 9. 历史 slow/fast disturbance rejection

- 参考 varying-payload self-tuning regulator、LQR + L1 adaptive predictor/projection/filtered compensation，以及成熟自平衡项目的控制边界组织。
- 没有机械照搬；保留强 nominal ID-LQR，使用 nominal one-step residual 估计 equivalent input disturbance。
- 该阶段结构曾为 `u = u_lqr + u_slow + u_fast`；后续确认分路 clip/shared slew 会破坏频带互补并引入额外 lag，因此已由单一 matched innovation → Q-filter → `u_dr` 通道彻底替代，旧执行逻辑与逐点数据在 Sensorization V1 收口时删除。

## 10. Authority 与碰撞实验

- 单独放宽 projection、slow、fast 和 combined 软件限幅，没有从根本上消除高能持续碰撞；失败不只是软件 authority 不足，还包含 payload hidden-state 与 unmatched/contact dynamics。
- 原始低摩擦场景出现“无碰撞”和长期乒乓之间的分岔。仅缩小 payload 到 80% 尺寸仍不能得到稳定的有限碰撞次数。
- 保持 0.20 kg，将 payload 设为 64×32×32 mm，并把 μ 从 0.024 适度提高到 0.040 后，C 在固定初态下得到 2 次有效碰撞、无饱和、载荷留篮且运动衰减。
- A/B 的更多碰撞与 B 的载荷逃出被原样保留，没有通过改变 baseline 或缩小冲击来美化结果。

## 11. V1 冻结结论

- 最终保留：MuJoCo plant、Cascade PID、analytic LQR reference、Offline full-state ID-LQR、sensorized fixed LQR + single-Q matched disturbance rejection。
- Auto Probe 保留为诊断工具；Online adaptive A/B/K 与 affine theta_eq 不进入最终 moving-payload actuator 主链。
- 当前设计域覆盖 nominal、温和自由滑动和有限低能碰撞；不承诺处理持续高能乒乓碰撞。

## 12. Sensor timestamp 与未来 output predictor 路线

- Sensorization V1 的 IMU packet 带真实 `sample_time` / `available_time`；1 ms availability latency 使 complementary-filter 姿态位于测量时刻，而 controller 入口位于当前控制时刻。当前最小修复只在 estimator 输出边界做 `theta_now = wrap(theta_delayed + theta_dot_delayed * measurement_age)`，`theta_dot_now = theta_dot_delayed`，再用 control-time 姿态与当前 encoder PLL 一起构造 `p_hat/v_hat`。`measurement_age` 始终取 packet timestamp 差值；stale/invalid guard 触发时禁止继续外推。
- 本阶段仍使用冻结的 2 ms nominal `A/B`，LQR 与 DOB one-step residual 必须消费同一套 `x_hat_control_time[k]` / `x_hat_control_time[k+1]`。不实现 1 ms `A/B`，也不把 GT 或 post-hoc residual 引回 production path。
- 未来候选：**model-based delayed-state → current-state output predictor**。当 latency 增加到数毫秒、多传感器 delay 不同或 constant-rate 外推不足时，考虑使用已知 input history 与 nominal dynamics 做 `x_now = A_delta x_delayed + B_delta u_history`，或引入 delayed fusion horizon + current-horizon output predictor。
- 成熟参考：[PX4 ECL EKF](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf) 使用 delayed fusion time horizon、FIFO-buffered measurements，并用 buffered IMU / output complementary filter 把状态传播到当前时刻；[ArduPilot EKF2](https://ardupilot.org/dev/docs/ekf2-estimation-system.html) 在 delayed horizon 融合，再用计算成本更低的 output predictor 将 delayed filter state 预测到 current horizon；Khosravian, Trumpf, Mahony, Hamel, “[Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements](https://doi.org/10.1109/ACC.2015.7171825),” ACC 2015，给出 sampled/delayed vector measurement 与 observer 之间的递归 output-predictor 结构。本阶段仅记录，不移植完整 EKF 或 predictor。
- 若后续 post-hoc 频谱确认 physical matched disturbance 与 sensor/estimator contamination 在低频严重重叠，单一 low-pass Q 无法只靠 cutoff 分离；届时再评估 NRDOB、higher-order/shaped Q、data-based sensitivity shaping 或 estimator-confidence/innovation-aware compensation。本阶段不实现这些升级。

## 13. Sensorization V1 最终冻结 benchmark

旧 boxcar、slow/fast、Q sweep、未对齐 IMU 状态和逐点 transient JSON 已清理；以下表格是其需要长期保留的工程结论。

### Nominal ID-LQR：GT 与最终 PLL sensorized

| Initial pitch | GT settle / RMS | PLL sensorized settle / RMS | Real drift | Peak torque / saturation |
|---:|---:|---:|---:|---:|
| -5° | 0.154 s / 0.258° | 0.277 s / 0.291° | +6.32 mm | 0.378 N·m / 0% |
| -2° | 0.127 s / 0.104° | 0.289 s / 0.149° | +2.50 mm | 0.151 N·m / 0% |
| +2° | 0.127 s / 0.104° | 0.255 s / 0.118° | -2.73 mm | 0.151 N·m / 0% |
| +5° | 0.154 s / 0.258° | 0.289 s / 0.302° | -6.48 mm | 0.378 N·m / 0% |

以上是 ideal-IMU PLL takeover 阶段的冻结结果，用于证明 sensor-only state assembly 可稳定接管；不是后来加入 ADIS16448 noise/latency 后的重新回归。PLL velocity error RMS 为 2.82–7.28 mm/s，peak 为 0.0316–0.0795 m/s；残余累计位置误差支持 wheel-ground slip 是独立 odometry limitation。

### Moving-payload 路线收敛

| 版本 | Pitch RMS / peak | Terminal RMS | Drift | Collisions | Payload decay | `u_dr` RMS / peak | Authority hit | 结果 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Noise-free sensorized Q=0.6 Hz | 1.91° / 7.61° | — | -188.9 mm | 2 | 0.00178 | 0.071 / 0.180 N·m | 3.80% | PASS |
| Noise-free sensorized Q=2 Hz | 3.05° / 7.60° | 0.212° | -155.8 mm | 2 | 0.00082 | 0.116 / 0.180 N·m | 13.58% | PASS |
| Full IMU V1、timestamp 未对齐 | 5.31° / 8.42° | 4.790° | -161.1 mm | 3 | 0.82172 | 0.148 / 0.180 N·m | 49.10% | FAIL |
| **Final timestamp-aligned V1、Q=2 Hz** | **4.38° / 9.37°** | **0.550°** | **-113.2 mm** | **5** | **0.01773** | **0.133 / 0.180 N·m** | **33.53%** | **PASS** |

最终 estimator error RMS / peak：`p` 0.978/1.938 mm，`v` 0.01245/0.07113 m/s，pitch 1.324/2.485°，pitch-rate 1.703/24.626°/s。IMU typical/max age 均约 1 ms，无 invalid、stale 或 saturation。Phase-B post-hoc Welch PSD 在 0–5 Hz 内没有 sensor-over-physical crossover，故 Q 保持 2 Hz且没有第二次 final run。

## 14. Stage 3A flat-ground commanded-motion closure

- 首先拆开了 plant state 与 tracking-error state。兼容的 stationary API 保留不变；新路径中
  LQR 使用 `x_plant-r`，DOB 明确只使用前后 sensorized plant state 和实际 held/applied
  sum torque。因此 moving reference 不会以坐标替换的形式直接进入 innovation。
- 使用经典 1D acceleration-limited trapezoidal profile；velocity mode 限速率后的 `v_ref`
  被梯形积分成一致 `p_ref`。500 Hz 全部序列满足 0.20 m/s、0.25 m/s² 限制，离散位置积分
  误差约 `1e-16 m`，没有引入 Ruckig、PID/LQI、feedforward 或 MPC。
- Zero-reference A/Q 都通过，现有 fixed-LQR 和 frozen moving-payload acceptance regression
  也通过；冻结结果文件内容没有被 benchmark 结果替换。
- Required empty commanded cases 没有通过预先固定的 tracking/dwell gate。所有 run 稳定且
  wheel saturation 为 0%，但 A 的 +0.25 m velocity RMSE 为 0.0615 m/s；其余 A moving
  scenario 也至少有一个 transition 未在下一 command 前保持 0.5 s 双误差 dwell。Q 的
  p/v RMS 在每个 moving scenario 都高于 A；+0.50 m 和 ±0.20 m/s 中 `u_dr` authority hit
  分别为 34.4% 与 44.7%。
- Estimator p/v RMS 远小于 tracking error；加减速时 `theta_acc` 明显受 specific-force
  污染，而 fused pitch 对 GT 仍保持亚 1° RMS。当前证据优先分类为 nominal-model/reference
  transient mismatch 与 Q interaction；较大 Q cases 同时出现 augmentation authority bound，
  但不存在 wheel actuator saturation。reference semantics 和 estimator dynamic error 不是
  主因。
- 按“empty 通过后再 payload”的阶段门，fixed/free commanded cases 没有执行。冻结的
  free-moving acceptance 仍然保持 PASS；不能把它解释为 commanded tracking 已通过。

## 15. Stage 3A reduced Drive LQR architecture check

- 按标准 cascaded position/velocity-balance 思路，在 Old-4State-A 旁增加最小
  New-3State-A；参考 FINN 的慢比例 position-to-velocity correction 拓扑，但不移植其模型、
  gain 或其它算法。
- 直接抽取 frozen identified model 的 `[v,pitch_error,pitch_rate]` principal subsystem，
  并继承对应 Q 子权重与原 R。可控性秩 3、DARE 成功、最大闭环 pole 模 0.999197；零参考
  仿真稳定且无饱和，因此按预设 gate 进入四个 moving cases。
- 外环固定 `Kp_pos=0.15 1/s`、`v_corr_max=0.05 m/s`，运行后未调参；两个 position case
  的实际 correction peak 仅为 0.0252/0.0292 m/s，未触及 cap。
- New-3State-A 在四个 moving cases 都降低 pitch RMS/peak 和 torque，但 p/v tracking RMS
  均上升；velocity cases 的 p-v dwell 从 3/4、2/4 降到 1/4、1/4。这一失败现象不依赖
  position outer loop，最直接地表现为继承权重的 reduced K3 给出的 velocity tracking
  response 不足、误差继续积分成 position drift。
- 因目标仅为架构判别，本轮没有调 K3/K4、做 Q/R sweep、重辨识、引入 Q/DOB、PID、积分、
  theta/u feedforward 或扩大 actuator authority。

## 16. Stage 3A reference smoothing / nominal feedforward 2×2

- 参考 [Ruckig](https://github.com/pantor/ruckig) 的 current/target `p/v/a` 与 `v/a/j`
  状态语义，针对当前单轴、命令间隔充分的实验实现最小对称 S-curve；没有引入通用 OTG
  依赖。固定 `j_max=1.0 m/s³`，四个场景都保持原 duration。
- 参考 Drake 的 nominal trajectory/input + LQR error feedback 结构，但只实现题设标量
  weighted LS：`u_ff=(B'WB)^-1 B'W(x_ref_next-A*x_ref)`，其中 W 来自冻结 identification
  state scales；`u_ff` 不读取 estimator state。
- A 与旧 Old-4State-A 四场景指标逐项完全一致。A→C 的 p/v RMS 通常只改善 0–3%，
  position overshoot 和 p-v dwell 没有实质变化，因此 acceleration discontinuity 不是当前
  overshoot/settling 的主要来源。
- A→B 在两个 position cases 把 overshoot 从 52.4/62.5 mm 降到 20.5/18.9 mm，dwell
  从 1/2、0/2 提升到 2/2，并把 pitch peak 降约 16–19%；±0.10 m/s dwell 也从 3/4
  提升到 4/4。代价是 +0.50 m position RMSE/peak 上升，±0.20 m/s dwell 降到 1/4，
  所以 feedforward 是有价值但非一致增益。
- S-curve 的 normalized feedforward feasibility residual RMS 为 0.0436–0.0693，trapezoid
  为 0.0490–0.0732，说明 smoothing 对 nominal one-step compatibility 有小幅改善。
  B/D 的 `u_ff` RMS/peak 为 0.069–0.139/0.102–0.205 N·m；总 requested torque 仍无饱和。
- D 通常略优于 B 的 velocity RMSE、position overshoot 和 pitch，但差异小；在 ±0.20 m/s
  仍不优于 feedback-only C 的 tracking/dwell，因此两项改动只呈弱互补，不构成全面胜出。

## 17. D 的 dynamic nominal lean reference

- 先否决了直接以 `theta_ref=c*a_ref` 和差分 `theta_dot_ref` 的准静态构造：它虽有清晰物理
  符号，但 jerk 切换处的 pitch-rate transition 使离散模型 normalized residual 升到
  0.144–0.164，显著差于旧 D，因此没有进入 MuJoCo 对比。
- 实现的最小方案是每个已知 command segment 的 sparse joint model-consistency solve：固定
  S-curve `p_ref/v_ref`，联合求解中间 lean states 与 nominal input，并将段端 lean 固定为零。
  行尺度仅复用 frozen identification state scales，未加新正则化或性能权重。
- 四个新 D 的 normalized residual RMS 从 0.0436–0.0693 降到 0.0090–0.0132；
  `theta_ref` peak 3.01–3.88°，`theta_dot_ref` peak 13.14–13.93°/s，加速/倾角方向一致。
- p/v RMSE 和 peak 在全部场景改善，position overshoot 也降为零；但两个 position dwell
  从 2/2 降为 0/2，pitch RMS/peak 在四个场景都变差。
- 重要的负结果：`u_ff-v_ref` 斜率仍约 1 N·m/(m/s)，匀速段 `u_ff` RMS 几乎
  不变；`u_ff/u_fb` 反向占比只小幅降低，而幅值 cancellation ratio 在四场景都更低。
  因此零 lean reference 并非这两个现象的唯一或主要来源；当前 identified model 自身的
  velocity damping / pitch-rate coupling 仍将持续运动投影成较大 nominal torque。
- 所有新 D 均无跌倒、无 wheel saturation，Q 关闭。本轮在此停止，不调 K4、不重新辨识、
  不引入 Q 或新 controller。

## 18. Dynamic-D finite-position terminal lifecycle

- 有限 position goal 新增最小状态机：TRACKING 保持原 dynamic-D `x_ref/u_ff`；完整 nominal
  trajectory 结束后进入 TERMINAL_HOLD，固定 `[p_goal,0,0,0] / u_ff=0`；只有
  estimated p/v 在原 acceptance band 内连续 0.5 s 才标记 GOAL_REACHED 并允许下一目标。
- 第一个 goal 的 planner-finished 时刻仍为原 schedule 的 4.500/6.000 s；实机 estimate 分别
  再需 0.726/0.838 s 到达。return planner 因 gate 顺延，完成后再需 0.500/0.650 s。
- 两个 position dwell 由 0/2 恢复到 2/2，overshoot 仍为零。p/v RMSE 没有变差，
  pitch RMS 略降，peak 只增 0.07–0.10°；无跌倒、无 wheel saturation。
- velocity mode 不实例化该 lifecycle，±0.10/±0.20 m/s 结果对象与封装前完全相同。
  Q 仍关闭，K4、planner、dynamic lean、feedforward、estimator、timing 和 torque limit 均未调整。

## 19. Dynamic-D velocity feedforward smooth exit

- 研究问题限定为：dynamic feedforward 在 reference transition 完成后无扰退场，能否消除
  steady-cruise `u_ff/u_fb` 长期对抗并保留 transient 收益。实现采用 reference-side
  completion 加固定 150 ms cubic smoothstep；没有 preload，也不改 `e_p/p_ref`。
- 四次 fade start 为 2.444、5.488、7.688、9.938 s；其中三个分别在 2.596、7.838、
  10.090 s 进入 HOLD。5.488 s 的 stop fade 在 12 ms 后被固定 schedule 的 reverse
  command 中断，并从当前连续 reference 重新规划。所有完成的 HOLD cruise 中 `u_ff=0`。
- fade 起点的 total-torque jump 为 -0.00023、+0.00071、-0.00742、+0.00131 N·m；
  `e_p/p_ref` 前后完全相等。相邻控制采样在最终 HOLD 标志切换附近仍有 -0.0248、
  -0.0118、+0.0349 N·m 变化，这是 estimator/K4 feedback 在不同采样间的变化，
  不是直接截断非零 feedforward。
- 相对原 Dynamic-D，p RMSE/peak 改善为 0.0215/0.0415 m，v RMSE 基本持平为
  0.02268 m/s，dwell 由 1/4 增至 2/4；但 v peak 增至 0.0622 m/s，pitch
  RMS/peak 增至 2.33/5.57°，requested torque RMS/peak 增至 0.0325/0.2263 N·m。
  无跌倒、无饱和。本轮据此停止，不调 tolerance/K4/planner/feedforward。

## 20. Velocity profile-completion fade / duration-gated rearm

- 参考 Ruckig current/target/profile completion、WPILib goal/setpoint 分离和 Nav2
  smoother 职责边界，但不引入依赖。所有 accepted target 继续走同一 S-curve；新增唯一
  固定门限 `T_rearm=0.30 s`，planned duration 较短时保持 `u_ff=0`。
- 删除 fade 前的 `a_ref/theta_ref/theta_dot_ref` tolerance 与 dwell。四个 1.05 s plan
  均 rearm，fade 分别在 2.050、5.050、6.550、9.550 s 开始，与 shaped-reference
  completion 数值一致；150 ms fade 全部在下一命令前完成。
- 相对上一版 handoff：p RMSE 0.02147→0.02178 m、peak 0.04149→0.03847 m；
  v RMSE 0.02268→0.02208 m/s、peak 0.06224→0.06487 m/s；dwell 保持 2/4；
  pitch RMS/peak 2.333/5.574→2.301/5.483°；requested torque RMS/peak
  0.03246/0.22629→0.03320/0.21123 N·m。无跌倒、无 saturation。
- 提前退出时 dynamic lean 尚有约 0.75–0.83°、pitch-rate 约 7.12–7.17°/s，导致
  fade 起点 reference-manifold 切换的 total-torque step 达 0.052–0.064 N·m；这是本次
  明确保留的直接现象，不继续调 threshold、fade、K4 或 dynamic nominal。

## 21. Independent feedforward fade / dynamic lean-reference completion

- 本次同一 lifecycle follow-up 不引入新算法：FF 在 shaped completion 立即开始原
  150 ms fade；lean reference 保持原 nominal states 至段端。两个时钟互不阻塞。
- 四次 fade-start reference 前后相同，事件 torque jump 从上一版
  +0.05194/-0.06226/-0.06380/+0.05186 N·m 全部降至 0。原 nominal reference
  peak deviation=0，50 Hz FF waveform 与上一版逐样本完全相同，fade 后 FF 严格为零。
- p/v RMSE 从 0.02178/0.02208 变为 0.02271/0.02249（m / m/s），dwell 仍为 2/4；
  pitch RMS/peak 从 2.301/5.483° 略降至 2.294/5.460°，requested/applied peak
  从 0.21123 略降至 0.21016 N·m。无跌倒、无饱和；一次 benchmark 成功落盘。
- 固定 schedule 的 lean terminal 正好在下一命令边界，因此没有正时长 HOLD 区间；
  fade 后的零 FF lean-tail 区间单独记录。非零 lean 中途 retarget 不受现有零端点
  nominal projector 支持，本轮不改 dynamic nominal 接口。

## 22. GT-angle diagnostic 与 opt-in acceleration-compensated pitch

- 先只跑一次 GT-angle oracle ablation：现有 runner 在内存中仅将 control state 第三维
  替换为 `sim.longitudinal_state(theta_eq)[2]`，保留 sensorized p/v/pitch-rate；没有将 GT
  override 写入正式代码。配置、完整 reference 和 FF waveform 与 current LQR+FF 逐项相同。
- stop1/stop2 velocity overshoot 从 0.06482/0.06045 降到 0.02308/0.01976 m/s，overall
  velocity RMS 从 0.02249 降到 0.01177 m/s，dwell 2/4→3/4；因此通过用户的滤波实验条件门。
- 按 sin17-radar Adapt ROS `imu_complementary_filter` 的 adaptive correction 职责与
  ArduPilot DCM 的 independent velocity-derived acceleration compensation 思路，不移植完整框架。
  参考源码：https://github.com/CCNYRoboticsLab/imu_tools/blob/rolling/imu_complementary_filter/src/complementary_filter.cpp
  与 https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_AHRS/AP_AHRS_DCM.cpp 。
- 新补偿仅为 opt-in：既有 odometry velocity 差分经固定 50 ms first-order LPF，保留原
  IMU→odometry 顺序，因果延迟一个 2 ms control step；将 world -Y 加速度用 gyro prediction
  转到 chassis 后从 accel 中减去。修正 gain factor 固定 `1/(1+(a_lpf/0.25)^2)`，原
  complementary time constant、gyro、PLL 与 packet-age handling 不变，不读 GT 或 a_ref。
- 只再跑一次同一 velocity_0p20_stop_reverse_stop：stop overshoot 0.02276/0.01939 m/s，
  p/v RMS 0.00930 m / 0.01270 m/s，dwell 3/4，pitch RMS/peak 2.088/5.067°，requested/
  applied sum torque RMS/peak 0.03105/0.20945 N·m；无跌倒、无 saturation，Q off。
  stop1 的 shaped-completion→next-command 窗仍只有 0.45 s，短于冻结 0.5 s dwell。
- 默认 estimator 与补丁前 dirty-worktree 实现在 1000 组合成 sensor inputs 上逐位一致；
  补偿符号、gain、低通/因果延迟、reset 与 IMU age 检查通过，没有重跑 stationary acceptance。
  结果保存为 `stage3a_velocity_gt_theta_diagnostic_results.json` 和
  `stage3a_velocity_acceleration_compensated_filter_results.json`。没有 tuning、commit 或 push。
  复现补偿实验使用现有 runner 的 `--controller-mode lqr_ff --pitch-filter acceleration_compensated`
  和明确的新 `--results-path`，默认 `--pitch-filter current` 保持 frozen estimator。

## 23. Stage 3A longitudinal debugging 收口

- Original A/B 主要来自 near-equilibrium、tiny-velocity 数据；进入 Stage 3 motion envelope 后出现明显 OOD one-step residual。
- Residual 近似 `B*d(v)`，说明 velocity-state effect 曾被错误吸收到 input direction。
- 临时 `A[:,v] += c_v*B` 显著降低 motion residual，但没有解决完整模型结构问题。
- Original `A[pitch_rate,pitch]` 的符号与 inverted-pendulum physics 不一致。
- Supported-equilibrium finite-difference Jacobian 对 wheel-ground contact epsilon 高度敏感，因此停止 FD 路线。
- 复用 0、±0.2、±0.4 independent PRBS 与 aggressive ±0.4 GT maneuver，batch least-squares refit 单套 fixed A/B。
- New A/B 通过 physical sanity checks，并在完全 held-out ±0.6 trajectory 上大幅降低 one-step residual。
- New A/B 配原 Q/R 重算的 K tracking 反而退化；有限 Q/R retune 仍未击败 incumbent，因此正式保留 K_old。
- Bounded `lambda_ff` search 显示更高 authority 改善 transient tracking/overshoot，同时提高 FF/FB cancellation。
- 人工选择 `lambda_ff=0.6`，作为 tracking 与 cancellation 的工程折中，而非采用搜索输出的 0.8。
- Final baseline 为 new fixed A/B + retained K_old + `lambda_ff=0.6` + transient-only FF lifecycle；empty-payload Q disturbance rejection 保持 OFF。

## 24. Stage 3B V1 parallel yaw / steering baseline

- Yaw 与 longitudinal 采用并联控制；原 longitudinal common-mode `u_sum` 继续完全由 K_old + transient-only FF 生成。
- Yaw baseline 使用 relative-heading P + yaw-rate D：`K_psi=0.55 Nm/rad`、`K_r=0.20 Nm/(rad/s)`，没有积分或 yaw feedforward。
- `psi_hat` 只积分 startup-bias-corrected realistic IMU gyro-z；按实际 packet timestamp 只积分新 valid sample，并按 measurement age 外推到 control time。
- V1 暂不使用 encoder yaw、magnetometer、EKF 或 GT correction；MuJoCo yaw/yaw-rate 只进入 post-hoc evaluator。
- Torque allocator 先保护 longitudinal authority，再用剩余轮端 authority 分配 `u_diff`；正 yaw 的 MuJoCo mixer convention 为 left 增、right 减。
- 五次有限 symmetric tuning 后选择 C4；final mixed regression 无 fall、wheel saturation 或 differential clipping，heading-hold settling 约 0.8 s。
- Motor-response mismatch 仅存在于专门验证 wrapper，production 默认明确为 disabled，默认 MJCF/actuator 配置未修改。
- 仅有不同一阶 response speed 且 DC gain 相同时，直线巡航 mismatch 没有形成持续 heading drift；40/50 ms 与 100/130 ms 对照均保留为负结果，没有叠加 gain/radius/friction mismatch。
- Final production baseline 为 symmetric motors + yaw PD ON；Stage 3A A/B、K、lambda、v/a/j limits、FF lifecycle、estimator 与 Q-off 语义保持冻结。

## Stage 3C — gated matched-disturbance Q integration

- Reused the existing B-direction matched-disturbance observer unchanged; it runs continuously from sensorized plant state and interval actual applied wheel-torque sum.
- Derived the d_hat-only hysteretic gate threshold from one integrated nominal shadow run; payload/GT/contact data remained evaluator-only.
- Preserved nominal longitudinal priority over Q and yaw, with the frozen 2 Hz Q filter and 0.18 N m authority.
- Used the accepted free-moving payload directly and compared identical Q-OFF versus gated-Q-ON arms. Nominal active fraction was 0.000000; moving-payload u_Q RMS changed from 0.000000 to 0.026715 N m.
- Production decision: reject gated actuator augmentation; retain observer as diagnostic only. No Q/gate/controller/payload parameter was tuned after observing results.

## Stage 3C-R — mechanically conditioned moving payload

- The prior commanded free-payload failure was treated as repeated rigid-wall impact/slosh outside the Q observer's intended slow matched-disturbance responsibility.
- One fixed passive redesign changed the V1 payload to 0.10 kg with consistent box inertia, raised basket/payload sliding friction to 0.35, and used unified native MuJoCo compliant/damped wall contact; controller, estimator, A/B/K/FF/yaw/Q/gate remained frozen.
- Mechanical Q-OFF outcome: fall=False, contained=True, saturation=0.000000, large collision-correlated drops=0.
- retain Q observer diagnostic-only; actuator augmentation rejected. No friction, padding, payload-mass, Q, or gate sweep was performed.

## Stage 3 closeout — frozen baseline and artifact hygiene

- Stage 3 正式冻结为：new fixed A/B + retained K_old + acceleration-compensated estimator +
  transient-only dynamic nominal/feedforward (`lambda_ff=0.6`) + parallel yaw PD；velocity hold
  继续保持 `u_ff_used=0`，Q observer 继续 diagnostic-only。
- Stage 3C-R 的一次固定机械整形成为 moving-payload baseline：0.10 kg payload、0.35 滑动摩擦和
  柔顺/阻尼壁面；旧 0.20 kg 高能碰撞失败只保留关键摘要，避免把 Q 与机械 contact failure 混为一谈。
- Artifacts 按 `stage1/`、`stage2/`、`stage3/` 分层；跨阶段共享的 MJCF、plant、sensor 和 estimator
  配置不复制。`stage3/config/baseline.json` 是当前唯一 baseline manifest。
- 删除了约 123 MiB 可再生的 Stage 3 500 Hz raw trace、CSV/NPZ、绘图、参数扫描结果和相应一次性
  runner；3-state Drive LQR、临时 A-column patch、FD Jacobian、GT-theta diagnostic、有限 Q/R 与
  lambda 扫描的结论仍由本日志保留，但不再属于可执行 baseline surface。
- 本次只整理代码、配置、结果与文档路径，没有重跑实验、修改 controller 数学、commit 或 push。

## Stage 4 closeout — rejected paths and artifact hygiene

- Stage4B/B-R 曾尝试基于 proprioception 的 slope/slip/rough learning；修正 wheel/terrain
  friction 一致性后仍未过 slope Gate，因此 learning 路线永久拒绝。配置、生成数据和报告从独立
  `stage4b/` 归入 `stage4/stage4b/`，runner 仅为历史复现入口，不接 production controller。
- Stage4E/E-R 曾使用 `PAYLOAD_ID` environment state、continuous/shadow scalar RLS、
  candidate/accepted 双层 payload model、residual-improvement 与 information/covariance/stability
  多重 gates；payload removal 和 slope transient 暴露出复杂 lifecycle 的脆弱性。
- 最终实现删除上述逻辑，只保留 `FLAT/SLOPE`、`payload_id_pending/payload_id_active`、Q model-change
  trigger 与一次短时 50 Hz natural-transient batch ID。Q actuator/slip control 均 OFF，Stage4C EKF
  参数、LQR/FF/yaw/allocator、torque/friction baseline 冻结。
- 旧 Stage4E/E-R runner 与配置已删除；失败/过渡结果和 world 移入
  `stage4/reference/stage4e_legacy/`，仅供追溯，不再出现在现行 results/worlds 目录。
- 本次仅整理路径、历史材料和阶段标签，没有新增算法、修改门限或重跑实验。
