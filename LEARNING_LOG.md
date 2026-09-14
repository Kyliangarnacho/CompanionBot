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
