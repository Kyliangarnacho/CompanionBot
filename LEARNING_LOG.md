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
- 它对系统辨识仍有价值，但最终 slow/fast compensation 不依赖 probe 或 RLS 才能工作，因此降级为诊断/实验工具。

## 9. 上游启发与 slow/fast disturbance rejection

- 参考 varying-payload self-tuning regulator、LQR + L1 adaptive predictor/projection/filtered compensation，以及成熟自平衡项目的控制边界组织。
- 没有机械照搬；保留强 nominal ID-LQR，使用 nominal one-step residual 估计 equivalent input disturbance。
- 最终结构为 `u = u_lqr + u_slow + u_fast`：slow/fast 按频带区分，均经过 projection、带宽和软件 authority 限制；碰撞不得修改长期 operating point。

## 10. Authority 与碰撞实验

- 单独放宽 projection、slow、fast 和 combined 软件限幅，没有从根本上消除高能持续碰撞；失败不只是软件 authority 不足，还包含 payload hidden-state 与 unmatched/contact dynamics。
- 原始低摩擦场景出现“无碰撞”和长期乒乓之间的分岔。仅缩小 payload 到 80% 尺寸仍不能得到稳定的有限碰撞次数。
- 保持 0.20 kg，将 payload 设为 64×32×32 mm，并把 μ 从 0.024 适度提高到 0.040 后，C 在固定初态下得到 2 次有效碰撞、无饱和、载荷留篮且运动衰减。
- A/B 的更多碰撞与 B 的载荷逃出被原样保留，没有通过改变 baseline 或缩小冲击来美化结果。

## 11. V1 冻结结论

- 最终保留：MuJoCo plant、Cascade PID、analytic LQR reference、Offline full-state ID-LQR、fixed LQR + slow/fast disturbance rejection。
- Auto Probe 保留为诊断工具；Online adaptive A/B/K 与 affine theta_eq 不进入最终 moving-payload actuator 主链。
- 当前设计域覆盖 nominal、温和自由滑动和有限低能碰撞；不承诺处理持续高能乒乓碰撞。
