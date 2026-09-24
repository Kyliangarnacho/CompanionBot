# Stage 6.4：二维相对跟随与纵向 / yaw 分权

基于当前本地 Stage 6.x 工作树实施，Stage 3/4/5 冻结检查全部通过。观测为 20 Hz，物理步长 1 ms，MCU 控制与双通道合流为 2 ms。三组闭环均使用同一套 Governor、FULL 和底层 yaw 控制器；没有反向速度、UTURN 状态或 yaw 轨迹规划器。

## 实现边界

- 纵向 Governor 使用 `d=hypot(x_forward,y_left)`、`d_dot` 和 `v_master,radial=d_dot+v_hat*cos(beta)`，其中 `beta=atan2(y_left,x_forward)`；原有 d1/d2、动态刹车距离、稀疏事件、latest-wins pending 与 all-FULL 均保留。`v_cmd` 全程限制在 `[0,0.6] m/s`。
- Pi 对每个有效观测给出 `w_cmd=clip(K_beta*beta,±0.5 rad/s)`；`K_beta=1/s`，中心死区 0.01 rad。yaw 以独立 `YawCommandPacket` 到 MCU，后包覆盖前包，不受纵向 FULL 锁定影响。旧 ReferenceBlock 的 yaw 字段仅保留兼容，新二维运行不读取它。
- MCU 每个 2 ms tick 分别取纵向 ReferenceBlock / 本地安全刹车和 latest yaw / 本地回零，再交给原控制器。yaw freshness 用 MCU 接收 tick，超时 0.2 s 后按 0.8 rad/s² 回零。
- `LONGITUDINAL_STOP` 只清空纵向 ACTIVE/NEXT、递增纵向 epoch 并本地刹车，yaw 包仍可生效；`HARD_STOP` 还撤销 yaw 权限并回零。两者都需显式恢复；hard 恢复额外要求停机后新收到的有效 yaw 包。原有 `BRAKE` 名称仅作为 Stage 6.2/6.3 兼容入口，新的二维运行只使用两种 action。
- Synthetic sensor 内部持有世界坐标 GT，Pi 端只读取序列化后的 `(x_forward,y_left)`；GT 径向速度和 Master XY 仅供事后统计 / 绘图。二维传感器修正了 body forward 为 `-Y` 时左向应取 `+X` 的符号。

## 三组闭环结果

| 场景 | 最小 / 最终距离 m | `|beta|` 峰值 / 最终 deg | 事后径向速度 RMSE m/s | yaw 在 FULL 期间更新 | 安全 / 恢复 |
|---|---:|---:|---:|---:|---|
| [曲线与侧向](plots/curved_lateral.png) | 1.062 / 1.063 | 5.47 / -1.03 | 0.020 | 98 次 | 全程 NORMAL |
| [折返与最近点](plots/turnaround_pass_by.png) | 0.685 / 1.084 | 29.75 / 0.57 | 0.019 | 116 次 | 6.000 s 纵向接管，7.638 s 停稳，12.400 s 新 epoch 恢复 |
| [异步通道与 stale yaw](plots/asynchronous_safety.png) | 1.060 / 1.836 | 5.18 / 1.00 | 0.018 | 67 次 | 6.000 s 接管，8.204 s 停稳，11.600 s 恢复 |

曲线场景 360 个 yaw 包全部接收并应用，同时仅有 4 次纵向 FULL；`beta` 最终约 -1°，距离接近 d2=1.1 m。折返场景的径向估计从负值转正，`v_cmd` 最小始终为 0；最近点约 9.20 s，此附近有 725 个 2 ms tick 满足 `|v_ref|≤0.03 m/s` 且 `|w_ref|≥0.02 rad/s`。纵向刹车 / 停车期间仍有 3200 个 tick 使用非零 yaw，之后新 epoch 的 catch-up 恢复。没有专门的掉头状态。此场景的 0.685 m 最近距离低于 d2：Master 朝车靠近时，非倒车 V1 无法保证始终保持 d2，必须把它视为已观察到的边界，不能称为距离安全保证。

异步场景中，yaw 停发而 heartbeat 继续；MCU 在 0.2 s freshness 到期后只把 yaw 目标平滑回零，没有触发 hard stop。yaw 恢复后，旧纵向 epoch 包被拒收 1 次；纵向从新 epoch 的未来 block 显式恢复。停车期间非零 yaw 有 2064 个 tick。两次 safety 接管首 tick 的纵向 `v/a/theta/theta_dot/u_ff` 引用跳变均为 0；恢复首 tick 的 `p_ref` 跳变分别约 `-3.4e-8 m` 和 `5.1e-8 m`，`v_ref` 跳变均为 0。

三组均无跌倒、轮端扭矩饱和、总和命令饱和、reference underrun；没有负 `v_cmd`。每组各保存一张主响应图，以及完整控制历史、Governor、yaw 生成 / 接收、FULL、MCU 和 transport CSV。详见 [结果 JSON](stage6_4_2d_results.json)。

## 时序与限制

记录链为 observation source → yaw 生成 → MCU 接收 → 应用 tick，MotionIntent → FULL planning 起止 → block 生成 / 接收 → 执行 tick，以及 safety issued → received → authority switch。三个场景的 yaw source→应用最大约 **1 ms**；稳定 20 Hz yaw 接收间隔相对 50 ms 的最大偏差 **2 ms**。FULL 请求→执行为 **50–98 ms**；reference 接收时距执行起点最大 **96 ms**（含恢复预留）；safety issued→接管在此零 transport latency 模型下为同 tick **0 ms**。异步场景 yaw 停发期的最大 MCU 接收年龄为 **1.548 s**，超时后使用本地回零。三组的 reference packet 发送→接收最高约 10 ms，包括 FULL 计算预留，并非纯链路耗时；yaw / safety 的模拟 transport 耗时为 0。没有注入随机网络抖动，以上数值不能外推到真实 Pi、相机和 STM32。

曲线场景只到约 5.5° bearing；折返场景达到约 29.8°，由持续 yaw 跟踪避免了更大视线误差。Synthetic observation 始终标为 valid，本轮没有 FOV 丢失 / 重捕获，也没有验证真实相机、异步硬件时钟或执行器故障。`HARD_STOP` 的 yaw 撤权和 fresh-yaw 恢复门槛由定向单测覆盖，未增加第四个闭环场景。异步场景末端距离仍为 1.836 m，因此不能声称其在 19 s 截止时已重新收敛到 d2。

验证命令：`python -m unittest tests.test_stage6_packet_safety tests.test_stage6_2d_split_authority tests.test_follow_governor_wait -q`（14 项通过）；`python scripts/run_stage6_4_2d_follow.py`（三组完整闭环）。实际执行使用仓库 `.venv` 的 Python。未 commit、push，也未调整 Stage 3/4/5 frozen baseline。
