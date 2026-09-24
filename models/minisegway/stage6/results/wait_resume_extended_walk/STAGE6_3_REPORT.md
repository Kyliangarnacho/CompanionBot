# Stage 6.3：稀疏减速 / 等人 + Safety 恢复协议

本轮沿用当前本地 Stage 6.2、Stage 5 all-FULL 与 Stage 3/4/5 冻结运行参数。唯一闭环实验为 36.5 s 扩展走停走；Master 速度全程非负，20 Hz 相对观测。主响应图：[extended walk-stop-walk](plots/stage6_3_extended_walk_stop_walk.png)。GT 速度只在事后绘图出现，不进入 Governor、planner 或 MCU。

## 关键事件

| 时间 s | Master GT m/s | Governor 事件 | 新 `v_cmd` m/s | 相对距离 m | FULL 开始 → 退出 s |
|---:|---:|---|---:|---:|---|
| 2.25 | 0.18 | 开始追赶 | 0.283 | 1.406 | 2.30 → 3.794 |
| 10.05 | 0.10 | 动态距离切回 CRUISE | 0.099 | 1.156 | 10.10 → 11.404 |
| 11.65 | 0.00 | 减速 / 等人 | **0.000** | 1.059 | 11.75 → 12.814 |
| 15.50 | 0.40 | 开始追赶 | 0.506 | 1.419 | 15.60 → 17.424 |
| 19.45 | 0.22 | 动态距离切回 CRUISE | 0.221 | 1.249 | 19.50 → 20.996 |
| 21.65 | 0.06 | 减速 / 等人 | 0.063 | 1.105 | 21.70 → 22.940 |
| 25.95 | 0.28 | 开始追赶 | 0.381 | 1.405 | 26.00 → 27.536 |
| 28.55 | 0.12 | 动态距离切回 CRUISE | 0.122 | 1.270 | 28.60 → 30.058 |
| 31.65 | 0.00 | 减速 / 等人 | **0.000** | 1.047 | 31.70 → 32.840 |

**结果：**3 次 `slowdown_wait`，3 次 `catch_up_started`，3 次原有动态距离 `cruise_resumed`；共 9 次稀疏命令事件 / 9 条完整 FULL，对应 730 次 20 Hz 观测。稳定行走期间没有 20 Hz command churn。两次 Master 停车后最终均下达零速命令。最终距离 **0.992 m**，最小 **0.989 m**，最大 **1.700 m**；没有上一轮 Master 已停、命令仍锁存约 0.12 m/s、距离持续跌到 0.503 m 的现象。此次是更长的新速度序列，未复跑旧 22 s 场景，因此不把两者当严格同场景 A/B。

距离仍有瞬态偏差：大幅重新起步后最大达到 1.700 m，末段稳定在 d2=1.1 m 内侧约 0.108 m。没有跌倒、轮扭矩饱和、stream underrun、false SLOPE 或 safety 误接管；Stage 3/4/5 冻结检查全部通过。该扩展序列没有在 active FULL 中产生新的命令，pending 次数为 0，故不能把它当作 latest-wins 的新闭环证据。已有 Stage 6.2 双重追赶场景保留了该证据；本轮的定向 scheduler 测试另验证了 0.20→0.10→0.00 在 FULL 锁定时只接受最后一个目标。

## Safety 协议补齐

- Pi 一旦发 BRAKE，立即停止旧 epoch 的 ReferenceBlock 生产；heartbeat 在 Pi 存活时仍继续。MCU 接管时清空 ACTIVE/NEXT、永久拒绝旧 epoch，并在 SafetyStatus 中回报 `invalidated_epoch`。
- MCU 接收缓冲固定为 ACTIVE/NEXT，transport 最多容纳两个未送达 reference 包；过期和超容量包丢弃。BRAKING 独立完成本地制动；SAFE_HOLD 的 heartbeat 丢失只记录 `PI_LOST` health，不重复制动。
- 恢复须由 Pi 明确确认危险解除，并持有 SAFE_HOLD feedback 与更新的 RobotState；Pi 提供从该状态准备的新 producer、未来起始的 epoch+1 block，并发送 Resume。MCU 只在 SAFE_HOLD、收到同 epoch Resume 和 fresh block，且起始 tick 尚在未来时武装恢复，到起始 tick 才回到 NORMAL。Pi 收到 NORMAL feedback 后立即生产后继 block，以维持 ACTIVE/NEXT。单独的新 block 不会解除 hold。协议由定向测试验证；本轮没有自动危险解除检测，也没有另跑 safety 恢复闭环场景。当前 packet 仅能校验新轨迹的 epoch、时间和 block 几何，不能仅凭 RobotState 的有限字段证明轨迹全状态连续。

最初同一扩展场景试跑使用 0.03 m/s deadband，留下 0.0027 m/s 停车尾值和数次微小二次减速；原始 JSON 已归档到 `../../reference/stage6_3_deadband_trial/initial_governor_trial.json`。最终只把单个速度 deadband 定为 0.05 m/s，并把小于该值的 CRUISE 命令归零，没有增加距离门槛、FULL 抢占或额外规划器。
