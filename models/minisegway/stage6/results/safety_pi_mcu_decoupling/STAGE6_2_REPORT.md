# Stage 6.2：Safety + Pi/MCU 解耦简报

本轮基于当前本地 Stage 6.1 工作树，保留其 governor、动态制动、all-FULL 和 Stage 3/4/5 冻结参数。正常 FULL 继续按 quiet → fade → exit 完成；MCU safety 独立覆盖正常 reference。本轮没有实现 FULL splice、block 撤销或自动恢复。

## 四个压力场景

| 场景 | 正常命令 / FULL | 最小、最终相对距离 | Safety 结果 | 跌倒 / 轮饱和 / 未处理 underrun |
|---|---:|---:|---|---:|
| 双重追赶重触发 | 4 / 3 | 1.073 / 1.161 m | 两次 pending 更新，最终执行最新目标 | 0 / 0 / 0 |
| 低频走停走 | 4 / 4 | **0.503 / 0.503 m** | 无 pending；跟随距离失败 | 0 / 0 / 0 |
| Pi 前方障碍 | 1 / 1 | 1.508 / 1.655 m | 0.800 s 接管，5.742 s 进入 SAFE_HOLD | 0 / 0 / 0 |
| Pi/transport 失联 | 1 / 1 | 1.519 / 1.665 m | 0.838 s 本地接管，5.734 s 进入 SAFE_HOLD | 0 / 0 / 0 |

四场景均无 false SLOPE。配置和结果留在 Stage 6 目录；主结果仅各有一张响应图。第一次追赶试跑没有产生第二次 pending 替换，其数据保存在 `initial_case1_trial/`，没有被结果覆盖或当成成功。

## 关键结论

- **Retrigger 与 latest-wins：**FULL #1 从 2.600 s 开始。3.000 s 和 3.550 s 两次重触发先后提出 0.518、0.600 m/s；0.518 被覆盖，没有成为独立 FULL。FULL #1 于 4.046 s 退出，最新 0.600 m/s 在 4.050 s 才被接受，FULL #2 于 4.100 s 开始，最新 pending 等待 0.550 s。命令受原 0.6 m/s 限幅；最大相对距离 1.860 m。没有抢占或队列积压。Pi 端依据已生成 block 的预计 FULL 退出时刻和当前 observation 时间延后接受；这仍依赖 PC 仿真的统一时钟，真实双处理器应改用 MCU 确认的执行进度。
- **低频走停走：**四个稀疏事件均完整走 FULL，未出现 20 Hz retarget 或 pending。最后 Master 停止时 governor 已处于 CRUISE，继续锁存约 0.123 m/s，最终距离降至 0.503 m。因本轮锁定正常 follower 语义，此失败未通过调整场景或改 governor 掩盖；现有逻辑还不能安全覆盖这类停走序列。
- **Pi 安全命令：**OBSTACLE 包于 0.800 s 发出并在同一仿真控制 tick 接管 MCU；没有等待 FULL exit，也没有新 FULL 规划。旧 ACTIVE/NEXT 仍在 buffer，但不再有控制权。
- **Pi 失联：**注入失联时刻为 0.800 s。MCU 在 0.838 s 因 `REFERENCE_STARVATION_IMMINENT` 自主 BRAKING，距当前 ACTIVE 耗尽尚有 12 ms；未发生未处理 reference underrun。心跳超时监测也已接入，但本次由更早的 starvation guard 触发。两次 safety 接管均把旧 epoch 0 作废，保留新 epoch 1；本轮不恢复 NORMAL。
- **接管连续性：**两次 safety 的首样本相对前一正常样本，`Δv_ref / Δa_ref / Δtheta_ref / Δtheta_dot_ref / Δu_ff` 均为 0。随后本地制动的最大 `|a_ref|` 为 0.5 m/s²、最大 `|Δa_ref/Δt|` 为 0.8 m/s³；最终 SAFE_HOLD 的速度、加速度、hidden reference 和 feedforward 均为零。这里的 0 ms Pi BRAKE 延迟仅表示零 transport latency 的同 tick PC 仿真，不是硬件时延承诺。

## Packet 合同和仍存在的同进程依赖

六种 packet 均为 frozen DTO，跨端传输经过 bytes 编码/解码：TargetObservationPacket、RobotStatePacket、ReferenceBlockPacket、SafetyCommandPacket、HeartbeatPacket、SafetyStatusPacket。ReferenceBlock 采用 little-endian header 和按 `p/v/a/theta/theta_dot/u_ff` 排列的 float32 samples；包含协议版本、epoch、序号、控制 tick、生成时间、dt、样本数和 block yaw rate。MCU receiver 只读 packet 与本地估计，不持有 FollowGovernor、Pi source、TargetObservation 或 `_commands_by_sequence`。Pi 的 Master 速度估计只从解码后的 RobotStatePacket 更新。SafetyCommand 与 heartbeat 使用独立 channel；transport 仅搬运或按注入规则丢弃 bytes。

仍有两处仿真级共享：`Stage6PacketLink` 在同一个 Python 进程里按一个 MuJoCo 仿真时钟依次调度 Pi 和 MCU endpoint；它没有模拟独立 MCU 线程的真实 wall-time deadline，Pi 的 FULL 退出门槛也依据这个统一时钟。合成传感器在感知边界内部读取 MuJoCo 位姿来生成相对观测，GT 只进事后诊断。Pi crash 后该 endpoint 不再发送或接收 packet，MCU 继续独立制动；真实硬件还需要时钟对齐、完整性校验和通信驱动。本轮没有把这些扩成新架构。

正常规划沿用 [Ruckig](https://docs.ruckig.com/example_05.html)；MCU 安全制动是固定 2 ms 的限加速度/限 jerk 局部原语，未调用 Ruckig、FULL planner 或优化器。通信看门狗采用 [Zephyr task watchdog](https://docs.zephyrproject.org/latest/services/task_wdt/index.html) 所示的周期性存活检测思路；这些外部实现均未复制进仓库。

## 验证

- 四个闭环压力场景均完成；冻结 baseline 检查全通过。
- `python -m unittest discover -s tests -p test_stage6_packet_safety.py -v`：5 个 packet/接管不变量测试通过。
- 结果 JSON：`stage6_2_stress_results.json`；每场景的 history、governor、Pi intent、transport、MCU 事件 CSV 与四张主响应图保存在本目录。
