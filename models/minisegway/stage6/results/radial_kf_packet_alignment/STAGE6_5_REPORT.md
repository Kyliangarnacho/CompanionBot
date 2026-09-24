# Stage 6.5：径向速度 KF 与 Pi/MCU 采集时间对齐

本轮基于现有 Stage 6.x 本地工作树，未改变 Stage 3/4/5 冻结参数、FOLLOW 阈值、FULL、yaw 或 safety 策略。标准模型采用二状态线性恒速 KF；`P` 用 Joseph 形式更新以维持数值对称性。参考：[FilterPy 线性 Kalman predict/update 文档](https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html)。项目自定义部分仅是径向 ego 输入、packet 边界、状态包时间插值和丢帧语义。

## 数据链

```text
synthetic Camera bridge / future real camera
  → TargetObservationPacket(x_forward, y_left, capture_time, seq)
  → Pi observation receiver
  → MCU RobotStatePacket history at capture_time (linear interpolation)
  → d_meas=hypot(x,y), beta=atan2(y,x), u=v_robot(capture_time)*cos(beta)
  → RadialVelocityKalmanFilter[d_hat, v_master_radial_hat]
  → existing FollowGovernor → unchanged FULL / yaw / safety
```

MCU 估计速度仍先编码为 `RobotStatePacket`，经过现有 memory transport 到 Pi，再进入 2 s 短历史缓冲。Pi 的 KF 路径只读解码后的两个 packet 类型，不访问 MuJoCo 或 MCU controller 对象；相机到达时若状态历史不能覆盖 `capture_time`，记录 `STATE_HISTORY_MISS` 并丢弃测量，绝不以最新速度顶替。第一帧在仿真启动时预填，之后统一加入 60 ms 图像处理延迟；时间戳压力场景为 80 ms 加确定性 ±8 ms 抖动。`confidence` 保留于包中，`R` 固定在 Pi 本地配置，没有 adaptive-R。

滤波状态为 `[d, v_master_radial]`，预测使用相邻 **capture_time** 的实际 `dt` 和 `-v_robot*cos(beta)*dt`。本轮四个可填参数为初始距离标准差 **0.05 m**、初始速度标准差 **0.6 m/s**、测距标准差 **0.03 m**、Master 加速度标准差 **0.8 m/s²**；同一配置用于全部场景。第一帧令 `d0=d_meas`、`v0=0`。Governor 的距离和 Master 径向速度来自 KF，closing speed 来自 `v_master_radial_hat-v_robot_aligned*cos(beta)`；raw 差分仅记日志并驱动不接 actuator 的诊断 shadow Governor。

## 闭环结果

| 场景 | KF / raw 速度 RMSE m/s | KF / raw 峰值绝对误差 m/s | KF / raw 诊断 Governor 事件 | 跌倒 / 饱和 / underrun |
|---|---:|---:|---:|---|
| [无噪声回归](plots/noise_free_regression.png) | 0.029 / 0.015 | 0.180 / 0.136 | 4 / 4 | 0 / 0 / 0 |
| [带噪转弯](plots/noisy_turning.png) | 0.056 / 0.830 | 0.331 / 2.465 | 7 / 46 | 0 / 0 / 0 |
| [停车再起步，无噪声](plots/stop_restart_clean.png) | 0.052 / 0.027 | 0.333 / 0.326 | 5 / 4 | 0 / 0 / 0 |
| [停车再起步，带噪](plots/stop_restart_noisy.png) | 0.072 / 0.843 | 0.364 / 3.176 | 5 / 58 | 0 / 0 / 0 |
| [延迟与抖动包](plots/delayed_jittered_packets.png) | 0.056 / 0.829 | 0.331 / 2.465 | 6 / 38 | 0 / 0 / 0 |

无噪声曲线回归仍为 4 个稀疏 Governor 事件，最终 KF 距离 **1.047 m**，原 Stage 6.4 无噪声 baseline 为 **1.063 m**；未出现 fall、饱和或 underrun。KF 在无噪声条件下的速度 RMSE 高于 raw，这体现了响应滞后，结果保留。

带噪曲线的 Master 恒速转弯窗口（4–7 s，机器人 `|yaw_rate_hat|` 峰值 0.124 rad/s）中，KF/raw 径向速度 RMSE 为 **0.039/0.881 m/s**。Master 停止且机器人仍转向的 8.5–11.5 s 窗口，带噪结果为 **0.037/0.785 m/s**，机器人 `|yaw_rate_hat|` 峰值 0.112 rad/s。相对于对应无噪声运行，带噪曲线的事件数多 **3/42**（KF/raw shadow），带噪停车再起步多 **0/54**。这是“噪声诱发事件”计数代理：噪声也会改变闭环轨迹，不能把每个额外事件无条件判成物理误触发。raw shadow 从未进入 FULL 或 actuator。

## 变速响应与时间对齐

Master 在 8.0 s 停止后，以连续三帧 `|v_master_radial_hat|≤0.05 m/s` 为准：无噪声 KF **0.45 s**、raw **0.05 s**；带噪 KF **0.35 s**，raw 未能在 12 s 再起步前连续三帧满足此条件。无噪声 KF Governor 在停止后 **0.75 s** 发零速命令，零速 FULL 在 **0.90 s** 开始。带噪运行分别为 **0.45 s** 和 **2.05 s**：后一个长间隔主要来自已有 active FULL 的 deferred latest-wins 语义，不应归因于 KF 单独滞后。本轮没有调 Governor 或抢占 FULL。

每组首帧精确对齐，其余分别有 **358**（18 s）或 **458**（23 s）帧通过相邻 RobotState 包插值；全部运行 `STATE_HISTORY_MISS=0`、乱序拒绝=0。时间戳压力场景的 observation 到达年龄最大 **89 ms**，KF RMSE 与普通带噪场景几乎相同（0.0564 vs 0.0563 m/s）；若错误地使用到达时最新 RobotState 包，样本速度与正确插值结果的最大差异约 **0.054 m/s**。每条估计日志保留采集/到达时间、左右状态包时间与序号、插值权重、`d_meas`、`beta`、对齐速度、KF 状态、raw 诊断、innovation 及 Governor 输出/事件。详见 [结果 JSON](stage6_5_results.json) 与各场景 `*_estimator.csv`。

## 边界与验证

GT Master 径向速度仅在闭环结束后用于 RMSE 和绘图；运行结果标记 `GT_runtime_dependency=false`。Synthetic sensor 的噪声是在统一 body-frame `(x_forward,y_left)` 上注入，未实现相机检测、深度或 FOV 丢失。80 ms 延迟及抖动是单进程确定性 transport 测试，不能代表真实 Pi/STM32 带宽、时钟同步或视觉推理时延。KF 的停车检测代价和 FULL 锁定造成的额外执行等待仍是下一步实机前需要评估的风险，本轮未加第二层滤波。

定向单测 **17 项通过**；五组闭环全部完成。没有 commit、push 或改动 frozen baseline。
