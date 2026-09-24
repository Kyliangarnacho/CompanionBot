# Stage 6.1 — 纵向相对目标跟随

状态：**已实现并完成五个场景**。Stage 3/4/5 冻结的控制器、估计器、plant 和规划器参数均未重新调节。

## 实现内容

- `TargetObservation` 增加了版本号、序列号、有效标记和置信度，并保留采集时间及机身坐标系下的 `(x_forward, y_left)`。原有只传三个字段的构造方式仍然有效。
- `SyntheticTargetSensor` 在仿真边界内按预设轨迹驱动 Master，并只发布机身坐标系下的相对观测包。
- `FollowGovernor` 使用 `v_robot_hat + Δx_forward / Δt` 估计 Master 纵向速度，不做平滑。当距离 `> d1` 时锁存一次追赶速度；距离 `<= d2` 时锁存当前 Master 速度作为巡航速度。
- Stage 3B 增加可选回调，只向 reference producer 提供现有的纵向速度估计。原有调用不设置此回调，行为不变。
- Stage 5 scheduler gate 按 reference source 显式启用。FOLLOW 关闭候选稳定窗口要求和两处最小 Δv 检查；其他调用保留原默认值。冻结的 LIGHTWEIGHT/FULL 阈值没有改变。
- 首轮完整运行发现后续还有一处最小 Δv 检查会拒绝较小命令。现在该检查也遵从 FOLLOW 选项。首轮失败及其 CSV/图表保存在 `pre_bypass_fix/`。

## 技术选型简评

- **Adapt：**借鉴 [TurtleBot 4 follow controller](https://github.com/Hp092/human-following-robot/blob/main/person_follower/follow_controller.py) 中由距离和方位生成有界机身速度的结构。本阶段只实现项目所需的纵向锁存逻辑，yaw 保持为零。
- **Borrow：**借鉴 [Unitree Go2 follow controller](https://github.com/orisharabi/unitree-go2-follow-system/blob/main/follow_controller.py) 将距离与角度控制通道分开的做法；本阶段只采用其分离思路。
- **Reject（本阶段）：**[D-Robotics person-following stack](https://github.com/D-Robotics/tros_person_following) 包含目标跟踪、Nav2、搜索/重获和目标管理，超出当前 synthetic relative-target 接口范围。
- **复杂度门槛：**当前基线使用 Master 速度有限差分和 `d1/d2` 锁存。已测得的停车超程可作为后续专门研究制动的依据；本阶段未增加滤波、预测或 supervisor。

## Synthetic 观测与数据边界

观测包字段为 `version`、`sequence_id`、`capture_time_s`、`x_forward_m`、`y_left_m`、`valid` 和 `confidence`。Stage 6.1 控制逻辑只读取采集时间和机身相对坐标；测试场景中 `y_left_m` 为零，yaw rate 始终为零。Synthetic 观测为理想观测，置信度为 1.0。

仿真传感器内部读取机器人位姿以生成相对观测。另一路诊断数据在测量边界之外记录 `x_forward_GT_m` 和 Master GT 速度，仅用于评估；这些值不会传入 governor 或 controller。runner 只向 governor 传入当前观测包和现有的基于传感器状态得到的机器人纵向速度估计。

## 配置与场景结果

历史配置：`models/minisegway/stage6/reference/stage6_1/config/stage6_1_longitudinal_follow_config.json`。`d_target = d2 = 1.1 m`，`d1 = 1.4 m`，`T_catch = 3.0 s`，速度限幅为 `±0.6 m/s`。

| 场景 | Governor 事件与 Stage 5 路径 | 最终距离 | Master 速度估计 RMSE | 结果 |
|---|---|---:|---:|---|
| 目标静止且已在目标距离 | 无命令；无重规划 | 1.119 m | 0.0029 m/s | 保持零命令；无跌倒或饱和 |
| 目标静止，初始距离 2.3 m | 0.00 s 追赶命令 `0.399 m/s`；3.70 s 巡航命令 `0.0003 m/s`；两次 FULL，均退出 | 0.837 m | 0.0125 m/s | 到达 `d2` 触发点后，在停止参考执行期间继续超程 |
| Master 匀速 0.20 m/s | 1.65 s 追赶命令 `0.303 m/s`；7.00 s 巡航命令 `0.2003 m/s`；两次 LIGHTWEIGHT | 1.054 m | 0.0039 m/s | `0.103 m/s` 的命令变化立即被接受；接近 `d2` |
| Master 以 0.20 m/s 移动，并在 2.3 s 停止 | 1.65 s 追赶命令 `0.303 m/s`；4.10 s 巡航命令 `0.0003 m/s`；两次 LIGHTWEIGHT | 0.847 m | 0.0138 m/s | Master 停止后机器人仍在移动；接近 `d2` 后发出零命令，并在减速期间超程 |
| Master 速度按 0 → 0.15 → 0.25 → 0.15 → 0 变化 | 2.55 s 追赶命令 `0.252 m/s`；8.45 s 巡航命令 `0.0008 m/s`；两次 LIGHTWEIGHT | 1.004 m | 0.0163 m/s | 速度变化期间出现两次 governor 命令事件；未出现 20 Hz 命令抖动 |

五次运行均无跌倒、执行器饱和、false SLOPE 样本、reference underrun、stale block 或 sequence gap。场景 2、3、5 在块边界遇到重复观测包，governor 按序列号将其去重。每个场景的 CSV 和图表保存了完整历史、事后 GT、事件和 stream 数据。

## 尚存问题

Governor 在测得距离到达 `d2` 时触发停止命令，但 Stage 5 的 jerk-limited 速度参考和机器人本体仍需要时间减速。两个静止目标接近场景最终都进入 `d2` 内约 0.26 m；Master 中途停止场景最终进入约 0.25 m。这里观察到的是距离触发时机与 Stage 5 停止动态之间的交互。本阶段未改变参数或事件语义；提前制动或修改停止规则应作为后续单独假设验证。

## 验证与产物

- 五个配置的闭环场景均通过 Stage 5 V1.5 reference stream 运行。
- 直接 scheduler 检查确认：原默认 gate 仍要求稳定窗口和最小 Δv；FOLLOW 则立即接受 `0.100 m/s` 的变化，并仍选择 LIGHTWEIGHT。
- Python 编译与 `git diff --check` 通过。由于项目 `.venv` 未安装 pytest，无法运行 `pytest tests/test_rolling_reference.py`。
- 结果 JSON：`stage6_1_longitudinal_follow_results.json`。
- 各场景 history、governor observations/events、sensor evaluation CSV 和图表保存在结果目录及其 `plots/` 子目录中。
