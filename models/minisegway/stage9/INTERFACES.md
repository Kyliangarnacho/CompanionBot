# ROS 与 Pi / MCU 职责边界

ROS 2 负责开发机/Pi 的通信与进程运行。当前七个 package 按 interfaces、perception、follow、control、behavior、simulation、bringup 划分；核心算法位于对应 package 的 core/runtime，ROS wrapper 位于 ros，launch 统一位于 bringup。`companionbot_ros` 已归档，不在 active workspace。依赖方向、完整目录和旧路径迁移见 [ARCHITECTURE](ARCHITECTURE.md)。本次整理不改变以下数据、时间、frame 或授权契约。

## 职责与频率

| 边界 | 当前实现 / 更新 | Pi + STM32 方向 | 本阶段处理 |
| --- | --- | --- | --- |
| 感知、ReID、depth、geometry | 唯一 Camera owner，两个 latest-frame worker；频率由真实吞吐决定 | Pi/Linux 进程 | 原 worker 与状态机整体保留；ROS 从现有 observer tap 发布 |
| Agent / Catalog / Session / BehaviorSupervisor | 异步 task、输入 gate、0.5 s feedback/state poll、取消 | Pi 高层，Nav backend 后续由 Nav2 adapter 实现 | 一个 interaction node 包住原正式入口；Fake execution 标记保留 |
| radial KF / state-history / Follow Governor / bearing yaw servo | Stage 6 合成观测 20 Hz，source-time 对齐 | Pi 上层估计与跟随意图 | 原 Python / DTO / codecs 保留；不订阅真实 Stage 7 观测驱动它 |
| scheduler / FULL / LIGHTWEIGHT / reference producer | 稀疏 intent 生命周期；FULL cached-KKT 投影采样 250 Hz（不是 FULL 重规划频率）；25×2 ms block，即 50 ms | Pi 数值规划，MCU 只消费现成 reference | 不把每次 projection 或每个 sample 拆成 ROS node |
| reference receiver / yaw freshness / safety arbiter / heartbeat / resume | 2 ms tick；ACTIVE/NEXT、reference epoch、local braking 与状态包 | STM32 控制时间域，独立于高层执行是否阻塞 | 既有 packet authority / invalidation 不变；不把 DDS 当 500 Hz actuator transport |
| sensorized estimator / nominal LQR / yaw PD / allocator | 500 Hz，requested / clipped / actual 分离 | STM32 或同等实时执行侧 | 不改 K、Q/R、estimator、torque limits，不重写语言 |
| Stage 4 EKF / FLAT-SLOPE / payload lifecycle | control-time context；EKF 内部 100 Hz、payload ID 50 Hz，少量 session batch 计算 | 状态采样、mode/安全在控制时间域；batch 参数求解是否上移 Pi 需实测截止期与 atomic model-update 契约后再决定 | 现在保持已有组合 runtime；不提前搬走计算或新增参数写入 service |
| MuJoCo / synthetic sensor / GT / in-memory transport / viewer | physics 1 kHz；虚拟双方共享 sim clock | plant 和 GT 仅属仿真；真实 MCU 不应链接 MuJoCo/GT | 保留仿真组合调用链；ROS 只取 estimator/actual-command API 遥测 |

Stage 5/6 的 Pi / MCU 虽同处一个 Python 进程，已经用 packet bytes、reference epoch、receiver/safety 对象和传感状态边界表达职责。当前 producer 属于 follow/runtime，receiver/safety/latest-yaw 属于 control/runtime；in-memory transport 和 `before_control_tick` / physics callbacks 属于 simulation/runtime。这些模拟调度方法不要求 Pi/Python/ROS 在实机同步运行 500/1000 Hz，也不是硬件实时性承诺。MCU C/C++ 移植、驱动、传感器/电机 envelope 和 measured clock adapter 留待实机阶段。

ROS `frozen_sim` 是可选有限诊断 node，控制 worker 仍调用原 Stage 3 runner，并重用 Stage 5 已有 frozen-config resolver。ROS executor 不推进物理或控制 tick；只以 50 Hz 发布选定遥测与 `/clock`。GT 留在旧验收 runner 的 post-hoc history，不出现在 ROS controller inputs 中。

## 时间契约

`SourceStamp` 显式传递 contract version、source ID、source epoch、sequence/是否存在、source raw time/是否存在、clock domain + clock ID、time semantics，以及本 adapter 的 receiver perf time/clock ID和可选 producer-ready time/clock ID。ROS Header.stamp 是 ROS publication time；不替换 raw source time。

| 数据 | source time / clock | 含义 |
| --- | --- | --- |
| ColorFrame / Master | `host_read_complete` / `host_perf_counter` | 原 Camera 读帧完成时刻；不是曝光时刻，视频也不假装 media PTS |
| RelativeObservation | 同 source ID/sequence/time，另有 geometry result-ready time、source-time pitch basis/time | same-frame bbox-depth 配对；原时间与 provisional 信息完整保留 |
| RobotSnapshot / BehaviorFeedback | 原 DTO 的 version/source/epoch/sequence/time/domain | Fake 来源不能包装成硬件来源；旧缺省字段明确 has=false |
| ControlTelemetry | `control_sample` / `mujoco_sim` / `sim:<runtime epoch>` | 原控制采样时刻；receiver host time 独立；Header 默认系统 ROS time |
| PiMcuPacket | 当前只支持 `sim_packet_generated` / `mujoco_sim` | 保留既有 binary packet bytes 与其中 capture/generated time、control tick、reference epoch；不把它 retime 到 host |

本机 `clock_id=linux-boot:<boot_id>` 表示同一个 Linux kernel 的 Python perf_counter 实例；它只支撑同机的 raw monotonic 比较，不表示跨 Pi/MCU 可直接对齐。未知 clock domain 不自动分配 host clock ID。

`interaction/input` 当前只接受同 Linux boot clock、`text_submitted`、version 2、完整 source/epoch/sequence，年龄 `[0,1] s` 且有限的输入；duplicate、倒退、future/stale、未知时钟和 source/epoch 更换均拒绝，拒绝不推进 watermark。首个有效 source epoch 绑定到该运行，重启客户端 epoch 需要重新启动/rebind runtime；本阶段没有伪造跨机 offset。queued 只表示已进原 Session 队列，不表示唤醒成功、Supervisor 授权或实际执行。

ROS camera/perception epoch 是当前 publication owner 的运行 epoch；source sequence 保留原采集编号，不改成 ROS send count。Robot/Feedback 使用各自现有 backend epoch。ROS publisher enqueue 的 receipt time 与远端 DDS 实际到达不是一回事；消费者必须另外记录自己 arrival time，然后在确证的 clock mapping 下检查 source freshness。当前没有运行中的 perception→controller consumer。

`/clock` 只发布本次 MuJoCo sim time 的 50 Hz 采样，不改变 camera / ASR / Agent deadlines。`use_sim_time` 是 ROS clock 参数，不能作为 MCU/camera/host 时钟映射。进程重启、相机重新枚举和 MCU reboot 必须分别处理 source epoch / clock ID；reference epoch 是既有纵向 reference authority，不能与 source epoch 合并。

## Topics / services

以下均在默认 `/companionbot` namespace；没有 `cmd_vel`、地图 goal 或直接轮端 subscriber。camera/perception 的 publisher adapter 属于 perception，复用 behavior interaction 的同一个 node/context；robot/behavior/input 属于 behavior；sim/clock 属于 simulation。库包 control/follow/interfaces 不为此新增 node。

| 名称 | 类型 | QoS / authority |
| --- | --- | --- |
| `camera/color` | ColorFrame，内含 BGR8 Image | sensor_data best effort；一次正常 publication，不参与 Agent 取帧队列 |
| `perception/master` | MasterStatus | sensor_data；visible/state/source metadata；不授权 actuator |
| `perception/relative` | RelativeObservation | sensor_data；leveled axle point、pitch、外参/depth provisional、`stage6_connected=false` |
| `robot/state` | RobotSnapshot | reliable depth 10；原 Fake 状态，不包含伪造 pose；`hardware_execution_ready=false` |
| `behavior/feedback` | BehaviorFeedback | reliable depth 32；status/reason/task/intent/destination/input/turn/request/progress |
| `interaction/events` | std_msgs/String JSON | reliable depth 32；UI/诊断，执行授权以 typed feedback 和原 Supervisor 为准 |
| `sim/control` | ControlTelemetry | sensor_data；估计状态、allocator requested / actual wheels、saturation、1/2 ms timing；无 GT inputs |
| `/clock` | rosgraph_msgs/Clock | sensor_data；有限仿真诊断时间 |
| `interaction/input` | UserInput | 有 source guard 的异步文字入口；响应仅 queued / reason |
| `interaction/cancel`, `interaction/stop` | std_srvs/Trigger | 投递原本地 `/cancel`、`/stop`；返回 queued，未确认实体停止 |

反馈语义沿用 Stage 8：ACCEPT ≠ RUNNING ≠ COMPLETE，CANCEL_REQUESTED ≠ CANCEL；UNKNOWN、timeout、task ID 和迟到 feedback 校验仍由原 Supervisor / Backend contract 处理。ROS 不在收到 service response 时虚构执行完成，也不把模型文本当轮端指令。

`PiMcuPacket` 配套 `encode_packet/decode_packet` 为诊断/未来 measured transport 提供现成 byte envelope，reference/yaw/heartbeat/safety/status/state/observation/resume codecs 原样使用。默认不创建控制 topic，不 dispatch 到 MCU；当前 decoder 要求匹配 simulation clock/epoch、channel/direction/sequence，hardware authority 为 false。下一阶段若接真实 UART/USB，另加经过测量的 device-clock/time adapter，不能直接拿这个 sim envelope 当真实 transport。ROS version 2 envelope 与现有 binary protocol version 1 有各自含义。

## Frames / TF / 导航

- `base_link`：本阶段约定原 body 轴，原点轮轴中点，+X 前 / +Y 左 / +Z 上；不是 localization pose。
- `camera_optical_frame`：+X 右 / +Y 下 / +Z 前；已有 `P_body=R_body_camera P_camera+t_body_camera`。
- `robot_leveled_axle`：同一轮轴原点、保留机器人 heading 的 source-time pitch 去倾斜观测 frame；不等于 odom 或 map。
- camera static TF 默认关闭。显式开启才从原 provisional mount 配置广播 `base_link → camera_optical_frame`，日志指出未实测。未发布 leveled dynamic TF，因为没有实测 IMU/pitch history。
- 不发布 `map → odom`、`odom → base_link` 或里程计/地图 pose；没有 SLAM 就不产生 map 定位。

下一阶段 Nav2 应包装现有 `NavigationBackend`，在确定 map/odom/actual odometry、destination→map pose、时钟和 frame 后将 real task feedback 接回 Supervisor；Catalog 的 destination ID 与用户/Agent 请求身份仍保持。Stage 7→6 则另需曝光/receipt latency、pitch clock/history、stale/LOST 与几何精度验证。此处只有明确的接入边界，没有声称这两条真实闭环已完成。
