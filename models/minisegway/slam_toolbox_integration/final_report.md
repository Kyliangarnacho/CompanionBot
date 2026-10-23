# SLAM Toolbox 建图与机器人地图系统集成 final_report

完成日期：2026-10-22。代码与原始证据以 `/home/kylian/projects/CompanionBot` 为准。正式交互入口保持 `scripts/demo_stage8_interaction.py`；没有 commit/push、增加 remote、GUI 自动操作或实体执行声明。

## 已完成的功能

ROS 2 Jazzy 原版 SLAM Toolbox 已接入实际 MuJoCo 自平衡运动、encoder/IMU 估计 odom、保留物理 pitch 的 TF、VirtualPlanarLidar 与同一 `/clock`。机器人通过原规划器、Pi ReferenceBlock、包传输、MCU safety、冻结 LQR/yaw PD/allocator 运动；没有设置 GT 位姿、发布假传感器、重写 SLAM、增加转发 Node 或改造控制核心。

具备无 GUI 双圈室内建图、真实跨圈约束/图优化、标准 OccupancyGrid、YAML/PGM 与 `.posegraph/.data` 保存、标准 map_server 检查、pose graph 重新加载、运行异常传递和正常退出。RViz 配置提供 RobotModel / LaserScan / TF / Odometry / OccupancyGrid，由用户手动打开；未声称 GUI 人工观感通过。

直接运行：先 `source scripts/env.sh`、`bash scripts/build_ros.sh`、再次 source，然后 `ros2 launch companionbot_bringup slam_simulation.launch.py output_dir:=<本工程绝对成果路径>`。默认 82 s 有限运动后保留图供保存，Ctrl+C 关闭。保存使用 `bash scripts/save_slam_map.sh <绝对路径/map_basename>`；完整操作、接口、加载前提与人工 RViz 命令在 `docs/SLAM_MAPPING.md`。

## 验收平台与固定条件

- WSL2 Ubuntu 24.04 / Linux `6.18.33.2-microsoft-standard-WSL2`，x86_64，AMD Ryzen 9 7945HX，WSL 可见 8 CPU；不是 Pi 5 或 STM32 实测。
- `.venv/bin/python` 为项目 Python 3.12.3。Jazzy `slam_toolbox 2.8.5-1noble.20260615.161600`；map_server / lifecycle_manager 1.3.12 仅作地图工具，未启动 Nav2 导航或 AMCL。
- 原 plant、质量/摩擦/扭矩、Q-OFF、LQR/yaw/MCU 参数、1 ms physics / 2 ms control、ReferenceBlock 和原安全期限保持。原房间 x∈[−2,6]、y∈[−4,4] m 与箱体不变。
- 当前 LiDAR：原挂载、360 束、10 Hz、0.05–8 m、5 mm 测距噪声、固定 seed、瞬时扫描；没有姿态补偿、过滤、点云或地面投影。
- `acceptance.json` 在完整建图前固定门限：几何 P95≤0.15 m、10 cm 内 occupied precision≥90%、四墙 coverage≥85%、known interior≥80%、位置 RMSE≤0.15 m、yaw RMSE≤0.10 rad、路径≥15 m、xy span 各≥2 m、真实图回环与原控制门禁。

## 地图与定位质量

最终两个 run 均为同一 82 s 轨迹、21.7685 m 真实物理路径，xy span 3.3866×2.9996 m、累计 yaw 12.5842 rad；pitch 峰值 7.7033°。GT 只由单向 post-hoc 文件送离线工具，运行模块不能读取；误差仅用首帧原点/航向对齐，没有轨迹或地图 best fit。

| 指标 | final_mapping | final_repeat_dropout |
| --- | ---: | ---: |
| 主动扫描丢帧 | 0 | 40.0–40.5 s，5 帧 |
| OccupancyGrid | 161×161，5 cm | 161×161，5 cm |
| occupied cells | 973 | 956 |
| occupied geometry P95 | 0.059748 m | 0.059685 m |
| occupied precision（≤10 cm） | 100% | 100% |
| 四面室内墙 coverage（≤10 cm） | 100% | 100% |
| known interior | 98.944% | 98.944% |
| SLAM position RMSE | 0.011092 m | 0.010301 m |
| SLAM position peak | 0.043590 m | 0.042096 m |
| SLAM yaw RMSE | 0.001192 rad | 0.001274 rad |
| graph nodes / edges | 125 / 208 | 124 / 206 |
| 跨圈图边（source time 差>20 s） | 84 | 83 |
| 历史节点优化修正 peak | 0.012358 m | 0.012358 m |
| 原冻结控制门禁 / MCU authority | 全通过 / NORMAL | 全通过 / NORMAL |

回环依据为发行包 Karto API 直接反序列化得到的跨圈约束，加上早期节点从发布时位姿到最终优化位姿的实际修正。本次 fresh run 未调用 deserialize/manual closure；上游 2.8.5 的 TryCloseLoop→CorrectPoses 路径执行历史优化。84/83 是图约束数量，不冒称回环事件计数。

独立未校正 odom 在 normal run 的位置 RMSE 约 9.8 mm、yaw RMSE 约 0.0061 rad；本场景 odom 本来已很好，不能把接入 SLAM 宣称成所有位置指标都改善。导航地图、闭环和持久化是本轮工程成果。

## 数据有效性与资源

normal producer 实际发布 4100 state、4100 IMU、820 scan，probe 收到 4058 odom / 820 scan；820 帧中 812 可在保存的 TF 历史做 acquisition-time 查询（99.024%），启动前 0.1–0.8 s 的 8 帧返回 past extrapolation。没有回填历史或把 unavailable 算作覆盖。SLAM 的实际成功处理和图节点由它自己的 source-time filter 决定。

dropout producer 发布 815 scan，probe 收到 798（另有 best-effort / discovery 接收损失）；798 帧 TF 查询全部有覆盖。两次 producer 都是 state/scan superseded=0；接收差异保留在原始 JSON，不把 matched discovery 当作逐帧必达。

| 资源/速率（主机墙钟） | normal | dropout |
| --- | ---: | ---: |
| SLAM 平均 CPU，占一个核 | 6.08% | 5.97% |
| SLAM peak RSS | 58.24 MiB | 57.80 MiB |
| 整个 simulation 进程 peak RSS | 883.55 MiB | 958.43 MiB |
| spatial_state 平均 CPU / peak RSS | 9.31% / 108.89 MiB | 8.83% / 108.73 MiB |
| map wall frequency | 1.048 Hz | 1.057 Hz |
| accepted pose wall frequency | 1.730 Hz | 1.667 Hz |
| 同一 probe scan→pose arrival P95 | 12.80 ms | 11.14 ms |
| ReferenceBlock 生成 wall 最大值 | 0.864 ms | 1.071 ms |

这些是本次 WSL 进程采样值，含该平台调度/启动与 DDS 观察边界；scan→pose 是观察到的到达差，不是 WCET 或跨设备时间相减。不能拿它认证 Pi 5 算力、内存、硬实时或真实 LiDAR 的机械扫描延迟。

以 GT 姿态离线重建 ray endpoint、距平地≤2 cm 的几何判定，normal 有 4541/276794 个有限回波（1.6406%），单帧最多 105 个接近地面。它们确实保留在原始数据中；没有投影成墙或从日志排除。最终栅格通过不代表这些瞬态扫描满足严格水平测量假设。

## 运行、故障与回归测试

原始证据根：`models/minisegway/slam_toolbox_integration/results/session_20261022/`。`evidence_index.json` 区分最终通过、早期失败、诊断和回归；标准地图在 `final_mapping/maps/indoor.*` 与 `final_repeat_dropout/maps/indoor.*`。两个最终 run 各有 launch、参数输入快照/SHA、Graph/topic、TF/GID diagnostics、scan/odom/map/pose、arrival、CPU/RSS、truth_posthoc、control/history、地图、pose_graph、evaluation 和图表。

| 验证 | 实测结果 / 证据 |
| --- | --- |
| 完整 8-package build | `build_delivery.txt`，全部完成 |
| 全仓与各 owner pytest | `pytest_delivery.txt`：389 passed / 0 skipped；新增 mapping 中断后 GC 策略恢复和日志关闭 case |
| colcon | `colcon_delivery_result.txt`：417 tests / 0 errors / 0 failures / 0 skipped；含 CTest/pytest 外层，与389不可相加作为 coverage |
| 原 native baseline / radial / safety / dynamic / stream | 五套全部通过；199300 仲裁 tick 与原 Python mirror 一致，reference max delta 4.1633e−17；原4ms delay保持 |
| 原空间五场景与被动传感器 ablation | `spatial_regression/summary.json` 全部通过，未更改原 truth/tolerance |
| GC-on / 有限GC策略控制 ablation | `gc_policy_control_ablation.json`：41000 控制 tick 全字段逐值相等，最大数值差0 |
| 实际 ROS graph / TF authority | 最终 graph 为原 spatial owners + 原版 SLAM；map→odom 唯一 GID/owner为slam_toolbox，无 GT/cmd_vel topic |
| 标准地图加载 | `lifecycle/summary.json`：map_server 的25921个cell与保存trinary图像逐值相同，未发布TF |
| pose graph 加载 | 原版从保存图加载并运行新 balance session，正常退出、Graph干净 |
| IMU dropout / 重复 map authority / SLAM 进程故障 / 缺失图 | 全部预期非零退出、Graph无遗留；重复 authority 检出实际新GID，未放宽门禁 |
| 正常退出与短暂scan dropout | 最终两run exit code=0、runtime CLOSED、无错误；5帧主动丢帧后仍通过原地图质量门限 |
| 依赖、几何与冻结保护 | pip check、rosdep check、派生几何检查通过；1083开工保护文件SHA256均不变 |

工作区原有 `tests/test_stage8_agent.py`、`tests/test_stage8_interaction.py` 的 diff trailing whitespace 未代用户修改；全局 `git diff --check` 因此有原有提示。只检查本轮修改范围的 whitespace 与链接，并保留用户所有已有更改。

## 失败、偏差与解决处

1. 第一组 raw 运行虽出现 OccupancyGrid，空间数据因 wall catch-up burst 超出原 gap gate。修 simulation pacing，不改 source time/gate。最早 verifier 的 passed 字段不完整，`raw_initial` 整组明确列为失败，不能作成功证据。
2. 空可选 `map_file_name` 被传为非法空 ROS CLI override，SLAM 非零退出。改为只在实际请求加载时传入，并检查绝对 basename / 两个非空文件。
3. 初期多次 reference starvation 保留了真实安全接管。被动实际 ROS profiling 定位 sequence792 的 Gen2 GC：52.3947 ms wall、56.8581 ms thread CPU、collection delta=[0,0,1]。只在有限 mapping physics loop 前收集并延后循环 GC，所有出口恢复；原测得 latency 继续进入 transport、安全期限/容量/heartbeat不变。两次最终 run loop collections=[0,0,0]，原策略恢复。此策略不推广到无限运行或MCU固件。
4. 初始 3 m / scan barycenter 邻域覆盖整个小房间循环，图只有64节点/63边，无真实回环。阅读 Karto near-linked/candidate chain 实现后采用 pose locality、1.5m 邻域和更密真实扫描；solver、loop response/variance门限不变。
5. 稀疏图定位良好但最大x/y栅格边界缺墙，coverage仅69.84%；与上游#689和本地round/index边界实现相符。未 fork upstream、合成墙、改环境或降低门限。同场景最终足够扫描与闭环达到100%覆盖；upstream边界缺陷仍是后续数据必须复查的限制。
6. 早期 run 的 SLAM 参数以精确编辑历史重建并明确标注；两个最终 run 使用启动前复制并真正传入的参数快照/SHA。没有把重建文件冒称当时的运行时dump。

所有成功/失败原始证据保留。没有因局部故障引入另一套控制器、补偿算法、地图格式或独立导航系统。

## Modified files 与重要接口

| 本轮文件/函数 | 输入 → 处理 → 输出 |
| --- | --- |
| bringup `launch/slam.launch.py` | sim time、参数文件、可选graph basename → 验证加载前提、包含官方online_async_launch和失败退出策略 → 原版SLAM进程 |
| `launch/slam_simulation.launch.py`、`config/slam.yaml`、`config/mapping_route.json` | 同源SLAM参数与有限intent → 组合原spatial launch、显式map authority → 同一真实运动/传感器图 |
| `launch/map_review.launch.py` | 标准map YAML → 官方map_server/lifecycle → 静态OccupancyGrid，无定位TF |
| `spatial_simulation.launch.py` | 原空间参数＋完成等待/readiness/dropout/离线记录 → 原四owner参数与线程约束 → 兼容原诊断及SLAM组合 |
| simulation `runtime/mapping_motion.py` | 原common/config、route → 原planner/producer/packet_link/MCU arbiter，有限分配策略和被动CPU/GC计数 → 既有reference/yaw/safety接口，无GT输入 |
| `SpatialSession.run / physics_step` | 选定scenario、原估计与physics → 映射运动组合、wall pacing、不改physics/control → 原控制结果、source history、实际authority/timing证据 |
| `SpatialSimulation` | 原source frame → readiness、有界ROS交接、显式scan丢帧、完成保留/退出；GT回调仅独立写文件 → 标准sensor/clock、runtime、post-hoc文件 |
| control `src/tf_authority.cpp` | 实际TF＋rclcpp publisher GID → 可选fully qualified map owner检查 → 唯一authority diagnostics或非零退出，无TF输出 |
| `scripts/save_slam_map.sh` | 绝对basename → 官方map_saver、typed SerializePoseGraph，拒绝覆盖/检查result → 四个标准成果文件 |
| verification `verify_slam_integration.py` | 输出目录/实验参数 → 启动真实graph、采集与控制门禁、标准保存、退出 → 原始证据与summary |
| `evaluate_slam_map.py / inspect_slam_pose_graph.cpp` | 保存的DDS/scene/GT/上游图 → source-time TF、几何/漂移/资源/回环离线计算 → evaluation JSON/图，不回流运行 |
| `verify_slam_lifecycle.py`、`tests/test_slam_mapping.py` | 隔离运行/显式故障 → 加载、GID冲突、进程清理、GC中断恢复 → 实际测试证据 |
| 安装与文档 | bringup setup.py/package.xml、description rviz/slam.rviz；README/ros2_ws及owner README、docs/SPATIAL_STATE、RUNTIME_ARCHITECTURE、SLAM_MAPPING、CURRENT_STATE、tools README → 功能依赖/入口与边界 |
| 本工程文档 | acceptance.json、UPSTREAM_DECISIONS.md、此final_report、learning_log → 固定验收、来源与唯一工程交付 |

C99 native core、MCU firmware、Agent/Supervisor、ReferenceBlock 协议、原冻结参数和唯一正式交互脚本均未改。

## Remaining issues 与下一步最少前提

当前交付是可运行的控制仿真建图与地图系统，不是实体导航。没有验证真实机械旋转扫描、非平地、滑移/振动/材质、长期无限运行、Pi/STM32时序、真实LiDAR或GUI人工反馈。startup DDS损失、未覆盖TF历史、upstream最大栅格边界和动态外参的二维缓存限制均如实保留。

实体安装至少先实测刚性挂载/外参、有效高度与自遮挡、传感器轴、同源姿态与设备→ROS source-time映射/不确定性、真实 acquisition/time_increment 和完整pitch/速度包络，并验证MCU本地失联/物理输出 gate。无证据时不提前增加过滤/补偿；若失败，优先研究现成ROS处理或简单物理有效性门禁，禁止GT和地面伪墙。

接 Nav2 至少需要：合格持久地图；明确在线定位模式和唯一map→odom owner；实测footprint与obstacle/clearing语义；cmd_vel到既有授权reference/safety链的适配及停止/失联证明；实际算力和传感器/时钟覆盖。当前Fake Navigation/RobotBackend不因地图存在而变成实体执行。

桌面正式文字交付使用独立副本：`C:\Users\29603\Desktop\CompanionBot\stage report\SLAM_Toolbox_Integration\final_report.md` 与同目录 `learning_log.md`。仓库文档为源，副本SHA256及原始成果hash在工程会话的 `report_delivery.json` / `artifact_manifest.json`；复制不是两份独立演进报告。
