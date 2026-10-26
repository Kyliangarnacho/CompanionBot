# CompanionBot current state — Ubuntu / ROS 2 Jazzy

## 当前工程进行中：Native Runtime 与视觉陪伴（2026-10-26）

- MCU C99 已修有效 HARD_STOP 撤销旧 resume/reference 的本地安全屏障；原生/对照/故障注入及 ARM 交叉构建证据见本工程 `mcu_safety/summary.json`。控制律、1ms physics/2ms control、wire 与物理 arm 权限保持。
- 已实际构建安装 C++ MotionAuthority、rclcpp ingress、完整 TwoPath scheduler/lifecycle/ReferenceBlock、FollowController 和空间节点；正式导航仿真已消费原生链。90000 tick、61500 samples 与 wire/state 对照已有通过证据；根全量634 pytest、八包实际构建安装和colcon汇总797 tests/0 errors/0 failures/0 skipped通过（含外层，不相加计coverage）。新版native链8项20s物理故障全部通过，含pending-resume取消，旧权限未恢复。
- 最新190s Bookstore西书架→东书架→服务台全部通过，停稳误差6.26/8.18/11.01cm，零障碍接触/意外接管，RTF约0.995；日志14条TF过滤丢弃保留。首轮RTF约0.90的负结果不覆盖。固定参考块P95改善约95%、P99约94–97%，但同输入20s whole-physics对照wall/CPU退化约4.3%；10000tick控制/物理exact，两个SAFETY时间字段仍超过原2e−12容差，全面parity未过。
- Gazebo成熟人物真实RGB→原YOLO/ByteTrack/Master/OSNet＋独立Depth已执行，含LOST→ReID。单目heldout RMSE约0.388m失败。用户授权新增真实双目后，48位置/2064帧实际执行，heldout31/32可用，RMSE0.06084m/P950.11031m、age P95239ms，原0.03/0.09m/0.2s门禁仍失败；一次scale-only辅助退化并Reject。原生双目45项单测/CTest/sanitizer通过不能代替测距验收，当前无合格motion证书。
- 正式唯一app最新60s双目/Agent/真实Nav2运行完成两次同服务台的新task GUIDE_TO、零碰撞/意外接管并正常清理；实际332组双目、311组metric pair/308组原YOLODepth诊断pair，FOLLOW正确拒绝。310条观测仅104条满足原200ms，P95约368ms，不能认证连续跟随。当前全向保守障碍guard在原起点被0.751m桌体阻挡（静止要求1.356m），已有有限审计，未裁扇区或降低门限。真实主人跟随、完整互斥切换与最终组合回归尚未验收。
- 最新82s原SLAM回归通过：C++空间节点实际消费，820/820扫描有source-time TF，原地图质量/控制门禁及保存加载/故障清理通过。VS Code已接五包编译数据库覆盖25个C++源码，八包构建和六个主要编译参数检查通过；未操作GUI或关闭诊断，1003份冻结成果SHA保持。
- `hardware_execution_ready=false`。审计：[Native Migration Audit](models/minisegway/native_runtime/NATIVE_MIGRATION_AUDIT.md)；统一证据根 `models/minisegway/native_runtime/results/session_20261023/`。唯一报告继续维护，桌面`stage report/Native_Runtime/`进展副本以该证据根`report_delivery_current.json`的SHA核验为准，非最终验收；未 commit/push。

## 已完成工程：商业书店场景与完整自主导航集成（2026-10-23）

- AWS Bookstore ros2 固定commit `f5a2aaac`：26原型、138实例、完整纹理/真实接触与射线几何；成熟库转换，非凸货架分解/开放墙面薄凸体保留通道。第三方和派生mesh仅在ignored cache，196源hash及派生manifest验证；原件不覆盖，Gazebo不接入。
- 原SLAM链实走37.737m主图和9.225m补图，标准地图/posegraph保存加载通过；最终5cm地图，几何P95=4.333cm，occupied precision=99.816%，补图SLAM位置RMSE=2.364cm。正式map version `b48c61b3b2bf76d5`，五个语义目标均经真实Smac Action验证；mapping/AMCL的map→odom互斥，GT只事后评价。
- 官方AMCL＋Smac2D＋RPP＋Costmap＋BT，经单authority租约/健康检查接原LIGHTWEIGHT Ruckig→ReferenceBlock→MCU safety；无第二速度平滑器、Spin/BackUp恢复。最终代码两轮“起始区→西书架→东书架→服务台”分别21.257/21.347m，六目标全成功，3s停稳误差6.13–11.55cm，零障碍接触/非预期接管；最大pitch4.443°，原控制门禁全部通过。
- 唯一正式app显式`--navigation-mode nav2`接实际Action，默认Fake保持。正式UserInput带路与同地点新task、实际误差、正常释放、10个目标markers均验；既有Agent取消及同task幂等通过。ACCEPT不等于完成，地图坐标不由LLM生成；本轮语言层为离线FunctionModel，未新增云端Qwen/语音验收。Nav2模式FOLLOW明确拒绝，hardware_execution_ready=false。
- STOP/WAIT/CANCEL、cmd/lease过期、scan/localization失效及resume等待窗口取消共8项物理夹具通过；5项实际DDS故障、显式重新授权、真实货架目标ABORTED(error208)与正常退出通过。保留初始碰柜、评分alias、BT/误差、启动gap、pending-resume及terminal轮询竞态的失败证据；只修职责/时序，不改安全门限。
- 八包build、397 pytest、colcon417 tests/0 errors/0 failures/0 skipped、五套native冻结回归通过；199300仲裁tick，reference最大差4.1633e−17。原空间、SLAM、typed协议、C++ PTY和正式Fake回归通过，587保护文件SHA不变。1ms physics/2ms control、冻结LQR、Q OFF与协议保持。
- WSL2 Ryzen 9 / 8 CPU的有限物理仿真；最终导航观测RTF约0.995–0.997，scan约10Hz/odom约50Hz，Nav2 controller10Hz。Pi 5/STM32实体、长期运行、真实传感器/玻璃与矮障碍覆盖、动态行人、GUI人工验收仍未完成。提供同模型passive MuJoCo viewer和RViz入口，由用户手动开启，Codex未操作桌面。
- 运行与完整数据流：[RETAIL_NAVIGATION](docs/RETAIL_NAVIGATION.md)。唯一 [final_report](models/minisegway/retail_navigation/final_report.md) / [learning_log](models/minisegway/retail_navigation/learning_log.md)；证据根`models/minisegway/retail_navigation/results/session_20261022/`，`evidence_index.json`区分最终通过和被审计否决的旧run。桌面独立副本`stage report/Retail_Navigation/`，SHA见`report_delivery.json`。未commit/push，保留用户已有改动。

## 历史：SLAM Toolbox 建图与机器人地图系统集成（2026-10-22）

- 八个 ROS package 保持；Jazzy 原版 SLAM Toolbox 2.8.5 接入 MuJoCo、原 sensorized odom/TF、VirtualPlanarLidar 和 /clock。`slam_simulation.launch.py` 是建图功能组合，有限运动走既有 planner→ReferenceBlock→packet_link→MCU safety→冻结控制，不改正式交互 Demo 或 Agent/Fake 权限。
- 两次82 s真实物理运行均通过：路径21.768 m、双圈、最大pitch7.703°；normal / 0.5 s主动scan dropout地图均为161×161、5 cm，四墙覆盖100%、occupied precision100%、几何P95为5.975/5.969 cm、位置RMSE为1.109/1.030 cm。图为125/124节点、208/206边、84/83跨圈约束，并有历史节点优化修正。
- 原始倾斜扫描直接输入，未增加补偿/地面投影/过滤算法；地面回波保留。map→odom只由SLAM发布，原spatial_state/RSP职责保持，C++实际GID审计扩展到该边。标准YAML/PGM和posegraph/data保存、map_server逐cell加载、上游图加载均实测；手动RViz配置已提供，未作GUI人工验收。
- 正常8包build、**389 pytest passed / 0 skipped**、colcon **417 tests / 0 errors / 0 failures / 0 skipped**通过（含外层，不相加计coverage）。五套原native冻结回归通过，199300仲裁tick、reference最大差4.1633e−17；原空间五场景/被动ablation通过，1083保护文件SHA不变。
- 异常IMU、重复map TF authority、SLAM进程失败、缺失图均预期非零退出且Graph无遗留；两个最终run正常CLOSED。normal收到820/820 scan、source-time TF覆盖812/820（8帧启动历史unavailable）；dropout发布815/收到798、已收帧TF覆盖100%，接收损失未隐藏。
- 有限调查定位并保留早期失败：wall catch-up数据突发、空参数、near-linked质心导致无回环、上游栅格最大边界缺墙，以及Gen2 GC导致52.39 ms reference停顿。只调整主机pacing/readiness、有限physics loop循环GC策略与有因果依据的SLAM参数；原deadline/gains/plant/truth/tolerance不变。GC策略恢复，41000控制tick对照逐值相等。
- WSL2 Ryzen 9 7945HX / 8 CPU下，SLAM平均CPU约6%单核、peak RSS约58 MiB、地图约1.05Hz、scan→pose观察到达P95约11–13ms；simulation进程883–958 MiB。不是Pi5/MCU实测。真实旋转LiDAR、非平地/大pitch、长期运行、实体安装/时钟/输出与Nav2仍待验证；upstream边界/动态二维外参限制保留。
- 运行：[SLAM_MAPPING](docs/SLAM_MAPPING.md)。唯一工程 [final_report](models/minisegway/slam_toolbox_integration/final_report.md) / [learning_log](models/minisegway/slam_toolbox_integration/learning_log.md)，[上游取舍](models/minisegway/slam_toolbox_integration/UPSTREAM_DECISIONS.md)；原始证据 `models/minisegway/slam_toolbox_integration/results/session_20261022/`，正式文字副本目录 `stage report/SLAM_Toolbox_Integration/`，SHA见会话report_delivery.json。未commit/push，用户已有改动保留。

## 历史：自平衡机器人空间状态与 SLAM 数据基座（2026-10-22）

- 新增 companionbot_description 后为八个 ROS package；新增 MJCF 派生几何/Xacro、JointState、raw IMU、估计 Odometry、保留物理 roll/pitch 的动态 TF、室内虚拟 LiDAR/LaserScan、diagnostics 与用户手动 RViz 配置。有限 spatial_simulation 诊断不替代唯一正式交互 Demo，也不接 Agent/actuator。
- 正旋转 MJ→ROS=[−Y,X,Z]，历史轮名只在空间 adapter 显式换序；复用原 encoder＋pitch 路径估计并按弧长积分。body-z control yaw 与 Euler navigation yaw 分开；没有 map、GT odom、假 identity TF 或清零 pitch。标准空间 Header 使用 source/horizon time，旧业务 publication-time 合同保持。
- 五组平衡/前进/转向/组合/俯仰物理验证通过，2200个50 Hz状态；最大 peak xy误差8.128 mm、yaw误差0.004084 rad，仅为固定seed平地短时评测。原控制门禁无跌倒/饱和，Q OFF；新增被动sensor/scene的7000行控制历史与原plant逐值完全一致。
- 正常与独立干净八包 ROS 构建通过；全仓/各owner **388 pytest passed / 0 skipped**，独立 colcon **397 tests / 0 errors / 0 failures / 0 skipped**（含CTest/pytest外层，不重复计coverage）。pip、rosdep、派生几何、冻结配置与板工程检查通过。
- 实际 ROS/DDS 验收通过：698组state/odom/joints/IMU、140 scan；8个TF child的实际GID/parent/owner唯一，source-time查询只缺首个0.019 s IMU早于TF历史的覆盖。IMU断流后最后odom为3.0 s，错误parent/重复GID非零退出；重复SIGINT下worker停止、runtime CLOSED、Graph无遗留。
- 10项独立typed DDS协议故障检查通过；clock暂停不推进，clock前进而源停止则timeout。Jazzy rclpy无GID，审计放在C++；发现Graph姓名可晚于GID，最多1 s限时等待且不批准unknown。原启动/数据流/清理失败日志保留，未改算法或放宽既有控制门限。
- 原native baseline/radial/safety/dynamic/stream五套回归及正式交互Fake/视频回放ROS运行回归通过；199300个C/Python仲裁tick维持旧parity，reference最大差4.1633e−17。四份baseline JSON和九份控制history完全一致；主机计时/arrival差异保留。
- 根 AGENTS 已去重重组为长期原则，动态状态留在本文件，接口由 [空间合同](docs/SPATIAL_STATE.md) 维护（含TF、节点和数据流图）。[final_report](models/minisegway/spatial_slam_foundation/final_report.md)、[learning_log](models/minisegway/spatial_slam_foundation/learning_log.md)、[上游取舍](models/minisegway/spatial_slam_foundation/UPSTREAM_DECISIONS.md)；原始证据为 `models/minisegway/spatial_slam_foundation/results/session_20261021/`，桌面独立副本目录为 `stage report/Spatial_SLAM_Foundation/`，SHA核对见报告。
- SLAM/地图/Nav2、真实传感器与设备时钟适配、Pi实时性、STM32运行/物理输出、GUI人工验收仍未完成。当前为会漂移的平面支持baseline，轴高固定轮半径、covariance未标定，倾斜scan的二维SLAM兼容性待验；hardware_execution_ready=false。冻结算法/资产与用户改动保留，未commit/push。

## 历史：Python / C++ / STM32 C/HAL 混合工程（2026-10-21）

- 七个 ROS package 保持；control 转 ament_cmake＋ament_cmake_python。C++ rclcpp 串口桥实际实现独占设备、worker、有界队列、session/clock/双 deadline、错误退出和 diagnostics；默认 TX off，不代发 heartbeat。500 Hz 控制留在 MCU 本地，不穿 DDS。
- 同一份独立 C99 源码实现 receiver、safety/resume、brake、互补姿态/encoder PLL/gyro yaw、固定 LQR、yaw PD 和 allocator，供主机 shared library 与 ARM 固件共用。Python 保留 AI/视觉/Agent、Pi planning、冻结对照和高级 slope/payload/Q 研究模块；正式 Demo 和默认 Python 行为不变。
- 参考硬件确定为 **Pi 5 4GB＋NUCLEO-G474RE**。真实 HAL 工程使用锁定 ST CubeG4 v1.6.1；170 MHz、USART2 DMA 921600、TIM6 2 ms、IWDG、SWD、调度与物理 gate 已实现并交叉编译。补充验证后的 Flash **39,140 B**，主 RAM **18,280 B**（含 8 KiB 栈），CCM 未占用。提供 `.ioc` / MDK5 Compiler 6 工程，尚未运行 CubeMX/MDK、未烧录。
- 新嵌入式 wire v2 使用有界 COBS/CRC32，完整复用原 640B ReferenceBlock v1；status 分开 requested/limited/allocated/actual、saturation、故障和拒绝原因。未知 IMU/encoder/motor/physical arm 只在 board_io 返回 unavailable，actual NaN，实体输出禁用；hardware_execution_ready=false。
- 正常七包 ROS 构建、独立空目录七包构建及重建通过；全量 **365 pytest passed / 0 skipped**。clean colcon **373 tests / 0 errors / 0 failures / 0 skipped**（含独立 C case 与 7 个 pytest 外层，不重复计 coverage）。19 项原生 C/Python回归、C CTest、ASan/UBSan、19 项实际 C++/DDS＋PTY 软件链路检查通过；pip/rosdep/上游锁/冻结配置检查通过。
- 原 moving Q-OFF、payload/slope/impulse、五 radial、三 safety、八 dynamic、stream/原 4 ms delay 验收复用通过；C 仲裁实际驱动 plant 并逐 tick 对照，共 **199,300 tick**，state/epoch 一致，reference 最大差 **4.17e-17**。原 Python 对旧 Linux 结果仍为 **76 项 exact checks** 一致，1538 个开工冻结文件 SHA256 不变。
- 数值差异未掩盖：恢复 encoder 原运算顺序后正式 governor 事件对齐；诊断 shadow 仍有一次 50 ms 变化。接触/冲激轨迹非逐位相等，pitch-rate 最大差约 0.025204 rad/s，terminal pitch 指标最大差约 0.005838°；全字段差异及初始不利证据保留，未改 gains/plant/truth/tolerance，Q actuator OFF。
- Pi 视觉吞吐、实体 UART/传感器/电机/供电、clock mapping、板上 WCET/栈/看门狗/arm 未实测；当前 PyTorch 三模型完整 20 Hz 未获证明。主机容量夹具不称作 Pi benchmark；真实视觉→follow、SLAM/Nav2、Agent 真机执行仍未开发。
- UART补充验证：921600 8N1最坏块664B/7.205ms；当前正向流15.54%、反向status9.01%。修复逐帧固定poll等待、kernel接受不等于线速完成的积压问题，保留原期限；12组严格限速PTY/C协议检查通过，120s标称7200/7200、60s高负载13200/13200，过载/带宽下降/失联/partial/阻塞均正确拒绝或本地降级。115200不适合现有流，不提高默认baud。
- 500Hz补充验证：修复只量函数体而遗漏释放等待/跨周期的漏洞，输出前检查本次ISR释放age/下一tick并锁存故障；新增SWD可读DWT分段最大值。9个真实app host逻辑case＋1个C core case经ASan/UBSan全部通过，365 pytest及原19项bridge复测通过。分析/正常ARM BIN相同，保留double；340,000cycles是待验收预算，尚不能认证板上2ms执行余量。C核心7个源码/头文件及1538冻结文件SHA不变。
- 当前合同及架构/数据流/协议/构建烧录：[CROSS_LANGUAGE_EMBEDDED](docs/CROSS_LANGUAGE_EMBEDDED.md)；[调研与选型](models/minisegway/cross_language_embedded/UPSTREAM_AND_HARDWARE.md)；唯一正式 [final_report](models/minisegway/cross_language_embedded/final_report.md) / [learning_log](models/minisegway/cross_language_embedded/learning_log.md)。原始证据在该目录 `results/session_20261021/`，补充验证在 `results/uart_timing_20261021/`；桌面独立报告副本目录为 `stage report/Cross_Language_Embedded/`，最新SHA256见补充验证目录 `report_delivery.json`。测试输出隔离、用户已有改动、冻结证据和有效 workspace 构建保留，未 commit/push。

## 历史：ROS 2 运行架构完善与数据流规范化（2026-10-21）

- 七个 package 保持，唯一正式入口仍为 `scripts/demo_stage8_interaction.py`；bringup 委托同一 app。启用相机时静态组合 interaction/perception 两 Node、各自独立 ROS executor；无相机只建 interaction。Camera/推理仍是原唯一 owner，本地 Frame/Master 共享，不通过 DDS 迂回消费。
- 感知 observer 改为三个有界 latest 槽，DDS 序列化移出 capture worker，无订阅者不序列化 RGB。frozen_sim 同样只交接估计/实际命令快照，ROS timer 发布遥测，不进入 500 Hz control / 1 kHz physics 调度；合并计数可见。
- 文字入口同步有界接纳，64 项、local control 优先、满载拒绝；UserInput response 新增 input_id，连通 provenance→Session→Feedback，消费前拒绝排队过期。旧 ROSIDL 客户端须随 workspace 重建；SourceStamp v2 / binary v1、Supervisor 授权与 Fake 执行合同保持。
- 加入运行状态/标准 diagnostics、后台错误传播、完整 cleanup、首帧前退出、SIGTERM 和 required-process 失败传递。安装的 ROS 参数文件、namespace、agent_config、独立输出目录已验。运行状态不是 ROS ManagedNode lifecycle，不支持热 reset/reconnect。
- 最终全量 Python/ROS **346 passed，0 skipped（9.49 s）**；独立 underlay clean build 七包通过（36.5 s），正常 workspace 构建七包通过；clean colcon **347 tests / 0 errors / 0 failures / 0 skipped**，含一个 CTest 外层，不增加 coverage 计数。pip check / rosdep check 通过。
- 冻结移动 payload Q-OFF、坡面/偏载/冲激、五个 radial 场景、八个动态窗口场景和 reference stream/原 4 ms delay 复现通过。与既有同版本 Linux 完整 metrics/history/事件/计数共 **76 项 exact JSON checks** 一致；最新 Windows 动态 latch 最大差约 8.55e-14 m/s，未改门限。
- 实际 launch / typed DDS / rclpy Graph / ros2 CLI 检查通过；perception 是感知 publisher，interaction 是输入与 feedback owner，二者 PID 相同、executor 分开。正式回放没有自动选 Master；显式 track-1 软件 fixture 收到 119 RGB / 102 Master / 68 relative。正常退出取消在途 Fake task，ROS 线程停止、Graph 不留项目 runtime；无效源非零退出，真实首帧前有限退出通过。
- ROS/异步验证最初受沙箱本地 socket 禁止影响，原失败/超时保留；在允许本地通信的环境复验通过。新 node 曾与 rclpy 的 publishers 只读属性重名，验收工具曾使用无可检查签名的 deque.append；均已修复并保留失败日志。无算法调参。
- 当前权威合同、两张架构/数据流图、接口/生命周期/依赖与运行命令：[运行架构](docs/RUNTIME_ARCHITECTURE.md)。完整工程报告：[final_report](models/minisegway/runtime_architecture/final_report.md)、[learning_log](models/minisegway/runtime_architecture/learning_log.md)。成果只在 `models/minisegway/runtime_architecture/results/20261021T1335/`；桌面交付目录为 `C:\Users\29603\Desktop\CompanionBot\stage report\ROS2_Runtime_Architecture\`，副本 SHA256 见报告。
- 真实视觉→follow、SLAM/Nav2、实际 odometry、Pi/STM32 transport/实时性、物理相机/音频/GUI 未新增认证；Robot/NavigationBackend 仍 Fake、hardware_execution_ready=false。共进程不隔离 native crash/OOM；同 boot monotonic proof 不等于跨设备同步。未 commit/push。

以下是已冻结的历史状态，反映当时架构；当前运行关系以上述文档为准。

## 历史：Linux 迁移与源码职责整理（2026-10-05）

- 主环境保持 WSL2 Ubuntu 24.04.4 / .venv Python 3.12.3 / ROS 2 Jazzy；Windows venv 不复用，数值/模型/Agent 依赖与初次迁移相同。source scripts/env.sh 后统一解释器；pip check、rosdep check 通过，未重复安装 Jazzy。
- 当前七个 package：**interfaces / perception / follow / control / behavior / simulation / bringup**，代码唯一归属 ros2_ws/src。原顶层 control/sim/perception/embodied_agent/learning 与混杂 scripts 已按职责搬迁；算法 core/runtime 与 ros adapter 分层，package 不强制等于 node。根 pyproject 仅 editable bootstrap 同一份源码。
- 唯一正式入口仍是 scripts/demo_stage8_interaction.py，薄委托 companionbot_behavior.app.main；ROS interaction 同样委托该 main。唯一正式 bringup：**ros2 launch companionbot_bringup interaction.launch.py**。simulation.launch.py 仅有限诊断，camera_tf 仅 opt-in。
- Pi reference producer / bearing yaw 属于 follow；MCU receiver/safety/latest-yaw/estimator/controller 属于 control；plant、virtual sensors、in-memory transport/模拟调度属于 simulation。control/follow 核心无需 MuJoCo/rclpy/视觉库，Agent 包不依赖 control/follow/simulation，无 actuator authority。保持 Python，未做 MCU/C++ 移植。
- Stage 3–6 的冻结算法、参数、plant、truth/tolerance、scheduler/reference/双窗口 follower 保持。1 ms physics / 2 ms controller 不拆 DDS。原算法306个完整定义 body及import alias一致，1,758个原配置/资产/冻结成功失败证据SHA256不变。Stage3/4完整四份metrics/history与重组前Linux完全相等，Stage6五场景KF/raw/事件/对齐/packet计数亦相等，共76项exact checks。
- Stage 7 Camera/YOLO/ByteTrack/Master/OSNet/depth/geometry仍原语义，perception publisher taps复用唯一Camera/interaction node。**真实Stage7→6视觉运动闭环仍未完成**；外参、pitch、axial-Z provisional。TF默认关闭，仅opt-in未实测camera mount，无map/odom定位。
- Stage 8 Agent/Catalog/Supervisor、sleep/stream cancel/freshness/source/task/feedback契约保留；ROS service只queued，不能等同授权/完成。Robot/NavigationBackend仍Fake、hardware_execution_ready=false；无SLAM/Nav2/真实地图导航。各source/perf/sim/ROS publication clock与source/reference epoch分别保留。
- 最终全量pytest **332 passed，0 skipped（8.11 s）**；七包clean build/rebuild与colcon全量test通过。colcon **333 tests / 0 errors / 0 failures / 0 skipped**是同一332pytest cases加接口CTest外层，不重复计coverage。新增4架构边界+1资产路径测试，原测试只迁import/CLI路径。
- MuJoCo headless、Stage3 Q-OFF移动payload、Stage4 final六场景、Stage5 V1.5三solver benchmark/replay/reference stream和原4ms delay、Stage6五径向场景与最新八动态完整历史复现通过；最新Windows双窗口event/sequence/FULL/alignment一致，max latch delta约8.55e-14m/s。
- 真实模型CPU fixture inference与完整感知typed DDS通过（120 Color / 82 Master updates / 40 relative observation，异步吞吐不固定）；正式root Demo回放+ROS正常退出，observer failure0且未自动选Master。Agent/Supervisor全量回归、input freshness/duplicate gate、Fake GUIDE_TO ACCEPT/CANCEL task身份、50Hz sim telemetry/独立clock/opt-inTF通过。
- 原生eSpeak cmn→paplay完成/取消、pending0/worker退出再验通过。WSL2仍无/dev/video*或USB bus；C920未验。初次16kHz mono PCM证据保留，物理麦克风/扬声器身份、真人wake/ASR/AEC/可听起声、GUI、真实Qwen Linux API、Pi/STM32实机链未验；fake/replay不视作硬件通过。
- 旧通用companionbot_ros、重复Stage7 full-mode wrapper、旧路径audit归档reference/pre_reorganization并COLCON_IGNORE。旧build/install和本轮源cache仅在新构建/验收后清理；历史runner、拒绝学习/失败证据、provenance与当前.venv/build/install保留。清理清单见本轮results/architecture/cleanup_scope.json。
- 整理初轮漏原import alias导致四runner NameError，仅恢复alias；Stage5工具漏PLOT_PATHS重定向曾覆盖四历史PNG，审计检出后按Windows原副本与开工SHA256精确恢复，修复wrapper重验全部hash不变。原失败/恢复证据保留，没有改算法或门限。
- 当前权威文档：[ARCHITECTURE](models/minisegway/stage9/ARCHITECTURE.md)、[RUNNING](models/minisegway/stage9/RUNNING.md)、[INTERFACES](models/minisegway/stage9/INTERFACES.md)、[STAGE9_REPORT](models/minisegway/stage9/STAGE9_REPORT.md)。本轮证据在results/architecture，初次迁移报告快照在reference/pre_reorganization，旧结果不覆盖。用户已有改动保留，未commit/push。

阶段文字交付索引（2026-10-21）：`C:\Users\29603\Desktop\CompanionBot\stage report\Stage9_Linux_ROS2\final_report.md`、同目录 `learning_log.md`。按 AGENTS 新增规则，同一大阶段固定维护这两份文件；原始证据留在仓库对应阶段成果目录。本次只整理文字与核对副本，未重跑机器人回归。

## Stage 8 冻结能力与 Windows 历史验收（2026-10-05）

- 唯一正式交互入口：`scripts/demo_stage8_interaction.py --camera-device 1`，交互窗口＋Stage7预览；`run_stage8_agent.py`是无设备文字诊断，`verify_stage82.py`是自动验收，不是另一套正式Demo。
- PydanticAI Slim2.53.0＋现有Qwen `qwen3.8-flash`，单Agent，保留C920唯一采集owner、Realtek、Vosk唤醒“你好小柒”、SenseVoice/Silero、Streaming与SAPI。睡眠零模型请求/零Agent问答取帧，感知继续运行。
- CatalogQuery/lookup_knowledge新增可选query_variants（最多3个，默认空）。首次Qwen Tool Call完成当前语义扩展；同一次Provider检索融合/去重，返回有界真实候选、query_index命中来源和截断计数。去掉预置待测口语alias，不引入词库、第二Agent或独立改写请求。
- 虚构目录为12件商品/展品、5品牌、8分类、7目的地，包含同品牌面食/饮料、同名不同包装及跨货架果汁；商品、分类和可导航destination稳定ID分离，没有地图坐标或真实库存。
- Resolver持有单个pending与当前turn的resolution，TTL120s、revision和候选交集；补充条件先重新检索，品类专区与具体商品歧义区分。GUIDE_TO须使用本轮真实destination_id/resolution_id；错误编号、失效候选和模型编造凭据不会获准。
- Supervisor、Robot/Navigation异步契约沿用：ACCEPT不等于完成，RUNNING/progress、CANCEL_REQUESTED/CANCEL、UNKNOWN、任务ID/回合关联、2s Backend deadline、300s task deadline、幂等和迟到反馈校验；任务由独立callback/0.5s poll继续反馈，模型回答结束不代表任务结束。内存队列/历史有界。
- Frame/Master/ASR与Robot/Feedback adapter已有source/epoch、sequence、接收与可选产生时间及clock域；Stage7原始ColorFrame/host_read_complete不是曝光时间。跨机时钟显式映射、destination→map pose和持久化重连协调仍待未来Adapter。
- 自然视觉按需capture_view，不参与Master tracking或运动控制；图片轮不能授权行为。FOLLOW仍要求新鲜LOCKED/visible Master。Robot/Navigation仍Fake，hardware_execution_ready=false，尚未接ROS2/SLAM或实际运动。
- Streaming/SAPI、本地优先STOP/WAIT/取消/打断与30s idle休眠保留。含行为权限的首响应可能延后显示，ACK后与取图后继续流式。无可靠AEC/远场噪声认证；无confidence语音执行行为须名字前缀。
- 本轮最终全仓302项通过（13.02s），pip check通过；真实Qwen最后语义11/11、行为/取消5/5、休眠与本地睡眠零请求3/3，合计32请求。当前检索对照与全部失败批次见[交付证据](models/minisegway/stage8/results/semantic_refinement_delivery.json)。扩展OFF/ON使用同一目录和过滤条件，不增加模型请求。模型仍可能填错凭据/工具名并耗尽既有预算，失败及取消必须如实保留，不能当作成功或未执行。
- 10-04 Realtek只读采集、合成音频SenseVoice及SAPI曾通过；10-05相机索引1首次打开失败，用户重新连接后正式入口headless复验通过：1280×720、自然capture_view、2请求、3.400s、Streaming正常。原失败保留；本次麦克风/TTS关闭，真人音频/AEC不作新认证，没有GUI自动操作。
- 收尾清理限于有明确范围的缓存/过时重复测试输出；原始模型、硬件失败与冻结实验成果保留。README只描述能力/限制和启动命令，阶段状态放本文件；规则已写入AGENTS。未commit/push。

当前权威文档：[最终架构与接口](models/minisegway/stage8/STAGE8_REPORT.md)、[运行与验收](models/minisegway/stage8/RUNNING.md)、[设计/失败/修复记录](models/minisegway/stage8/LEARNING_LOG.md)。Stage3–7冻结边界见下文。

## Stage 7 冻结能力与历史入口（当前路径见 Stage 9）

- 最终入口：`scripts/demo_stage7_perception.py --camera-device 1 --preview`，RGB-only。
  Camera → YOLO26n → ByteTrack → Master/OSNet ReID，与独立 YOLO26n-depth 按
  source ID/sequence/time/resolution 配对 → torso median → camera XYZ → robot-relative observation。
- Master/ReID 保留 Stage 7.6：三帧 reference 跨至少 0.30 s；LOST candidate 连续 3 次 tracker update，
  ≥0.68 接受，0.50–0.68 间隔约 0.20 s retry，低于 0.50 停止本 track 重试。阈值未做 negative-person calibration。
- Camera 为 BGR uint8；bbox/crop/depth 均用原始 source pixels。C920e K/D 为 1280×720，禁止自动缩放；
  detector imgsz=640、depth imgsz=768 与采集分辨率独立。
- 时间为 host read-complete / perf_counter，未宣称曝光时间。depth age 为 ready-source；pitch provider
  必须使用同一 source time 和时钟语义，缺少 coverage 返回 unavailable。
  Provider 可另外显式报告映射到 host clock 的曝光对齐 `pitch_time_s`；原始 source receipt 时间保持不变。
- camera 坐标为 +X右/+Y下/+Z前；body/leveled robot 为 +X前/+Y左/+Z上，原点为轮轴中点。
  `P_B=R_BC P_C+t_BC`，`P_L=R_y(theta) P_B`；theta 正值绕 +Y 右手旋转（nose down）。
  模拟安装为前方 0.08 m、轮轴上方 0.35 m、下倾 10°；pitch 为 simulated constant 0°。
  这些是 provisional 参数，YOLO depth 的 axial Z 解释也尚未最终确认。
- 最终无 GUI C920 smoke：300 帧、124 组同帧配对、104 条 robot observation；创建一次 reference，
  一次 ID 1→2 ReID reacquire（0.8963）。这是执行证据，未独立验证身份或物理距离。
  depth 14.03 Hz，infer median/p95 68.59/85.04 ms；detector 主链 27.23 Hz。
- 最终时间接口补完后的第二次 180 帧无 GUI 验证同样正常退出：56 组同帧配对、54 条 robot observation；
  depth 12.04 Hz、infer median/p95 82.19/86.75 ms，detector 主链 25.60 Hz。两次原始结果均保留，未调参。
- 当前完整 active tests：**150 passed**，统一在项目 `.venv`。最终 GUI 留待用户验收。
- Stage 7 配置/结果统一在 `models/minisegway/stage7/`，下设 `stage7_1/` 至 `stage7_7/`。
  最终配置为 `stage7/config/final_demo.json`。
- **Stage 7 尚未接 Stage 6**。需确认设备物理身份、真实 T、depth convention/精度、pitch sign/clock/history、
  相机 latency 与 stale/lost 接口策略，并测 Pi/MCU 实机时序。检测频率不要求固定 20 Hz。

详见 [Stage 7 final report](models/minisegway/stage7/STAGE7_FINAL_REPORT.md) 与
[Stage 7 learning log](models/minisegway/stage7/LEARNING_LOG.md)。

## Stage 3–6 冻结控制与跟随状态

Stage 3 已完成并冻结为 MuJoCo simulation baseline。权威索引是
`models/minisegway/stage3/results/config/baseline.json`；生产配置未自动替换为调参过程中任何临时候选。

## Plant、时序与数据边界

- MuJoCo physics step：1 ms；controller/estimator update：2 ms。
- 两轮纵向输入语义：`u = tau_left + tau_right`；单轮 hard peak 仍为 ±0.63 N·m。
- 控制状态顺序：`[p_hat, v_hat, pitch_error, pitch_rate_hat]`。
- Virtual IMU 保留 1 kHz packet、1 ms availability latency、启动 gyro bias calibration 和
  control-time age compensation。
- Controller/estimator 不读取 GT payload/contact/pose；GT 只进入 evaluator 与结果 history。

## Longitudinal baseline（Stage 3A）

- Nominal model：batch-refit 的单套 fixed A/B，来源为
  `stage3/results/stage3a_refit_ab_0p60_validation_results.json`。
- Feedback：保留 K_old；new-A/B 原 Q/R gain 和最多六组有限 Q/R challenge 均未形成更好的替代证据。
- Estimator：acceleration-compensated complementary pitch filter。
- Velocity reference：jerk-limited S-curve，baseline limits 为 0.6 m/s、0.6 m/s²、1.0 m/s³。
- Dynamic nominal 只服务于 command transient；`lambda_ff=0.6`。`T_full` 后执行 0.15 s
  smooth fade，`VELOCITY_HOLD` 中 `u_ff_used=0`，position-error memory 不被 preload/reset。
- Finite position goal 保留 `TRACKING -> TERMINAL_HOLD -> GOAL_REACHED` 语义。
- Q/DOB actuator contribution 在正常 longitudinal baseline 中关闭。

## Yaw baseline（Stage 3B）

- Parallel relative-heading/yaw-rate PD：`K_psi=0.55 N·m/rad`、
  `K_r=0.20 N·m/(rad/s)`。
- Yaw estimator 只使用 startup-bias-corrected gyro-z，并按 packet timestamp/age 对齐。
- Wheel allocator 先保护 longitudinal common-mode torque，再分配 differential torque。
- Production baseline 使用 symmetric motors；motor mismatch wrapper 只保留为专门诊断路径。

## Moving payload 与 Q 决策（Stage 3C-R）

早期 0.20 kg、低摩擦、刚性壁面场景出现高能重复碰撞、payload escape、fall 和 saturation；该失败
归类为机械 disturbance shaping/hidden contact dynamics，不作为 Q observer 的成功条件。

一次固定被动机械整形后采用 0.10 kg、64×32×32 mm payload、0.35 滑动摩擦，以及
`solref=[0.02,1.5]`、`solimp=[0.7,0.95,0.005,0.5,2]` 的壁面接触。冻结 controller 下：

- Q-OFF：无 fall、payload contained、wheel saturation 0、无 large collision-correlated velocity drop。
- Pitch GT RMS/peak：6.09° / 13.81°；velocity GT error RMS：0.0394 m/s。
- Q-ON 只产生很小的指标变化，同时增加 feedback burden；因此 actuator augmentation 被拒绝。
- Q observer 保留 diagnostic-only；production `u_Q_used=0`。

完整结果：`models/minisegway/stage3/results/stage3c_mechanical_payload_q_revalidation_results.json`。

## 当前边界

- 这是 nominal flat-ground MuJoCo research baseline，不是硬件安全认证或 payload rating。
- 未解决项包括真实 motor torque-speed envelope、电池/驱动动态、地面变化、真实视觉跟随闭环/导航和实机验证。
- 不应从已删除的 tuning artifacts 恢复临时 A patch、3-state Drive LQR、persistent cruise FF、
  GT-in-controller diagnostic 或 Q actuator augmentation。
- 后续阶段若改变 plant、payload、timing、estimator、K/A/B 或 torque limit，应新建独立公平验证，
  不覆盖 Stage 1/2/3 冻结结果。

## Stage 4 研究结论（不改写 Stage 3 baseline）

- Stage 4A：冻结 baseline 在 ±8° 通过；±15° 属于压力测试 envelope。现有 2 Hz Q actuator
  对坡面 tracking 无独立收益，production 继续 OFF；Q observer 保留。
- Stage 4B：68 个 episode、episode-exclusive split、100 Hz / 0.8 s window 的 proprioception
  pilot 已真实生成和训练。statistics+Ridge 的 test α MAE/p95 为 1.190°/3.348°；tiny raw
  Conv1D 为 2.007°/3.815°，两者均未过 α Gate，payload counterfactual drift 也为 1.070°。
- Stage 4B-R 发现旧 generator 只改 terrain friction、未改 wheel collision friction；修正版以新 ID
  生成 108/9/40 train/val/test episode，旧数据不覆盖。2.4 s Ridge 的 online MAE/p95/max 为
  2.615°/8.187°/21.954°；Ridge+±3° bounded residual 为 2.579°/7.284°/24.744°，仍有
  53 个 window 的绝对误差超过 5°。Slip CNN recall 0.689，rough CNN F1 0.441。
- Stage 4B-R 最终定案：**PERMANENT REJECT proprioceptive slope learning**；slip detector 不进
  production；rough cheap baseline 只保留结果/代码。按 Stop Rule 未把 estimator 接回 controller，
  也未执行条件式 A/B/C/D。解析 α→θ_eq 的 0.237°验证结果保留，但不代表 online sensing 可用。
- Stage 4C 没有恢复学习路线，而是按 Parravicini/Corno/Savaresi 的解耦结构增加独立 physics
  slope EKF。clean empty、smooth、`mu=1.0` 的 held-out ±3/±6/±10/±14°、0.2/0.4/0.6 m/s
  上，TWIP-EKF MAE/p95/steady bias 为 0.206°/0.400°/0.143°，48/48 有效 episode 在恒坡
  入口后最坏 0.08 s 收敛；因此定案 **KEEP TWIP-EKF**。STATIC-INVERSE 因 4 个有效 episode
  未满足 5 s hard convergence 而不保留。
- Stage 4C oracle `x_eq/u_eq` 和 estimated-alpha A/B/C 闭环均无 fall/saturation；estimated 闭环
  steady velocity RMSE 为 0.0031–0.0054 m/s。fixed 0.10 kg payload 则使坡度 MAE/p95 恶化到
  1.888°/2.564°，明确需要 payload-aware model parameters，本轮未做 payload adaptation。
- Stage 4D 的 scalar `[v_body,d_slip]` IMU/encoder observer 在严格截断 45° fall 后的 held-out
  pure-slip test 上 precision/recall/F1 = 0.947/0.419/0.581、FPR=0.0006、median latency=0.10 s；
  recall 未过 0.90 Gate。按 Stop Rule 未接 safe slowdown、未跑 slope HOLD integration，production
  不保留 slip path，并记录 future camera/visual-odometry independent velocity dependency。
- Payload Q 最终 A/B 对 fixed/free 没有一致明确收益，且 free payload 的 pitch/velocity 指标略退化；
  Q actuator 定案 **PERMANENT PRODUCTION OFF**，observer 仅 diagnostic。Frozen LQR 对 rough 与
  12/12 个 0.5/1.0/1.5 N、0.10 s 双向 stationary/moving push 自行恢复；现有 modest bump 使车体
  无 fall/无 saturation 但停滞在障碍前，属于当前 V1 traversable envelope 之外。
- 最终职责：Q observer 继续 diagnostic/physics logging；Q actuator production OFF；Stage4B-R
  的 proprioceptive **learning** 永久拒绝不变。TWIP-EKF 只在 clean empty nominal-traction
  假设内保留；slip coupling 留给 Stage4D，rough/bump/free-payload 和几何传感不在 Stage4C 范围。
- Stage 4 最终 runtime 只保留 `FLAT/SLOPE` 与 `payload_id_pending/payload_id_active`。Q one-step
  innovation 只触发一次短 session；payload ID 以 50 Hz 使用 31 次自然 transient 更新，直接冻结
  两个 sagittal 参数。本次 0.25 kg、前置 15 mm 真值的估计为 0.3476 kg、15.18 mm；质量偏高作为
  baseline limitation 保留，不再增加 acceptance layer 或调参。
- 最终 robustness smoke 中，中心/左右 ±10 mm 固定偏载与左右各 1 N×0.10 s 单轮纵向冲激均无
  fall、无 wheel saturation。左偏载的 yaw drift/peak 最大（0.0627°/s、0.718°）；左右冲激相对
  冲激前稳态带的 settling 分别为 0.062 s / 0.002 s。Stage 4 据 Stop Rule 收口。

最终记录：`models/minisegway/stage4/stage4b/results/STAGE4B-R_REPORT.md`。
Stage 4C 记录：`models/minisegway/stage4/results/STAGE4C_SLOPE_REPORT.md`。
Stage 4 最终记录：`models/minisegway/stage4/results/final_baseline/STAGE4_FINAL_BASELINE.md`。

## Stage 5 V1.5 — 上层命令生命周期与 reference-block stream

Stage 5 在 frozen Stage 3/4 runtime 上增加上层速度命令调度和参考流，不替换 LQR、estimator、yaw controller、allocator、torque limit、坡度/payload runtime 或 Stage 3/4 参数。

- 当前配置：`models/minisegway/stage5/config/stage5_v1_5_pi_mcu_stream_config.json`；当前计算 runner：`scripts/run_stage5_v1_5_pi_mcu_stream.py`；人工试玩入口：`scripts/view_stage5_v1_5_manual_demo.py`。
- 输入保持 20 Hz observation；candidate stable-window 后再 accept。小变化走 LIGHTWEIGHT，较大稳定变化走不可普通打断的 FULL_DYNAMIC；FULL 完成 quiet snap 与既有 FF fade 后再处理 pending command。
- 当前控制时基仍为 2 ms，reference block 为 25 samples（50 ms）。FULL projection 在 250 Hz 规划，horizon 先取 50 ms bucket，再向上补齐到 4 ms 网格；selected backend 为 cached KKT。V1.5 不改 frozen Ruckig 速度/加速度/jerk limits、A/B、K、`lambda_ff=0.6` 或 fade 曲线。
- 同一 V1.4 episode 的 cached-KKT 250 Hz replay：planning warm=3.75 ms、post-interpolation 500 Hz residual=0.01776、v error RMS/peak=0.05540/0.16734 m/s、pitch tracking RMS=0.06060 rad；无 saturation、无 fall。V1.5 参考流和 4 ms synthetic-delay check 均无 underrun、stale block 或 sequence gap。开发机耗时不是 Pi 5 实测承诺。
- Stage 5 slope-entry evidence 仅在 `|a_ref|≤0.03 m/s²` 时累计，persistence=0.50 s；该约束只作用于 Stage 5 scheduler/runtime 接入，不改 Stage 3/4 默认行为。V1.5 验证 episode false SLOPE=0。
- 本阶段结果：`models/minisegway/stage5/results/STAGE5_V1_5_SUMMARY.md`；完整 JSON/CSV 与图位于同一 `results/` 目录。V1.0–V1.4 的旧入口、旧门限配置和实验输出仅归档在 `models/minisegway/stage5/reference/`，不属于当前快速运行 surface。

## Stage 6 — 2D relative-target follow 与 Pi/MCU radial KF

Stage 6.5 在冻结 Stage 3/4 controller 与 Stage 5 planner/reference stream 上完成当前 follower baseline。观测频率为 20 Hz，机器人 body frame 为 `+x forward / +y left`。运行数据链为：

```text
TargetObservationPacket(x_forward,y_left,capture_time)
  → Pi 接收 observation
  → MCU RobotStatePacket 经 transport 到 Pi state-history
  → capture-time 线性插值 / 对齐失败则丢弃
  → radial KF [d_hat, v_master_radial_hat]
  → Follow Governor
  → FULL longitudinal ReferenceBlock + independent yaw command
  → MCU safety merge / frozen controller
  → MuJoCo plant
```

Controller/estimator 不读取 Master GT、MuJoCo pose 或 controller 对象。GT 只在 synthetic sensor 显示和闭环完成后的评估中使用。KF 参数为 `initial_distance_std=0.05 m`、`initial_velocity_std=0.6 m/s`、`measurement_distance_std=0.03 m`、`master_accel_std=0.8 m/s²`；Governor 只消费 KF 距离和径向 Master 速度，以及按 capture time 对齐的机器人速度。当前 synthetic KF 配置使用 60 ms camera processing delay；延迟与 jitter 压力测试使用 80 ms 与确定性 ±8 ms jitter。

### 最终 Governor 双窗口锁存

CATCH_UP→CRUISE 时保留最近 20 个有效 KF Master radial-velocity 样本，默认锁存 20 帧 median；若 5 帧 median 比 20 帧 median 低超过已有 `slowdown_velocity_deadband_m_s=0.05 m/s`，则快速通道采用 5 帧值。CRUISE 的锁存语义保持不变；非停车 slowdown 还需要 long-window 证据，5 帧 median 进入 deadband 时可走原快速停车路径。Follower 参数位于 `stage6_4_2d_follow_config.json`：`d_target=d2=1.1 m`、`d1=1.4 m`、`T_catch=3.0 s`、速度范围 `[0,0.6] m/s`、有效制动减速度 `0.25 m/s²`、重触发 margin `0.05 m/s`、连续判定 3 帧。

60 s 的 0.40 m/s 直线对照（只计 20–60 s）：

| 锁存方案 | Vx Governor 事件 / FULL | Vy Governor 事件 / FULL | Vx 距离均值±std | Vy 距离均值±std |
|---|---:|---:|---:|---:|
| 原单帧 KF Vm | 9 / 9 | 10 / 10 | 1.253±0.110 m | 1.231±0.109 m |
| 仅 5 帧 median（被拒绝） | 11 / 11 | 12 / 12 | 1.263±0.108 m | 1.216±0.126 m |
| 当前 5/20 双窗口 | **0 / 0** | **0 / 0** | **1.144±0.029 m** | **1.113±0.014 m** |

双窗口在这两个稳态方向场景中把 Governor/FULL 重复事件清零，并把距离标准差分别降低约 **74% / 87%**；这不是任意 Master 运动下无 hunting 或距离安全的证明。当前 transition 日志还保留 5/20 median、单帧估计和旧单帧 latch 的诊断 shadow，shadow 不接 actuator。

### KF 数据与动态变速结果

带噪转弯场景中，KF/raw finite-difference radial-velocity RMSE 为 **0.056/0.830 m/s**，Governor 事件为 **7/46**；机器人持续转向时的 KF 噪声抑制明显。无噪声回归反而是 KF RMSE **0.029 m/s**、raw **0.015 m/s**，体现滤波响应滞后。所有 KF 运行均有 capture-time RobotState 覆盖，没有 history miss；到达时使用最新 RobotState 会与正确插值产生最高约 0.054 m/s 差异。

最近的零噪声直线速度阶跃验证中，0.20→0.40 m/s 后，追赶状态持续约 6.1–6.2 s，CRUISE 锁存误差小于 0.001 m/s；转入 CRUISE 后延长观察 11.0–11.8 s，没有新增 Governor 事件。0.40→0.20 m/s 后 0.10–0.25 s 即进入 CRUISE，但 20 帧历史仍约 0.401 m/s；仅在最晚一组，5 帧通道把锁存值降至约 0.350 m/s。随后 slowdown 事件把命令带入新速度 ±0.05 m/s 的时间分别约 0.85、0.90、3.65 s，最后一组期间距离降到 **0.759 m**。无周期性 hunting、fall、轮饱和或 reference underrun，但减速 transient 的距离超程仍是当前风险。

### 当前入口与边界

- 当前 2D viewer：`scripts/view_stage6_5_manual_2d_follow.py`；独立 Master 控制窗口以 forward/lateral 速度 slider 连续移动，范围 ±0.50 m/s、步进 0.02 m/s；MuJoCo HUD 与右侧内置面板默认隐藏。
- 当前配置：`models/minisegway/stage6/config/stage6_4_2d_follow_config.json` 与 `stage6_5_radial_kf_config.json`。Stage6.2–6.5 的 packet/safety、wait/resume、2D/yaw 与 KF 验收产物留在 `stage6/results/`。
- 早期 Stage6.1、旧 deadband、窗口候选和 hunting 诊断现已归档到 `models/minisegway/stage6/reference/`；当前 quick-start 只指向 Stage6.5 viewer。
- 折返最近距离曾到 0.685 m，动态减速测试曾到 0.759 m；非倒车 follower 不提供固定 d2 安全保证。Stage 6 尚未连接 Stage 7 真实视觉；独立硬件时钟、真实 UART/USB transport 与 STM32 实测仍待实现或验证。

## 当前可试玩 demo

- Stage 4 frozen slope compensation：`scripts/view_stage4_slope_compensation_demo.py`，依次展示 ±8°、±15°，Q actuator OFF；使用 Stage 4 controller/estimator 的在线坡度估计与 `theta_eq` 补偿。
- Stage 5 V1.5 manual command：`scripts/view_stage5_v1_5_manual_demo.py`，slider raw `v_cmd` 走 candidate/scheduler、planner、50 ms stream 与 frozen controller，不直接改 `v_ref`。
- Stage 6.5 2D relative follow：`scripts/view_stage6_5_manual_2d_follow.py`，独立窗口控制 Master forward/lateral velocity；完整 observation/KF/Governor/FULL/yaw/MCU 链保持生效。

以上 demo 只用于仿真可视化，不表示额定实机坡度、安全或 Pi/STM32 性能认证。
