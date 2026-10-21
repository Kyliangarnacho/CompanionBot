# Stage 9 — Linux/Jazzy 迁移与职责架构整理

2026-10-05。项目已在 WSL2 Ubuntu 24.04 / ROS 2 Jazzy 继续开发，并将初次迁移的通用 adapter 重组为按职责归属的七个 package。此轮是代码搬迁、明确接口与清理；没有增加算法、调参、连接真实视觉控制、SLAM/Nav2 或嵌入式实现。唯一正式用户入口保持 `scripts/demo_stage8_interaction.py`。

当前 [目录/依赖/数据流](ARCHITECTURE.md)、[运行命令](RUNNING.md)、[时间/frame/authority](INTERFACES.md) 为权威文档。初次两包迁移的完整环境决策、失败与验收文档保留在 [整理前报告](reference/pre_reorganization/STAGE9_REPORT.md)，原结果仍在 results 根；本轮结果统一在 `results/architecture/`。

2026-10-21 文字交付整理：本阶段在 `C:\Users\29603\Desktop\CompanionBot\stage report\Stage9_Linux_ROS2\` 统一交付 `final_report.md` 与 `learning_log.md`。前者合并本报告及 ARCHITECTURE / INTERFACES / RUNNING 的当前文字，后者记录可复用结论；原始数据和已有阶段文档保留。以下测试数字为既有验收记录，不代表 10-21 重新执行回归。

## 实际修改

- 初次 Linux 环境、依赖/模型版本、Windows 平台隔离与已完成迁移保持；本轮没有重新安装 Jazzy、升级数值/模型库或改环境策略。根 editable bootstrap 更新到 0.9.1，从七包源码发现 canonical namespaces。
- `companionbot_interfaces`：原 messages/services 不变，增加独立 ROS-free `companionbot_contracts`，归属 binary codecs、source stamps、资产 root helper。CMake 使用官方 Python install-dir helper 避开 rosidl/ament 同包 target 冲突。
- `companionbot_perception`：原 Camera/YOLO/ByteTrack/Master/OSNet/depth/geometry 整体归属 core/runtime；原 publisher taps 从通用 bridge 提取到 perception/ros，仍复用同一个 node/context。camera_tf 是明确 opt-in 的原 provisional mount。
- `companionbot_follow`：原 radial KF/Governor/reference/scheduler/数值规划、Pi producer、bearing yaw；库包，无新增 DDS node、真实视觉 consumer 或 actuator authority。
- `companionbot_control`：原 LQR/yaw/allocator/Stage 4 runtime、安全/reference receiver、latest-yaw；独立 longitudinal/slope/slip estimator 从 sim 归属到这里，encoder profile/loader 为单份共享 sensor contract。核心不导入 MuJoCo/rclpy/视觉库。
- `companionbot_behavior`：原 embodied_agent 与唯一正式 app；ROS interaction entry/bridge/input guard。Agent 不依赖 control/follow/simulation，不获得低层权限。原 Robot/NavigationBackend 仍 Fake。
- `companionbot_simulation`：原 MuJoCo/virtual sensors、in-memory transport/调度、控制 scenario/viewer/benchmark；原离线学习实现归 experiments。frozen_sim 仍独立 control worker + 50 Hz 只读 ROS 遥测，physics 1 ms/controller 2 ms。
- `companionbot_bringup`：统一 launch；interaction 为正式 bringup，simulation 为有限诊断。没有新增第二应用或一算法一 node。
- 两处混合 Pi/MCU 文件按整类定义拆归 producer/follow、receiver+safety/control、transport+composition/simulation；BearingYawServo/follow、MCULatestYawSource/control。其余算法没有为架构重写。
- 根 scripts 仅保留正式 Demo 薄入口和 env/build；安装与资产恢复归 tools/environment，Stage 9 验收归 tools/verification，原阶段场景入口按职责移到 package 的 scenarios/diagnostics/benchmarks。
- 原测试只更新导入/CLI 路径；17 项 ROS 合约按 owner 拆放。新增四项实际依赖/独立导入/权限/唯一入口检查和一项资源定位/encoder 类型检查。colcon 直接收集原根回归，不复制测试。
- README、CURRENT_STATE、Stage 9 三份原文档、新 ARCHITECTURE、workspace/七包/tools/archive README 与当前结构一致。

## 职责与兼容性

package 是源码职责/安装边界，不强制等同进程。默认正式运行只有 /companionbot/interaction；感知 publishers 与之共享 node。可选 /companionbot/provisional_camera_tf；独立 /companionbot/frozen_sim 为诊断。interfaces/follow/control 没有常驻 node。

500 Hz safety/estimator/controller 和 1 kHz plant 没有拆进 DDS。Pi 规划、MCU 消费/安全、plant 调度分别归属 follow/control/simulation；Stage 4 少量 batch ID 保留原时序，未来上移需实测截止期与 atomic update 契约。

根正式命令/CLI 参数保持；Python import 的 `control.* / sim.* / perception.* / embodied_agent.*`、旧独立诊断脚本路径和原 `companionbot_ros` launch 路径变更。外部工具需按 relocation.json 或 ARCHITECTURE 的新归属更新。没有通用旧 namespace 别名。根 editable 与 colcon 安装指向同一份 package 源码，算法不存在双份实现。

source time/sequence/epoch/clock ID、receipt、producer-ready、publication Header 与 MuJoCo clock 各自保留；binary version 1 和 ROS envelope version 2 不混用。Agent queued/ACCEPT/COMPLETE/cancel/task identity 与 Supervisor gate 不变。TF 默认关闭，不伪造 map/odom。Stage 7→6 真实闭环、Nav2、SLAM、MCU 实机 transport 均未实现。

## 审计与 stale 清理

开工读取当前源码、各阶段文档、现有入口/测试与 Git 历史；历史 commit 从最初 V1/sensorization 到 Stage 3–8，记录在 `history_review.txt`。先保存开工 Git 状态、源码 ZIP 与保护清单，再搬迁，不按旧印象设计。

- `relocation.json` 记录 115 个源码文件搬迁及 split/export 对照；原算法 51 文件中的 **306 个完整 class/function 定义 AST** 全相同，原项目 import 的本地名字/alias 无丢失。
- 整理前 **1,758 个**模型、配置、baseline、成功/失败原始证据/provenance 文件全部保持 SHA256，不以更新文档为由改历史结果。
- 原通用 `ros2_ws/src/companionbot_ros` **归档**到 `reference/pre_reorganization/companionbot_ros`；退出 active workspace，保留源码/launch/测试的历史对照价值。
- 原 `scripts/demo_stage7_perception.py` 是只固定 full mode 的重复 wrapper，归档；当前分层诊断直接用 perception/runtime/session。
- 原 `scripts/audit_stage9_boundary.py` 按旧路径/字节比较，整理后失效，归档；由新快照/AST/alias/hash + 数值回归代替。
- 初次两包 Stage 9 文档保留快照；reference 添加 COLCON_IGNORE，不能误构建旧包。
- 旧 build/install 先搬到 /tmp 隔离，待新七包 build/test 完成才删除；清理29处本轮源目录 cache及一次性/tmp搬迁helper。范围见 `cleanup_scope.json`。新活动 build/install、.venv 和成功/失败验证证据保留。
- 其余历史 runner、Stage 4 拒绝学习实现、早期感知层诊断有实验/测试/失败解释价值，保留并分类，没有为了减少数量而删掉。

用户开工已有 Git 改动保留；未 commit、push、创建 remote 或修改 Git 配置。

## 最终验证

以下结果均使用同一仓库 .venv/bin/python 3.12.3，执行前核对 Python/pip。pytest/colcon 中的 fake 是契约 fixture，不是硬件通过。

| 验证 | 结果与本轮证据（results/architecture 下） |
| --- | --- |
| pip / rosdep / shell | pip check 无冲突，ROS 系统依赖满足；env/build/audio/rosdep helper bash -n 通过；pip_check_final.txt、rosdep_check_final.txt |
| 全项目 pytest | **332 passed，0 skipped，8.11 s**；原 327 项迁移回归 + 4 架构 + 1 资源；pytest_final.txt |
| ROS build | **7 packages finished**；首次干净 build 与最终 rebuild 均通过，ros_build.txt、ros_build_final.txt |
| ROS/ament 全量测试 | **0 errors / 0 failures / 0 skipped**；colcon_result_final.txt。333 个汇总结果是同一批 332 pytest cases 加 interfaces 外层 CTest，不相加当额外 coverage |
| 源码/证据审计 | 306 完整定义 body、alias、1,758 文件保护全部通过；source_audit.json |
| Stage 3 / Stage 4 精确对照 | Q-OFF moving payload 18 s 原 health gate，Stage 4 final closeout 原六场景；完整四份 metrics/history JSON 与整理前 Linux **完全相等**；extended_control/、behavior_comparison.json |
| Stage 5 V1.5 | 原三 solver benchmark/closed-loop replay + selected cached_kkt_250hz stream/finalize；无 underrun/stale/gap/fall/saturation/false slope，原 4 ms delay 无 underrun；reference_stream/verification.json、stream_regression_final.txt |
| Stage 6 五场景 | 原 finite/no-fall/no-saturation/no-underrun/alignment/无 GT runtime gate 通过；KF/raw 数值、事件、delay、history alignment、simulation、packet counters 与整理前 Linux完全相等；control/、behavior_comparison.json（合计76项 exact checks） |
| Stage 6 最新八动态场景 | 完整原时长与 -1 ms sensor phase；Windows 对应最新双窗口 event 时刻到1 μs、sequence/FULL/alignment 数量相同；最大 latch 差8.55e-14 m/s，原比较上限5e-5未改；dynamic_control/comparison.json |
| MuJoCo headless | 原3 s torque/contact smoke通过，±0.63限幅/有限状态；无控制自然倒下是原 smoke 预期，非平衡验收；plant_smoke.json |
| 实际模型 CPU inference | YOLO 3人，OSNet512维 unit feature，depth720×1280/921600 finite pixels；SenseVoice 官方wav、Silero silence rejected、Vosk silence empty；models/summary.json |
| 完整感知 + typed DDS | 原120帧软件回放，**120 Color / 82 Master updates / 40 relative observations**；同源/ready/provisional flags preserved、明确选择fixture track1；perception_ros/summary.json。worker吞吐/配对数量受并行CPU负载影响，非固定计数或真实身份/距离验收 |
| 唯一正式 Demo + ROS 回放 | 同一根入口、Fake、无GUI/mic/TTS，120帧 observer失败0，未自动选Master，正常退出；formal_replay.txt、formal_replay/ |
| ROS topic/service/launch/clock/TF | input duplicate/stale rejection、无Master FOLLOW拒绝、Fake GUIDE_TO ACCEPT/CANCEL task/destination身份保留；sim收到30 telemetry，raw sim与ROS Header区分、1/2ms；仅opt-in camera TF、launch子进程清理；ros_smoke/summary.json |
| Agent/Supervisor | 原全量 Agent/Catalog/interaction/dataflow/Linux 回归经 pytest 与 package test：sleep零请求、消歧、stream cancel、freshness/source/clock、异步身份/超时/迟到反馈保留 |
| Native TTS | 原eSpeak cmn→paplay 完成4.410s、取消0.105s、pending0/worker退出；audio/tts.json。只验证进程/FIFO，没有人为确认声学输出 |
| WSLg PCM / Windows | 初次迁移的真实16kHz mono PCM证据保留，本轮未新增真人测试；Windows原MSMF/SAPI设备证据保留，平台策略fixture通过 |

完整运行命令在 RUNNING。没有修改 frozen configuration、truth/tolerance、场景 seeds 或接受标准。Stage 4 的原质量偏高、Stage 6 最小距离和感知 provisional 限制没有因架构整理被消除。

## 整理中发现的问题及修复

1. 第一次控制场景运行发现原 root control export 的 `config_from_dict as disturbance_rejection_config_from_dict` 搬迁时漏了 alias，四个 runner NameError。只补原 import alias，函数 body 不变；保留 control_regression.txt，最终 control_regression_final.txt 通过。新审计同时检查本地 import binding，避免只比较 AST body 漏掉此类问题。
2. 新 Stage 5 验收 wrapper 最初只重定向 PLOT_DIR，原模块 import 时已构造的 PLOT_PATHS 仍指向历史目录，四张PNG被覆盖；保护审计及时检出。先核对 Windows原目录 /mnt/d/project/CompanionBot 对应四张原图与开工SHA256完全一致，保存本轮图到Stage9，再恢复原图。修正仅验收wrapper的PLOT_PATHS，重新运行完整benchmark/stream/finalize后1,758文件全hash不变。恢复计划/证据见 historical_plot_restore_plan.json、historical_plot_restore.json，首次/最终stream日志均保留。
3. 初次 Linux 迁移曾将当前双窗口 follower 与早期单帧结果误比；那些原失败/版本差异证据保留。此次精确对照用整理前对应Linux版本，另按最新Windows八场景验证；没有复活单帧算法或改门限。
4. 原 AnyIO/PydanticAI 测试在隔离沙箱的IPC限制下超时；同一解释器/同一测试在正常本机执行通过。没有改测试timeout或跳过；原迁移失败记录仍保留。

## 未验证与下一阶段

- WSL2 无 /dev/video* / USB bus，C920真实capture、标定设备身份、GUI人工Master选择、真人LOST/ReID和物理距离精度未验证。
- WSLg RDPSource/RDPSink桥不确认物理麦克风/扬声器身份；真人wake/ASR、声学起声/可听效果、AEC/远场噪声、重连/长期稳定性未验证。
- 未发真实Qwen Linux云端请求，未重做Windows SAPI/MSMF/Realtek硬件试验；只保留既有真实证据与跨平台fixture。
- 无真实Pi/STM32/UART/USB、电机/IMU、measured clock offset/drift/exposure latency、pitch history或Stage7→6真实闭环；仿真频率/规划wall time不等于实机deadline认证。
- SLAM/Nav2/map/odom localization/destination→map pose/导航action/实体安全停止均未实现，不是本轮缺失的软件迁移测试。
- 未重新训练拒绝的Stage4学习模型，未全量复跑每个旧扰动/坡面网格或negative-person calibration；冻结历史证据原样保留。
- 没有GUI/桌面自动操作；preview/viewer/真人交互由用户按RUNNING手动执行。

本轮没有观察到被冻结算法或Supervisor契约的行为差异。可观察的兼容性变化是包/import/诊断路径；异步感知吞吐、planning与TTS wall time随机器负载变化，不作为算法语义相等的固定数值。
