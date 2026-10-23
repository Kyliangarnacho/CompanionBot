# ROS 2 运行架构完善与数据流规范化 — final report

2026-10-21。当前实现、两张系统/实际数据流图、接口、生命周期、依赖与调试命令统一维护在 [运行架构](/home/kylian/projects/CompanionBot/docs/RUNTIME_ARCHITECTURE.md)。本次是一次完整架构工程，不另设编号阶段或逐步正式报告。历史源码/失败/冻结证据保留，未 commit/push。

## 实现结果

保留七包和成熟算法，将原 interaction 的通信责任拆为高层 `interaction` 与感知 `perception` 两个 Node，默认同进程静态组合、各自独立 executor。原 Camera owner、推理 workers、Frame/Master 本地消费者保持；不引入 RGB DDS 回环或算法一 Node 的机械拆分。无相机只启动 interaction。

相机 observer 只交接三个 latest 槽，序列化/DDS 由感知 executor 完成；无订阅者不序列化。仿真 adapter 同样通过一个数值槽交接遥测；500 Hz 控制和 1 kHz plant 原循环保持。运行诊断可见 age、错误、输入容量、superseded/no_subscriber 和 PID。

文字 service 同步接纳到 64 项 inbox；满载拒绝、local control 优先且拒绝不推进 source watermark。新增 response.input_id，保留客户端 provenance / 服务端到达时刻；同一 ID 进入 Session、Agent turn 和 typed task feedback。排队后超过原 1 s freshness deadline 的普通 ROS 文字在消费前拒绝。queued 仍不表示授权/执行。

配置安装到 bringup share，namespace / params_file / agent_config 可明确指定，ROS adapter 参数只读。建立 STARTING/RUNNING/STOPPING/CLOSED/ERROR、标准 diagnostics、后台异常传播、所有资源 cleanup、首帧前有限退出与 SIGTERM；required-process 失败传递到 launch exit code。每次运行独立目录和 runtime manifest。旧重复 ROS smoke 归档，当前只有一份实际运行验收工具。

## 关键类/函数的输入、处理与输出

| 实现 | 输入 | 处理 / 输出 |
| --- | --- | --- |
| `InteractionROS` | enqueue、with_perception、原始 ROS args | 共享 Context、按需建 Node、独立 executors；check_health 传播异常；反序关闭线程/ROS handles；snapshot 输出真实运行归属 |
| `InteractionBridge` | UserInput / Trigger，原 RobotSnapshot / Feedback DTO | source/epoch/sequence/clock/freshness gate；同步 TextInput 入队并返回 input_id；发布高层 typed state/feedback/events/diagnostics，不再发布感知 |
| `SessionInbox` / `TextInput` | 文字/ASR 或带 ID、provenance、deadline 的 ROS text | 有界同步接纳、control 优先、容量拒绝/关闭/计数；Session 单 owner 消费，无未受限 asyncio callback backlog |
| `runtime.session.main` | 原 AgentConfig、FrameProvider、camera_runner、UI/audio 选项 | 保留原 Agent/Supervisor 调度；检查 ROS/monitor/camera 错误、消费 deadline、资源关闭；JSONL 和 runtime manifest 输出 |
| `PerceptionNode` | ColorFrame / Master / relative DTO observer | 每种一槽、保留 adapter receipt；独立 timer 按订阅情况调用原 publishers；输出 typed Topics、计数/age/error；无运动消费者 |
| `PerceptionPublishers` | 原 DTO 与可选 received_at_s | 原 source time/sequence/epoch 与 produced time 保留，Header 仍为 ROS publication time；不改像素/geometry/authority |
| `run_perception` / app `camera_runner` | 正式 app 参数、shutdown、observers | 转换到原完整 session，保留唯一 Camera；统一 ROS/本地 source epoch；CLI SystemExit 转成可清理的 worker 异常；退出清空 buffers |
| perception `session.main` 的 stop 检查 | shutdown_event | 模型加载前后与零帧结束处识别正常停止；原推理/配对/阈值不变，不将用户退出当成无数据失败 |
| `FrozenSimulation.observe/publish_pending` | 仅 estimate_after、requested/actual wheels、time/dt/saturation | 500 Hz observer 按原 10:1 抽样后只存数值；ROS timer 发布 telemetry/clock，记录合并，不读取 sim/GT 或回写 actuator |
| `positive_period/require_host_time/publish_status` | Node、只读 adapter 参数、状态字典 | 验证有限正周期、拒绝 use_sim_time=true、输出标准 DiagnosticArray；不供控制核心导入 |
| bringup `exit_with_runtime` | required process exit event | 正常结束关闭 launch，非零退出让 launch 失败；不自动重启或重置任务/source epoch |

## 验收结果与证据

所有实际命令均使用 `/home/kylian/projects/CompanionBot/.venv/bin/python`，Python 3.12.3，pip 24.0 属于同一 venv；每批测试前核对解释器/版本/pip。没有安装或切换依赖。数据目录为 [`results/20261021T1335`](/home/kylian/projects/CompanionBot/models/minisegway/runtime_architecture/results/20261021T1335/)。

| 验证 | 实际结果 | 原始证据 |
| --- | --- | --- |
| 独立 clean build | 七包通过，36.5 s；隔离 build/install/log，清除旧 overlay 环境，只用 Jazzy underlay | `clean_build.txt`、`clean/` |
| 正常 workspace build | 七包通过，28.0 s；常用 scripts/env.sh overlay 可用 | `workspace_build.txt` |
| 最终完整 Python/ROS | **346 passed / 0 skipped，9.49 s**；原 332 用例保留并增加 14 个边界用例 | `pytest_final.txt`；更早批次也保留 |
| 七包 clean colcon test | **347 tests / 0 errors / 0 failures / 0 skipped**；346 pytest cases + 一个接口 CTest 外层，不重复计算 coverage | `colcon_test.txt`、`colcon_results.txt`、`clean/test_log/` |
| Python/ROS 依赖 | pip check / rosdep check 通过；rosdep 有既有 pkg_resources deprecation warning | `pip_check.txt`、`rosdep_check.txt` |
| 冻结控制复现 | moving-payload Q OFF、坡面/偏载/冲激六场景、五个 radial 场景全部原 gate 通过 | `extended_control/`、`control/` |
| reference stream | 原三 solver / replay / stream，原 4 ms delay，通过；无 fall、saturation、underrun | `reference_stream/verification.json` 与完整原结果 |
| 八个动态窗口场景 | 原配置与 -1 ms sensor phase；事件/计数/sequence/对齐一致，最大 latch delta 8.554268404736831e-14 m/s | `dynamic_control/comparison.json` 与每案结果 |
| 完整冻结行为比较 | **76 项 exact decoded JSON equality**，完整 metrics/history、事件、数值与 packet 计数，全部一致 | `behavior_comparison.json`，对照原同版本 Linux 证据 |
| 实际正式 launch / Graph | 正确 publisher/service owner、namespace / 参数生效、同 PID 独立 executors、无相机不建感知 publisher；退出后无 runtime Node | `ros_runtime_run2/*_graph.json`、`summary.json` 与 logs/manifests |
| 输入→任务身份 | service input_id == ACCEPT feedback input_id；turn/task/destination 保留，cancel 与 shutdown cancel 关联正确 | `ros_runtime_run2/summary.json`、JSONL |
| 正式感知回放 | 自然 capture_view 成功；没有自动选择 Master；typed RGB / Master 有真实消费 | `ros_runtime_run2/replay/` |
| 完整感知 DDS fixture | **119 RGB / 102 Master / 68 relative**；明确选择 fixture track 1，几何 provisional，stage6_connected=false | `perception_dds/summary.json`、`fixture_metrics.json` |
| 仿真 ROS telemetry | 4 s，**2,000 control samples**，1/2 ms timing；并行模型推理期间 168 telemetry publications / 32 superseded，原完整 history 留存 | `ros_runtime_run2/simulation/runtime.json`、`frozen_sim.json` |
| 失败/退出 | 无效源 launch 非零；真实首帧前 .05 s lifetime 正常 CLOSED、0 frame、线程停止；SIGTERM code 0 / 线程停止、ros2 node list/info 实际检查通过（`signal_shutdown/`） | `invalid_source.txt`、`early_exit/`、`signal_shutdown/` |
| 源/历史/链接审计 | 99 份数值实现/完整原场景 + 1,915 个历史文件 SHA256 比较；15 份共享 tracked model/config 对 HEAD 一致；15 份当前文档本地链接均有效 | `before_hashes.json`、`source_audit.json` |

队列与故障用例还验证：满载不丢已接纳普通输入、stop 清旧意图且保留优先级、多线程容量严格有界、拒绝不污染 watermark、启动/退出正常输入 gate、排队过期不产生 FOLLOW、慢感知 serializer 不阻塞文字服务、无 RGB consumer 不序列化、executor 异常传播、非法配置 rollback、camera 异常仍关闭 ROS 和 Session。

完整 clean build 可从仓库根复现（更换为一个尚不存在的会话目录，保留旧证据）：

```bash
env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH -u PYTHONPATH bash -c '
source /opt/ros/jazzy/setup.bash
.venv/bin/python -c "import sys; print(sys.executable); print(sys.version)"
.venv/bin/python -m pip --version
.venv/bin/python -m colcon --log-base models/minisegway/runtime_architecture/results/YOUR_SESSION/clean/log build \
  --base-paths ros2_ws/src --build-base models/minisegway/runtime_architecture/results/YOUR_SESSION/clean/build \
  --install-base models/minisegway/runtime_architecture/results/YOUR_SESSION/clean/install --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$PWD/.venv/bin/python" -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python"
'
```

## Deviations、失败与取舍

- [upstream 取舍](/home/kylian/projects/CompanionBot/models/minisegway/runtime_architecture/UPSTREAM_DECISIONS.md)基于 ROS 2 Jazzy Composition、Nav2 Jazzy bringup/lifecycle 与 ros2_control Jazzy 官方实现/文档。借鉴分层和顺序，不引入完整 ManagedNode manager 或 controller_manager 移植；当前没有经过验证的热 reset/reconnect/真实驱动合同。没有为形式增加 package 或 planner/controller Node。
- `UserInput.Response.input_id` 是需要全客户端重建的 ROSIDL schema 增量，未提供旧 wire 兼容 shim；原 source/time/authority 与 binary codecs 保持。后续以当前明确合同为稳定基座。
- 初次沙箱执行拒绝本地 sockets，DDS 报 Operation not permitted、离线 anyio 测试也超时；原受限全量运行被中断，失败日志保留，不记为产品回归通过。在自动审批允许本地通信后原测试和最终回归均通过，没有修改算法迁就沙箱。
- 新 PerceptionNode 初版使用 rclpy 保留的 `publishers` 属性名，真实 Node 构造测试发现后改为 outputs；Graph 工具初版把 deque.append 直接用作 ROS callback，其签名无法检查，已改为显式 lambda。这些失败日志保留。
- 当前旧 radial wrapper 会打印早期 Windows 单窗口结果的版本差异；判定使用既有同版本 Linux 完整 JSON exact comparison，并另复现最新 Windows 八场景双窗口结果，没有重写旧 truth/tolerance 或丢弃差异。
- 诊断 ROS timer 会合并旧遥测/图像；实际数量取决于 CPU/DDS。样本合并有计数，完整控制 history 独立保留，不能将名义 50 Hz 当成实时保证。
- 所有旧 stage 文档及历史证据原字节保留；当前长期合同移到 docs，CURRENT_STATE/README/package README 指向它。旧重复 smoke 的源代码归档，仅当前验收工具可运行。原 runner 自带摘要作为原始 run 证据保留，不另写逐 Step 正式报告。

## Remaining issues / 能力边界

真实视觉→follow、实体 Pi/STM32 transport/实时控制、SLAM/Nav2、实际 odometry / destination→map pose / measured clocks 未实现。现有 Backend 仍 Fake，hardware_execution_ready=false；explicit fixture/replay 不等于实体闭环。

camera read-complete 不是曝光时间；1280×720 K 不缩放，外参、pitch、axial-Z 仍 provisional。本次没有物理相机/音频/GUI、真人识别/AEC、真实云 API 的新增认证。原成功/失败证据和来源/许可未删。

静态组合不隔离 native crash/OOM/GIL；ROS diagnostics 不是实体 watchdog。驱动/模型的不可中断 native 阻塞需要未来实测故障策略；当前不提供自动重连、source epoch rebind 或热生命周期管理。

## 清理与文字交付

只清理本轮可证明归属的源码/测试 Python cache，路径/引用核对清单保存在 `cleanup_scope.json`；本轮 colcon test-result 意外生成的根 log 移入本会话，保留日志，不删除有效 workspace build/install/log。所有成功/失败、原配置/history/plot 保留。默认每类临时完整 run 最多三份，本轮未超过。

文字交付主目录：`C:\Users\29603\Desktop\CompanionBot\stage report\ROS2_Runtime_Architecture\`（WSL `/mnt/c/Users/29603/Desktop/CompanionBot/stage report/ROS2_Runtime_Architecture/`）。仓库这份 final_report.md 与 learning_log.md 是独立来源副本；桌面同名文件是本工程交付副本，核对 SHA256 见 `results/20261021T1335/delivery_sha256.json`。每次迭代更新这两份，不建立逐 Step 报告。报告中的外部资料链接指向本仓库绝对路径，保证仓库与桌面副本在当前 WSL 环境均可定位；同目录两份报告互链保持相对路径。当前修改文件精确清单附后。

## Modified files

本工程修改/新增/归档范围（结果目录为 ignored 本地证据）：

```text
.gitignore
CURRENT_STATE.md
README.md
docs/RUNTIME_ARCHITECTURE.md
models/minisegway/runtime_architecture/UPSTREAM_DECISIONS.md
models/minisegway/runtime_architecture/final_report.md
models/minisegway/runtime_architecture/learning_log.md
models/minisegway/stage9/reference/pre_runtime_architecture/README.md
models/minisegway/stage9/reference/pre_runtime_architecture/smoke_stage9_ros.py
ros2_ws/README.md
ros2_ws/src/companionbot_behavior/README.md
ros2_ws/src/companionbot_behavior/companionbot_behavior/app.py
ros2_ws/src/companionbot_behavior/companionbot_behavior/ros/entry.py
ros2_ws/src/companionbot_behavior/companionbot_behavior/ros/input_guard.py
ros2_ws/src/companionbot_behavior/companionbot_behavior/ros/interaction_bridge.py
ros2_ws/src/companionbot_behavior/companionbot_behavior/ros/runtime.py
ros2_ws/src/companionbot_behavior/companionbot_behavior/runtime/inbox.py
ros2_ws/src/companionbot_behavior/companionbot_behavior/runtime/session.py
ros2_ws/src/companionbot_behavior/package.xml
ros2_ws/src/companionbot_behavior/setup.cfg
ros2_ws/src/companionbot_behavior/test/test_ros_input.py
ros2_ws/src/companionbot_behavior/test/test_runtime_ros.py
ros2_ws/src/companionbot_bringup/README.md
ros2_ws/src/companionbot_bringup/companionbot_bringup/processes.py
ros2_ws/src/companionbot_bringup/config/interaction.yaml
ros2_ws/src/companionbot_bringup/launch/interaction.launch.py
ros2_ws/src/companionbot_bringup/launch/simulation.launch.py
ros2_ws/src/companionbot_bringup/package.xml
ros2_ws/src/companionbot_bringup/setup.py
ros2_ws/src/companionbot_control/README.md
ros2_ws/src/companionbot_follow/README.md
ros2_ws/src/companionbot_interfaces/CMakeLists.txt
ros2_ws/src/companionbot_interfaces/README.md
ros2_ws/src/companionbot_interfaces/companionbot_contracts/ros_diagnostics.py
ros2_ws/src/companionbot_interfaces/package.xml
ros2_ws/src/companionbot_interfaces/srv/UserInput.srv
ros2_ws/src/companionbot_perception/README.md
ros2_ws/src/companionbot_perception/companionbot_perception/ros/node.py
ros2_ws/src/companionbot_perception/companionbot_perception/ros/publishers.py
ros2_ws/src/companionbot_perception/companionbot_perception/runtime/owner.py
ros2_ws/src/companionbot_perception/companionbot_perception/runtime/session.py
ros2_ws/src/companionbot_perception/package.xml
ros2_ws/src/companionbot_simulation/README.md
ros2_ws/src/companionbot_simulation/companionbot_simulation/ros/frozen_sim.py
ros2_ws/src/companionbot_simulation/package.xml
scripts/env.sh
tests/test_runtime_inbox.py
tests/test_runtime_lifecycle.py
tools/README.md
tools/verification/audit_runtime_architecture.py
tools/verification/compare_architecture_results.py
tools/verification/smoke_stage9_perception_ros.py
tools/verification/smoke_stage9_ros.py
tools/verification/verify_runtime_ros.py
tools/verification/verify_stage9_audio.py
tools/verification/verify_stage9_control.py
tools/verification/verify_stage9_dynamic.py
tools/verification/verify_stage9_models.py
tools/verification/verify_stage9_stream.py
```
