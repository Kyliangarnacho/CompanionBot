# Stage 9 — Windows → WSL2 Ubuntu / ROS 2 Jazzy 迁移

2026-10-05。迁移已达到 Linux 软件开发交付状态：同一项目、同一正式交互入口、原 Stage 3–8 算法/任务语义，新的项目 Python 环境和可构建/运行的 ROS interfaces / adapter / launch。真实设备与实体控制闭环的验证边界见下文，未把回放、Fake 或 skipped 写成硬件通过。未 commit、push、添加 remote 或创建新仓库。

日常运行见 [RUNNING.md](RUNNING.md)，职责、时间、消息和 frame 见 [INTERFACES.md](INTERFACES.md)。README 保留能力/限制和正式启动方式，CURRENT_STATE 记录当前状态；不改写 Windows 历史成果为 Linux 验收。

## 1. 开工状态与复用判断

审阅 AGENTS、CURRENT_STATE、README、Stage 3 manifest / Stage 4 final 与 rejected learning 记录、Stage 5 V1.5、Stage 6.2–6.5 及最新双窗口动态验收、Stage 7 final/config/RUNNING、Stage 8.2 report/RUNNING、当前运行链和测试。关键发现：

- Stage 3 config 已在 `stage3/results/config`，部分历史脚本的旧路径过时；当前 Stage 4/5 resolver 已适配，迁移复用它，不复制一套配置、不修改冻结 manifest。
- Stage 6.5 早期 radial-KF JSON 是单帧锁存，当前 follower 是后续 5/20 双窗口；应比较相同语义的最新动态验收。
- Stage 7 真实视觉未接 Stage 6；depth/geometry 外参、pitch、深度 convention 均有 provisional 边界。
- Stage 8 的 Catalog/Resolver/Supervisor 已有完整 async task 与 source/freshness 契约，执行/导航仍 Fake；不能因增加 ROS 就生成地图或实体任务成功。
- 系统已有 Jazzy desktop/base、rclpy、rosidl、launch、tf2、colcon、rosdep 与构建工具；本阶段检查后复用，没有重复安装 ROS，更没有启动预装的导航组件。

Borrow：官方 Jazzy deb / rclpy / rosidl / ament / colcon、现有 SciPy/MuJoCo/Ruckig/Ultralytics/OSNet/SenseVoice/Silero/Vosk、eSpeak NG/PulseAudio。Adapt：项目 venv ABI、observer taps、source-clock envelope、Linux device backend 与正式入口包装。Reject：ROS 分拆每个控制 tick、重新设计算法、C++/MCU 全量移植、SLAM/Nav2/map pose、DDS 直接 actuator transport、新模型/新控制参数。

官方依据：[Jazzy Ubuntu deb](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)、[ROS Python venv/ABI](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)、[ROS clock/time design](https://design.ros2.org/articles/clock_and_time.html)、[Jazzy QoS upstream](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)、[MuJoCo Python](https://mujoco.readthedocs.io/en/stable/python.html)、[Microsoft WSL USB](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)、[WSLg upstream](https://github.com/microsoft/wslg)。ROS 文档网页在工具中有反爬限制时读取同一官方仓库 Jazzy 源文档；未据第三方教程改控制器。

## 2. 实际环境与改动

| 项目 | 实际状态 |
| --- | --- |
| OS / kernel | Ubuntu 24.04.4、WSL2 6.18.33.2 |
| Python | `/usr/bin/python3` 3.12.3 建立新 `.venv --system-site-packages`；项目均使用 `.venv/bin/python` |
| ROS | Jazzy 系统 deb 复用，生成 Python extension 与 rclpy 使用相同 3.12 ABI |
| 数值 | NumPy 2.2.6、SciPy 1.14.1、MuJoCo 3.2.0、Ruckig 0.19.4 |
| 视觉 | CPU Torch 2.14.1+cpu / torchvision 0.29.1+cpu、Ultralytics 8.4.173、OpenCV 5.0.0.93、现有 OSNet pinned implementation/weight |
| Agent / audio | 原 PydanticAI Slim 2.53.0、Vosk 0.3.45、sherpa-onnx 1.13.8、sounddevice 0.5.6；Linux eSpeak NG / paplay |
| packaging | setuptools 79.0.1（满足 Torch >=77.0.3）、empy 3.3.4；colcon 配置显式 pytest |

MuJoCo 版本依据继承 Stage 1 smoke 中记录的 3.2.0；Ruckig 0.19.4 来自原 Stage 5 结果。控制参数与原数值求解器接口保持不变。具体安装日志、resolved pip snapshot、environment 和 `pip check` 在 `results/`。

真实修改：

- `requirements.txt` 补核心数值/测试依赖；`requirements-linux.txt` 复用现有 camera/yolo/agent/audio requirements，补 ROS build Python tools。`pyproject.toml` 仅增加可编辑安装，未搬移/重写核心包。
- `scripts/env.sh`、`build_ros.sh`、`configure_linux_venv.py` 与 `ros2_ws/colcon.defaults.yaml` 统一 venv / overlay / cache / build / test。ROS packages 的生成与 console-script shebang 使用项目解释器。Matplotlib 的 Ubuntu namespace 冲突仅在 `.venv` 修复，3D import 通过，不改系统包。
- `install_linux_audio.sh` 提供 Ubuntu amd64 官方 deb 的项目内解包 fallback；`init_local_rosdep.sh` 在 `.venv` 建立官方 metadata/cache。本机 sudo 需要密码，所以未改系统音频包、全局用户配置、设备权限或 Windows 设备绑定。
- `embodied_agent/audio.py` 平台选择 Windows SAPI/COM 或 Linux eSpeak；WSLg 有 PulseServer 时采用 eSpeak WAV synthesis → paplay。原 FIFO、generation、echo-text veto、streaming、half-duplex/打断/取消策略保留。Linux implicit input 仅配置默认输入，camera/monitor 禁止隐式回退。
- `embodied_agent/devices.py` 隔离 winreg/pycaw，Linux 诊断仅做枚举/PCM 统计，不保存原始麦克风录音。`ui.py` 与启动提示按配置的数据源说明状态，不把回放/无相机描述为 C920 或默认 Realtek。
- `demo_stage8_interaction.py` 仍唯一正式入口，增加 `main(argv)`、`--ros`、Linux backend/device path、`--video`、诚实的 `--no-camera` 和有限诊断退出；Windows 默认 MSMF 路径保留并在 Linux 拒绝。无相机时真实 Master buffer 不可用，不授予 FOLLOW。
- `run_stage8_agent.py` 在原 session 中接 ROS executor / feedback / state taps、统一输出目录与清理；launch 的非 TTY 不当作 stdin EOF 退出。`demo_yolo26n_depth.py` 只增已有 geometry 结果 observer tap，没有重新实现 detector/tracker/depth/geometry。
- `smoke_test_minisegway.py` 增显式 `--output`，避免迁移测试覆盖冻结 Stage 1 报告。其物理与验收条件未变。
- 增 ROS message/service/package/launch、8 项平台 backend 回归、17 项 ROS 契约回归及有限诊断/asset restore scripts。没有另一个交互产品入口。
- 更新 AGENTS 的主环境约束、CURRENT_STATE、README、Stage 7/8 运行文档的 Linux 指引，并新增本阶段运行/接口/报告。

开工 SHA256 对照检查了原 1,676 个文件；当前只有 13 个既有文件属于本阶段预期修改。**1,093 个冻结控制/仿真/感知算法、原测试、Stage 3–6 成果与 plant 文件 SHA256 完全一致**。见 [boundary audit](results/boundary_audit.txt)。Git 中原先已有很多 Windows/Stage 8 改动与 CRLF，未把它们还原或当作本次修改；`pre_migration_git_status.txt` 保留开工状态。检查 whitespace 时用 `cr-at-eol`，不全仓归一化冻结文件。

第三方代码、模型权重和本地派生 CAD 仍在 ignored 缓存；原许可证/来源记录未删除。ROS 包的本地 `LicenseRef-CompanionBot` 元数据不为原项目或第三方新增许可授权。

## 3. 运行与接口

```bash
cd ~/projects/CompanionBot
source scripts/env.sh
bash scripts/build_ros.sh
source scripts/env.sh
python scripts/demo_stage8_interaction.py --no-camera --fake --headless --no-mic --no-tts --ros
# 或同一入口的 ROS launch：
ros2 launch companionbot_ros interaction.launch.py
```

Pi 上层保留 perception / Agent / Catalog / Supervisor / radial KF / Governor / FULL planning；reference receiver、MCU safety / sensor-time estimator / LQR / yaw PD / allocator 仍是未来实时控制侧职责。MuJoCo、synthetic GT、共享 sim clock 与 in-memory transport 属于仿真。当前 Python 足以表达这些边界，因此没有机械拆分节点或改语言。Stage 4 batch 参数求解未来落到哪侧须以截止期/atomic update 测量决定，当前保持原 runtime。

ROS 发布 Color/Master/relative、Fake robot state、async behavior feedback 和可选 frozen control telemetry；typed text input 只投递原 Session，取消/停止 service 只投递原 local control。source receipt/sequence/epoch、clock domain/instance、producer-ready 和 ROS publication time 分开。既有 Pi/MCU bytes 原样封装，仅 diagnostic simulation transport，未部署 UART/USB 或 MCU actuator subscriber。

TF 默认关闭；显式打开时仅广播原 provisional camera mount。无 `map→odom` / `odom→base_link`、没有假 pose、没有 cmd_vel/Nav2 执行路径。`robot_leveled_axle` 仍 source-time pitch leveled 的相对 frame，不是地图 frame。完整 authority、QoS、frame 和 freshness 说明在 [INTERFACES.md](INTERFACES.md)。

## 4. 验证结果

| 验证 | 结果 / 证据 |
| --- | --- |
| 最终 Python dependency check | `No broken requirements found`，`results/pip_check_final.txt`；同一 `.venv` |
| 全项目 pytest + ROS contracts | **327 passed**，无 skipped；原 302、8 Linux backend、17 ROS contract，`results/pytest_delivered.txt` |
| ROS package build | **2 packages finished**，`results/ros_build_delivered.txt` |
| colcon / ament tests | **17 tests, 0 errors, 0 failures, 0 skipped**，`results/colcon_result_delivered.txt`；与上一行 pytest 中的 17 项是同一套，不重复计为新 coverage |
| rosdep / shell syntax | 所需系统依赖满足；env/build/audio/rosdep helper `bash -n` 通过 |
| MuJoCo passive headless | 3 s、状态有限、两轮 analytic cylinder 接地、visual mesh 无碰撞；direct torque 0.12/-0.23 N·m，hard limit ±0.63；`results/plant_smoke.json`。无控制器自然倒下属于原 plant smoke 预期，不当作平衡通过 |
| Stage 3 moving payload Q-OFF | 18 s 原场景、原 seed/history、原 mechanical health gate 全通过；pitch GT RMS/peak 6.088°/13.810°、无 fall、payload contained、wheel saturation 0、`u_Q_used=0`；`results/extended_control/stage3_moving_q_off.json` |
| Stage 4 final closeout | 原 payload ID + centered/left/right 偏载 + left/right impulse 共六场景；31 次自然更新、accepted，估计 0.347625 kg / 15.176 mm；五个 robustness 场景无 fall/轮饱和，Q/slip OFF；`results/extended_control/`。质量偏高的原限制保留 |
| Stage 6 radial replay | 原 5 个 complete 场景 finite、无 fall/saturation/underrun/history miss，GT_runtime_dependency=false；`results/control/` / `control_audit_delivery.txt` |
| 最新双窗口历史复现 | 原 8 个动态变速场景（包括完整延长观察，原 -1 ms observation phase）；event 时刻到 1 μs、CRUISE sequence、FULL 数量与 alignment count 一致；最大锁存差约 8.55e-14 m/s；`results/dynamic_control/comparison.json`。场景/时长/门限未变 |
| 实际模型 CPU inference | YOLO 检出 fixture 中 3 人、OSNet 512 维 unit feature、depth 720×1280 / 921600 finite pixels；SenseVoice 识别官方中文 wav，Silero silence rejected、Vosk silence empty；`results/models/summary.json` |
| 完整感知 + DDS 回放 | 合成 120 帧、1280×720/10 Hz，沿原 full chain；最终 DDS **120 Color / 105 Master updates / 75 relative observations**，同源时间/sequence、ready time 与 provisional flags 保留；`results/perception_dds_delivery.txt`、`perception_ros/`。明确 fixture track 1；不认证身份/距离 |
| 正式入口回放 | 原正式 Demo + Fake Agent + ROS + original Stage 7 worker 正常退出，无 GUI/麦克风/TTS；Master 未人工选择；`results/formal_replay.txt` |
| ROS launch/service/topic/clock/TF | 同机 freshness/duplicate/stale rejection、无 Master 的 FOLLOW 拒绝、GUIDE_TO Fake ACCEPT/CANCEL task 身份保留、正常退出；只 opt-in camera TF，/clock 与 raw sim time / ROS header 分开，1/2 ms frozen telemetry；`results/ros_smoke_final.txt` / `ros_smoke/summary.json` |
| WSLg PCM | 实际 3 s / 48000 samples / 16 kHz mono s16，peak 366、RMS 23.75，未保存录音；`results/microphone_pcm.txt`。只证明 RDPSource 桥读到信号 |
| Native TTS | eSpeak Mandarin + paplay，COMPLETED 4.202 s、cancel 0.104 s、pending=0、worker 停止；`results/native_tts_delivery.txt`。是 process/FIFO 完成与取消，非声学起声/可听确认 |

原 Agent/Catalog/Supervisor 回归包括 sleep 零请求/零自然问答取帧、流式取消、检索消歧、source/clock/freshness、task/progress/cancel/unknown/late feedback 等已有测试。Linux 回归不发云端请求，不接硬件，不更改原 acceptance limits。

## 5. 迁移失败与修复记录

保留失败日志，不把它们算通过：

1. 初选旧 Ruckig 0.15.3 触发其 build config 与现代 build backend 冲突；查原 Stage 5 结果后恢复原 0.19.4 wheel。MuJoCo 也依据原版本记录选择 3.2.0，不用最新版替换 plant 行为。
2. 沙箱内 PydanticAI FunctionModel / AnyIO 测试出现 30 s timeout；同一套、同一解释器在正常环境通过。临时 AnyIO 4.12.1 仍不能解决沙箱问题，因此恢复 4.15.1。不是更改测试 timeout 或 Fake 结果掩盖失败。
3. 初次 ROS sim 走历史 Stage 3 旧路径而失败；改 adapter 复用原 Stage 5 resolver，不改冻结 config 或历史脚本。交互退出后最后一个 UI event 访问已关闭 node，清理后清空 bridge 引用修复。
4. 初轮把当前双窗口代码与早期单帧锁存 JSON 比较，事件数不同。历史 event 没有 long-window fields，源码/最新报告已有这些字段。原五场景对照差异仍在 `control_regression.txt`；正确 audit 标记 same_behavior_version=false，另复跑最新对应历史八场景获得相同行为证据。没有复活单帧算法、调门限或挑选通过场景。
5. Ubuntu system Matplotlib namespace 遮蔽 pip mplot3d；用 venv `.pth` 的正常 namespace import 修复。[Python site 官方说明](https://docs.python.org/3.12/library/site.html)支持这一机制；只写 venv，不改系统。
6. TF adapter 遇 SciPy 对 read-only rotation matrix 的 buffer 要求；给转换函数传副本，原不可变 extrinsic 对象保持。
7. colcon 默认选择 unittest，未发现 pytest。尝试旧 setuptools 后最终 pip check 发现与 Torch >=77.0.3 冲突，拒绝保留此组合。最终恢复 79.0.1，用官方 colcon `python-testing: pytest` defaults；build/test/pip check 全部重新通过。setup 保留现代 test extra，未全局升级/修改 apt 的 colcon。
8. 本地 eSpeak data 实际位于 Ubuntu multiarch lib 下；修正 lookup。随后 WSLg 原生 eSpeak/PortAudio 输出偶发 timeout；使用独立 WAV synthesis + 原生 Pulse paplay adapter，实际 completion/cancel 和双进程错误/取消回归通过。诊断脚本自己的回调签名与缺 import 失败也保留，最终结果无 callback exception。

初次失败与最终记录都在本阶段 `results/`，旧 Stage 3–8 原始成功/失败成果未删除或覆盖。

交付收尾只清理核对过的本轮源码 `__pycache__` 与 ROS pytest cache（9 处），保留 `.venv`、ROS build/install 与所有成功/失败证据。误写到仓库根 `log/` 的两次 colcon test-result 日志移至 `results/colcon_result_logs/`，运行文档显式指定 `--log-base ros2_ws/log`；清理范围记录在 `results/cleanup_scope.json`。

## 6. 未验证与下一阶段边界

- **C920 in WSL2**：无 `/dev/video*` / USB bus，本轮未验证真实 capture、GUI Master 选择、真人 LOST/ReID、设备对应 K/D 或距离精度。回放和 source/geometry fixture 只验证软件链。
- **真实麦克风/扬声器**：WSLg RDPSource/RDPSink 可读/播放进程可完成，物理设备身份未确认；真人 wake/ASR、扬声器可听效果、声学起声、噪声/远场、AEC、重连与长期稳定性未验证。
- **真实 Qwen Linux 请求与 Windows 实机**：本轮未读取密钥/调用 API；Windows SAPI/MSMF/Realtek 的原证据保留，只有平台策略 fixture 回归，不声明重新做了 Windows 硬件测试。
- **嵌入式与视觉运动闭环**：没有真实 Pi/STM32/UART/USB、电机/IMU、measured clock offset/drift/exposure latency、真实 pitch history 或 Stage7→6 connection；仿真 timing 与 planning wall time 不等于硬件实时性。
- **SLAM / Nav2**：未实现，未启动。没有 map、odom pose、destination→map pose、navigation action 或安全实机执行证据。此阶段提供接入边界与 task/feedback 契约。
- **历史实验范围**：未重新训练已拒绝的 Stage 4B/B-R 学习模型，未全量复跑所有旧坡面/扰动/negative-person calibration 网格。被冻结的历史 operating envelope 与 Stage 6 最近距离 0.685/0.759 m 风险不被迁移扩大或消除。
- **GUI**：没有任何 GUI/桌面自动操作。需要 MuJoCo viewer、preview 和真人交互时，由用户按运行文档人工执行。

可以在当前 Linux/Jazzy 主环境继续开发下一阶段，但下一阶段仍需明确选定真实 odometry/SLAM frame 与 clock 来源、实测 camera/pitch 对齐、Nav2 adapter 的 destination / authorization / async feedback，以及 MCU 的独立 safety / measured transport。当前接口没有用虚构数据填补这些缺口。
