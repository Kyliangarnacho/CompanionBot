# 跨语言控制架构与嵌入式平台落地 — final_report

日期：2026-10-21。工作区：WSL2 Ubuntu 24.04、ROS 2 Jazzy、仓库 `.venv/bin/python` 3.12.3。未 commit、push、添加 remote，未操作 GUI 或实体设备。开工时已有的运行架构修改继续保留；本报告只归纳本次增量。

本报告末已追加 UART / 500 Hz 补充验证；初交付表格保留其原始构建与证据。补充修复后的当前固件为 **Flash 39,140 B、主 RAM 18,280 B（含8 KiB栈）**，仍未获得板上 WCET 或实体 UART 认证。

## 交付结论

已经形成可构建、可继续开发的 **Python 应用 / C++ ROS 串口基础设施 / 共享 C99 控制核心 / STM32 C/HAL** 工程。七个 ROS package 保持；control 成为 ament_cmake＋ament_cmake_python 混合包。C++ 实际拥有串口、异步读写、有界队列、deadline/session 校验、错误退出和诊断。MCU 的 receiver、safety、brake、estimator、固定 LQR、yaw、allocator 使用同一份 C 源码，同时编译为主机 shared library 和 ARM 固件。

参考硬件确定为 **Raspberry Pi 5 4GB＋NUCLEO-G474RE**。固件使用锁定的 **ST 官方 STM32CubeG4 v1.6.1 HAL/CMSIS/startup/system**，已生成 ELF/HEX/BIN/map；不是自制寄存器驱动。提供真实 `.ioc`、MDK5/ARM Compiler 6 工程和 Linux 构建工具，但尚未安装/运行 CubeMX、MDK，也没有烧录或实物认证。

原 Python 默认路径、Agent/Supervisor 权限、Fake Robot/Navigation、1 ms physics / 2 ms controller、冻结参数和 Q actuator OFF 保持。500 Hz 环在 MCU 本地；DDS 只交换块、心跳、yaw、安全消息和降采样状态。没有新增真实视觉→follow、SLAM/Nav2、真实执行 Backend 或自动 arm。

架构图、数据流、完整字节协议、跨语言目录和可复制的构建/烧录命令统一维护于 [CROSS_LANGUAGE_EMBEDDED](../../../docs/CROSS_LANGUAGE_EMBEDDED.md)。研究中的 Borrow / Adapt / Reject、价格及 benchmark 来源见 [UPSTREAM_AND_HARDWARE](UPSTREAM_AND_HARDWARE.md)，经验取舍见 [learning_log](learning_log.md)。

## 调研后的工程决策

先审查七包源码、实际 app/worker/executor 所有权和既有控制/规划/权限合同，再实施。采用 rclcpp 原生 Node/QoS/参数/退出机制；参考 ros2_control 的 read/write、故障和设备边界，但当前没有可靠实体 joint feedback，不建立空 SystemInterface，也不引入绕过 ReferenceBlock 的 wheel effort/velocity 控制通道。Micro-ROS、RTOS、FOC 和重新设计 controller 均没有本轮必要性证据。

Python 保留 AI/视觉/Agent、Pi reference producer、成熟规划、离线 SciPy ZOH/DARE/least-squares、高级 slope/payload/Q 研究模块与冻结对照。没有按文件比例迁移；高频数值核心和设备边界才迁往 C/C++。Python 诊断通过显式 native backend 调用 C，C 输出真实进入 MuJoCo actuator；它不是只计算后丢弃的 wrapper。

ST 上游不进入仓库，在 ignored `.venv/embedded/upstream` 中获取；顶层及 HAL/device 子模块均锁提交、核验 HEAD、保留许可。GCC linker 从官方模板派生，修正为 **96 KiB DMA 可用主 SRAM＋32 KiB CCM**，8 KiB 栈、0 heap。Keil 使用官方 ARM startup 的外部派生副本，仅调整 stack/heap。来源与许可见 [firmware/UPSTREAM](../../../firmware/UPSTREAM.md)。

## 实现与 modified files

下表是本次主要增量，不把用户开工前已有的 perception/behavior/运行架构改动计入本次实现。精确相对开工快照的源文件 SHA256/增量清单保存在 `results/session_20261021/source_audit.json`。

| 文件 / 目录 | 实际新增或修改 |
| --- | --- |
| `ros2_ws/src/companionbot_control/native/{include,src,test}` | 独立 C99 控制/估计/安全/协议库和无 ROS CTest；无 heap、OS、HAL、GT 依赖。 |
| `control/{CMakeLists.txt,package.xml,include,src}` | ament 混合构建、C++17 SerialPort、rclcpp mcu_bridge、原 Python 安装及既有测试接续。旧 setup.py 保留为 legacy pip 元数据。 |
| `control/companionbot_control/runtime/{native.py,embedded_link.py}` | ctypes ABI/version/size 校验；显式旧 packet→嵌入式 v2、COBS/CRC、状态解码。 |
| `interfaces/{msg/McuFrame.msg,CMakeLists.txt}` | 有界 payload 672B、session、sequence、MCU tick、Linux clock identity 和本地 deadline；旧 PiMcuPacket / SourceStamp 不改。 |
| `simulation/.../runtime/native_backend.py`；`scenarios/run_stage3b_yaw_control.py` | 显式 C 验收 backend；原 runner 只加可选 feedback_function，默认 NumPy 表达式不变。 |
| `bringup/{launch/mcu_transport.launch.py,config/mcu_bridge.yaml,package.xml}` | 显式设备、默认只读的有限职责通信入口、required-process 失败传播；不是第二个正式 Demo。 |
| `firmware/{CMakeLists.txt,cmake,app,boards/nucleo_g474re}` | 实际 G474 HAL clock/UART DMA/TIM6/IWDG、调度/故障/物理 gate、board_io、CubeMX/Keil 工程、交叉编译。 |
| `firmware/{generated,upstream.lock.json,reference_hardware.json,UPSTREAM.md,README.md,COLCON_IGNORE}` | 冻结配置及来源 SHA256、依赖锁、参考硬件、许可与边界；固件独立于七包 ROS 构建。 |
| `tools/embedded/*`；`scripts/build_firmware.sh` | 锁定依赖、无 root 的本地 ARM GNU/newlib 工具链、参数导出/检查、官方板级模板适配/检查和构建。 |
| `tools/verification/{verify_cross_language,compare_cross_language,probe_native_roundoff,verify_embedded_bridge,benchmark_embedded_perception}.py` | 原场景完整复用、全字段差异、单变量定位、真实 rclcpp/DDS＋PTY、原模型 CPU 容量夹具。 |
| `tests/test_embedded_native.py` | 19 项 C/Python 参数化回归，含 sensor trace、安全恢复、yaw 时序、协议互通、状态分层。 |
| `tests/test_stage8_{agent,interaction}.py` | 两个既有 Fake 测试显式写 pytest tmp_path，避免测试输出落入历史成果；不改产品逻辑。 |
| `.gitignore`、README、CURRENT_STATE、包 README、`docs/*`、本目录报告 | 当前能力/限制、架构协议、验证和报告索引；本轮 build/log/history/图表只存本目录 results。 |

### 重要类 / 函数的输入、处理、输出

| 接口 | 输入 → 处理 → 输出 |
| --- | --- |
| `cb_estimator_init/reset/update` | 冻结 config、按合同轴定义的 accel/gyro、展开 encoder counts、sample age → 互补姿态、因果加速度补偿、encoder PLL、pitch geometry / control-time extrapolation → 本地 estimate 及诊断；无 truth 输入。 |
| `cb_feedback` / `cb_yaw_estimate` / `cb_yaw_pd` / `cb_allocate` | 固定 gain/error、有效 gyro 时间戳、yaw reference、Nm limit → 保留运算顺序/旧样本不重复积分/纵向优先限幅 → feedback、yaw、请求/限幅/分配及 saturation flags。 |
| `cb_brake_init/step` | 最后生效的六字段 reference＋冻结 brake config → jerk-limited 速度停止及隐藏 reference cubic handoff → 每 2 ms 安全 reference。 |
| `cb_mcu_reference/heartbeat/yaw/safety/resume/step/peek` | 原 binary ReferenceBlock、序号、epoch、本地 tick、本地估计状态 → active/next、失联/饥饿接管、authority、恢复与 yaw rearm → reference、safety state/reason、epoch 和 rejection counters。 |
| `cb_frame_encode` / `cb_parser_feed` / `cb_mcu_dispatch` | 有界 frame 或任意字节流 → COBS、CRC32、格式/会话/本地期限校验 → 完整 frame 或具原因的拒绝；畸形流在 delimiter 后恢复。 |
| `SerialPort` | 显式 path/baud → 独占 fd、termios、poll、partial write 的绝对期限 → 字节或异常；RAII 关闭。 |
| `McuBridge` | 有界 McuFrame＋设备 status → host clock/session/status freshness/双 deadline gate、8 项队列、串口 worker → 低频 RX/diagnostics；worker 故障使进程非零退出。默认 TX off，不生成 heartbeat。 |
| `command_from_packet` / `decode_status` | 原 packet bytes＋显式 session/deadlines，或 144B status → v1 reference 完整复用、JSON 小包显式二进制化，分字段解码 → MCU 命令 / requested、limited、allocated、actual、flags、原因计数。 |
| `Native*` / `CheckedNativeArbiter` / `native_backend` | 同一 Python 测量/场景 → 已构建的 C 库；保留 Python 仲裁作诊断 mirror、逐 tick 核对 → C 实际控制输出和对照证据；上下文退出恢复原实现。 |
| `cb_app_init/poll/control_tick/publish_status` | HAL DMA ring、TIM6 release、本地 board_io → 有界 ingress、C 主干、DWT overrun/看门狗/物理 arm/故障 gate → 实际输出请求与 50 Hz status；未接硬件时不启用输出。 |
| `cb_board_*` | 尚待确认的实体测量、驱动、物理 arm 和转矩反馈 → 明确板级适配边界 → 当前 unavailable、actual NaN、ready=false；不能以 allocated 冒充 actual。 |

## 协议与安全落地

新增嵌入式外层 **wire v2**，不改冻结的 simulation binary v1。小端 16B header＋payload＋CRC32，COBS＋0 delimiter；ReferenceBlock 仍是原 640B、25×2 ms、六字段 float32。yaw 小包保留原 JSON roundtrip 的 binary64 精度。status 144B 明确区分请求、软件限幅、分配、真实反馈、saturation、故障和 rejection 原因；未知 actual 为 NaN 且 valid=false。

Pi host CLOCK_MONOTONIC、ROS time 和 MCU tick 各归其域，不直接相减。bridge 要求同 Linux boot clock identity、≤100 ms host deadline、status ≤250 ms、正确 session 和保守的 MCU expiry；MCU 再按本地 receipt/tick 裁决。未把 status arrival 叫作传感器采样时间，也未声称实现跨设备时钟同步。

MCU safety 刹车保留平衡能力，physical output disable 是另一层。物理 arm 只能来自 board_io 本地边沿及健康/近静止/reference 条件，ROS/Agent 无 arm 字段；关键故障锁存。bridge 不自动重连、重放、恢复运动或代发心跳。`allow_command_tx` 是内部部署开关，不是身份认证。UID＋备份计数 boot session 不宣称跨断电绝对唯一；实际 clock/链路/arm/驱动效果仍须实测。

## tests 与实际结果

证据根：`models/minisegway/cross_language_embedded/results/session_20261021/`。机器可读索引为 `validation_summary.json`；失败、初始实现和不利对照均保留。

| 验证 | 结果与证据 |
| --- | --- |
| 正常 ROS workspace | 七包构建通过，32.1 s：`ros_build_final_verified.txt`。 |
| 独立 underlay clean workspace | 从空目录七包构建 61.0 s；最终增量重建 32.8 s：`clean_build.txt` / `clean_rebuild_verified.txt`。 |
| 全量 Python/ROS | **365 passed，0 skipped，15.64 s**：`pytest_verified.txt`。两处输出隔离修复后再验 2 passed；最终 status 分层改动后 C/Python 19 passed：`test_output_isolation.txt` / `native_status_verified.txt`。 |
| clean colcon | **373 tests / 0 errors / 0 failures / 0 skipped**：`clean_test_result_verified.txt`。这是同一 365 个 pytest case＋独立 C core CTest＋7 个 pytest CTest 外层，不把 wrapper 重复计 coverage。 |
| 主机 C / ASan＋UBSan | CTest 通过。673 个 payload 长度、溢出/截断/CRC/恢复和 200,000 随机畸形字节；`sanitized_verified.txt`。容器下 leak detection 关闭，不冒称 leak sanitizer 检查通过。 |
| C/Python 组件对照 | 4000 tick sensor traces×2 compensation 模式、3000 tick brake×5 初速、2000 随机 LQR/yaw/allocator、gyro freshness、安全/心跳/饥饿/escalation/resume traces；binary64 yaw、原 640B block、独立 zlib CRC/COBS 对照均通过。 |
| 原 Python 不变性 | 与前一轮同 Linux 完整 baseline/history/事件/计数 **76 项 exact checks 全通过**：`python_frozen_compare/behavior_comparison.json`。 |
| 原完整物理场景，C 数值核心 | `native_baseline_final/`：原 moving-payload Q-OFF、payload ID、slope、三 lateral payload、两 wheel impulse；原 runner 验收通过，未更换 plant/gains/truth/tolerance。高级 slope/payload 部分继续是 Python。 |
| 原 radial / safety / dynamic | `native_radial_final/` 五场景 50,000 tick；`native_safety_final/` 三场景 32,000 tick；`native_dynamic_final/` 八场景 117,300 tick。共 **199,300 个 C/Python 仲裁 tick**，state/epoch 一致、reference 最大差 **4.17e-17**；原适用的 finite/fall/saturation/underrun、event/sequence/FULL/alignment 条件通过。动态 latch 最大差 **7.56e-14 m/s**。 |
| 原 stream / planner / 4 ms delay | `native_stream_final/verification.json` 全部原 gate 通过；三 solver benchmark/replay/完整 reference stream 保留。此套使用 C 数值核心，producer/stream 仲裁仍为原 Python，不计入上述 C 仲裁 tick。 |
| 实际 C++＋ROSIDL/DDS＋PTY | **19 checks 全通过**：`bridge_pty_verified/summary.json`。含 TX off/on、session、clock、双 deadline、stale device、NaN/schema、CRC 后恢复、不合成 heartbeat、正常退出、断连非零、sim_time/缺失设备拒绝、原因计数。MCU 对端是软件夹具，hardware_tested=false。 |
| STM32 ARM GNU 交叉编译 | GCC 13.2.1、Cortex-M4F hard-float；最终 **Flash 38,532 B / 512 KiB、主 RAM 18,208 B / 96 KiB（含 8 KiB 栈）、CCM 0**。`firmware_delivery_verified.txt` 与 `firmware_build/companionbot_g474.{elf,hex,bin,map}`。 |
| 依赖/模板/冻结资产 | pip check、rosdep check、上游 commit、导出参数和板配置检查通过。**1538 个开工冻结文件 SHA256 全不变**；旧成果目录无本轮新增输出：`frozen_verified.json`。 |

ARM 链接器对 newlib-nosys 的 `_close/_lseek/_read/_write` 给出未实现提示，并注明可能被 section GC 去除；这是当前无文件系统/stdio 后端的工具链提示，不是已实现的设备驱动。未通过造假的 syscalls 消除提示。构建尺寸不包含未来 IMU/电机驱动，也不能代替栈水位和 WCET 实测。

### C/Python 不是逐位等价：保留的不利证据

初始 C 将 encoder 角度 `(tau * counts) / cpr` 因式分解为 `counts * (tau / cpr)`，虽然只改变末位，却在完整 noise-free 闭环引发主 governor 一次 **50 ms** 离散时刻偏移。用 estimator-only、feedback-only 的单变量实验定位后，恢复冻结 Python 的运算顺序；不改 gains、场景、扰动或验收门限。回归测试固定该顺序，初始结果与 ablation 留在 `native_radial_initial/`、`roundoff_ablation/`、`comparison/`。

最终 moving-payload 的 `v_hat`、`theta_hat`、actual sum 最大差分别 **1.48e-13 m/s、1.75e-13 rad、3.93e-12 Nm**；完整 history 含诊断量的最大差 3.21e-10。正式 radial governor 事件序列对齐，但 **raw shadow governor** 一次 `cruise_resumed` 仍从 sequence 344→343、17.201→17.151 s。它是诊断 shadow 的剩余差异，未隐藏或调门限抹平。

接触/轮冲激闭环轨迹差异更明显：`theta_dot_hat` 最大差约 **0.025204 rad/s**，terminal pitch deviation 指标最大差 **0.005838°**；接近零残差上的 normalized `matched_residual_fraction` 差可达 **0.970575**。这不是可统一称为“浮点 epsilon”的结果；当前证据证明组件对照和原行为 gate 通过，**没有证明所有接触轨迹逐位复现**。Q actuator 仍 OFF，normalized observer 诊断不进入 actuator。所有数值字段、路径差异、离散变化及原始 history 均在 [comparison_final](results/session_20261021/comparison_final/summary.json)，未过滤不利字段。

## 硬件选择与实际容量边界

比较 Pi 4 4GB、Pi 5 4GB、Orange Pi 5，以及 G431、G474、F446 后，选择 Pi 5 4GB＋G474 Nucleo。调查日同一经销商 Pi 4 / Pi 5 4GB 为 $100 / $110；G474 ST 官店 $19.73，参考板卡合计 **$129.73**。另预留 $30–50 给主动散热、可靠卡、台架电源和线材，总工程预算约 **$160–180**，不含底盘/电机/驱动/传感器/电池，且未含运税。Orange Pi 当地更低的可靠官店报价与完整 RKNN 实测可能改变后续选择；F446 是成熟可复用替代，本轮未落地其 board target。

选型没有把硬件参数当 benchmark：原 YOLO26n 640、OSNet x0.25、depth 768 在本机同 1280×720 fixture 的单模型 p50 为 **17.09 / 10.79 / 49.12 ms**；三者并发 p50 **103.52 / 130.71 / 143.00 ms**，进程 peak RSS **580.99 MiB**。这是 WSL x86、2 torch threads 的容量夹具，非 Pi、非整套 Agent/ASR 内存。原始样本/模型 hash 见 `host_perception_capacity/capacity.json`。

官方 Pi 5 YOLO26n 的 PyTorch / NCNN inference-only 数据约 299 / 67 ms；NCNN 检测 FPS 也不能保证含 ReID、depth、tracking 的完整 20 Hz 管线。RKNN depth FP16 现有上游支持，也不能用 detect INT8 benchmark 替代其验证。**Pi 5 4GB 是初版工程/协议/感知测量基座；当前 PyTorch 三模型完整视觉速率仍是实物待测项。** 既有云 Agent 保留，不假设本地 LLM。NCNN/RKNN 优化是可选后续，尚未导出/切换正式 backend，没有降低现有分辨率或 source-age 条件。

## deviations、未验证及 remaining issues

| 项目 | 本轮完成 / 保留边界 |
| --- | --- |
| MCU 高级算法 | nominal 控制主干已 C 化；高级 slope EKF/supervisor、payload ID/Q/Auto Probe 继续 Python，不宣称固件拥有完整高级研究运行时。固定 LQR、Q actuator OFF 不变。 |
| CubeMX / Keil | 从真实 ST 模板适配 `.ioc`、`.uvprojx`、startup/scatter 并静态检查引用；未打开/再生成 CubeMX、未 MDK 编译。由用户日后按合同中的简短步骤亲自确认。 |
| 实体 IMU / encoder / motor | 型号、轴、引脚、物理 arm/急停、驱动使能、Nm→current/PWM/actual 标定未确定；board_io 明确 unavailable。没有“假 IMU/假 torque”充数。 |
| 板上实时与电气 | 170 MHz/HAL 初始化已编译；UART、DMA IRQ、LSI watchdog、DWT WCET、clock drift、栈水位、复位/失联/倒地和供电均未实测。G474 double 数学先保行为，不能用 x86 速度保证 2 ms WCET。 |
| 生产 Pi/MCU 运行组合 | 显式 packet adapter、真实 transport、固件已实现。尚无经过实测时钟映射/链路时延的自动 producer 设备运行组合；正式 Agent 仍无执行权限。 |
| 真正视觉闭环 / SLAM / Nav2 | 均按原范围保持未开发；不以软件夹具或 typed DDS 收发冒称机器人闭环成功。 |
| 数值等价 | 原验收 gate、组件回归通过，保留接触轨迹差异及诊断 shadow 一次 50 ms 变化。ARM libm 运行数值和实测 WCET尚待板上验证。 |

购板后沿这套工程继续：先 SWD/只读 status 与实测周期，再接 board_io 的传感器和本地安全条件，最后完成真实 torque/反馈标定；不必重构 ReferenceBlock、共享 C core 或 ROS 接口。板级替换适配 HAL/MSP/startup/linker/board_io，上位机替换适配部署或推理 backend。

## 清理、成果与正式副本

本轮 build、history、配置、日志、图表和成功/失败证据集中在 `results/session_20261021/`；保留正常 ROS workspace 与独立 clean build/install/log。两处旧测试泄漏的 22 个本轮 Fake 日志先核对 Owner、开工快照、无网络调用及文档引用，再复制/校验 SHA256 后迁离历史目录；本轮误落根目录的 colcon test-result/manual install 元数据和误发现固件的失败 rc 同样迁入本轮目录。范围清单见 `cleanup_scope.json`。未知 Owner 的旧日志和所有原冻结证据保留。

正式文字仅本目录这份 `final_report.md` 与 `learning_log.md`，并按要求保存独立副本至 `C:\Users\29603\Desktop\CompanionBot\stage report\Cross_Language_Embedded\`。来源、绝对副本路径和 SHA256 一致性见本轮 `report_delivery.json`。构建/烧录操作与限制不另建重复报告。

## 补充验证：UART 持续压力与 500 Hz 预算

本节更新同一工程，不另立阶段。证据目录：`results/uart_timing_20261021/`；本节的机器索引为 `supplement_validation.json`，最新正式副本校验为该目录的 `report_delivery.json`。原成功/失败证据、冻结控制参数和 Python 默认路径均保留。

### 结论与必要修复

**默认921600 8N1的持续带宽足够当前块流；不提高baud，不放宽安全期限。** 但平均带宽不能覆盖软件排队问题，本次复现并修复三处具体时序缺陷：

1. worker原先每发一帧前固定最多等待5ms。20Hz reference＋100Hz heartbeat＋100Hz yaw的8秒初测中，1760条输入只发出1609条，151条被queue-full拒绝；正向最坏字节需求仅约20%线速。改为有pending时非阻塞读串口、空闲时仍poll等待，取消无必要的逐帧等待。
2. 原write_all把kernel接受全部字节当作发送完成。115200限速初测出现约22.6KB外部积压、最长约1.82s晚到；8项应用队列不能界定driver/USB缓存。现在按8N1向上取整计算传输预算，host/MCU expiry gate也计入它；完整写入后在原期限内轮询TIOCOUTQ并保守等待一帧线速，再处理下一帧；关闭时best-effort清理driver待发数据。未使用无限阻塞的tcdrain。TIOCOUTQ并不覆盖所有转接器私有FIFO，故这不是远端收包确认，MCU自己的deadline仍负责最后拒绝。[Linux TIOCOUTQ](https://man7.org/linux/man-pages/man2/TIOCOUTQ.2const.html)、[tcdrain](https://man7.org/linux/man-pages/man3/tcdrain.3.html)。
3. 原固件仅比较control_tick函数自身是否耗时2ms，漏掉主循环开始晚及跨入新tick。真实app.c的host HAL逻辑夹具注入“等待300,000 cycles＋函数100,000 cycles”，原实现仍arm且写actuator一次、overrun=0；长函数路径也会先写一次才事后禁用。现在记录TIM6 ISR释放周期，写actuator前检查释放至当前已到2ms或下一tick已释放，并锁存FAULT_OVERRUN；返回后继续检查实际board写入期间超时。三个上述坏路径均变为 **actuator writes=0、overrun=1、fault=4**。这些是注入虚拟周期的逻辑结果，绝非测得CPU速度。

所有C99算法源文件SHA256不变。协议字段、payload长度、25×2ms、heartbeat150ms、starvation12ms、yaw freshness、authority、epoch、host100ms上限、status250ms上限及MCU125tick上限均未放宽；Agent/Fake边界不改。

### 持续吞吐、突发与DMA预算

最坏COBS长度按 `raw + floor(raw/254) + 2`，raw含16B header和4B CRC；delimiter已计入，不把CRC/COBS当成免费。精确计算与原C codec零样本编码对照见 `analysis/uart_budget.json`。

| 消息 | payload / raw / worst wire | 921600 8N1线速时间 |
| --- | --- | --- |
| ReferenceBlock | 640 / 660 / 664 B | 7.205 ms |
| heartbeat | 0 / 20 / 22 B | 0.239 ms |
| yaw binary64 | 8 / 28 / 30 B | 0.326 ms |
| safety / resume | 3 / 23 / 25 B；5 / 25 / 27 B | 0.271 / 0.293 ms |
| status | 144 / 164 / 166 B | 1.801 ms |

当前20Hz reference＋20Hz heartbeat＋20Hz yaw，正向最坏 **14,320 B/s / 15.54%**；反向50Hz status **8,300 B/s / 9.01%**。全双工分别核算，不相加后误当半双工。20Hz reference＋100Hz heartbeat＋100Hz yaw为18,480 B/s /20.05%；再加入各20Hz safety/resume为19,520 B/s /21.18%，仅作字节包络，并非推荐如此触发状态切换。

115200下，当前正向流需求为容量的 **124.31%**，单块57.64ms已经超过块自身50ms时长；460800则为31.08%、单块14.41ms。测试没有降低控制频率或放宽期限补偿115200不足。协议没有限制任意publisher的持续产生率，不能声称任意输入都有吞吐保证；超载必须有界拒绝和本地降级。

两块reference＋heartbeat＋yaw突发约14.97ms；8块reference约57.64ms，8 pending加1在途约64.84ms。MCU仍只有active/next，八项串口队列不是八块执行缓冲。有效safety清除队列，已在途半帧不能撤回，链路阻塞时依靠MCU本地失联/饥饿，而不是承诺主机一定送达stop。

1024B circular RX DMA在满线速下HT/TC间隔约5.56ms、全缓冲11.11ms；4095B可用ring约44.43ms。每2ms最多约184.3B到达，而poll的768B是单次字节处理上限；它不能证明CPU足够快。TIM6优先级1、UART/DMA为2、SysTick为15；HAL＋RX copy仍会抢占main。continuous无IDLE时，末尾字节还可能等至下一半DMA批次。next reference须在active开始后38ms前收到；扣7.21ms线速、5.56ms批次、2ms控制相位，剩约23.24ms给host/USB/parser/clock不确定性，另计idle worker5ms后约18.24ms。该预算尚未用实体链路认证。

170MHz / USART2 oversampling16名义BRR约184，对应约923,913 baud，较配置值高约0.251%；预算保守使用921600。HSI实际误差、USB端UART容差、CDC buffering/包调度均未实测，不能由该整数计算推出误码率或延迟保证。

### 持续压力及降级结果

采用实际安装的C++ mcu_bridge、真实ROSIDL/DDS、PTY，并在两个方向按8N1限速；MCU端使用同一C parser / dispatcher / MCU safety core，每2ms推进本地tick。零reference和明确的静止输入只用于通信/安全夹具；无MuJoCo plant、真实传感器或actuator。测试知道自身MCU时钟，不宣称实现生产跨设备clock mapping；并未模拟HAL执行速度或USB电气特性。

最终正式压力结果为 **12组通过**：以`strict_*`开头，backpressure使用`strict_backpressure_isolated`，另有独立`strict_heartbeat_loss`。阻塞/故意慢写组按设计提前非零退出，不把配置时长称为完整运行时长。

| 工况 | 实际结果 |
| --- | --- |
| 标称流120s | 2400 reference＋2400 heartbeat＋2400 yaw，**7200/7200全部收到**，无MCU拒绝/应用队列拒绝；生产期间MCU始终NORMAL。publish→C dispatch P99 **13.934ms**、max19.958ms；reference最短7.498ms，大于实际662B线速下限7.183ms。 |
| 高负载60s | 1200 reference＋6000 heartbeat＋6000 yaw，**13200/13200全部收到**，无拒绝/非NORMAL；P99 **12.627ms**、max13.868ms。 |
| 30s超载 | 各200Hz reference/heartbeat/yaw，字节需求约155%线速；应用queue-full拒绝12,890条，队列仍≤8；C接收端明确capacity/epoch拒绝并BRAKING→SAFE_HOLD，无自动恢复。 |
| 有效服务带宽骤降20s | 从921600降为等效115200服务率，仍保留原期限。数据在中继/driver中滞留，C接收端拒绝epoch/stale；在生产仍继续时本地BRAKING→SAFE_HOLD。最长晚到约1.804s，**不是成功命令时延**；host write完成无法替代MCU接受，不能据此声称90ms端到端保证。 |
| 配置115200，8s | 80个reference全因stale拒绝；core从未启动，保持startup inhibited。证明该baud不适合现有流。 |
| 八帧突发＋显式HARD safety，10s | safety收到并保留reason=1；8个旧排队意图被superseded，MCU制动/hold，新旧epoch拒绝可见。 |
| producer停止 / heartbeat单独停止 | reference饥饿按reason=3；仅断heartbeat、保留reference/yaw时按reason=2，均BRAKING→SAFE_HOLD。未合成heartbeat或自动resume。 |
| status停止，10s | bridge的MCU期限及随后device-stale拒绝可见，device-stale计数285；MCU本地饥饿制动。 |
| 仅阻塞读端，标称流 | 先正常运行，再停止读取PTY；MCU本地制动/hold，bridge writer按deadline以code1退出。与超载分开检验。 |
| 强制partial write / EAGAIN，10s | 最多17B每次写、每5次强制EAGAIN；**8000次partial、2149次EAGAIN，600/600命令收到**，生产期间无MCU拒绝或非NORMAL。 |
| 强制慢写 / 半帧，8s上限 | 每次1B并延迟1ms，原90ms命令deadline触发，进程code1退出；只有64B被接受，首块未完成，MCU未启动，未重放半帧。 |

夹具也经历了审查修正：最初token credit会在空闲时积累，使首帧提前；该批`*_initial`/`*_final`只保留为探索/失败证据，不支撑最终线速结论。修正为新空闲帧开始时清零额度后，完整重跑`strict_*`矩阵，并检查reference到达时间不小于物理字节下限。最初backpressure还叠加了过载，因此另外用标称流单独阻塞读端，避免把两种原因混在一起。

### ARM静态证据、2ms预算与待实测项

从实际GNU Cortex-M4F Release产物审查ELF属性、nm、objdump、GCC `.su` / `.ci` / compile_commands。保持 **O3、hard-float ABI、fpv4-sp-d16、no-fast-math、ffp-contract=off**；分析构建额外使用`-g -fstack-usage -fcallgraph-info=su`，其BIN与正常Release **SHA256完全一致**。无未定义符号，未链接malloc/calloc/realloc/free。[GCC callgraph/stack说明](https://gcc.gnu.org/onlinedocs/gcc/Developer-Options.html)。

当前Flash **39,140 B /512KiB**，主RAM **18,280 B /96KiB**（含8192B预留栈），CCM0；比初交付增加608B Flash、72B RAM。linked source frame最大为cb_app_poll944B；parser736B、frame encode720B、reference/validation各664B。已知源码调用链局部stack和为1696B，**不包含libm/libgcc汇编、异常/FPU压栈、间接调用和未来BSP，不能称为完整栈上界或用来宣称实测栈余量**。

G474的FPU是单精度；hard-float ABI不意味着double由硬件完成。当前ELF仍链接`__aeabi_dmul/ddiv/dadd`、double atan2/sin/cos/fmod/floor；estimator的静态调用位点含两处atan2、sin/cos和多处double helper。位点包含分支/循环，不能当每tick调用次数，更不能乘一个假定IPC推出WCET。未改float32、CORDIC、滤波器、LQR增益或编译数学语义。[ST G474 datasheet](https://www.st.com/resource/en/datasheet/stm32g474re.pdf)。

170MHz名义时钟下，2ms为340,000 cycles。以下是**购板后的待验收分配**，目前没有任何一项板上测量值：

| 项目 | 预算 | 名义cycles |
| --- | --- | --- |
| TIM6 ISR释放→main控制开始 | ≤200µs | 34,000 |
| control body，含真实BSP与抢占 | ≤1400µs | 238,000 |
| 其他前台不可抢占服务片段 | ≤150µs | 25,500 |
| 预留，尚未证明实际存在 | 250µs | 42,500 |
| 合计 | 2000µs | 340,000 |

`cb_timing`保存DWT最大值，供SWD观察：release latency、control、release-to-output、sensor、estimator、arbitration、feedback/allocator、actuator、parser/dispatch、status、RX callback及ring高水位、control/full-control计数。未知硬件时full-control快路径不能伪装成完整负载；callback不含HAL前导/异常入口，分段观测可能含抢占，不能简单相加。DWT定义复用现有CMSIS，而非自写计时器。[Arm CMSIS Cortex-M4定义](https://github.com/ARM-software/CMSIS_5/blob/develop/CMSIS/Core/Include/core_cm4.h)。

购板后必须测：estimator reset/compensated update及libm数据范围；正常块切换、brake/hold/resume/yaw rearm；LQR/allocator saturation；真实sensor/driver正常与超时路径；完整reference delimiter的COBS/CRC＋两次校验/decode/dispatch；50Hz status编码/CRC/DMA busy；RX HT/TC/IDLE/重复SIZE/wrap/ring-full；TIM6/UART/DMA/SysTick干扰、IRQ入口延迟和被合并的更新事件；冷启动、DWT wrap和带嵌套IRQ的栈水位。需记录build hash、实际clock、模式/armed/full-control计数、测试时长和每工况max，并用GPIO/TIM counter辅助检查硬件tick；计划保存在`measurement_plan.json`。

**当前不能认证G474已具备2ms的执行余量。** 已经证明构建/资源可用、已知超时逻辑漏洞修复，并准备好了可追踪的板上测量入口；没有用PC benchmark、静态指令数或理论周期填补WCET证据空缺。

### 本次 modified files / 复测与限制

- `control/{include/.../serial_port.hpp,src/serial_port.cpp,src/mcu_bridge.cpp}`：transmission_time输入frame byte count→向上取整8N1纳秒；TX gate和write_all计入线速/有期限排空；pending时取消额外poll等待。I/O或期限异常仍传播非零退出，无权限变化。
- `firmware/app/{app.c,app.h}`：control_tick接收捕获的release cycle→输出前/后deadline检查和fault latch；新增只供SWD的cb_timing计数/最大值，不改变wire。未选硬件的board_io不改。
- `firmware/test/{CMakeLists.txt,app_host_test.c,hal_stub/main.h}`：仅host逻辑夹具，执行真实app.c；**9个app case＋1个C core case共10/10，ASan/UBSan通过**（leak detection关闭）。含晚启动、跨周期、长计算、漏释放、DWT wrap、DMA重复回调/协议/ring overflow。
- `tools/verification/{stress_embedded_uart.py,serial_fault_inject.c,analyze_embedded_timing.py}`：有限、无硬件诊断；分别输出逐帧时序/原因、强制TTY partial/EAGAIN计数、真实协议预算和ARM静态证据。
- 当前合同/状态和本报告、learning_log增补；无README能力扩张，无新Demo。

复测：七包ROS构建通过26.7s；全量pytest **365 passed /0 skipped，16.99s**；既有bridge **19/19**；上述host CTest **10/10**；正常及分析ARM交叉编译均通过；上游锁/冻结配置/board project检查通过。**1538个原冻结文件和7个C core源码/头文件SHA256保持不变**。本次未重跑无算法变更的全部物理场景；原完整场景证据继续有效，未声称有新硬件回归。

复测入口（独立诊断，不启动正式Demo或实体设备；ROS需要本机socket）：

```bash
source scripts/env.sh
.venv/bin/python -c 'import sys; print(sys.executable); print(sys.version)'
.venv/bin/python -m pip --version
.venv/bin/python tools/verification/stress_embedded_uart.py --scenario nominal --duration-s 120 \
  --output-dir models/minisegway/cross_language_embedded/results/YOUR_SESSION/uart_nominal
# 另选load/overload/deficit/slow_baud/burst/producer_loss/heartbeat_loss/status_loss/backpressure。
cmake -S firmware/test -B models/minisegway/cross_language_embedded/results/YOUR_SESSION/app_logic
cmake --build models/minisegway/cross_language_embedded/results/YOUR_SESSION/app_logic -j4
ctest --test-dir models/minisegway/cross_language_embedded/results/YOUR_SESSION/app_logic --output-on-failure
```

ARM分析构建须对**全部**C源加`-fstack-usage -fcallgraph-info=su`并导出compile_commands；精确构建命令保存在`arm_fixed/CMakeCache.txt`、`compile_commands.json`，不改变O3/数学语义。`analyze_embedded_timing.py --build-dir <该ARM build> --output-dir <本工程会话目录>`会拒绝缺少核心stack文件的输入。正常烧录仍按既有build_firmware.sh和SWD说明，不能把host HAL夹具烧入MCU。

无板条件下仍未验证UART误码、ST-LINK USB调度/私有FIFO、真实DMA/IRQ延迟、时钟容差、IWDG复位、板上stack/WCET和BSP电气输出。限速PTY、host HAL和静止core fixture分别证明软件链路与逻辑，不替代这些实体结果。
