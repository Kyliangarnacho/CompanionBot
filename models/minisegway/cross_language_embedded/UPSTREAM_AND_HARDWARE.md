# 跨语言控制与嵌入式平台：调研及选型

调查日期：2026-10-21。软件/硬件事实优先采用维护者、芯片厂商和实际销售方的一手资料；价格是当日网页报价，USD、未含运费/地区税费，不是采购承诺。完整实现与验收见同目录 `final_report.md`；当前合同见 [跨语言架构](../../../docs/CROSS_LANGUAGE_EMBEDDED.md)。

## 源码审查与术语

七个 ROS package 是安装/职责单位，不等于七个 node。正式 interaction 静态组合两个 Python node；Camera/推理 worker 本地共享 Frame/Master，Agent 与 Supervisor 仍调用 Fake Robot/Navigation。follow 没有真实视觉 consumer，MCU 侧过去是 Python 库；MuJoCo 1 ms、controller 2 ms 在同一本地循环，DDS 仅降采样观测。

已有算法应区分：离线 nominal ID-LQR 与在线固定反馈；Pi 上的 reference planning/rolling block；MCU 上的 reference receiver、safety arbiter、jerk-limited brake、互补姿态融合、encoder PLL、gyro relative yaw、PD、longitudinal-priority allocator。payload ID、slope EKF/supervisor、Q diagnostic 不能被笼统称为“LQR”。本次固件迁移 nominal 实时主干，保留高级 slope/payload 模块的 Python 实现和仿真验收；没有把尚未移植的功能写成固件能力。

## Borrow / Adapt / Reject

| 上游 / 一手来源 | 决定与实际使用 |
| --- | --- |
| [ROS 2 Jazzy executors](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Executors.rst)、本机 rclcpp headers/API | Borrow rclcpp Node、QoS、参数与信号退出；Adapt 独立串口 worker、有界队列和 latest telemetry，ROS executor 不运行控制 tick。源码为 `control/src/mcu_bridge.cpp`、`serial_port.cpp`。 |
| [ros2_control hardware components](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html)、[不同更新率](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/different_update_rates_userdoc.html)、[官方 demos](https://github.com/ros-controls/ros2_control_demos) | Adapt read/write、故障与设备生命周期边界。暂不安装/创建 SystemInterface 或 controller_manager：当前没有真实 joint feedback；直接导出 wheel velocity/effort command 会绕开已有 ReferenceBlock 和 MCU 仲裁，空插件也没有实际价值。以后可在现有 transport 上加真实 state 接口，平衡控制继续留在 MCU。 |
| [Casini 等，ECRTS 2019](https://drops.dagstuhl.de/entities/document/10.4230/LIPIcs.ECRTS.2019.6) | 调研 ROS 2 processing chain 的响应时间分析。工程判断：选用 C++ 本身不证明 DDS/通用 executor 满足 2 ms deadline，因此不拆高频闭环到 DDS；本轮没有借论文宣称 WCET 认证。 |
| [STM32CubeG4 v1.6.1](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.1)、[UART ReceptionToIdle 官方例子](https://github.com/STMicroelectronics/STM32CubeG4/tree/v1.6.1/Projects/NUCLEO-G474RE/Examples/UART/UART_ReceptionToIdle_CircularDMA)、NUCLEO-G474RE Templates | Borrow ST 原始 HAL、CMSIS device/core、system、GCC/ARM startup、linker 和 MDK 模板；Adapt USART2 DMA、TIM6、IWDG、170 MHz HSI PLL。版本/子模块锁定，代码放 ignored `.venv/embedded/upstream`；没有把完整第三方固件复制进仓库。 |
| [STM32CubeF4](https://github.com/STMicroelectronics/STM32CubeF4) | 作为 F4 替代方案调查。保持相同 C core 和 wire，替换 HAL/BSP/startup/linker；本轮只落地/编译 G474，未假称 F4 board target 已实现。 |
| [ODrive encoder PLL](https://github.com/odriverobotics/ODrive/blob/master/Firmware/MotorControl/encoder.cpp) | 复核现有 Python 已采用的 linear PLL 语义，C 严格迁移当前 baseline 的 kp/ki、floor、snap-to-zero、pitch geometry。没有引入 ODrive 电机控制栈或更改机械参数。 |
| [Linorobot2 hardware](https://github.com/linorobot/linorobot2_hardware)、[STM32 balance LQR 示例](https://github.com/phandien289/2wheel_balance_LQR) | 参考 host/MCU 分层、编码器与通信实践；Reject 直接移植 differential-drive PID、cmd_vel 路径、不同 plant/state-order 的 LQR 增益与 FreeRTOS 配置。它们不能代替本机冻结行为。未复制其代码。 |
| [SciPy DARE](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.solve_discrete_are.html) 与既有 SciPy ZOH/least-squares | Borrow 已有离线设计与辨识，导出冻结 `K_id`、尺度和参数 SHA256；MCU 不重写求解器，也不在线重调 Q/R。 |
| [Cheshire/Baker COBS 论文](https://www.stuartcheshire.org/papers/COBSforToN.pdf) | Adapt 标准 COBS 有界分帧＋CRC-32/ISO-HDLC；项目只实现固定协议胶水，以 zlib CRC 及独立解码交叉核验。Reject JSON 进入 MCU，以及在本次额外引入 micro-ROS/XRCE agent 和 RTOS；它们尚无必要性证据。 |

## 上位机比较

| 方案 | 内存 / 接口 / 软件 | 当日价格依据 | 判断 |
| --- | --- | --- | --- |
| Raspberry Pi 5 4GB | 四核 A76 2.4 GHz，USB3、GbE、PCIe、Wi-Fi/BT；Ubuntu Server arm64/Jazzy 路径成熟 | [PiShop 官方经销商](https://www.pishop.us/product/raspberry-pi-5-4gb/)：$110、网页有货 | **本次参考平台**。内存够初版有限模型/ROS/云 Agent 开发，CPU 比新购 Pi 4 更有余量；须主动散热及实测模型吞吐。 |
| Raspberry Pi 4 4GB | 四核 A72，USB3/GbE，旧生态成熟；无 Pi 5 的 PCIe 扩展 | [同一经销商](https://www.pishop.us/product/raspberry-pi-4-model-b-4gb/)：$100 | 新购只省 $10，无法抵偿视觉/规划余量下降；已有旧板可做控制/串口开发，不建议本次新购。 |
| Orange Pi 5 4GB / 8GB | RK3588S、四 A76＋四 A55、NPU、M.2、USB3/GbE；原版的 Wi-Fi/BT 不应当作标配；厂商镜像/NPU runtime 需成套维护 | 官方产品站本轮反复不可达，未取得可靠官店现货报价。[实际第三方卖家页面](https://www.newegg.com/p/3C6-01ZY-01FC9) 为 $177.79；另有更高价，不能用历史 $66 当当前价 | 可作为 NPU 优先替代。若当地官店报价明显更低，值得重新核价；本轮没有其完整 YOLO26/ReID/depth 实测，也未用不可信报价断言它必然更便宜。 |
| Pi 5 8GB / H7 起步 | 额外 RAM 或 MCU 算力不直接解决 depth 吞吐、真实传感器/电机标定 | Pi 历次调价可见 [2025-12](https://www.raspberrypi.com/news/1gb-raspberry-pi-5-now-available-at-45-and-memory-driven-price-rises/)、[2026-02](https://www.raspberrypi.com/news/more-memory-driven-price-rises/)、[2026-04](https://www.raspberrypi.com/news/a-new-3gb-raspberry-pi-4-for-83-75-and-more-memory-driven-price-increases/) | 不作为初购默认。未来实测出现内存/调度瓶颈再升级，ROS/ReferenceBlock/C core 不需重构。 |

Ubuntu 24.04 arm64 属 Jazzy 的目标平台，依据 [REP-2000 官方源码](https://github.com/ros-infrastructure/rep/blob/master/rep-2000.rst)。厂商 NPU Linux 支持不等于该板型的 Ubuntu/Jazzy、相机及 RKNN 环境已由本项目验证；Orange Pi 支持范围另见 [官方 build 工程](https://github.com/orangepi-xunlong/orangepi-build)及其原厂用户手册。

### 必须结合完整模型负载

- 当前 detector 是 YOLO26n 640，OSNet x0.25 是 256×128 单 crop、只在 reference confirmation/LOST reacquire 使用，不是每帧对全部行人重算；depth 为独立 YOLO26n-depth 768，严格同 source frame 配对。没有为了选型缩分辨率或更改 Master/几何阈值。
- [Ultralytics 的 Pi 5 benchmark](https://docs.ultralytics.com/guides/raspberry-pi) 给出 YOLO26n FP32、640：PyTorch 299.09 ms，NCNN 67.03 ms，ONNX 125.99 ms。它们是 inference-only；67 ms 约 14.9 FPS，不能写成完整 20/30 Hz 系统。
- [YOLO26 depth 官方表](https://docs.ultralytics.com/tasks/depth) 的 n-depth 768 为 6.3M 参数、46.9 GFLOPs；272 ms CPU ONNX 来自 32-core Xeon，**不是 Pi**。[RKNN 官方集成](https://docs.ultralytics.com/integrations/rockchip-rknn)现已支持 YOLO26 depth FP16，INT8 当前只支持 detect；不能用检测 INT8 benchmark 替代 depth FP16 的速度/精度验证。
- [Rockchip Model Zoo](https://github.com/airockchip/rknn_model_zoo) 的 benchmark/模型导出说明及 [维护仓库内实测报告](https://github.com/airockchip/rknn_model_zoo/issues/454)显示 NPU 对检测有明显潜力，但导出图、量化、预后处理、线程/DDR 竞争都会改变结果。其 YOLO11/8 数据不是当前 YOLO26 完整管线保证。
- 本轮用原三个 adapter、相同 1280×720 fixture、2 torch threads 做可重复 WSL x86 CPU 测量：单独 p50 为 detector **17.09 ms**、OSNet **10.79 ms**、depth **49.12 ms**；三者并发 p50 为 **103.52 / 130.71 / 143.00 ms**，进程 peak RSS **580.99 MiB**。原始样本、模型 hash 与平台在 `results/session_20261021/host_perception_capacity/capacity.json`。这说明“单模型 FPS 相加”不成立；不是 Pi 性能预测，也不是包含 Agent/ASR 的整机内存数字。

因此参考 Pi 5 4GB 的定位是初版 ROS/协议/控制开发和感知测量基座，仍采用现有云 Agent，不默认加载本地 LLM；没有把其标为完整实时视觉运动闭环平台。NCNN 可经已有 Ultralytics adapter 接入，但本轮未装新推理 backend、未改正式模型。购板后原样运行 `benchmark_embedded_perception.py`，再测正式 app 的并发吞吐、峰值 RSS、温升及 source-age；若要维持既有 20 Hz 观测假设，必须取得完整管线证据，必要时用独立加速器、RKNN 或实测 RGB-D 方案。不会靠把采样/验收门限改小来证明成功。

## MCU 比较与参考 BOM

| 方案 | 厂商能力 | 价格/可购性 | 判断 |
| --- | --- | --- | --- |
| [NUCLEO-G474RE / G474RE](https://www.st.com/en/microcontrollers-microprocessors/stm32g474re.html) | M4F 170 MHz，512 KiB Flash；128 KiB 总 SRAM含32 KiB CCM；FDCAN、编码器/电机定时器、ADC、CORDIC/FMAC | [ST 官店](https://estore.st.com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-g474re.html)：$19.73、有货；[Mouser](https://www.mouser.com/en/c/embedded-solutions/engineering-tools/embedded-processor-development-kits/development-boards-kits-arm/?m=STMicroelectronics&series=NUCLEO)约 $19.90 | **实际配置/交叉编译目标**。Nucleo 自带 ST-LINK/VCP，无需初期另购调试器；RAM 余量比 G431 更适合后续驱动、诊断。 |
| [G431RB / Nucleo](https://www.st.com/en/microcontrollers-microprocessors/stm32g431rb.html) | 同为 M4F 170 MHz，128 KiB Flash / 32 KiB 总 SRAM，具 FDCAN/电机相关外设 | Mouser 约 $19.90、网页为 on-order；[DigiKey UK](https://www.digikey.co.uk/en/products/detail/stmicroelectronics/NUCLEO-G431RB/10231584)网页有货 £15.71（不同币种不硬换算） | 能做紧凑控制板，当前 Nucleo 级别省钱有限；新固件含预留栈约18 KiB主RAM，后续驱动余量更紧，不选作初版参考。 |
| [F446RE / Nucleo](https://www.st.com/en/microcontrollers-microprocessors/stm32f446re.html) | M4F 180 MHz，512 KiB Flash、128 KiB SRAM，成熟 HAL/CMSIS/Keil 生态，传统 CAN；不等同 G4 FDCAN/HRTIM | [ST 官店](https://estore.st.com/en/products/evaluation-tools/product-evaluation-tools/mcu-mpu-eval-tools/stm32-mcu-mpu-eval-tools/stm32-nucleo-boards/nucleo-f446re.html)：$18.66；[DigiKey](https://www.digikey.com/en/products/detail/stmicroelectronics/NUCLEO-F446RE/5347712)：$18.82、有货 | 很合理的替代，已有 F4 时应复用；新购只差约 $1，G474 更利于后续驱动/总线扩展。F4 board target 未实现/验证。 |

参考购置：Pi 5 4GB $110＋NUCLEO-G474RE $19.73，板卡合计 **$129.73**。另预留约 $30–50 给 [主动散热](https://www.raspberrypi.com/products/active-cooler/)、可靠 64GB 卡、USB-C 台架电源与数据线；这部分是工程预算，非逐项现货报价，总计约 **$160–180**，不含已有相机、电机/驱动、IMU、编码器、底盘、电池电源方案。没有为了给出“完整 BOM”擅自确定电机电流、驱动器或未知传感器引脚。

未来升级：Pi 只替换 Linux部署/模型 backend；F4/G431/更高 MCU 替换 board HAL、startup/linker 与 `board_io`。升级电机驱动只替换“Nm请求→真实电流/转矩→actual feedback”边界及物理标定，不能把 Nm 当 PWM duty。SLAM/Nav2、实物传感器、FOC/电流环均未在本次顺手实现。
