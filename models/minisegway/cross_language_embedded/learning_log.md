# 跨语言控制与嵌入式平台 — learning_log

日期：2026-10-21。本文件只记录可复用的工程结论、失败机制与取舍；实际结果、文件增量与未验证项见 [final_report](final_report.md)，当前合同见 [架构与协议](../../../docs/CROSS_LANGUAGE_EMBEDDED.md)。

## 语言边界由运行责任决定

七个 package 不等于七个进程，更不等于应该平均分配语言。AI/视觉/Agent 和已有数值研究依赖 Python 生态；串口所有权、partial write、线程退出、deadline gate 等设备基础设施适合 C++；MCU 的确定状态/固定容量数值核心适合 C。把 500 Hz controller 改成 rclcpp timer 并穿过 DDS，并不能因为使用 C++ 就取得实时性。

实际可复用单元应是没有 ROS/HAL/OS/heap/GT 的 C 核心。主机 shared library 和固件静态库编译同一份源文件，Python 作为冻结行为参考逐步退出生产 MCU 路径。主机验收必须把 C 输出接入 actuator，不能只计算后继续用 Python 结果；保留 rich Python 诊断 mirror 时，要明确它不是独立 MCU 运行依赖。

ros2_control 的设备 read/write、故障边界值得参考，但真实反馈缺失时建立 SystemInterface 并无价值。直接接受 wheel effort/velocity 还会绕开现有 ReferenceBlock/safety 合同。先把可用 transport、shared core 和板级边界做实，未来再接确有用途的 state 接口。

## 末位差异可能改变离散闭环事件

`(tau * counts) / cpr` 与 `counts * (tau / cpr)` 数学相等，浮点运算顺序不同。在本工程完整 noise-free 闭环中，初始因式分解足以改变 governor 一次 50 ms 的触发时刻。定位应使用 estimator-only / feedback-only 的公平 ablation，而不是降低扰动、放宽门限、改 plant 或偷看 truth。

恢复冻结运算顺序后，正式 governor 事件对齐，但诊断 shadow 仍可能在临界点跨一个 observation 周期。组件 trace 一致、闭环验收通过和全轨迹逐位复现是三个不同强度的结论。接触求解会放大微小数值差异，归一化近零残差又会放大诊断比例；这些不能统一包装成“epsilon”。保留全字段差异和初始不利结果，才能解释后续板上 libm/优化变化。

因此本轮保留 double、禁用 fast-math/FMA contraction，先取得行为证据。G474 只有单精度 FPU，double 的板上 WCET 仍须实测；只有明确的性能失败才有理由另做 float32 改造及独立公平对照。

## 协议的时间和权限不能靠序列化解决

host CLOCK_MONOTONIC、MCU tick、ROS time 各有自己的时钟域。status 的 host arrival 不是 MCU sample time；同 Linux boot identity 不能推导出跨设备同步。设计时应同时保留 host queue deadline 和 MCU 本地有效期，让最终接收端按自己的时间裁决。设备重启/session 改变要清旧队列，不能自动重放、arm 或重新生成心跳。

保持原 ReferenceBlock binary v1 完整复用，有利于对照。新增的 embedded v2 是外层 framing 与小控制包；显式 adapter 比让 MCU 解析 JSON 或偷偷改变既有 binary 字段更清晰。yaw 原 JSON 保存 binary64，没有带宽证据时不应主动降低精度。

有界 COBS/CRC 需要畸形流、溢出和 delimiter 后恢复测试，也需要独立 CRC/解码交叉检查。只用同一 encoder/decoder 做 roundtrip 可能让共同错误相互抵消。CRC 是完整性检查，不是认证；默认 TX off 也不是权限认证系统。

partial write 的超时不能只丢软件队列后继续发送下一帧；已发出半帧时应终止当前串口会话，由上层明确重启。串口 worker 失败必须传播到进程/launch，不能剩下一个仍显示运行的 ROS node。

## HAL 模板要复用，也要核对目标内存和回调语义

ST 官方 HAL、CMSIS、system/startup、板级模板可显著减少维护成本，但模板不等于目标硬件已验证。例如 G474 的 128 KiB 总 SRAM包含 CCM，DMA 不能把全部当成连续主 SRAM；本轮将主区限制到 96 KiB，独立列 CCM。Keil 和 GNU 必须分别核对 startup/stack/scatter/linker，不能用 GNU 成功冒充 ARMClang 通过。

官方 ReceiveToIdle circular DMA 的回调位置可能在 TC 和 IDLE 重复报告 buffer SIZE。若把 SIZE 立即映射成 0，下一次同 SIZE 回调会重放整块旧数据。保留 HAL 回调位置的 `[0, SIZE]` 语义、用单生产者 ring/DMB 和有界主循环解析，才能保持字节流可解释。电气时序仍需板上确认。

定时器 ISR 只释放调度，不在中断里运行整个算法。主循环先处理控制 deadline，再做有界 ingress；错过周期要记录并禁用输出，不能拿一个旧传感器样本补跑多个控制 tick。DWT overrun、watchdog 是已实现的软件机制，其真实上限/复位仍要实测。

## 启动、安全刹车和物理输出是不同状态

尚无首块时不能假装已运行而立刻触发旧 epoch 的 underrun；同时也不能一直冻结 receiver 等待用户恰好按中某个 2 ms tick。估计有效后按时消费 reference，即使电机禁用；物理 arm 用本地边沿、健康、fresh heartbeat 和近静止连续 reference 条件裁决。

MCU 的纵向 brake/hold 保留平衡控制；传感器/驱动/时序故障触发的是更外层输出禁用。Agent 或 ROS 参数不能成为 physical arm。未知 actual 必须为 NaN＋valid=false，不能复制 allocated torque。status 还应显式记录 sum/diff/guard saturation、故障、epoch、安全 reason 与 rejection histogram；仅有 requested 或“安全”布尔值不足以复盘。

未知驱动器、引脚和传感器不是补几行 PWM/随机 IMU 的理由。明确返回 unavailable 的 board_io，使工程现在可以编译和诊断，又不会把没有实体依据的实现冒充完成。它也是未来换板、驱动、电机标定时应修改的唯一物理边界。

## 选型要比较完整负载和当前可购价格

Pi 4 / Pi 5 的实际价差很小的时候，单看旧售价会误导选择；Orange Pi 的历史低价同样不能代替当前可靠现货报价。G431 和 G474 的 Nucleo 价格接近时，应把 RAM、调试器、通信和后续驱动余量算进升级成本，不只看芯片单价。

检测 inference FPS 不等于 YOLO＋tracking＋ReID＋depth＋ROS 的观测频率。当前 OSNet 只在确认/重识别时工作，既不能当成每帧全行人负担，也不能完全忽略并发竞争。本机三个模型并发显著慢于单独运行，说明参数量或各自 FPS 相加不能替代部署测量。

官方 NCNN detection 和 RKNN INT8 detection 的表格不能证明 depth FP16 或完整 pipeline 可达 20 Hz。参考 Pi 5 4GB 可支撑初版工程、协议和感知测量，但完整吞吐、温升、source age 与 Agent/ASR 峰值内存仍须按原负载实测。没有实物时，保存可在板上原样运行的测量工具比给出未经支持的 FPS 更有用。

## 构建证据与成果隔离同样属于架构

Python/ROS/C/ARM 共同使用一套可追踪的配置来源，比手填第二份 gain 更可靠；配置导出应保存原文件 hash，并提供只检查的模式。上游锁顶层 tag 还不够，HAL/device 子模块也要锁提交。工具链可本地解包到 ignored venv 区域，避免改系统和污染 Python 包环境。

ament_python 转 ament_cmake 后，旧 build 的 pytest.xml 可能留着，造成 test-result 重复计数。独立空目录构建/验收并区分 pytest case、独立 C case 与 CTest wrapper，才有可核验的 coverage 数字。独立固件工程加 COLCON_IGNORE，避免从仓库根发现成第八个 ROS package。

旧测试默认输出路径可能在全量验收时向历史目录新增日志，即使所有原文件 hash 不变。冻结审计既要检查 modified/deleted，也要检查 unexpected added；给测试显式 tmp_path，迁移已确认 Owner 的本轮产物并校验 hash。未知 Owner、有效 build/install/log 和支持结论的失败证据应保留，不做跨工程通用清理。

## 补充：平均带宽不能代替传输与调度期限

8项应用队列只界定应用内存。nonblocking write返回成功表示kernel接受，driver/USB FIFO仍可能积压；tcdrain又可能无限等待。可维护的基础约束是向上取整8N1线速预算、在原deadline内查询driver queue、限制逐帧发送速度，并始终让MCU本地tick作最后裁决。TIOCOUTQ不是远端ACK，host deadline也不是跨设备端到端保证。

队列有待发数据时仍为每帧固定poll等待，会制造与UART字节带宽无关的包率瓶颈。应让idle时等待、pending时及时服务，同时用实际线速约束发送，避免用忙循环或无限kernel缓存换取好看的吞吐。满线速、部分写入、读端停止、带宽下降、heartbeat丢失与status丢失要分别验证；local safety reason能帮助区分真正的接管原因。

限速PTY的token credit不能在空闲时累积后花到未来首帧，否则所谓线速测试可能瞬时交付整个reference。两个方向都应从新空闲帧重新计时，并交叉检查reference实际延迟不短于字节物理下限。软件夹具需要审查自己的模型；修正后重跑，保留此前错误/不利证据，比把夹具结果叫作硬件测试更可靠。

故障实验也要一次只改变一个主要条件。最初“停止读端”叠加了过载，MCU早已制动，不能用来定位读端阻塞；应再以正常流单独停止读端。单独停止heartbeat而保留reference，才能确认reason=heartbeat，不能用同时停所有流后的starvation结果代替。

## 补充：500Hz应量释放至输出，而不只量函数体

函数开始晚1.76ms、计算只用0.59ms，函数本身没有超过2ms，但本周期输出已过期。只在函数返回后检查duration还可能先写一次旧actuator请求。应在写出前同时检查本次释放的age和是否已有新tick，事后继续检测BSP写入超时。真实app的host虚拟cycle夹具可确定性复现这类逻辑漏洞，但注入cycle绝不能被解释成执行速度。

170MHz乘2ms只给出340,000cycles槽位。Cortex-M4F hard-float ABI没有把double变成硬件双精度；ARM disassembly里的libgcc/libm调用、GCC stack文件和调用图可以识别负担与未知项，不能直接推出WCET。源码已知stack链不含库汇编、异常/FPU压栈和未来BSP，也不是完整上界。保持数学语义，只有板上测得明确性能失败后才决定是否做float/CORDIC等另一次公平改造。

DWT观测应标注路径、mode、armed、full-control计数和build hash；未接传感器时跑过的startup短路径没有代表性。分段时间可能含IRQ抢占，callback还漏掉异常入口/HAL前导，最大值不能简单相加当作严格证明。GPIO/TIM counter、IRQ干扰和真实BSP/栈水位测量仍不可省略。软件deadline guard用于检测失败，不等于系统已经具有可认证的实时余量。
