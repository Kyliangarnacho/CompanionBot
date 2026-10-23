# ROS 2 运行架构 — learning log

2026-10-21。本文件只记录可复用结论和取舍；测试明细、修改文件与剩余问题见 [final_report](final_report.md)，当前合同见 [运行架构](/home/kylian/projects/CompanionBot/docs/RUNTIME_ARCHITECTURE.md)。

- package、Node、进程与算法是不同边界。先检查谁消费数据、谁持资源、什么频率、故障怎样传播，再决定是否拆分。一个相机 owner 的 detector/depth/Master 生命周期适合保留组合；高频 controller/physics 没有理由绕 DDS。
- Graph 只显示 ROS endpoints，不能证明本地算法数据流。既要记录 endpoint owner/QoS/service client，也要检查 observer、最新槽、不可消费的 frame buffer、调用链和最终 actuator authority。没有消费 relative 的地方必须明确标未接通，不能靠增加空 subscriber 掩盖。
- 独立 executor 解决 ROS I/O 对 Agent event loop 的依赖；它不提供 native crash/GIL/内存隔离。共进程保留本地 RGB 消费有实际成本优势，未来拆进程必须同时完成时钟、epoch 和 freshness 的消费者设计。
- latest-slot tap 应只交接引用/数值，序列化和 DDS 在 observer 外执行。没有 consumer 时避免多 MB RGB 编码；有背压时保留最新并记录 superseded。完整控制证据与降采样遥测各有用途，不能用稀疏 Topic 消息数冒充完整 history。
- “queued” 需要同步、可界定的接纳点。先 call_soon_threadsafe 再返回成功，会把真实容量隐藏在另一个无界 callback backlog。容量拒绝不应推进 source watermark；本地 control 要能清除旧意图；排队不能延长原 freshness deadline。
- request identity 应在通信边界生成并贯穿 Session/turn/task/feedback。文本内容和到达时间不足以区分重复、取消、迟到反馈或不同客户端。保留 source epoch/sequence 与服务端 receipt，避免每层重新生成彼此无法关联的 ID。
- lifecycle 状态只有配合实际资源顺序才有意义。CLI SystemExit 不能直接逃入 asyncio worker Task；首帧前退出不是设备失败。先关闭 admission，再取消任务和 worker，最后销毁 ROS handles；一个 cleanup 错误不应跳过其他资源。
- 没有 reset/reconnect 合同就不提供 ManagedNode 空壳或自动 respawn。重启会改变 source epoch 与任务身份；实机 transport 的重连应以后续测量和 reconciliation 合同为依据。
- 同一 Linux boot 的 perf clock 可比较，不代表 Pi 与 STM32 的 clock 已映射。source time、producer-ready、adapter receipt、ROS publication、reference epoch 和 source epoch 都要分别保留。
- 必须实际构造 Node、交换 DDS 并检查进程退出。单纯 import 不会发现 rclpy 保留属性冲突、callback 签名不被接受、全局 node remap 把两个 Node 重命名为同一名字等问题。
- 沙箱拒绝 socket 会制造产品之外的 DDS/异步超时。先用同一个离线用例在允许本地通信的环境确认，保留受限失败，不为测试环境调控制器、放宽 deadline 或假造成功。
- 冻结复现要对照同一行为版本。旧单窗口 Windows 数据不能直接充当当前双窗口的 exact oracle；保留版本差异，用匹配的 Linux 全量 history 和最新 Windows 场景分别验证，不挑有利片段。
- 成果路径属于实验合同。复现 wrapper 要求显式输出目录，包括原 runner 提前物化的 plot/summary 路径；clean build 与日志也留在当前会话。当前合同独立于历史编号，历史证据字节不应为文档整洁被改写。
