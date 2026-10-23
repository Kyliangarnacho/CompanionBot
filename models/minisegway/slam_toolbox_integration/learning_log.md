# SLAM Toolbox 集成 learning log

## 成熟算法的适配首先是输入与运行合同

Jazzy 原版 SLAM Toolbox 可直接消费当前原始 LaserScan 和 sensorized TF。没有理由因为车身会 pitch 就先研发补偿：固定平地双圈中最大 pitch 7.70°，仍产出合格地图。这个结论只适用于已测场景；回波中确有地板命中，不能推导出动态倾斜输入天然满足平面假设。SLAM 对外参的二维缓存也不是姿态补偿。

## 小房间中的扫描质心不是可靠的近邻位置

全视野扫描的 centroid 常接近房间中心。`use_scan_barycenter=true` 配合 3 m 搜索半径，可能让 near-linked 图遍历覆盖整圈，使回环候选链为空；两圈路径不等于两圈图约束。实际读 FindNearLinkedScans / FindPossibleLoopClosure 后，改用传感器 pose 的 locality、与路径尺度一致的邻域及足够扫描密度，保留 matcher 的响应/方差质量阈值。图中的跨圈边与历史节点优化修正才是可复查的闭环证据。

## 定位精度、地图几何与覆盖率必须分别验收

早期地图位置误差只有厘米级，最大栅格边界却缺墙。Karto 的尺寸和 WorldToGrid round 可将边界端点落到 width/height 之外，与 upstream #689 相符。位姿没有丢失并不保证地图适合规划。保留独立的 occupied precision、geometry P95、room boundary coverage 和 known-area 指标；不补绘墙、不降低门限，也不因某次最终栅格通过就宣称 upstream 缺陷已修复。

## 有界 source queue 需要正确的仿真墙钟节奏

规划停顿后追赶墙钟会把正确的 20 ms source samples 突发交给 latest 槽，产生 source gap。解决处是 simulation pacing：停顿延长墙钟而不是追赶；不能放宽空间样本 age/gap 门禁，也不能重打时间戳。DDS matched readiness 与消息真正到达仍有区别，实际接收损失和启动前 TF unavailable 必须保留统计。

## Python 循环 GC 也属于 reference 生成的实际执行成本

重复安全停车不能归结为“随机慢”。被动 profiling 明确记录一次第 2 代 GC：52.39 ms wall、56.86 ms thread CPU、`gc_collections_delta=[0,0,1]`，触发原 reference starvation。单线程 BLAS 只约束线程竞争，不能消除此机制。

本工程仅在有界的 mapping physics loop 前收集、期间延后循环 GC、所有退出路径恢复原进程策略；引用计数清理保持。测得的 producer wall time 仍进入原 transport 模型，MCU deadline、队列容量、heartbeat 和 safety 不变。与 GC-on 的无 ROS 对照，41000 个控制 tick 历史逐值相同。对无限运行、Pi 内存或硬实时保证不能照搬此有限仿真策略。

## 上游 API 包含对象生命周期责任

离线读取 pose graph 应使用发行包 Karto API，并复用上游 sensor registration 顺序；仅 LoadFromFile 后让 Dataset 析构，会因 sensor 未注册而异常。检测工具不应实现另一套文件解析。标准 OccupancyGrid 加载仅发布地图，不创造 map→odom；pose graph 从首节点恢复也要求对应已知起点，不能充当未知位置的自动定位。

## 验证与证据的作用

失败记录不能被最终成功覆盖：source gap、空参数解析、GC safety takeover、缺墙和未回环都有各自证据。GT 只进入显式单向 post-hoc 文件和离线评测，不能反馈给 SLAM、规划器或控制。GUI 配置、软件故障夹具、WSL CPU 统计与实体执行/人工观感是不同证据范围。
