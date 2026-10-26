# Native Runtime — learning log

工程进行中。本文件只保存可复用机制与取舍；验证范围和原始证据见同目录唯一final_report。

- 安全屏障必须属于唯一MCU本地状态机。STOP不能只在NORMAL接受；已stopped也可能持有未来armed恢复资格。接受新HARD时立即撤销epoch/旧reference/pending与armed恢复，重复/陈旧sequence则明确拒绝。Pi重复STOP不能代替本地性质。
- 有界反重放表不能简单淘汰最旧identity，否则被淘汰的旧sequence会变成“首次输入”。本轮容量耗尽拒新identity，STOP仍通过并锁后续恢复；日志ring的覆盖必须可见。
- 原因字符串是诊断，不足以证明恢复权限。外部STOP与内部fault可有同名reason；生产transport应携带明确provenance，readiness与MCU真正可恢复条件一致。
- 原生化有效边界是完整责任，不是逐函数包装。500Hz authority/scheduler/MCU调度同进程共享，FULL数学solver只在接受计划时跨一次低频绑定；不能把每个2ms采样交还Python。
- C++持mutex后进入低频Python solver需要GIL；其他Python入口若持GIL等待同mutex会反锁。所有阻塞native锁入口先释放GIL，取完native快照再恢复GIL做JSON转换。
- 原Governor valid=False保留latched command是已有算法语义。LOST/stale/depth不确定应由独立运行权限门禁撤销，不能借移植悄改Governor数学或用失效观测继续移动。
- 检测/Depth独立慢通道需要exact-frame pair和统一发布水位。新成功结果之后到达的旧LOST不能越过水位撤销新task；当前fresh invalid仍须立即撤权。source epoch、capture time、task grant与authority epoch分别检查。
- 图像read-complete不等于acquisition。CPU renderer可能花400ms，之后100ms推理不能报告“source age100ms”。记录capture begin/read complete/ready，安全deadline覆盖完整acquisition→ready；pitch来自同capture horizon的sensorized estimate。
- 成熟人物模型被渲染，不保证检测模型或米制深度成立。Cesium域差异、Gazebo raw尺度错误及heldout标定失败都是真实边界。一次离线fit只能在预先固定heldout达标后成为runtime calibration，不能在线读GT纠偏。
- llvmpipe是软件renderer；720p和只降640p均失败，显式关闭阴影/反射及1 sample才明显改善。该实验改变成像质量，单独记账，不能与同输入Python→C++收益混算。
- 小矩阵/批量外围原生化可减少对象和绑定成本，但Ruckig、SciPy、Torch、OpenCV、MuJoCo本来已编译。空间节点和参考块已有收益，首轮整机RTF反而退化；必须保留负结果并定位剩余owner/通信/physics成本。
- 反应距离加上“当前速度的制动距离”并不足够：反应期间仍可能加速，必须从反应末端v/a预测原C99制动，保留尾部残距、sensor offset和source年龄。不以弱guard换取演示移动。
- 同步软件GL放在physics回调会直接破坏odom receipt freshness；应以成熟MuJoCo state API复制完整渲染快照、私有MjData＋mj_forward，在同一renderer线程管理GL生命周期。不能只复制qpos，也不能用mj_step后可能滞后的派生pose当成同source图像真值；snapshot roundtrip需在相同canonical forward定义下比。
- 异步只改变阻塞位置，不保证完整视觉deadline。原单目并行run的344条acquisition→authority全部超过0.2s；后续真实双目并行run仍只有104/310满足，P95368ms。两者都完成两次GUIDE_TO，但FOLLOW保持拒绝。read-complete→ready合格、部分fresh帧或新增C++测距来源均不能掩盖快照排队/渲染成本和连续有效性不足。
- 库的首次CPU模型setup可覆盖OMP环境变量；本机Ultralytics默认实际为7线程。线程数应以真实worker每call读数证明，通过公开setter与双方lazy setup屏障改变，不能补丁第三方常量。有限对照中1线程使depth显著变慢，2线程节省CPU而4线程改善pair尾延迟；不存在“线程越少越快”的统一结论。
- 线程预算会改变浮点归约。相同模型/input的2/4线程depth输出在本次逐值exact，detector仍有约6.1e−5差异；应记录实际误差与类别/数量，不能从相同权重推断bitexact，更不能据此放宽感知门限。
- CPU微对照必须固定计时窗口的边界：process rusage若在GC恢复后采样、wall却在恢复前停止，两者不再是同一工作窗口。v3修复该归属；旧v2结果保留且不与v3混算。P99改善也不能抹掉1.799ms真实最大尾值，局部收益不等于全系统RTF改善。
- 低频资源采样也有开销。本机/proc全PID树及线程扫描约耗5.1%单核CPU，最大单次73.56ms；RSS求和重复计算共享页，process status的context switch只属于leader。必须同时保留monitor成本、per-TID边界和未观测退出尾部，不把这些数据称为GIL等待或真实唯一物理内存。
- 测试退出挂住应先查运行环境。默认sandbox的AF_UNIX send EPERM会丢失asyncio call_soon_threadsafe的self-pipe唤醒，使shutdown_default_executor长期等待；最小socket探针和线程栈比修改应用异步逻辑更能定位。允许本机socket后550项原回归通过，原失败现场保留。
- 单目depth heldout失败后，用户明确授权扩大到真实双目图像米制来源。新增来源须独立验证，原模型链、冻结跟随算法及精度/时效门限不变；不得以GT代替图像测距，也不得把旧单目失败改写成成功。
- Native软件合同通过与真实测量精度是两件事。OpenCV4.6官方matcher逐像素parity、45pytest/CTest和ASan/UBSan通过，真实双目48pose仍只有31/32有效，RMSE0.06084m/P95误差0.11031m，原门禁失败。不能把C++实现正确解释成视觉闭环成立。
- 稀疏有效深度的median可能有空间偏差。近距同像素matching残差很小，纹理有效样本却偏向较远皮肤/背景；评测GT必须保持整个原visible-torso，不能按算法有效mask选GT。事后分解不等于可在线减去的常数，也不授权改冻结torso统计。
- 同像素对应解决了旧LiDAR proxy与RGB surface不一致的一个问题，但不保证原单目形状可用单scale恢复。预先固定的正scale-only LS、无shift/GT/outlier筛选在同31样本退化至RMSE0.17920m，按停止条件Reject；没有继续拟合变体或以调参替代信息增益。
- 相机source必须覆盖整条acquisition→authority。双目最后144measurement的P95为239ms；即使单帧180ms满足200ms，也只余约20ms有效窗口。低吞吐会在帧间过期，不能从多数单帧fresh推出500Hz控制可连续跟随。
- 验证文件不能只比较source文本或样本数量。实际运行可能加载旧overlay ELF，32条记录可能重复同一pose；应绑定实际module/共享库/OpenCV对象、原detector/depth/ReID模型、camera/protocol/asset，并比较完整distinct Cartesian pose keys。对真实失败文件必须观察生产validator拒绝，不能只测一个伪造失败夹具。
- 跨OpenCV版本只交换NumPy像素，不能把Python cv2 5.x的Mat ABI或数值当成native4.6 parity依据。绑定先做有界输入快照再释放GIL，返回独立拥有存储的只读数组；相同source stamp并不能自行证明左右像素来源，采集owner仍须维护物理camera/snapshot身份。
- 从ament_python迁到ament_cmake时要显式保留旧setup.cfg中的全部Stage7测试注册。根pytest通过不能覆盖ament在build cwd及各包setup.cfg下的导入行为；本轮只修测试path并移除没有Python测试的空注册，保留真实C++测试，colcon复验797/0失败通过。不能把诊断工具变成生产包、给runtime增加隐藏source fallback或用空测试掩盖收集错误。
- 局部加速与whole-window可相反。固定20s physics P/N/N/P中scheduler P99从约385降到39μs，整体wall/CPU仍退化约4.3%；command DTO P99也变差，最大4.079ms。先分清方法计时、模型setup、必要report/日志与调度窗口，记录负结果，不从一个候选JSON复制路径直接推断根因。
- 名称不证明频率。旧`history_50hz`在loaded Stage5实验实际保留500Hz history；必须以10000row/20s和真实dt核查，不能把原本已记录的高频history算成新增功能或声称性能测量只保留50Hz日志。空间DTO的50Hz消费频率是另一个事实。
- parity总布尔值也要审计conjunction。macro所有reference/控制/物理数组exact，但两个时间DTO字段有3.43e−12差，超过固定2e−12；原工具漏检查这些列，补审必须保留原件并标全面失败。映射到同整数tick只能解释时钟表示差异，不能把失败改写成旧epsilon通过，更不应伪造时间戳追求bitexact。
- 低收益Python外围可通过责任消费频率去掉实际浪费。DTO原每500Hz构造、仅50Hz使用；将构造留在消费者路径，原500Hz attitude/visual观察继续。真实mixed14s的7000control history、700sensor DTO逐值保持，当前7000observe仅700次构造且7001attitude更新。旧计数源码推论与新计数实测须分别标注。
- 局部失败不自动授权更复杂的替代方案。双目精度、完整时效和全向clearance是三个独立边界；改renderer、改保护层或下载新库都不能替代原测量/控制验收。有限审计能确认阻塞后，应保留拒绝行为和停止条件，先收齐现有链的构建、故障、导航与文档证据，避免靠新模块不断转移问题。
- 多包colcon工作区的编辑器类型诊断应消费每包CMake编译数据库。只选interfaces项目、手填include或关闭报错都不能代表真实依赖；导出数据库并交给已安装C/C++插件即可复用真实编译器、标准和defines。编译参数覆盖与syntax检查是命令行证据，不能代替未操作GUI的目视确认。
- 原生函数释放GIL后仍须恢复才能返回Python。固定忙线程诊断把恢复wall区间P99从0.271μs推到4.029ms，而控制/物理数组保持exact；无pacing的零underrun与2ms墙钟实时性是不同验收。wall与thread CPU探针顺序也不完全相同，不能直接相减宣称纯锁等待，更不能用外围微优化消除这个架构事实。
