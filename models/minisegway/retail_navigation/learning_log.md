# Retail Navigation — learning log

维护日期：2026-10-23。只记录本工程可复用的机制与取舍；实测数值、文件清单和未完成边界统一见同目录 `final_report.md`。原始成功与失败证据均在 `results/session_20261022/`。

## 商业资产应复用布局，运行环境应重新选择

AWS Bookstore 的 SDF、DAE、材质与碰撞可以独立于 Gazebo 使用。先审查实际实例引用，再转换使用到的原型，能避开未使用而损坏的 BookB 文件，也不会把“完整场景”误解成必须导入仓库中所有备用模型。固定 commit、逐文件 hash、转换库版本与派生 manifest，运行时再检查；第三方和派生 mesh 留在忽略的 cache。

COLLADA 的节点变换、长度单位与 SDF instance pose 是三层不同信息。这里的厘米单位必须显式应用；遗漏它会造成百倍尺度错误。使用 pycollada/trimesh 的 graph，局部 XML 注释兼容修复留在内存中，不重写网格解析器。Spotlight URI 的大小写修复、远在 343582m 的单个物体排除、地板对齐均须有原始尺寸依据和记录，不能手调成一套新的布局。

根 LICENSE 的 Amazon permissive 文本与 package.xml 的 Apache2.0 声明不一致。应同时保留，不能用包元数据替代实际许可证，也不能因已能加载而省略 provenance。

## MuJoCo 的视觉、接触与射线必须分别检查

显示 mesh 完整，不代表其物理空间正确。MuJoCo 的单 mesh 凸包会封住非凸墙体、门洞、货架孔隙；开放墙面应沿原三角面生成有限薄度凸体，闭合非凸部分借用 CoACD。其 threshold 是库中的归一化参数，不能直接称为“2cm 精度”。没有需要发明新的通用凸分解算法。

碰撞组设置 alpha=0 会让 MuJoCo ray 跳过几何。正确做法是保留碰撞 alpha，用 viewer 的 geomgroup 隐藏它；同一组物理表面供接触与 LiDAR 使用。地板沿用冻结摩擦，不把 AWS floor 的极高摩擦悄悄带入 plant。

独立栅格评测若只检查 cell center，会漏掉 2mm 墙面。这里曾因此把真实墙面回波判为地图离群点。核对多个高度的实际射线后，修正评测为保守的 cell intersection，并保留原失败评分；没有修改 SLAM、墙体或验收门限。独立评测同样需要验证，不能天然视为真理。

## 用证据决定路线与组件，避免调控制器掩盖接口问题

最初开环建图路线撞到真实展示柜，属于路线/障碍认识错误，不能靠降低碰撞或加大力矩修复。修正路线后沿原控制器重建，再从同一已知 dock 加载 pose graph 补北侧区域。重启 dock 是明确初始化条件，不是传送到上一轮终点。

linorobot2 提供了导航组件与接口组合的成熟参考，但其底盘速度、footprint、EKF、micro-ROS 与恢复动作不属于本机器人。借用 Nav2 的 AMCL/Smac2D/RPP/Costmap/BT；几何从冻结 MJCF 派生，运动意图进入原 Ruckig/ReferenceBlock/safety。不要叠加第二套速度平滑器，也不要为成功率启用未验证的 Spin/BackUp。

官方无条件周期重规划 BT 会反复重置本例的最终到点/朝向过程。查看官方 controller 与 goal checker 实现后，改用已有的“路径无效才重规划” BT，避免自写 BT 或修改跟踪算法。初始 12cm checker 曾产生 Action 成功但实际停稳误差 15.0459cm；应收紧 checker 留出定位/制动余量，保留 15cm 的物理验收标准与失败证据。

## 不同层的 ACK 不能互相替代

UserInput queued、Supervisor ACCEPT、Nav2 goal accepted、MCU resume armed、NORMAL 和目标 COMPLETED 是不同事件。文本回答结束不能代替实际导航反馈。初始默认 distance_remaining=0 也不表示已经到达，必须等有效路径距离反馈；只有 SUCCEEDED 给出进度 1.0。

DDS 服务 discovery 早于 lifecycle ACTIVE。正式 app 测试暴露了“服务存在但 BT Navigator 未激活”的竞态，因此 readiness 查询五个官方 lifecycle 状态，再结合 motion owner CHECK。独立 probe 也不能根据自己已收到 scan，就推断另一个 executor 已处理它。

租约续期与 Action 结束响应也不能共用一个睡眠周期。原 backend 每0.35s续约并轮询结果，但0.3s命令 watchdog 可先在正常到点后撤销任务，造成 RELEASE owner mismatch。保持两个既有时限，单独及时处理 terminal result；正式入口复验要求成功、实际误差、正常释放与零非预期接管同时成立。

正式 ROS 模式故意不读取非 TTY stdin。自动验证应使用已有带 SourceStamp 的 UserInput 服务，不应为测试方便新增第二个输入入口。默认 Fake 与显式 nav2_simulation 要在 manifest、状态和提示中准确区分；`--fake` 只选择离线语言模型，不覆盖显式运动后端。

## 撤销权限必须覆盖尚未完成的 resume

极快的规划失败暴露了重要的时序边界：SAFE_HOLD 中已有未来 resume armed，新的 HARD_STOP 在原 MCU 中被作为“已安全”忽略；随后旧 resume 仍可激活。不能让 Nav2 修改 MCU gate，也不能通过持续发旧 reference 避免 starvation。

本次只修正导航 link 的职责：有租约但没有有效运动意图时不请求 resume；如果撤销任务后收到迟到的 NORMAL 确认，在发送更多 block 前重新发同一停止原因。保留原 MCU、协议、周期与安全参数。新增实际物理夹具覆盖这个窄窗口：resume 激活后下一次 2ms 通信处理即重申 STOP，旧任务不再恢复运动，且没有独立 starvation 接管。

停止不是瞬间速度清零。原自平衡车有制动与姿态调整过程，早期约 0.034m/s 的残余不能被丢弃；分别记录早期峰值、后续停稳窗口、前向制动距离与重新授权。未知 actuator feedback 不写成零；requested、软件限幅与 actual history 分开存。

## 故障调查要有边界，资源证据要限定平台

暂停本测试自己的 controller/AMCL、有限 scan 断流、实际 Action 取消能验证真实职责边界；测试不能发布假 scan/TF/GT，也不能暂停 physics。停止后恢复传感器不会重放旧命令，必须有新的 task/epoch 与显式授权。独立 MCU 故障不是 Nav2 可以恢复的原因。

并行重型仿真夹具曾让空间源序列间隔检查 fail closed，并在上游 planner 激活中退出时触发其 class_loader 清理错误。保留日志，独立运行复验；不扩大 freshness 容忍度。WSL 调度与内存压力不等于 Pi 实时能力。

资源采样会看到同主机其他 domain 的进程，必须按本轮 launch PID 过滤。控制输出频率、Action 往返时间与采样 ROS age 各有语义；不能把量化为零的输出 age 或 Action 总耗时称为 RPP 算法执行耗时。250s 的有限运行也不等于长期无人值守。

GT 只进入独立 post-hoc 文件与评价。地图、定位、代价图、路径、参考与 MCU 闭环都消费实际模拟传感器/估计。共享 C 核心与 PTY/DDS 回归证明软件兼容性，不能冒充 STM32 上板、Pi 5 性能或实体物理输出验收。
