# FOLLOW swept-clearance audit — implementation decision pending

本附件属于同一个 Native Runtime 工程，是对现有保护层的有限审计与实现建议，**没有修改生产guard、冻结算法、场景或任何门限**。全部数据为WSL/MuJoCo软件几何/共享C99计算；`hardware_execution_ready=false`。

## 原始保护层与真实阻塞

当前 `navigation_ingress.cpp::scan` 在完整360°覆盖、原frame/range/header检查通过后，将所有返回值压成单个全向最小range；REP117 +Inf内部只作range_max下界，原消息不改。`NavigationRuntime::update_follow_braking`每25个2ms tick运行，将radius、反应位移与原C99 brake绝对行程/尾界相加。反应时间仍是0.2s视觉source期限＋0.35s原scan source期限＋0.002s MCU tick。当前radius0.369016926m是原0.174016926m attitude-invariant机器人半径＋0.18m LiDAR原点三角界＋3×0.005m原噪声标准差。

旧正式并行run的第一个真实projection：v=a=0，但假设反应末端可达v=0.331200013m/s、a=0.600000024m/s²；反应位移0.091411204m，四个原C99角点最大制动绝对行程0.895527114m，合计required1.355955243m。该日志完整保留在 `results/session_20261023/visual/formal_parallel_attempt3/simulation/motion_authority.json`。

有限静态审计直接加载旧完整visual_model.xml，在旧run实际0.02s的posthoc初始pose重建传感器几何，actor依原route分别放在[1.8,0]、[2.7,0]、[3.3,.4]m，调用官方`mj_multiRay`。没有mj_step、GL、AI或控制器GT输入；model编译约0.734s。三次最近距离均为0.750566728m，LiDAR左前40°的`DeskA_01_005`，hit ROS位置约[.569390,.482479,.237176]m。前±30°最小值仍为0.981459259m。真实scan噪声只有5mm sigma，不能解释约0.605m的拒绝差。

因此即使深度/身份/时效都通过，**原Bookstore初始位置的当前scalar guard也会拒绝FOLLOW，人沿原route走远不能解除这张桌子的约束**。该结论是几何反事实，不是此前正式run已经尝试并通过FOLLOW；此前实际还被未验证视觉阻断。证据 `results/session_20261023/follow/clearance_geometry_audit/static_scan_summary.json` 及三份原始360-ray NPZ。没有移动spawn、删桌子或换容易场景来隐藏失败。

还存在可证的参考轨迹保守量：初始参考v=a=0、原jmax=1m/s³，T=.552s小于达到amax=.6的时间，所以在这段时间内参考|a|≤.552、|v|≤.152352、|p|≤.028032768，而当前角点(.3312,.6)不可达。通过同一已编译`cb_brake_init/step`计算(.152352,.552)的绝对行程为.468567000m，连同原radius与反应位移得到.865616694m，仍大于上述全向最小range。该例证明只收紧反应角点也不一定解除原spawn阻塞；它**没有证明所有初始状态下角点最大值完备，也没有用参考jerk证明真实plant加速度界**。原始计算见同目录`rest_reaction_projection.json`。

## 成熟机制的具体取舍

核查本机安装的nav2_collision_monitor版本1.3.12及对应官方tag源码，而非只读README。`Polygon::getCollisionTime`先做当前footprint检查，再逐步推进机器人pose并将障碍点变换到该pose坐标检查；这保留了方向和扫掠几何，未把全部障碍压成一个半径。其轨迹是给定twist的运动学外推，不是本项目C99制动模型。[官方polygon.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/polygon.cpp)

`Source::getTransform(base_shift_correction=true)`通过固定odom frame将scan source stamp的坐标变换到current base；缺少TF或source过期则失败。该机制只校正机器人自身运动，不估计障碍物运动。[官方source.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/source.cpp) `Scan::getData`将原scan点经该transform映射到base。[官方scan.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/scan.cpp)

| 判断 | 成熟机制及本项目适配 |
|---|---|
| Borrow | 官方source-time/TF/base-shift校正与缺失即停语义；继续原0.35s scan期限、未来时刻拒绝、clock-domain/epoch与完整360°有效性检查。 |
| Borrow / Adapt | 官方按轨迹pose变换全部障碍点、测试swept footprint的机制；本项目输入改为同一owner实际承诺的ReferenceBlock/FULL计划与原C99 brake/yaw decay，输出仅binary权限/STOP。旋转不变原radius可继续使用，不为通过而缩小。 |
| Reject | 直接另启CollisionMonitor velocity-output链或使用`processApproach`的速度rescale；它会成为额外速度owner并改变冻结Governor/轨迹。该函数确实以collision_time/time_before_collision缩放twist。[官方collision_monitor_node.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/collision_monitor_node.cpp) |
| Reject | 把官方constant-twist Euler外推称为本项目jerk-limited brake/未来yaw变化/实体追踪误差的证明；官方`projectState`只负责运动学积分与速度方向旋转。[官方kinematics.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/kinematics.cpp) |

不推荐简单裁剪front sector：桌子可在转向扫掠范围内，某些制动参考还会反向；丢弃侧后数据没有消除风险的证明。只用当前yaw预测也不成立：当前`receive_follow_cmd`能改变yaw，而guard只每50ms更新，旧校验可能先于新intent。原MCU HARD使yaw_output按0.8rad/s²衰减，并非瞬时清零；从|ω|=.5的理想参考到零仍有至多.15625rad参考转角，必须计入。

## 推荐的有限实现责任

推荐优先研究**唯一native producer在block进入唯一MCU队列之前，对实际承诺轨迹作binary swept-footprint检查**，而非再写控制器或安全速度重规划器。应先定义并验证以下闭合条件，再替换旧scalar gate；目前只提出审计建议。

1. 取原始完整scan及其source pose、当前有效sensorized state、已在途active/next块和实际候选block/FULL计划，全部绑定相同task/source/reference epoch与计划generation。由现有估计/TF历史对齐scan，不从本审计GT或仿真model取控制输入。没有覆盖、映射不确定性上界、finite有效输入或完整已承诺轨迹时拒绝。
2. 在原.552s时间责任内包含scan测量→当前的自身运动/不确定性、可能仍执行的在途块及后续承诺计划，之后接原C99纵向制动与原yaw_decay，保留反转和剩余尾界。不得凭“最近新scan通常100ms到达”缩短.35s失效容忍期，也不得把尚未校验的新指令当作已固定未来。
3. 任何会改变真实输出的后续intent、yaw包、pending FULL切换或ReferenceBlock，在生效前重新校验同一generation；校验后至enqueue/consume之间的task/scan/plan变更必须使旧结果失效。若依赖另一个线程异步完成计算，须有明确提交条件/截止时间，不能让new command越过old proof。单纯每2ms“以后会再检查”不是该性质的证明。
4. 所有future accepted changes均重新检查时，可通过每次提交的有限轨迹保护闭合；如仍允许新变化先进入MCU再检查，就必须包络这段窗口内所有允许的v/a/jerk/yaw变化。无法求出有限包络则fail-closed，不能用四个未证明完备的角点或单条time-optimal Ruckig轨迹替代可达集合。
5. 保留完整scan有效性和未知空间边界，检查全部返回点而不是只取某方向最小值；采样步长之间的扫掠覆盖、角分辨率、原噪声与姿态/时间误差必须有显式保守余量。成熟point-footprint检查本身不证明离散ray之间不存在薄障碍，也不证明过期点云中的动态障碍仍在原位；这些不能被“TF成功”隐藏。缺少动态障碍运动界时保留受限仿真验证范围，不宣称任意行人/实机安全认证。
6. 原12750步投影上限、bounded360输入、复用buffer及有限计算deadline需保留/来源检查；耗尽、越界、NaN、source故障或提交超时经当前权限/MCU HARD链停止。ReferenceBlock字段/wire、控制周期、MCU本地安全owner不因guard变化而改变。评估软件计算开销和2ms/50ms调度headroom，不能因为C++就默认实时。

应先用保留旧失败的同初始状态静态/轨迹对照验证方向信息的价值，再做原bookstore物理回归：桌旁直行与转入桌体、前/侧/后障碍、yaw反向、允许加速末端STOP、scan .35s边界、.2s视觉失效、无TF/历史、旧epoch与新block竞态、投影耗尽；成功与拒绝都记录。实际body姿态、碰撞、制动误差和停止轨迹与原控制基线一起评分。当前尚未执行或授权上述生产改造，原始global guard继续fail-closed。

## 入队前 admission shield 的具体设计（2026-10-25；未实现）

以下是上面建议的可落地接口/数学边界，**不是生产变更或安全验收**。本次只追加本附件；没有运行物理、GL、viewer 或新建另一份工程报告。目标是由现有 `NavigationRuntime` 对它即将承诺的 Reference/Yaw 事务返回 PASS/REJECT，再走原 `MotionAuthority`/C99 HARD 链；shield 不输出较小速度、不改变轨迹、不规划绕障、不恢复任务。现有视觉距离仍未验收，故几何实验即使 PASS 也不使 FOLLOW 具备授权条件。

此处PASS是**静态平面 sampled-scan 工程碰撞准入条件**，沿用原固定margin和C99模型，不能写成任意环境/实体的完整防碰证明。原scalar guard同样没有Gaussian噪声、ray间/平面外障碍与真实plant跟踪的确定性保证；新方向性机制不应为缺少原baseline从未具备的形式化认证而停在接口设计。应完成shadow实现、负例与相同场景物理比较，再决定是否满足有限软件范围的切换验收。下文将输入确实缺失的source/TF/FOV/free-cell故障与该传感器模型的固有限制分开记录。

### 实现追踪与缺失数据

| 当前真实输入/owner | 已有字段与实际行为 | shield 必须补齐或明确拒绝的部分 |
| --- | --- | --- |
| `follow/src/navigation_ingress.cpp::scan` | 标准 `LaserScan` 已有 source Header、frame、360 ranges、角度/范围、`time_increment`、`scan_time`；当前仅保留全向最小值。现有有效性检查容许 ≥180 束且角间隔 ≤2°，并非固定360容量合同。 | 当前限定仿真模型应要求与 canonical `sim_lidar.yaml` 的360束/瞬时采样/角几何一致，保留全部范围和 +Inf/no-return 标志。reject 大包，不能先无界复制再检查。有效 source 与 local receipt 分别保存。NaN/−Inf/不完整覆盖继续拒绝。 |
| `navigation_ingress.cpp::odom/transforms` | Odometry 原消息已有 quaternion、pose/twist covariance；当前只验 x/y、linear.x/angular.z 并保留时间。`/tf` 当前只看 map→odom 的 stamp，未查变换值、history、publisher authority；没有 `/tf_static` subscriber。 | 使用官方 tf2 Buffer/TransformListener 查询历史，不把当前 `health()` 等同于 TF 证明；增加源/authority/finite quaternion 检查，保留有界 source pose history。shield 只需唯一 odom，不需 map→odom；AMCL 的跳变不能改变本地障碍相对几何。 |
| `interfaces/msg/BalanceState.msg` 与 `control::spatial::SourceGate` | 已有 source_id/epoch/sequence、raw clock、ROS mapping id/uncertainty、geometry_id、物理 roll/pitch/navigation yaw、yaw rate、路径弧长、sensor validity。`SpatialState` 是唯一 odom/动态 body TF owner。 | 复用该 gate 与其 canonical geometry 检查，不在 shield 再积分轮子/gyro、不另建 odom。现 runtime 500Hz `step` 只接收 v/pitch/pitch_rate，不能从这三个量推造导航 yaw。必要时将**既有 sensorized spatial snapshot**作为 typed input 传入，不用 MuJoCo qpos 或参考 yaw 冒充观测。 |
| `spatial_sim.py::publish_pending` | 同一空间 producer 发布 BalanceState 和原 LaserScan，scan header 是 ray acquisition time；`time_increment=0`。LaserScan/Odometry Header 本身不带 SourceStamp 的 epoch/mapping/geometry。 | 可在当前固定仿真组合中 pin 两个 topic 的唯一 producer GID/Graph identity、单次运行 session 与 BalanceState identity，并明确证明 source time 的同域绑定；restart/GID/clock/epoch 变化锁存失效。无法绑定时报告 `scan_source_identity_unavailable`，不能仅按时间接近猜同源。跨设备迁移需真正的 source envelope/驱动合同，不能沿用该仿真假设。 |
| `simulation/native/src/navigation_runtime.cpp` | runtime mutex 内拥有 MCU active/next、最多2个在途 reference、32个总 transport 槽和唯一 TwoPathProducer。Reference block 内的 yaw 字段**不驱动 C99 yaw**；`after_pi` 每25tick另发 `Kind::Yaw`，`cb_mcu_yaw` 独立接收。 | Reference 与可能生效的 Yaw 必须组成同一 admission 事务；单验 block.yaw 不够。正常态 `mcu.c::yaw_sample` 可直接切换到 yaw_target，只有 stale/HARD 使用原 .8rad/s² decay，不能假设正常态也有 yaw 加速度限制。 |
| `full_reference.cpp::produce` | 调用会改变 accepted/candidate/FULL/pending/lifecycle/index/sequence，返回25行 float64诊断及640B float32 wire。FULL解由现有solver提供，producer持有所有权。 | shield 验证 decode 后的**实际wire**及原C99将消费的行，保留float32误差；不能仅验float64。为 producer 增加单调 `plan_generation`/`pending_generation` 身份，不复制算法。未入队的FULL余段不是MCU承诺；它的每个后续block仍必须重新验。 |

当前 `nav2_geometry.json` 的 owner 是冻结 MJCF 的导出：物理姿态不变球界 .154016925591m，加既有 .02m geometric margin 得到 .174016925591m。新shield第一版继续使用这个圆盘footprint，避免新增polygon几何/姿态近似。若以后改polygon，须重新证明所有允许roll/pitch下仍包络整车，不能用可视化URDF轮廓缩半径。

### 冻结安全值与本轮 adaptation 派生量

| 分类 | 来源与数值 | 允许的结构性替换 |
| --- | --- | --- |
| 保持冻结 | 1ms physics、2ms C99 control、25行/640B v1 ReferenceBlock；原参考v/a/jerk limits；Governor/KF/Ruckig/FULL生命周期；stage6配置的 brake .5m/s²、jerk .8m/s³、handoff 1s、heartbeat .15s、starvation .012s、yaw .5rad/s、freshness .2s、decay .8rad/s²。 | 不由shield改变。`navigation_motion.py::build_navigation_reference` 继续唯一组合 canonical common/stream 与 stage6 safety/yaw 配置。 |
| 既有空间/导航有效性 | scan source/receipt .35s、odom source覆盖 .12s，以及现FOLLOW的future tolerance .02s。原空间倾角/IMU/source/epoch gate与TF唯一authority。 | 保留期限/拒绝语义；不能以10Hz名义scan周期代替 .35s失效责任，也不能从Nav2 upstream默认值覆盖本项目值。 |
| 当前adaptation几何派生量 | .369016925591m = .174016925591m robot radius + .18m LiDAR原点三角界 + .015m三倍仿真range sigma。 | .18m不是机器人半径，也不是必须重复加入的制动margin：只有拿到source-time的完整真实 `odom→base_footprint→base_link→lidar_link` 变换并覆盖外参/姿态误差后，才可用变换后的点代替该三角界；不能只减去 .18。继续保留 .02与原 .015基准。高斯3σ只给统计模型，不构成有界误差证明。 |
| 当前adaptation反应包络 | .552s = .2s视觉期限 + .35s scan期限 + .002s MCU tick；全向反应位移、四个v/a角点、当前reference与measured-v的scalar brake最大值。 | .552不是原Governor/C99的一项参数。可用实际已承诺reference/yaw及真实失效截止路径替换“该窗口内任意角点”的过度近似，但必须完整说明同一 .552s上界内为何没有未校验输出；不能缩短期限来通过。四角不是已证明完备的可达集。 |
| 当前adaptation资源量 | 12750 maximum projection steps由canonical limits、C99 brake及machine epsilon公式派生；braking receipt .15s复用heartbeat期限。 | 12750作为单分支上限保留，并新增**总分支/总工作量/提交deadline**上限；它不是“每次一定运行25.5s物理”的控制要求。保留来源hash与消耗计数；计算不足REJECT，不能截尾当零。 |

旧 `follow_clearance_derived.json` 是原scalar guard的证据，不覆盖它或改写旧run。若实现新shield，给同一工程的派生配置增加显式schema/semantics与新字段，由 `native_follow.clearance_configuration` 或其同一配置组合边界输出；所有数值继续由canonical source派生并校验hash，不手填第二套安全配置。

### 轨迹与二元判断

令 `t0` 为带source epoch的当前控制tick，`S` 为已验证的完整scan快照，`G` 为一次候选提交的身份。只对该事务实际可能执行的有限参考集合求 swept footprint：

```text
G = (task_id, task_source_epoch, authority_epoch, intent_revision,
     observation_epoch/sequence, spatial_epoch, geometry_id, mapping_id,
     scan_generation, pose_history_generation, reference_epoch,
     producer_plan_generation, pending_generation, transport_generation)

C = MCU剩余active/next + 所有尚可能到达的Reference/Yaw包 + 候选事务
P = C的正常消费前缀 + 各合法HARD/expiry/starvation时刻的原C99 stop后缀
W(P) = union over paths and time [center_ref(t) + disk(R_robot + E(t))]

PASS_software iff source/identity/deadline valid
         and every path is finite, fully covered and within configured work budget
         and W(P) intersects neither sampled occupied set nor model-unknown cells
         and G still matches atomically at commit/receive.
```

这里的 `E(t)` 是已声明且实际采用的工程margin，不包含尚不存在的真实plant确定性误差界。结果同时带 `criterion=static_planar_sampled_scan_engineering`、`physical_guarantee=false`、`hardware_execution_ready=false` 与 `validation_status=not_run/shadow_validated/...`。不能仅凭数学检查PASS就更改后两项或宣布正式FOLLOW通过。

第一版不需要读完整FULL余段或预测下一条人指令：MCU只能执行已到达/在途/该候选的有限block；不再接受后续block时，它会按原starvation或更早的明确HARD停止。每条后续reference/yaw在入队之前重新证明其“有限前缀＋stop”即可递推闭合。若实现选择承诺整个FULL余段，则必须由同一producer导出其真实采样/lifecycle结果，且pending/FULL退出重规划必须先使旧generation失效；不能把target velocity或一条另算的Ruckig轨迹替代承诺内容。

必须纳入早停分支。只验“最后deadline才HARD”不完备：早停会更早衰减yaw，中心路径可能偏向另一边。当前控制tick离散，可枚举从当前到有限承诺前缀末端所有可能的HARD tick（包含显式STOP、source/receipt到期、heartbeat/starvation/underrun），每支调用原 `cb_brake_init/step`，同时保留原yaw decay。正常段及stop段均保留负v/反转；不要将follow目标非负误当制动参考不反转。显式STOP还保留本轮修复的立即epoch/revoke barrier，不能让shield的计算等待阻塞它。

可复制**一份固定大小的 `cb_mcu` POD**作消费/包到达dry-run，调用原 `cb_mcu_reference/yaw/safety` 来取得实际选择、rejection与安全转换；这是只读预测，不能发heartbeat、resume或写真实MCU。原C99 `cb_mcu_step` 的SAFE_HOLD还依赖未来 measured v/pitch/rate，未知时不能传零让它提前settle；纵向后缀应独立复用 `cb_brake_step` 保守保留到tail证明成立，不凭虚构未来观测缩短。传输延迟/到达顺序在本软件runtime是已知逻辑ready time；若拓展到有抖动的实体链，必须枚举其有界到达分支或报告 unavailable，不能沿用单条仿真顺序。

中心参考从唯一source-time odom pose开始，将原 `p` 的差量当作**有符号路径弧长**，以实际C99输出yaw的逐tick序列在SE(2)推进；`p`绝不是world x。常值twist只能作为某个2ms小段的数值积分，不能作为未来全部轨迹。轮地滑移、yaw控制误差、纵向LQR追踪误差没有被这些参考序列消除：它们进入 `E_track(t)`；当前传入measured v与reference a再跑一次brake只是一条诊断分支，并不是物理加速度可达集合证明。

离散采样之间也要覆盖。使用圆盘时，最简单可靠的实现是每个区间起点圆盘加该区间绝对弧长上界 `L_k`，而不是只检查端点。若采用精确segment/capsule求距，可减小同一数学界的数值保守量，但要保留曲率/积分误差界；不能用两端无碰撞推出中间无碰撞。`L_k` 必须来自该段可验证的参考/积分插值合同，例如C99 brake的线性v梯形积分；若FULL wire的p/v几何不自洽则fail。每个有限stop后缀达到原settled/handoff/unsaturated one-jerk条件后，原尾界 `abs(v)/2` 作为末端圆盘额外半径保留；这仍覆盖余尾朝任意方向的位移，不能只沿当前yaw补一点。yaw参考尾段还必须继续到零或被同一圆盘位移界覆盖。

### 坐标、未知空间与真正的证明边界

原始点 `q_i=(r_i*cos(alpha_i), r_i*sin(alpha_i),0)` 只在source LiDAR frame中成立。使用官方tf2历史查询获得 `T_odom_lidar(t_scan)`，将三维点变换到固定odom后再做XY几何；查询当前/估计horizon的 `T_odom_base_footprint(t0)`。这与Nav2 [source-time/base-shift机制](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/source.cpp)相同，但本项目额外要求epoch、future/receipt、bounded source identity和不确定性。source时实际body pitch/roll不能清零；static mount来自同一sim_lidar配置。TF异常、缺少source历史、非正交/非finite姿态或两个authority都拒绝，不用latest transform、identity或now重盖stamp。

误差清单至少分解为 range noise、extrinsic/attitude、scan→now odom drift、time mapping/interpolation、reference/plant tracking和数值扫掠覆盖误差，分别写明单位、来源、实际margin和 `bounded/statistical/assumption/unavailable` 状态。距离为r的ray由角误差ε造成的位置界可取 `2*r*sin(min(pi,abs(ε))/2)`；有已知time uncertainty时按已声明profile传播。当前仿真identity time mapping可用已有零映射不确定性；不是因此把姿态/plant误差也置零。协方差不是确定性误差界，`spatial.yaml` 的velocity/yaw过程噪声也明确是未标定假设。原 .015m三σ继续作为统计工程margin；reference/plant tracking没有验证上界时原样标记unavailable，并以post-hoc物理比较评估，不能编造一个数字声称已覆盖。缺少必要source时间/坐标、非finite数据或超出当前profile仍须fail closed；未具备的普适物理误差证明使 `physical_guarantee=false`，而非禁止有限软件criterion的shadow计算。

完整360°与endpoint避碰仍不足以产生安全证明。Nav2官方 [scan.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/scan.cpp)只收集有效返回点，[Polygon/Circle](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.12/nav2_collision_monitor/src/polygon.cpp)检查这些点是否进入footprint；它没有证明ray之间不存在薄障碍或平面外没有障碍。当前MuJoCo360条ray是零宽、单一倾斜平面的瞬时量测，Gaussian range noise无确定性有界支持。因而不能直接把相邻ray端点连成“已知自由polygon”，也不能把+Inf表示为8m处假障碍或8m内整个角扇区已知自由。

实施时分别输出 `observed_point_clear`、`sampled_coverage_valid` 与 `physical_coverage_proven=false`；软件PASS要求前两项为真。沿用成熟Nav2的sampled点/footprint准则，第一版只建立当前固定360 ray模型的有界覆盖域：完整有效角度、各束的测距范围、有限返回之前的可见区和+Inf到range_max的no-return区；range_max之外、FOV/坏束缺口、遮挡之后、缺TF/source/history等显式unknown一律REJECT。若使用polar/free-cell实现，可将相邻ray的最小有效range作为该角cell的保守**软件采样约定**，对边界扣原 .015m margin并做footprint包含检查；必须把该约定记录为 `adjacent_min_sampled_cells`，不能称作ray之间连续空间已被传感器测过。近场盲区不靠Inf补齐：新增扫掠进入未观测near-field时拒绝，当前footprint的起始占用/无碰撞假设也单列。不得将一张existing map中的unknown/无数据cell转换成free；若不消费map，就不虚构其free cells。

当前零宽ray确实**无法证明**ray间及平面外的完整free space；这是与原baseline共享的静态平面ODD限制，必须在每次验收结论中保留，而不是将 `sampled_coverage_valid` 改名为物理证明。先实现完整shadow shield，保留旧scalar生产gate，针对**当前静态Bookstore＋该传感器模型**将所有软件PASS/REJECT与独立post-hoc完整碰撞几何比较。只有实际负例、原spawn/route物理碰撞/跟踪证据、source/覆盖故障与计算预算通过后，才考虑有限软件范围切换；不要求它先取得Nav2/原guard均不具备的普适形式化认证。场景几何/GT只用于离线验收反例，不能给运行shield输入障碍位置、hit id、actor pose、contact或GT纠偏。动态actor仍可产生真实scan返回，但缺少其未来速度/遮挡进入界时不能把静态结论外推为行人安全；硬件同理。

### 提交原子性、锁顺序和deadline

推荐将shield实现为现有simulation native库内的纯函数/小helper（例如 `collision_admission.hpp/.cpp`），复用该库已经链接的C99，只接typed snapshot并返回有界POD结论，无线程、ROS node、重连、任务模式或速度输出。环境DTO放在follow库，避免让MotionAuthority依赖C99/ROS。`MotionAuthority`继续唯一grant/revoke owner；其内保留一个immutable latest `ShieldScanSnapshot` 和generation，`NavigationRuntime`继续唯一producer/transport/MCU owner。

建议窄接口如下；名字为本设计建议，不表示当前源码已经提供。DTO中的身份字符串继续执行现有长度上限；scan/history/message先验容量后复制；runtime一次只持一份有界输入快照，不能用不受控shared_ptr历史积累替代固定资源。

```cpp
// follow库：无ROS/C99。Scan360含Header时间/frame、receipt、角/范围、
// 360 ranges及finite/no-return状态；PoseHistory64含source pose/epoch与covariance。
struct AdmissionInputs {
  AdmissionToken token;                 // 上面G的所有身份/版本
  MotionIntent desired;                // mailbox请求；不是输出许可
  Scan360 scan;
  PoseHistory64 poses;
  SourceValidity validity;             // source/receipt/mapping/TF/FOV状态及截止
};
AdmissionInputs MotionAuthority::admission_inputs(double source_now, double receipt);
std::optional<AdmissionCommitGuard> MotionAuthority::try_commit_admission(
    const AdmissionToken&, double source_now, double receipt);

// simulation native库：QueueSnapshot为cb_mcu POD及最多32个已知逻辑到达包；
// CandidateTransaction为真实wire Reference/Yaw/必要Resume及预留槽数。
AdmissionResult evaluate_collision_admission(const AdmissionInputs&,
    const QueueSnapshot&, const CandidateTransaction&, const AdmissionLimits&);
// result = PASS/REJECT + bounded reason enum + G + sampled coverage/risk flags
//        + minimum margin clearance + offending branch/ray/tick + work/time.
// 通过try_commit后才授予本事务输出权限；evaluate不能改变任何owner状态。
```

1. ingress在它既有rclcpp executor中完成消息验证与TF查询，构造有界snapshot，短暂调用authority发布。查询时不持runtime/authority锁。原BalanceState gate/TF authority异常先invalidate；新scan即使snapshot准备失败也必须发布失效generation，不能把上次成功永久留作当前数据。
2. runtime在现有 `mutex_` 内读取authority的 `AdmissionInputs`，快照含所有G字段与未授权intent；desired intent只是mailbox中的请求，不是actuator许可。`ACQUIRE_FOLLOW`可以用当前静止/stop几何证书作任务readiness检查，但grant本身不发moving packet；第一个真实运动事务仍必须验证。不存在原guard被一个永久 `clear=true` 替代的阶段。
3. 单一producer调用原 `produce` 得到候选block，同时收集将发出的Yaw/Resume与已在途清单；计算shield时不持authority锁。`produce`已推进生命周期，故第一版若失败/超时/generation竞态，直接既有C99 HARD与task revoke，禁止该producer epoch再生产；以后沿现有显式重新获取任务→MCU停止态→producer.reset恢复。无需复制所有FULL数组/log来回滚，也不写第二套trial scheduler。
4. PASS后调用authority的窄提交API（建议move-only `AdmissionCommitGuard`，内部持其mutex），重新做当前lease/source/receipt/health检查，并核对G。调用者已持runtime mutex，故唯一嵌套顺序为 **runtime → authority**；authority/ingress绝不反向调用runtime。获得guard后只做已预检容量的固定槽插入并发布certificate，随即释放；不得在guard中调用ROS/TF/solver或再次调用authority，避免自死锁/停止阻塞。Python所有等待runtime mutex的入口继续在加锁前释放GIL。
5. Reference/Yaw/必要Resume作为一个已预留槽位的事务提交；先核对32槽总容量和2个在途reference容量，再写任何项，避免yaw已发但block失败的半提交。HEARTBEAT仍由原Pi scheduler生成；shield不代发。STOP永远不受普通运动容量或proof计算结果限制。packet携带G/certificate id，实际receive/consume前再次核对reference/transport/task/scan等身份及source和receipt有效期；旧epoch按原C99拒绝，不能只靠producer日志证明。
6. 采用严格版本语义时，新scan/pose revision会使尚未提交或尚未消费的旧proof失效；必须在下一次moving消费之前对**实际剩余prefix**重验，或立即原HARD。不得等到下个50ms producer周期再检。若2ms内算不完，fail closed并记录budget fault，而不是先消费旧block。若以后允许“已提交proof在其原scan有效期内继续成立”的静态环境优化，须显式保存该immutable snapshot及其有效ODD，证明新信息不会推翻旧证明；它是另一项合同变更，不能偷偷把最新scan generation检查删除。
7. 失效责任包含原 .2s视觉、.35s scan的source与receipt期限、odom/history覆盖、.15s既有证书receipt看门狗、task lease与transport/compute延迟。每个2ms step在moving deliver/consume之前做O(1)失效检查；deadline不能只由50ms健康timer触发。计算时保留原 .552s窗口为最大未覆盖责任上界，并精确记录实际触发tick；若有限队列更早starvation则按真实C99早停分支，不虚构后续reference填到 .552s。stop命令到MCU的最晚可达时刻必须有界且纳入prefix；本软件可用当前已知logical latency，硬件未知时reject。

若整个step因同步FULL数学/几何计算占用wall time而不能保证失效检查，不能把source-time模拟步进误称为实体实时保障。先量测whole-block、shield和step wall/thread-CPU；原transport availability应包含shield计算时间，不能只增加日志而仍按旧block.compute_wall发送。missed deadline必须让原starvation/HARD显式出现。若之后需要异步计算，仅使用现有runtime/ingress可交接的immutable快照与单个bounded结果槽；在结果未到时不授予新输出，不因异步再建motion owner。

### 有界容量和文件落点

| 文件 | 最小责任变更（均尚未实施） |
| --- | --- |
| `follow/include/.../motion_authority.hpp`、`src/motion_authority.cpp` | source/intent/environment revision，bounded scan/pose输入，二元certificate状态，短提交guard；现有task/epoch/lease/MCU provenance不变。旧scalar API保留给既有oracle/夹具时标明用途，生产不能同时有两个互不一致的guard事实源。 |
| `follow/src/navigation_ingress.cpp`、header、CMake/package | 保存完整360 scan；正式source/GID/epoch、BalanceState既有gate、tf2历史变换与失效快照；复用官方tf2而非自写TF插值系统。TF cache按有限时间配置并记录实际frame/source容量，异常frame不写入guard可用集合。 |
| `follow/include/.../full_reference.hpp`、`src/full_reference.cpp` | 仅增加plan/pending generation与只读manifest；保留原produce/scheduler/FULL算法。manifest包含候选block wire hash/sequence和已用plan identity，避免日志字符串作为协议。 |
| `simulation/native/include/.../navigation_runtime.hpp`、`src/navigation_runtime.cpp` | 在唯一produce/enqueue/receive/step边界调用pure shield；Reference/Yaw事务、固定transport快照、立即HARD失败路径、deadline与证据日志。不改C99内部。 |
| `simulation/native` 内 `collision_admission.hpp/.cpp`（若新增） | 固定大小typed DTO、C99 brake/replay与SE(2)扫掠/点距计算；无通用planner/plugin抽象，不新增follow→C99依赖。圆盘点距和segment距离足够简单，不引入另一个costmap/world state；若性能需要空间索引，先量测360点线性循环，只有确实不足再用已有Boost.Geometry等成熟实现。 |
| `native_follow.py` 与 bindings | 同一canonical配置派生、geometry/limit/来源hash检查和typed接口组合；Python不承担新的任务/参考/guard状态机。 |

可直接采用固定360 ray槽、latest scan双缓冲/单读快照；history按原最大.35s scan期限＋.12s允许样本间隙＋两端bracketing推导至少 `.35+.12+2*.02=.51s` 的source跨度，并在50Hz名义样本下给固定64槽（约1.28s）及实际time coverage检查。64是资源上限，不允许稀疏历史伪装有覆盖；高频/重复源不许无限增长，超出当前单一仿真profile拒绝。frame链只接受合同中的固定链，拒绝任意新frame。tf2按时间缓存仍需守住输入authority与最大消息/transform数量，不能宣称“有cache duration就有固定内存”。

承诺block数的理论工作上限取2个MCU槽＋2个在途reference＋1候选，共125行（重叠sequence去重，gap按C99触发处理）；若无法证明重叠/到达关系则拒绝。逐tick早停最多126支，每支最多12750 brake步，直接上界约160.65万纵向步；360点全遍历可达约5.78亿次点距，**不能默认适合2ms**。应流式检查分支、固定复用buffer并按几何AABB作保守早拒/宽相位筛选；只需保留最大工作量/最小clearance/首个失败与其time、ray索引，不保存百万采样日志。计算优化不得丢分支/反转/tail；新guard的总工作deadline与2ms/50ms可用headroom先由软件benchmark确认，耗尽REJECT是验收结果的一部分。

每次certificate日志至少保存G、各source/receipt age与有效截止、sampled coverage状态/使用的free-cell模型、误差项状态、固定margin、静态平面ODD及未覆盖风险、prefix block/yaw identity、stop触发分支、工作量/耗时、最小带margin clearance、失败原因和实际是否enqueue；actual actuator feedback仍来自独立frozen history。旧桌体失败、拒绝与预算失败一起留证，不能只保留PASS。

### 有限实现与比较顺序

1. **纯定向几何/协议夹具**：先固定相同scan和实际wire，检验桌旁直行与转入桌体、正/负v、yaw跳变/反向/HARD decay、早STOP与deadlineSTOP、尾界和离散步间碰撞。旧scalar必须保留原 .750566728m/1.355955243m拒绝证据；新算法不预设一定放行原spawn。若新算法也拒绝，输出具体ray/branch/unknown原因，不调margin找通过。
2. **source/竞态夹具**：.35/.2/.15s边界、future .02s、receipt倒退、缺TF首样本、source epoch/geometry/mapping/GID变化、odom/bracketing缺口、非finite quaternion、+Inf/NaN/−Inf、359/超360/角几何错误；在evaluate与commit/receive之间依次注入STOP、task替换、scan更新、yaw/FULL pending变更，证明无旧certificate的moving包穿越。软件夹具不冒充真实感知或物理执行。
3. **有界工作量benchmark**：最多槽位/分支、最长brake、全部ray靠近路径等不利输入；记录whole producer＋shield latency、2ms step p95/p99/max、截止遗漏、queue高水位和fault计数。只提高并发/数据局部性不改变真实逻辑到达次序；不能关闭budget gate取得漂亮时延。
4. **原Bookstore相同spawn/route的shadow比较**：只在既有runtime输出链旁被动记录新shield，冻结原scalar/控制器/Q/R/plant/actor/摩擦/限幅/视深门限。使用原始sensorized输入，post-hoc完整几何仅评估false admission、最小真实车体间距、碰撞、倾角、路径/停止误差和参考/实际actuator差异。覆盖桌旁直行、转桌、侧后障碍及scan dropout，不把actor移远就当静态桌子已解决。
5. **条件满足后的生产A/B**：只有source/TF/FOV/free-cell合同、固定margin、实际负例/原场景物理比较与预算先通过，且静态平面软件ODD/未关闭风险在结果中明确，才显式切换唯一生产guard；旧scalar继续作为诊断以便解释差异，不形成第二owner。不以形式化物理保证作为已取得成果。视觉门尚未通过时，该阶段FOLLOW继续拒绝；可用明确标记的传感器/协议夹具验证几何接口，不能伪造perception health来声称正式FOLLOW。

停止条件明确：若64槽历史不能覆盖、source/TF/FOV/软件free-cell覆盖缺失、输入超profile或保守完整分支无法在deadline内完成，则记录为source/coverage/profile/compute blocker，保持fail-closed，继续不依赖该blocker的C++边界与回归。ray间/平面外、高斯非有界噪声、未验证追踪误差与动态障碍无运动界分别作为本有限工程criterion的未关闭风险；它们禁止普适安全/实体保证，但不阻止保留margin的静态软件shadow/负例/物理比较。新guard若实际出现物理碰撞或更差停止轨迹，先保留失败证据并定位数据/模型/追踪原因，不调原场景、安全门限或算法来掩盖。
