# CompanionBot current state — Stage 8 finalized baseline

## Stage 8 已完成：独立 Agent 与真实交互

- 正式单入口：`scripts/demo_stage8_interaction.py --camera-device 1`，交互窗口 + 原Stage7预览，保留Master点击选择；首帧之后初始化本地TTS/麦克风，模型加载与首次推理就绪另有提示。唯一camera capture owner，Agent SLEEP时感知与frame tap持续，零Agent读帧/模型请求。
- PydanticAI Slim2.53.0（OpenAI extra）+ Qwen `qwen3.8-flash` non-thinking；官方async/events/streaming/usage/cancellation。简单问答通常1 request，自然视觉通常2 requests；无第二个分类Agent、旧agent-core或MCP。
- 唯一逻辑唤醒词“你好小柒”，本地Vosk保留“你好小”前缀并容忍qi同音名字；普通唤醒句级0.35，严格控制/行为保留更严门控。ACTIVE为SenseVoice int8 + Silero VAD，16kHz mono s16le，停顿0.9s整句提交；缺confidence显式null/unavailable，无confidence语音行为需名字前缀。
- 内置Realtek按实际设备名称选取，保留CLI override，失败不回退C920麦克风；设备/ASR原文/RMS、输入来源与具体ACCEPT/REJECT/IGNORED可见。ASR原文、真人PCM和图像不写交互日志。
- Streaming窗口 + 中文句段SAPI FIFO；行为轮等Supervisor ACK和最终结果再播。文字/按钮/预览空格可取消generation、清队列、停当前播放；口头仅允许完整名字+明确打断口令，并保留自身TTS文本veto。无可靠AEC或任意语音Barge-in。
- ACTIVE空闲30s本地休眠并只提示一次“小柒先走啦”；生成、播放、VAD用户语音保护计时。自然缄默由typed `enter_sleep {action:SLEEP}`直接结束，不告别；引用负例正常回答，休眠清历史/epoch并取消活动任务。
- 按需`capture_view`复用非消费式FrameProvider，source/sequence/host read-complete、BGR uint8、同帧原像素ROI、源尺寸与JPEG尺寸分开；编码后freshness复核、历史移除图片。图片轮不能执行行为或授权休眠；VLM不参与Master tracking。自然capture当前整帧，ROI由显式入口提供。
- 高层工具覆盖本地知识、真实Master、robot/task状态、FOLLOW/WAIT/STOP_REQUEST/GUIDE_TO与取消。FOLLOW执行前要求新鲜LOCKED/visible Master；Supervisor为真实确定性实现，Robot/Navigation仍fake，`hardware_execution_ready=False`。GUIDE_TO保留destination ID、task ID、status、cancel，无SLAM/地图/实际运动。
- 已有真实Qwen语义smoke：6 requests/6 attempts、8392 input/191 output tokens，0重试/异常；自然视觉首事件0.916s、首文本2.070s、首SAPI提交2.340s、生成结束2.410s。首语音是命令提交而非声学起声，有限测试不是普遍性能承诺。
- 对应真实C920 full：1256帧、28.74Hz，tap1256/0failure，detector27.99Hz、depth14.76Hz。新ASR相同8条合成PCM对照CER Vosk1/71、SenseVoice0/71，median decode0.844/0.093s；用户基本功能通过，但真人固定语料、环境误漏唤醒率、回声与听感未作量化认证。
- 收口active测试**260 passed**（冻结工程150 + Agent54 + interaction56），pip check通过；历史helper断言另11项通过。视觉关键词hint和旧名字专用映射退出Runtime，早期设备smoke移入`stage8/reference/`；当前门限/config、依赖和原结果保留，没有新增算法、commit/push或GUI自动操作。
- 接口已预留FrameProvider、RobotBackend、NavigationBackend，尚缺完整分布式契约：RobotSnapshot时钟字段、稳定source/epoch、destination→map pose、ROS Action进度/取消终态、真实ACK deadline/幂等/ID匹配、重连恢复和资源上界；HTTP发送前未再次验证图像年龄。Stage7→6真实geometry/pitch/latency/transport仍需独立验收，不能自动开启实体execution_ready。

当前权威文档：[最终报告与白盒图](models/minisegway/stage8/STAGE8_REPORT.md)、[运行与验收](models/minisegway/stage8/RUNNING.md)、[Learning Log](models/minisegway/stage8/LEARNING_LOG.md)。历史仅在reference和原results追溯；Stage3–7冻结边界见下文。

## Stage 7 当前状态

- 最终入口：`scripts/demo_stage7_perception.py --camera-device 1 --preview`，RGB-only。
  Camera → YOLO26n → ByteTrack → Master/OSNet ReID，与独立 YOLO26n-depth 按
  source ID/sequence/time/resolution 配对 → torso median → camera XYZ → robot-relative observation。
- Master/ReID 保留 Stage 7.6：三帧 reference 跨至少 0.30 s；LOST candidate 连续 3 次 tracker update，
  ≥0.68 接受，0.50–0.68 间隔约 0.20 s retry，低于 0.50 停止本 track 重试。阈值未做 negative-person calibration。
- Camera 为 BGR uint8；bbox/crop/depth 均用原始 source pixels。C920e K/D 为 1280×720，禁止自动缩放；
  detector imgsz=640、depth imgsz=768 与采集分辨率独立。
- 时间为 host read-complete / perf_counter，未宣称曝光时间。depth age 为 ready-source；pitch provider
  必须使用同一 source time 和时钟语义，缺少 coverage 返回 unavailable。
  Provider 可另外显式报告映射到 host clock 的曝光对齐 `pitch_time_s`；原始 source receipt 时间保持不变。
- camera 坐标为 +X右/+Y下/+Z前；body/leveled robot 为 +X前/+Y左/+Z上，原点为轮轴中点。
  `P_B=R_BC P_C+t_BC`，`P_L=R_y(theta) P_B`；theta 正值绕 +Y 右手旋转（nose down）。
  模拟安装为前方 0.08 m、轮轴上方 0.35 m、下倾 10°；pitch 为 simulated constant 0°。
  这些是 provisional 参数，YOLO depth 的 axial Z 解释也尚未最终确认。
- 最终无 GUI C920 smoke：300 帧、124 组同帧配对、104 条 robot observation；创建一次 reference，
  一次 ID 1→2 ReID reacquire（0.8963）。这是执行证据，未独立验证身份或物理距离。
  depth 14.03 Hz，infer median/p95 68.59/85.04 ms；detector 主链 27.23 Hz。
- 最终时间接口补完后的第二次 180 帧无 GUI 验证同样正常退出：56 组同帧配对、54 条 robot observation；
  depth 12.04 Hz、infer median/p95 82.19/86.75 ms，detector 主链 25.60 Hz。两次原始结果均保留，未调参。
- 当前完整 active tests：**150 passed**，统一在项目 `.venv`。最终 GUI 留待用户验收。
- Stage 7 配置/结果统一在 `models/minisegway/stage7/`，下设 `stage7_1/` 至 `stage7_7/`。
  最终配置为 `stage7/config/final_demo.json`。
- **Stage 7 尚未接 Stage 6**。需确认设备物理身份、真实 T、depth convention/精度、pitch sign/clock/history、
  相机 latency 与 stale/lost 接口策略，并测 Pi/MCU 实机时序。检测频率不要求固定 20 Hz。

详见 [Stage 7 final report](models/minisegway/stage7/STAGE7_FINAL_REPORT.md) 与
[Stage 7 learning log](models/minisegway/stage7/LEARNING_LOG.md)。

## Stage 3–6 冻结控制与跟随状态

Stage 3 已完成并冻结为 MuJoCo simulation baseline。权威索引是
`models/minisegway/stage3/results/config/baseline.json`；生产配置未自动替换为调参过程中任何临时候选。

## Plant、时序与数据边界

- MuJoCo physics step：1 ms；controller/estimator update：2 ms。
- 两轮纵向输入语义：`u = tau_left + tau_right`；单轮 hard peak 仍为 ±0.63 N·m。
- 控制状态顺序：`[p_hat, v_hat, pitch_error, pitch_rate_hat]`。
- Virtual IMU 保留 1 kHz packet、1 ms availability latency、启动 gyro bias calibration 和
  control-time age compensation。
- Controller/estimator 不读取 GT payload/contact/pose；GT 只进入 evaluator 与结果 history。

## Longitudinal baseline（Stage 3A）

- Nominal model：batch-refit 的单套 fixed A/B，来源为
  `stage3/results/stage3a_refit_ab_0p60_validation_results.json`。
- Feedback：保留 K_old；new-A/B 原 Q/R gain 和最多六组有限 Q/R challenge 均未形成更好的替代证据。
- Estimator：acceleration-compensated complementary pitch filter。
- Velocity reference：jerk-limited S-curve，baseline limits 为 0.6 m/s、0.6 m/s²、1.0 m/s³。
- Dynamic nominal 只服务于 command transient；`lambda_ff=0.6`。`T_full` 后执行 0.15 s
  smooth fade，`VELOCITY_HOLD` 中 `u_ff_used=0`，position-error memory 不被 preload/reset。
- Finite position goal 保留 `TRACKING -> TERMINAL_HOLD -> GOAL_REACHED` 语义。
- Q/DOB actuator contribution 在正常 longitudinal baseline 中关闭。

## Yaw baseline（Stage 3B）

- Parallel relative-heading/yaw-rate PD：`K_psi=0.55 N·m/rad`、
  `K_r=0.20 N·m/(rad/s)`。
- Yaw estimator 只使用 startup-bias-corrected gyro-z，并按 packet timestamp/age 对齐。
- Wheel allocator 先保护 longitudinal common-mode torque，再分配 differential torque。
- Production baseline 使用 symmetric motors；motor mismatch wrapper 只保留为专门诊断路径。

## Moving payload 与 Q 决策（Stage 3C-R）

早期 0.20 kg、低摩擦、刚性壁面场景出现高能重复碰撞、payload escape、fall 和 saturation；该失败
归类为机械 disturbance shaping/hidden contact dynamics，不作为 Q observer 的成功条件。

一次固定被动机械整形后采用 0.10 kg、64×32×32 mm payload、0.35 滑动摩擦，以及
`solref=[0.02,1.5]`、`solimp=[0.7,0.95,0.005,0.5,2]` 的壁面接触。冻结 controller 下：

- Q-OFF：无 fall、payload contained、wheel saturation 0、无 large collision-correlated velocity drop。
- Pitch GT RMS/peak：6.09° / 13.81°；velocity GT error RMS：0.0394 m/s。
- Q-ON 只产生很小的指标变化，同时增加 feedback burden；因此 actuator augmentation 被拒绝。
- Q observer 保留 diagnostic-only；production `u_Q_used=0`。

完整结果：`models/minisegway/stage3/results/stage3c_mechanical_payload_q_revalidation_results.json`。

## 当前边界

- 这是 nominal flat-ground MuJoCo research baseline，不是硬件安全认证或 payload rating。
- 未解决项包括真实 motor torque-speed envelope、电池/驱动动态、地面变化、真实视觉跟随闭环/导航和实机验证。
- 不应从已删除的 tuning artifacts 恢复临时 A patch、3-state Drive LQR、persistent cruise FF、
  GT-in-controller diagnostic 或 Q actuator augmentation。
- 后续阶段若改变 plant、payload、timing、estimator、K/A/B 或 torque limit，应新建独立公平验证，
  不覆盖 Stage 1/2/3 冻结结果。

## Stage 4 研究结论（不改写 Stage 3 baseline）

- Stage 4A：冻结 baseline 在 ±8° 通过；±15° 属于压力测试 envelope。现有 2 Hz Q actuator
  对坡面 tracking 无独立收益，production 继续 OFF；Q observer 保留。
- Stage 4B：68 个 episode、episode-exclusive split、100 Hz / 0.8 s window 的 proprioception
  pilot 已真实生成和训练。statistics+Ridge 的 test α MAE/p95 为 1.190°/3.348°；tiny raw
  Conv1D 为 2.007°/3.815°，两者均未过 α Gate，payload counterfactual drift 也为 1.070°。
- Stage 4B-R 发现旧 generator 只改 terrain friction、未改 wheel collision friction；修正版以新 ID
  生成 108/9/40 train/val/test episode，旧数据不覆盖。2.4 s Ridge 的 online MAE/p95/max 为
  2.615°/8.187°/21.954°；Ridge+±3° bounded residual 为 2.579°/7.284°/24.744°，仍有
  53 个 window 的绝对误差超过 5°。Slip CNN recall 0.689，rough CNN F1 0.441。
- Stage 4B-R 最终定案：**PERMANENT REJECT proprioceptive slope learning**；slip detector 不进
  production；rough cheap baseline 只保留结果/代码。按 Stop Rule 未把 estimator 接回 controller，
  也未执行条件式 A/B/C/D。解析 α→θ_eq 的 0.237°验证结果保留，但不代表 online sensing 可用。
- Stage 4C 没有恢复学习路线，而是按 Parravicini/Corno/Savaresi 的解耦结构增加独立 physics
  slope EKF。clean empty、smooth、`mu=1.0` 的 held-out ±3/±6/±10/±14°、0.2/0.4/0.6 m/s
  上，TWIP-EKF MAE/p95/steady bias 为 0.206°/0.400°/0.143°，48/48 有效 episode 在恒坡
  入口后最坏 0.08 s 收敛；因此定案 **KEEP TWIP-EKF**。STATIC-INVERSE 因 4 个有效 episode
  未满足 5 s hard convergence 而不保留。
- Stage 4C oracle `x_eq/u_eq` 和 estimated-alpha A/B/C 闭环均无 fall/saturation；estimated 闭环
  steady velocity RMSE 为 0.0031–0.0054 m/s。fixed 0.10 kg payload 则使坡度 MAE/p95 恶化到
  1.888°/2.564°，明确需要 payload-aware model parameters，本轮未做 payload adaptation。
- Stage 4D 的 scalar `[v_body,d_slip]` IMU/encoder observer 在严格截断 45° fall 后的 held-out
  pure-slip test 上 precision/recall/F1 = 0.947/0.419/0.581、FPR=0.0006、median latency=0.10 s；
  recall 未过 0.90 Gate。按 Stop Rule 未接 safe slowdown、未跑 slope HOLD integration，production
  不保留 slip path，并记录 future camera/visual-odometry independent velocity dependency。
- Payload Q 最终 A/B 对 fixed/free 没有一致明确收益，且 free payload 的 pitch/velocity 指标略退化；
  Q actuator 定案 **PERMANENT PRODUCTION OFF**，observer 仅 diagnostic。Frozen LQR 对 rough 与
  12/12 个 0.5/1.0/1.5 N、0.10 s 双向 stationary/moving push 自行恢复；现有 modest bump 使车体
  无 fall/无 saturation 但停滞在障碍前，属于当前 V1 traversable envelope 之外。
- 最终职责：Q observer 继续 diagnostic/physics logging；Q actuator production OFF；Stage4B-R
  的 proprioceptive **learning** 永久拒绝不变。TWIP-EKF 只在 clean empty nominal-traction
  假设内保留；slip coupling 留给 Stage4D，rough/bump/free-payload 和几何传感不在 Stage4C 范围。
- Stage 4 最终 runtime 只保留 `FLAT/SLOPE` 与 `payload_id_pending/payload_id_active`。Q one-step
  innovation 只触发一次短 session；payload ID 以 50 Hz 使用 31 次自然 transient 更新，直接冻结
  两个 sagittal 参数。本次 0.25 kg、前置 15 mm 真值的估计为 0.3476 kg、15.18 mm；质量偏高作为
  baseline limitation 保留，不再增加 acceptance layer 或调参。
- 最终 robustness smoke 中，中心/左右 ±10 mm 固定偏载与左右各 1 N×0.10 s 单轮纵向冲激均无
  fall、无 wheel saturation。左偏载的 yaw drift/peak 最大（0.0627°/s、0.718°）；左右冲激相对
  冲激前稳态带的 settling 分别为 0.062 s / 0.002 s。Stage 4 据 Stop Rule 收口。

最终记录：`models/minisegway/stage4/stage4b/results/STAGE4B-R_REPORT.md`。
Stage 4C 记录：`models/minisegway/stage4/results/STAGE4C_SLOPE_REPORT.md`。
Stage 4 最终记录：`models/minisegway/stage4/results/final_baseline/STAGE4_FINAL_BASELINE.md`。

## Stage 5 V1.5 — 上层命令生命周期与 reference-block stream

Stage 5 在 frozen Stage 3/4 runtime 上增加上层速度命令调度和参考流，不替换 LQR、estimator、yaw controller、allocator、torque limit、坡度/payload runtime 或 Stage 3/4 参数。

- 当前配置：`models/minisegway/stage5/config/stage5_v1_5_pi_mcu_stream_config.json`；当前计算 runner：`scripts/run_stage5_v1_5_pi_mcu_stream.py`；人工试玩入口：`scripts/view_stage5_v1_5_manual_demo.py`。
- 输入保持 20 Hz observation；candidate stable-window 后再 accept。小变化走 LIGHTWEIGHT，较大稳定变化走不可普通打断的 FULL_DYNAMIC；FULL 完成 quiet snap 与既有 FF fade 后再处理 pending command。
- 当前控制时基仍为 2 ms，reference block 为 25 samples（50 ms）。FULL projection 在 250 Hz 规划，horizon 先取 50 ms bucket，再向上补齐到 4 ms 网格；selected backend 为 cached KKT。V1.5 不改 frozen Ruckig 速度/加速度/jerk limits、A/B、K、`lambda_ff=0.6` 或 fade 曲线。
- 同一 V1.4 episode 的 cached-KKT 250 Hz replay：planning warm=3.75 ms、post-interpolation 500 Hz residual=0.01776、v error RMS/peak=0.05540/0.16734 m/s、pitch tracking RMS=0.06060 rad；无 saturation、无 fall。V1.5 参考流和 4 ms synthetic-delay check 均无 underrun、stale block 或 sequence gap。开发机耗时不是 Pi 5 实测承诺。
- Stage 5 slope-entry evidence 仅在 `|a_ref|≤0.03 m/s²` 时累计，persistence=0.50 s；该约束只作用于 Stage 5 scheduler/runtime 接入，不改 Stage 3/4 默认行为。V1.5 验证 episode false SLOPE=0。
- 本阶段结果：`models/minisegway/stage5/results/STAGE5_V1_5_SUMMARY.md`；完整 JSON/CSV 与图位于同一 `results/` 目录。V1.0–V1.4 的旧入口、旧门限配置和实验输出仅归档在 `models/minisegway/stage5/reference/`，不属于当前快速运行 surface。

## Stage 6 — 2D relative-target follow 与 Pi/MCU radial KF

Stage 6.5 在冻结 Stage 3/4 controller 与 Stage 5 planner/reference stream 上完成当前 follower baseline。观测频率为 20 Hz，机器人 body frame 为 `+x forward / +y left`。运行数据链为：

```text
TargetObservationPacket(x_forward,y_left,capture_time)
  → Pi 接收 observation
  → MCU RobotStatePacket 经 transport 到 Pi state-history
  → capture-time 线性插值 / 对齐失败则丢弃
  → radial KF [d_hat, v_master_radial_hat]
  → Follow Governor
  → FULL longitudinal ReferenceBlock + independent yaw command
  → MCU safety merge / frozen controller
  → MuJoCo plant
```

Controller/estimator 不读取 Master GT、MuJoCo pose 或 controller 对象。GT 只在 synthetic sensor 显示和闭环完成后的评估中使用。KF 参数为 `initial_distance_std=0.05 m`、`initial_velocity_std=0.6 m/s`、`measurement_distance_std=0.03 m`、`master_accel_std=0.8 m/s²`；Governor 只消费 KF 距离和径向 Master 速度，以及按 capture time 对齐的机器人速度。当前 synthetic KF 配置使用 60 ms camera processing delay；延迟与 jitter 压力测试使用 80 ms 与确定性 ±8 ms jitter。

### 最终 Governor 双窗口锁存

CATCH_UP→CRUISE 时保留最近 20 个有效 KF Master radial-velocity 样本，默认锁存 20 帧 median；若 5 帧 median 比 20 帧 median 低超过已有 `slowdown_velocity_deadband_m_s=0.05 m/s`，则快速通道采用 5 帧值。CRUISE 的锁存语义保持不变；非停车 slowdown 还需要 long-window 证据，5 帧 median 进入 deadband 时可走原快速停车路径。Follower 参数位于 `stage6_4_2d_follow_config.json`：`d_target=d2=1.1 m`、`d1=1.4 m`、`T_catch=3.0 s`、速度范围 `[0,0.6] m/s`、有效制动减速度 `0.25 m/s²`、重触发 margin `0.05 m/s`、连续判定 3 帧。

60 s 的 0.40 m/s 直线对照（只计 20–60 s）：

| 锁存方案 | Vx Governor 事件 / FULL | Vy Governor 事件 / FULL | Vx 距离均值±std | Vy 距离均值±std |
|---|---:|---:|---:|---:|
| 原单帧 KF Vm | 9 / 9 | 10 / 10 | 1.253±0.110 m | 1.231±0.109 m |
| 仅 5 帧 median（被拒绝） | 11 / 11 | 12 / 12 | 1.263±0.108 m | 1.216±0.126 m |
| 当前 5/20 双窗口 | **0 / 0** | **0 / 0** | **1.144±0.029 m** | **1.113±0.014 m** |

双窗口在这两个稳态方向场景中把 Governor/FULL 重复事件清零，并把距离标准差分别降低约 **74% / 87%**；这不是任意 Master 运动下无 hunting 或距离安全的证明。当前 transition 日志还保留 5/20 median、单帧估计和旧单帧 latch 的诊断 shadow，shadow 不接 actuator。

### KF 数据与动态变速结果

带噪转弯场景中，KF/raw finite-difference radial-velocity RMSE 为 **0.056/0.830 m/s**，Governor 事件为 **7/46**；机器人持续转向时的 KF 噪声抑制明显。无噪声回归反而是 KF RMSE **0.029 m/s**、raw **0.015 m/s**，体现滤波响应滞后。所有 KF 运行均有 capture-time RobotState 覆盖，没有 history miss；到达时使用最新 RobotState 会与正确插值产生最高约 0.054 m/s 差异。

最近的零噪声直线速度阶跃验证中，0.20→0.40 m/s 后，追赶状态持续约 6.1–6.2 s，CRUISE 锁存误差小于 0.001 m/s；转入 CRUISE 后延长观察 11.0–11.8 s，没有新增 Governor 事件。0.40→0.20 m/s 后 0.10–0.25 s 即进入 CRUISE，但 20 帧历史仍约 0.401 m/s；仅在最晚一组，5 帧通道把锁存值降至约 0.350 m/s。随后 slowdown 事件把命令带入新速度 ±0.05 m/s 的时间分别约 0.85、0.90、3.65 s，最后一组期间距离降到 **0.759 m**。无周期性 hunting、fall、轮饱和或 reference underrun，但减速 transient 的距离超程仍是当前风险。

### 当前入口与边界

- 当前 2D viewer：`scripts/view_stage6_5_manual_2d_follow.py`；独立 Master 控制窗口以 forward/lateral 速度 slider 连续移动，范围 ±0.50 m/s、步进 0.02 m/s；MuJoCo HUD 与右侧内置面板默认隐藏。
- 当前配置：`models/minisegway/stage6/config/stage6_4_2d_follow_config.json` 与 `stage6_5_radial_kf_config.json`。Stage6.2–6.5 的 packet/safety、wait/resume、2D/yaw 与 KF 验收产物留在 `stage6/results/`。
- 早期 Stage6.1、旧 deadband、窗口候选和 hunting 诊断现已归档到 `models/minisegway/stage6/reference/`；当前 quick-start 只指向 Stage6.5 viewer。
- 折返最近距离曾到 0.685 m，动态减速测试曾到 0.759 m；非倒车 follower 不提供固定 d2 安全保证。Stage 6 尚未连接 Stage 7 真实视觉；独立硬件时钟、真实 UART/USB transport 与 STM32 实测仍待实现或验证。

## 当前可试玩 demo

- Stage 4 frozen slope compensation：`scripts/view_stage4_slope_compensation_demo.py`，依次展示 ±8°、±15°，Q actuator OFF；使用 Stage 4 controller/estimator 的在线坡度估计与 `theta_eq` 补偿。
- Stage 5 V1.5 manual command：`scripts/view_stage5_v1_5_manual_demo.py`，slider raw `v_cmd` 走 candidate/scheduler、planner、50 ms stream 与 frozen controller，不直接改 `v_ref`。
- Stage 6.5 2D relative follow：`scripts/view_stage6_5_manual_2d_follow.py`，独立窗口控制 Master forward/lateral velocity；完整 observation/KF/Governor/FULL/yaw/MCU 链保持生效。

以上 demo 只用于仿真可视化，不表示额定实机坡度、安全或 Pi/STM32 性能认证。
