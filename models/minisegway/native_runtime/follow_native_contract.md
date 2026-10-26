# 原生 Follow 合同（本工程接口附件）

本附件记录实际实现，不是独立工程报告。`companionbot_follow::MotionAuthority` 是 FOLLOW/GUIDE_TO 共用唯一运动权限 owner；`FollowController` 是原 Stage 6.5 KF、最新 Governor 与 BearingYaw 的组合 owner。算法 Python 文件继续供冻结诊断和测试 oracle 使用，正式视觉 consumer 只调用 C++ 组合。

## 输入与职责

- `MotionAuthority(..., optional<FollowAuthorityConfig>)`：无 Follow 配置时保留原 Nav2 行为；显式 Follow 配置携带独立的最大速度/yaw、command、视觉 health、scan、braking 期限，均由组合层从 canonical 配置传入。
- `ACQUIRE/RENEW` 对应 GUIDE_TO；`ACQUIRE_FOLLOW/RENEW_FOLLOW` 对应 FOLLOW。跨 mode 的 acquire 被拒绝，即使 task ID 相同。先 RELEASE/STOP/WAIT 后，由新的显式 acquire 生成 authority epoch。
- `CHECK_FOLLOW` 同时检查原公共 scan/odom/pose health 与 MCU ready、最新有效视觉观测、真实 scan clearance 和由原 C99 制动律推演的 required clearance。Agent 不提供 Master locked；它读取这些实际消费方的 readiness。
- `update_follow_health` 接受同一观测 epoch 的严格递增 sequence/source time，保存 source 与本机 receipt 时间；LOST、深度/外参/历史/clock 无效撤销 FOLLOW，恢复健康本身不恢复 task。camera epoch 改变撤销现有 FOLLOW，已退休 epoch 不可回放，容量有界且满时锁恢复。
- provenance 水位和 pregrant 检查先于 valid/invalid 分支：旧 LOST、旧深度失败、同 seq 重复失败与退休相机结果不能撤销新观测或新 task。无法证明 frame 归属的结果保持 unavailable，由既有 source freshness watchdog 约束；当前 source 真正的 worker 失败/关闭使用原子 `invalidate_follow(reason,receipt)`，只撤 FOLLOW，不影响 GUIDE_TO。
- `receive_follow_cmd` 必须匹配 task、task source epoch、authority epoch、最新健康 observation epoch/seq/time，且采样时间严格晚于 task grant。旧计算结果不能影响新 task。GUIDE_TO 的 cmd 入口在 FOLLOW 模式直接拒绝，包括 malformed 输入，不改变当前 FOLLOW。
- `update_follow_obstacle(clearance, source, source_now, receipt)` 接收 native ingress 从真实全周有效 LaserScan 得到的保守最小距离；`update_follow_braking(required, receipt)` 接收 native runtime 从已消费 reference 与 sensorized local state、原 C99 制动律推演的距离。视觉有效不能覆盖该独立 guard。
- `execution_state()` 在 mutex 中返回纯 POD `{authorized, follow, authority_epoch}`，供 500 Hz 调度读取；完整 string/event snapshot 仅用于低频诊断/组合。
- 正式 MCU 状态通过 `update_mcu_status_with_provenance(state,reason,task_owned_stop,receipt)` 传递 C99 reason namespace 证明：只有 native runtime 本地产生的有效 external reason code 且属于该 owner 已发 STOP，才允许匹配 SAFE_HOLD 恢复资格。与本地 HEARTBEAT_TIMEOUT 恰好同名的外部文本不能让本地故障通过 CHECK/ACQUIRE。旧三参数接口仅保留软件 oracle 兼容，正式 native runtime 不使用它。

`FollowController.accept_state(FollowRobotState)` 只接 Pi 实际收到的 MCU RobotState（20 Hz），并保存有界历史；不能接 actor、plant GT 或 requested 当 actual。`accept_measurement(epoch, seq, capture_source, source_now, receipt, x, y, valid, reason)` 是单次低频边界：验证观测、capture-time ego 插值、Radial KF Joseph 更新、Governor 原状态转移与 5/20 median、BearingYaw，再调用唯一 authority。原 Governor 在 valid=False 时保持速度 latch 的数学定义不改，组合层在该条件下先撤权、拒绝消费该 latch。

## 重置与线程

新 task authority epoch 清 KF/Governor 观测状态，保留同一 source clock 下的真实 RobotState 插值历史。pregrant 图像不能 seed 新 task 的算法。camera epoch 候选只在 provenance/health 被接受后提交，退休相机结果不擦当前算法。

锁顺序是 `NavigationRuntime -> FollowController -> MotionAuthority`。Controller/Authority 都不会回调 runtime、Python planner 或 ROS executor；不存在反向 owner 调用。sensor-state 与视觉 measurement 可来自不同线程，Controller 用自己的 mutex 串行保护整段算法。配置、标准库或 GIL 释放不构成实时性认证。

## 时间和 guard

`capture_source` 与 RobotState 必须在同一个已声明映射的 source clock；`receipt` 是本机单调时钟。source age 与 receipt age 分别检查，不能互相替代。snapshot age 基于最后显式 sample/check/health 输入的时钟；快照本身不偷偷选另一 clock domain。

视觉 health 的 `master_locked` 只表示最新健康观测是否仍在其窗口内，`follow_obstacle_clear`/`follow_safety_reason` 独立展示；`CHECK_FOLLOW` 才是共同 readiness 结果。无 sensor coverage、无 scan、无 braking prediction、过期或 clearance≤required 都不可运动。

本工程 root 已冻结来源：视觉 source age 0.2 s；原导航 scan health 0.35 s；制动预测每20 Hz更新，失效期限采用原 heartbeat 0.15 s；reaction time为0.2+0.35+0.002=0.552 s。clearance offset由 canonical robot radius、LiDAR 安装平移范数、3σ已声明 scan noise 合成。具体数值由正式组合层读取配置并检查，不由算法类硬编码。

## 已验证与边界

`results/session_20261023/follow/final_parity.{txt,xml}`：最终 71 项 pytest 通过，包含13项算法 parity、500个完整组合测量、模式互斥、guard、MCU reason provenance、旧 invalid 竞态、原导航权限与 ReferenceBlock 回归。算法比较覆盖5000 KF步、6000随机Governor步、2000历史样本、1001角度，绝对与相对 tolerance 2e-12。`final_native_ctest.txt`：真实CMake构建/安装的两项原生 authority/Controller CTest 通过。ASan/UBSan的最终证据和源hash见同目录summary。软件夹具不代表视觉准确率或物理跟随成功；实体、Pi5/STM32执行尚未认证，`hardware_execution_ready=false`。
