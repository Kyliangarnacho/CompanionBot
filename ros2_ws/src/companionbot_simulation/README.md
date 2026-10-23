# companionbot_simulation

仿真 plant、virtual device/transport、实验组合与验收，不是实机控制组件。

- `core/`：原 MuJoCo MiniSegway、dynamic payload、virtual IMU；encoder/独立 estimator 已归 control。
- `runtime/packet_link.py`：原 in-memory transport、Stage6PacketLink 模拟调度；producer/receiver/safety 从 follow/control 导入。
- `scenarios/`：原 Stage 1–6 benchmark/runner/viewer。用 `python -m companionbot_simulation.scenarios.<文件名>`，GUI/viewer 由用户人工操作。
- `experiments/learning/`：原 Stage 4B 离线/拒绝学习实现，不接 production actuator。
- `ros/frozen_sim.py`：executable frozen_sim，node /companionbot/frozen_sim；独立 worker 跑冻结控制，ROS 只 50 Hz 发布估计/actual-command 与 /clock。
- `test/` + setup.cfg：资产路径/encoder 类型与原 packet/safety 回归。

依赖 control/follow/interfaces。physics 1 ms、controller 2 ms，DDS 不推进 tick、无 actuator subscriber。GT 保持 post-hoc-only。frozen_sim worker 只交接单槽数值快照，ROS timer 降采样发布并记录合并计数。tools/verification 的复现工具要求显式 --output-dir；直接原 scenario 仍按原设计写历史阶段。models 资源通过 companionbot_contracts.paths 定位。参见 [运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。

`core/spatial_scene.py` 添加室内碰撞几何和无质量 LiDAR，使用 MuJoCo mj_multiRay 生成瞬时真实场景射线；`runtime/spatial_session.py` 复用冻结 runner，被动读取已消费传感器；`ros/spatial_sim.py` 发布 source-time BalanceState/raw IMU/scan 和唯一 /clock。有限诊断不消费外部运动命令，GT 不作为状态输出；见 [空间合同](../../../docs/SPATIAL_STATE.md)。

建图复用 SpatialSession：`runtime/mapping_motion.py` 只组合既有规划器、ReferenceBlock producer、packet_link、MCU receiver/safety 与独立 yaw freshness，固定 intent 由 bringup/config/mapping_route.json 维护。无 GT 运动反馈、无 DDS 闭环控制，物理状态仅用于原传感器生成和显式单向 post-hoc 文件；运行见 [建图合同](../../../docs/SLAM_MAPPING.md)。
