# companionbot_simulation

仿真 plant、virtual device/transport、实验组合与验收，不是实机控制组件。

- `core/`：原 MuJoCo MiniSegway、dynamic payload、virtual IMU；encoder/独立 estimator 已归 control。
- `runtime/packet_link.py`：原 in-memory transport、Stage6PacketLink 模拟调度；producer/receiver/safety 从 follow/control 导入。
- `scenarios/`：原 Stage 1–6 benchmark/runner/viewer。用 `python -m companionbot_simulation.scenarios.<文件名>`，GUI/viewer 由用户人工操作。
- `experiments/learning/`：原 Stage 4B 离线/拒绝学习实现，不接 production actuator。
- `ros/frozen_sim.py`：executable frozen_sim，node /companionbot/frozen_sim；独立 worker 跑冻结控制，ROS 只 50 Hz 发布估计/actual-command 与 /clock。
- `test/` + setup.cfg：资产路径/encoder 类型与原 packet/safety 回归。

依赖 control/follow/interfaces。physics 1 ms、controller 2 ms，DDS 不推进 tick、无 actuator subscriber。GT 保持 post-hoc-only。Stage 9 wrapper 在 tools/verification 重定向成果；直接原 scenario 仍按原设计写原阶段。models 资源通过 companionbot_contracts.paths 定位。参见 [架构](../../../models/minisegway/stage9/ARCHITECTURE.md)、[运行](../../../models/minisegway/stage9/RUNNING.md)。
