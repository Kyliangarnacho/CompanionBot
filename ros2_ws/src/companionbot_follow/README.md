# companionbot_follow

Pi/上位机的跟随估计与数值规划库，不执行 actuator；没有单独 node。

- `core/`：原 radial KF、Follow Governor、motion/reference 生命周期、FULL/LIGHTWEIGHT scheduler、cached-KKT/Ruckig 与 bearing yaw servo。
- `runtime/reference_producer.py`：原 PiReferenceBlockProducer，输出现有 binary reference packets。
- setup.cfg：直接引用原根 rolling reference、wait/resume、alignment 回归。

运行依赖 interfaces/contracts 与既有数值库；不加载 MuJoCo/ROS/视觉库。synthetic observation/RobotState history 由 simulation 提供；Stage 7 真实观测尚未成为运行输入。source time、clock、history alignment、双窗口 cruise latch 保持。

规划与 follow 共用生命周期，保留一包；MCU 消费/安全属于 control，plant/调度属于 simulation。参见 [架构](../../../models/minisegway/stage9/ARCHITECTURE.md)。
