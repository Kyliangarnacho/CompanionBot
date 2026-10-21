# companionbot_control

底层控制侧库；本阶段保留 Python，没有物理 MCU/actuator node。

- `core/`：原 nominal ID-LQR、yaw PD、allocator、安全制动、Stage 4 runtime；probe/ID 保持原诊断与 Q-OFF 边界。
- `estimation/`：原 longitudinal/slope/slip estimator，从旧 sim 归属过来，不依赖 MuJoCo/GT。
- `sensors/encoder.py`：单份 encoder profile/loader，plant 与 estimator 共用类型。
- `runtime/mcu.py`：原 wire reference DTO、ACTIVE/NEXT receiver、MCUSafetyArbiter；`yaw_reference.py` 为 latest-yaw freshness。500 Hz 语义不变。
- `ros/packets.py`：原 simulation-only byte envelope，无 DDS command subscriber。
- `test/` + setup.cfg：packet wire 与根 estimator/runtime/yaw authority 回归，不复制测试。

运行只依赖 interfaces/contracts 与既有数值库；follow 仅为原 yaw 交叉测试的 test dependency。核心导入无需 ROS/plant/视觉库。未来 STM32 移植以原 packet/reference/safety 边界为依据，真实 transport/电机/IMU 未实现。参见 [架构](../../../models/minisegway/stage9/ARCHITECTURE.md)、[接口](../../../models/minisegway/stage9/INTERFACES.md)。
