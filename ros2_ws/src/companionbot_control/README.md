# companionbot_control

底层混合包：共享C99控制核心、C++串口/ROS基础设施及冻结Python行为对照；没有已验证的物理actuator。

- `core/`：原 nominal ID-LQR、yaw PD、allocator、安全制动、Stage 4 runtime；probe/ID 保持原诊断与 Q-OFF 边界。
- `estimation/`：原 longitudinal/slope/slip estimator，从旧 sim 归属过来，不依赖 MuJoCo/GT。
- `sensors/encoder.py`：单份 encoder profile/loader，plant 与 estimator 共用类型。
- `runtime/mcu.py`：原 wire reference DTO、ACTIVE/NEXT receiver、MCUSafetyArbiter；`yaw_reference.py` 为 latest-yaw freshness。500 Hz 语义不变。
- `native/`：独立C99 estimator/LQR/yaw/allocator/reference/safety/brake及v2 wire库，供主机和STM32链接同一源码。
- `src/` + `include/`：C++ SerialPort与rclcpp `mcu_bridge`，独占串口、有界队列、时钟/session/deadline gate、错误和诊断；不运行500Hz闭环。
- `runtime/native.py`、`embedded_link.py`：有ABI校验的主机binding、显式simulation packet→embedded wire转换；`ros/packets.py`原v1诊断envelope保持。
- CMake/CTest + `test/`：packet wire、estimator/runtime/yaw authority及C/Python数值/安全回归，不复制测试。

ROS构建使用ament_cmake＋ament_cmake_python；原Python源码仍可通过根editable bootstrap导入，`setup.py`不再是ROS build入口。核心导入无需ROS/plant/视觉库；follow仍仅为测试依赖。真实串口软件已实现并做PTY验证，STM32G474 HAL工程见根`firmware/`；传感器/电机adapter保持unavailable。协议、构建与边界见 [跨语言合同](../../../docs/CROSS_LANGUAGE_EMBEDDED.md)，应用上游见 [运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。

`estimation/spatial.py` 复用既有 pitch/encoder path 与姿态积分核，提供被动空间姿态/SE(2) 积分；`ros/spatial_state.py` 消费 BalanceState，输出 odom、JointState、动态 body TF 与诊断。没有 actuator authority；无效源或时间抑制输出并退出。`src/tf_authority.cpp` 用实际 rclcpp GID 审计 TF owner/parent，冲突时关闭 required graph；未知 Graph 身份只可限时等待，不猜测来源。见 [空间合同](../../../docs/SPATIAL_STATE.md)。
