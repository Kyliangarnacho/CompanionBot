# companionbot_interfaces

共享 ROS / Pi-MCU 数据契约，不依赖具体算法/应用，无 node。

- `msg/`：SourceStamp、ColorFrame、MasterStatus、RelativeObservation、RobotSnapshot、BehaviorFeedback、ControlTelemetry、PiMcuPacket。
- `srv/UserInput.srv`：source/clock/sequence 的异步文字投递，响应只表示 queued/reason。
- `companionbot_contracts/packets.py`：原 binary version 1 codecs；`stamps.py` 为 ROS-free source/clock helper；`paths.py` 定位仓库资产。
- `test/`：原 source stamp 合约；其他 wire/gate 测试随实际 adapter owner 安放。

CMake 生成 companionbot_interfaces.msg/srv；标准 Python install-dir 安装不同 namespace 的 companionbot_contracts，避免与 rosidl targets 冲突。contracts 导入不需要 rclpy。ROS version 2 与 binary version 1 各有含义。参见 [接口](../../../models/minisegway/stage9/INTERFACES.md)、[架构](../../../models/minisegway/stage9/ARCHITECTURE.md)。
