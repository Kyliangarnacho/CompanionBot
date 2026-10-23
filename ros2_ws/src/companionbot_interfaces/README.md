# companionbot_interfaces

共享 ROS / Pi-MCU 数据契约，不依赖具体算法/应用，无 node。

- `msg/`：SourceStamp、ColorFrame、MasterStatus、RelativeObservation、RobotSnapshot、BehaviorFeedback、ControlTelemetry、PiMcuPacket。
- `srv/UserInput.srv`：source/clock/sequence 的异步文字投递，响应表示 queued/reason/input_id，不表示执行授权。
- `companionbot_contracts/packets.py`：原 binary version 1 codecs；`paths.py` 定位仓库资产；`stamps.py` 与 `ros_diagnostics.py` 只用于 ROS adapter。
- `test/`：原 source stamp 合约；其他 wire/gate 测试随实际 adapter owner 安放。
- `McuFrame.msg`：新embedded transport v2的有界ROS envelope，带本机clock proof、session、MCU tick和host deadline；独立于原simulation PiMcuPacket/JSON v1。布局见 [跨语言合同](../../../docs/CROSS_LANGUAGE_EMBEDDED.md)。

CMake 生成 companionbot_interfaces.msg/srv；标准 Python install-dir 安装不同 namespace 的 companionbot_contracts，避免与 rosidl targets 冲突。packets/paths 是 ROS-free；stamps/ros_diagnostics 仅供 ROS adapter 使用。UserInput response 返回 input_id，旧生成代码客户端须重建。SourceStamp version 2 与 binary version 1 各有含义。接口及实际消费者见 [运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。

`BalanceState.msg` 是被动空间估计合同，Header 为映射后的 estimate horizon，meta 保留原 clock/epoch；几何 hash、IMU sample/available/age 与 validity 必须显式。`spatial_geometry.py` 校验 MJCF 派生几何和仿真 LiDAR 参数。设备端 producer 尚未实现；见 [空间合同](../../../docs/SPATIAL_STATE.md)。
