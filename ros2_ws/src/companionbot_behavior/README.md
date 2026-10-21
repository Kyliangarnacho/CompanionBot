# companionbot_behavior

交互与高层行为的唯一应用实现，不依赖控制/规划/仿真包。

- `app.py`：原正式交互程序；根 demo 薄入口和 ROS entry 均调用同一个 main。
- `core/`：原 Agent/Catalog/BehaviorSupervisor、task/feedback、wake/ASR/TTS、frames/inputs/UI/config；原 sleep/freshness/cancel/authority 保持。
- `ros/entry.py` + `interaction_bridge.py`：executable interaction，node /companionbot/interaction；typed robot/feedback、events、input/cancel/stop；复用 perception publishers。
- `ros/input_guard.py`：同机 clock/source/epoch/sequence/freshness gate。
- `runtime/session.py`：无设备文字诊断；`diagnostics/`：原 cloud/Stage 8 验收，非第二 Demo。
- `test/` + setup.cfg：ROS input 与原 Linux/Agent/Catalog/Supervisor/interaction 回归。

依赖 perception/interfaces。Agent 无 actuator authority，Robot/NavigationBackend 仍 Fake，hardware_execution_ready=false。queued 不等于授权/完成，异步任务身份保持；未来 Nav2 应包装既有 Backend 契约，当前没有 map/odom/导航。参见 [运行](../../../models/minisegway/stage9/RUNNING.md)、[接口](../../../models/minisegway/stage9/INTERFACES.md)。
