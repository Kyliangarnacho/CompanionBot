# companionbot_behavior

交互与高层行为的唯一应用实现，不依赖控制/规划/仿真包。

- `app.py`：原正式交互程序；根 demo 薄入口和 ROS entry 均调用同一个 main。
- `core/`：原 Agent/Catalog/BehaviorSupervisor、task/feedback、wake/ASR/TTS、frames/inputs/UI/config；原 sleep/freshness/cancel/authority 保持。
- `ros/entry.py` + `interaction_bridge.py`：executable interaction，node /companionbot/interaction；只负责 typed robot/feedback、events、input/cancel/stop、diagnostics。
- `ros/runtime.py`：正式 app 的静态组合 owner；共享 context，每个 node 独立 executor 线程，处理异常和关闭。perception node 属于 perception 包。
- `runtime/inbox.py`：同步有界 ingress，64 项、local control 优先、满载拒绝；保留 ROS provenance / input_id / 消费 deadline。
- `ros/input_guard.py`：同机 clock/source/epoch/sequence/freshness gate。
- `runtime/session.py`：无设备文字诊断；`diagnostics/`：原 cloud/Stage 8 验收，非第二 Demo。
- `test/` + setup.cfg：ROS input 与原 Linux/Agent/Catalog/Supervisor/interaction 回归。

依赖 perception/interfaces。Agent 无 actuator authority，Robot/NavigationBackend 仍 Fake，hardware_execution_ready=false。queued 不等于授权/完成，异步任务身份保持；未来 Nav2 应包装既有 Backend 契约，当前没有 map/odom/导航。上游、下游、接口与运行方式见 [运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。
