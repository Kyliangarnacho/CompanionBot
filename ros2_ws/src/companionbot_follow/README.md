# companionbot_follow

Pi/上位机运动运行库，不执行actuator或物理arm。用ament_cmake＋ament_cmake_python构建；安装的原生模块缺失时明确报错，无Python运行fallback。

- C++ `MotionAuthority`：唯一FOLLOW/GUIDE_TO租约、epoch、freshness与撤销；`FollowController`组合原KF/history/Governor/BearingYaw。
- C++ `TwoPathProducer`：原FULL/LIGHTWEIGHT scheduler/lifecycle与25sample ReferenceBlock；直接复用Ruckig0.19.4。FULL只在接受计划时低频调用原SciPy cached-KKT数学solver。
- `runtime/navigation_motion.py`、`runtime/native_follow.py`：薄DTO/时钟/配置与数学solver组合。rclcpp ingress直接共享同一native owner，不经500Hz DDS往返。
- Python `core/`与`reference_producer.py`保留冻结实验和oracle；不再是正式导航/视觉运动状态机。

真实渲染视觉观测已接入native权限，但本轮depth heldout未达到原KF条件，FOLLOW不得授权运动。AI与Agent继续Python；hardware_execution_ready=false。接口、owner和构建见[运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。
