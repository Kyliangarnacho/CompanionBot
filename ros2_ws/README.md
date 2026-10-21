# CompanionBot ROS 2 workspace

七个 package 的源码是算法的唯一当前归属，不再从根目录旧 control/perception/sim/embodied_agent 导入。package 是软件职责与安装单位，不要求每个 package 启一个 node。

| Package | 责任 | 默认运行单位 |
| --- | --- | --- |
| companionbot_interfaces | ROS messages/services + ROS-free stamps/binary codecs | 类型/库，无 node |
| companionbot_perception | 感知算法、session、observer publishers | 与 interaction 共用 node；可选 camera_tf |
| companionbot_follow | 上位机相对估计、Governor、规划与 reference producer | 库，无真实视觉 consumer |
| companionbot_control | 冻结控制/估计、MCU reference/safety/yaw runtime | 库，无 DDS actuator subscriber |
| companionbot_behavior | Agent/Catalog/Session/Supervisor 与交互 ROS bridge | interaction |
| companionbot_simulation | MuJoCo、virtual sensors、packet scheduling、scenario | frozen_sim 有限只读诊断 |
| companionbot_bringup | launch 运行管理 | interaction.launch.py 正式；simulation.launch.py 诊断 |

在仓库根使用 `source scripts/env.sh`、`bash scripts/build_ros.sh`、再次 source，然后 `ros2 launch companionbot_bringup interaction.launch.py`。各 package README 指出本地目录和 adapter。完整运行/测试命令：[RUNNING](../models/minisegway/stage9/RUNNING.md)；依赖/dataflow/职责：[ARCHITECTURE](../models/minisegway/stage9/ARCHITECTURE.md)；时钟/frame/authority：[INTERFACES](../models/minisegway/stage9/INTERFACES.md)。

`build/`、`install/`、`log/` 是 ignored 生成产物。`colcon.defaults.yaml` 明确启用 Python pytest，各包 setup.cfg 收集原根 tests 的所属部分；接口包用 ament CMake pytest。没有重复测试副本。500 Hz controller / 1 kHz plant 仍在原循环内；Nav2、SLAM、MCU 实机 transport 和 Stage 7→6 真实闭环未实现。
