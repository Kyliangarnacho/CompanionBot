# CompanionBot ROS 2 workspace

八个 package 的源码是算法的唯一当前归属，不再从根目录旧 control/perception/sim/embodied_agent 导入。package 是软件职责与安装单位，不要求每个 package 启一个 node。

| Package | 责任 | 默认运行单位 |
| --- | --- | --- |
| companionbot_interfaces | ROS messages/services + ROS-free stamps/binary codecs | 类型/库，无 node |
| companionbot_perception | 感知算法、session、latest-slot ROS adapter | perception Node（启用相机时），同进程独立 executor；可选 camera_tf 独立进程 |
| companionbot_follow | 上位机相对估计、Governor、规划与 reference producer | 库，无真实视觉 consumer |
| companionbot_control | C99共享控制/估计/reference/safety；Python对照；C++串口基础设施 | 库＋显式opt-in mcu_bridge；被动 spatial_state 与 TF audit；无DDS平衡控制环 |
| companionbot_behavior | Agent/Catalog/Session/Supervisor 与交互 ROS bridge | app 静态组合；interaction Node 只负责高层 I/O |
| companionbot_simulation | MuJoCo、virtual sensors、packet scheduling、scenario | frozen_sim / spatial_sim 有限只读诊断 |
| companionbot_description | MJCF 派生几何、Xacro、RViz 配置 | 官方 robot_state_publisher，由空间诊断启动 |
| companionbot_bringup | launch 运行管理 | interaction.launch.py 正式；simulation.launch.py 诊断 |

在仓库根使用 `source scripts/env.sh`、`bash scripts/build_ros.sh`、再次 source，然后 `ros2 launch companionbot_bringup interaction.launch.py`。各 package README 指出本地 owner 与 adapter。系统/实际数据流图、接口/生命周期/依赖、参数与调试命令统一见 [运行架构](../docs/RUNTIME_ARCHITECTURE.md)。原 stage9 文档保留为迁移记录。

`build/`、`install/`、`log/` 是 ignored 生成产物。`colcon.defaults.yaml` 明确启用Python pytest；control现用ament_cmake安装C/C++/Python与CTest，其余owner测试不复制。500Hz controller / 1kHz plant仍在本地循环内；Nav2、MCU物理链路和真实视觉跟随未验证；原版 SLAM Toolbox 的仿真建图/地图管理见 [建图合同](../docs/SLAM_MAPPING.md)。C++ transport、STM32 HAL及C/Python共用核心见 [跨语言工程](../docs/CROSS_LANGUAGE_EMBEDDED.md)。

空间数据链的节点/TF/time/迁移合同和有限运行命令见 [空间状态](../docs/SPATIAL_STATE.md)。description 不复制第三方 CAD，标准 robot_state_publisher 负责轮关节与传感器挂载。
