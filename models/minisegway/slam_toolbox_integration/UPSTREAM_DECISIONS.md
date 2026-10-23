# Borrow / Adapt / Reject

检视日期：2026-10-22。路线按用户确定为 2D LiDAR + Jazzy SLAM Toolbox，不重新比较 SLAM 算法。没有克隆、vendor 或维护第三方 SLAM 源码。

| 实际阅读的实现 | 判断与本工程用法 |
| --- | --- |
| [linorobot2 jazzy slam.yaml](https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_navigation/config/slam.yaml)，blob `34017964cc9ac9bc9a4e28dc4882de4d87adb667` | Borrow 标准 frames、Ceres、scan matching / pose graph / loop 配置组织；保留原质量阈值。Adapt 当前 LiDAR 上限和小室内图的空间采样/回环邻域，不照搬 debug=true |
| [slam.launch.py](https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_navigation/launch/slam.launch.py)，blob `fdf526f3933561426fd0ef839ac6a1e0e1d7c7e4` | Borrow include 上游 online_async_launch；Reject 其完整 Nav2 bringup、初始位姿未被当前官方 launch 接收的参数和自动 GUI |
| [bringup.launch.py](https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_bringup/launch/bringup.launch.py)、[sensors.launch.py](https://github.com/linorobot/linorobot2/blob/jazzy/linorobot2_bringup/launch/sensors.launch.py) | Borrow 清楚的 bringup/sensor/TF 职责；Reject micro-ROS 实体底盘、robot_localization 重复融合、普通底盘静态 body pitch 和 depth-to-scan 支路 |
| [mapping.md](https://github.com/linorobot/linorobot2/blob/jazzy/docs/mapping.md)、[transforms.md](https://github.com/linorobot/linorobot2/blob/jazzy/docs/transforms.md)、[sensors.md](https://github.com/linorobot/linorobot2/blob/jazzy/docs/sensors.md) | Borrow 标准 scan/odom/TF/use_sim_time 接口、map_saver 文件与运行组织；Reject 普通差速底盘控制入口。文档关于 TF timeout 的建议不代替实际 source-time / Graph / QoS 调查 |
| Jazzy deb `slam_toolbox 2.8.5-1noble.20260615.161600` 的实际 online_async_launch/config、[laser_utils.cpp](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/laser_utils.cpp)、[slam_toolbox_common.cpp](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp) | Borrow 原版生命周期、Best Effort scan filter、TF 查询、map/pose、Serialize/Deserialize 服务。确认 laser metadata 只缓存二维 offset/yaw，不把它说成动态 pitch 补偿 |
| [Karto Mapper.cpp](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/lib/karto_sdk/src/Mapper.cpp)，blob `6ef0688b4f19bcea4b3eaab9fa930e4d9a8fe572`；已安装 Karto.h / Mapper.h / serialization.hpp | 阅读实际 FindNearLinkedScans、FindPossibleLoopClosure、TryCloseLoop→CorrectPoses、BoundingBox/WorldToGrid/RayTrace 与数据注册/析构责任。Adapt 小房间 locality 参数；只用上游 API 离线读图，不改算法或文件格式 |
| 官方 `nav2_map_server` / `nav2_lifecycle_manager` deb `1.3.12` | 只 Borrow map_saver_cli、map_server 和其生命周期管理，未启动 Nav2 controller/planner/costmap/AMCL |

本次 raw/稀疏图失效原因分开保存：1）wall catch-up 导致空间 source gap；2）可选空 map_file_name 不能作为空 ROS CLI override；3）主机 reference 生成停顿触发旧 safety，后续实际 CPU/GC profiling 明确复现 52.39 ms 和 `[0,0,1]` 第 2 代收集；4）3 m + scan barycenter 的 near-linked 遍历覆盖整圈，图只有链；5）轴对齐地图最大边界栅格缺墙。没有把这些归因于 scan matching 或发明新算法。

第 5 项与上游 [issue #689](https://github.com/SteveMacenski/slam_toolbox/issues/689) 一致；本地代码中 ComputeDimensions 和 WorldToGrid 都 round，边界点可能落到 width/height 之外。没有把讨论中的补丁带进本仓库、没有合成墙或更换测试环境。最终更密的真实建图数据和正确回环在同一场景达到原覆盖门限；这一通过不能保证所有稀疏/轴对齐数据都避开该 upstream 限制。

有限调查停止条件：先检查 topic/QoS/time/TF 与实际图；对 near-linked 用源码和图边验证；对 occupancy 区分点的定位误差和 raster coverage。最终同一路线通过后停止参数试验，不引入补偿支路、laser_filters 或新栅格生成器。SLAM Toolbox 上游 Jazzy head 已晚于实测 deb，因此本工程证据以 2.8.5 tag / 已安装版本为准，不把新版功能当作现有能力。

许可证记录：linorobot2 [Apache-2.0](https://github.com/linorobot/linorobot2/blob/jazzy/LICENSE)；SLAM Toolbox 根 [LGPL-2.1](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/LICENSE)，各 upstream 文件另有原作者声明。仅适配参数/启动组织并动态使用发行包，没有复制维护其算法源码、mesh 或完整项目。
