# ROS 2 运行架构：upstream 取舍

2026-10-21。实现前阅读当前七包、正式 app/session、感知 worker/observer、仿真 adapter、接口和既有回归。基线及本工程证据只写 `results/20261021T1335/`，不改旧实验。

| 上游 / 标准术语 | Borrow / Adapt / Reject | 本项目决定 |
| --- | --- | --- |
| [ROS 2 Jazzy Composition](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Composition.rst)：Node 是通信单元，process 是部署/故障单元 | Adapt | rclpy 静态组合 interaction 与 perception，独立 ROS executor 线程。相机/模型仍由原 worker 唯一持有，帧和 Master 本地共享，不绕 DDS 回来。这里不是 rclcpp_components container，不声称支持 ros2 component load。 |
| [Nav2 Jazzy bringup](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/navigation_launch.py)：namespace、配置文件、launch 管进程 | Borrow | bringup 安装配置，明确 namespace / params_file / 输出目录，正式进程结束即关停附属进程；失败不自动 respawn，不悄悄重置 source epoch 或任务身份。 |
| [Nav2 lifecycle manager](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_lifecycle_manager/README.md)：按依赖启动、反序退出 | Adapt | 显式 STARTING / RUNNING / STOPPING / CLOSED / ERROR 运行状态、输入 gate、错误传播与完整资源清理。暂不实现 ROS ManagedNode 接口或热重配置：Camera/模型与 Agent task 还没有 reset/reconnect 合同，空壳 lifecycle 服务会误导消费者。 |
| [ROS managed node design](https://design.ros2.org/articles/node_lifecycle.html) | Reject（本轮引入完整 manager） | 运行状态不命名为 ROS lifecycle；重启通过完整进程退出与重新启动，保留新 epoch。后续实机资源确有独立 deactivate/reconfigure 需求再引入。 |
| [ros2_control Jazzy controller manager](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/userdoc.html)：read/update/write、高频循环与非实时接口分开、命令与状态有独立责任 | Borrow 思路 / Reject 移植 | 保留既有 1 ms MuJoCo / 2 ms control、receiver/safety/allocator 同一调用链。ROS 只拿降采样遥测；本轮不添加 controller_manager、URDF 硬件插件或 DDS actuator subscriber。 |

主要失败机制：ROS spin 曾依赖 Agent asyncio 调度；原始 RGB 序列化/发布曾发生在 capture observer；interaction 发布全部感知 Topic，ROS Graph 隐藏了责任归属；后台任务错误和 cleanup 顺序缺乏统一收口。新架构必须用实际 Graph、背压/故障与退出测试证明改善，不能只增加目录或节点。

故障边界：感知与交互共享进程，Python 异常由 owner 传播并有序退出；不能隔离 native segfault、GIL 或 OOM。将来测得故障/性能需求时可将感知独立进程，但需一起实现带时钟、epoch、freshness 的消费者，不能只把 RGB 搬到 DDS 即宣称完成。
