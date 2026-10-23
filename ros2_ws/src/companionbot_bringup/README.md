# companionbot_bringup

统一 launch/进程管理，无算法/node 或第二正式应用。

- `launch/interaction.launch.py`：唯一正式 bringup，启动 behavior app，按相机配置组合 interaction/perception；默认无设备/Fake/headless/no_mic/no_tts。可选 camera_tf 独立进程。
- `config/interaction.yaml`：安装的 ROS adapter 参数；支持 namespace/params_file/agent_config，算法沿用原配置；`processes.py` 统一 required-process 退出，保留失败状态。
- `launch/simulation.launch.py`：有限冻结控制遥测诊断，启动 simulation frozen_sim。
- `launch/mcu_transport.launch.py`＋`config/mcu_bridge.yaml`：显式设备通信调试，默认command TX关闭，不启动planner/Agent/电机；协议及操作见 [跨语言合同](../../../docs/CROSS_LANGUAGE_EMBEDDED.md)。
- setup.cfg：原根依赖/authority/唯一入口检查。

依赖 behavior/perception/simulation、官方 launch/launch_ros。从仓库根 source scripts/env.sh 后：

```bash
ros2 launch companionbot_bringup interaction.launch.py
# 有限诊断：
ros2 launch companionbot_bringup simulation.launch.py duration_s:=8.0
```

launch 不接终端 stdin；终端交互用根正式脚本。配置、参数/service 合同与启动退出见 [运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。正式 interaction 不启动建图或 Nav2，也不虚构 map/odom TF。

`spatial_simulation.launch.py` 是有限无 GUI 空间诊断，启动 spatial_sim、spatial_state、C++ tf_authority 与官方 robot_state_publisher；同一几何/LiDAR 配置路径供仿真与 TF 消费。参数、手动 RViz 与能力边界见 [空间合同](../../../docs/SPATIAL_STATE.md)。

`slam.launch.py` 包含上游原版 online_async_launch；`slam_simulation.launch.py` 组合既有空间图与有限 mapping intent 序列，保留 MCU safety；`map_review.launch.py` 仅使用官方 map_server。参数、四种地图文件、手动 RViz 与边界见 [建图合同](../../../docs/SLAM_MAPPING.md)。
