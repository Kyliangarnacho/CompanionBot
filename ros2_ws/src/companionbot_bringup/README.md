# companionbot_bringup

统一 launch/进程管理，无算法/node 或第二正式应用。

- `launch/interaction.launch.py`：唯一正式 bringup，启动 behavior interaction，与根 Demo 共用 app；默认无设备/Fake/headless/no_mic/no_tts。可选 perception camera_tf；正式程序退出时清理子进程。
- `launch/simulation.launch.py`：有限冻结控制遥测诊断，启动 simulation frozen_sim。
- setup.cfg：原根依赖/authority/唯一入口检查。

依赖 behavior/perception/simulation、官方 launch/launch_ros。从仓库根 source scripts/env.sh 后：

```bash
ros2 launch companionbot_bringup interaction.launch.py
# 有限诊断：
ros2 launch companionbot_bringup simulation.launch.py duration_s:=8.0
```

launch 不接终端 stdin；终端交互用根正式脚本。参数/service 契约见 [RUNNING](../../../models/minisegway/stage9/RUNNING.md)。不启动 Nav2/SLAM 或虚构 map/odom TF。
