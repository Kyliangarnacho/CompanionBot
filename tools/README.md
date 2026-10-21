# 工具用途

没有正式交互入口；唯一用户 Demo 是 scripts/demo_stage8_interaction.py。

- `environment/`：venv namespace hook、官方音频 deb 本地 fallback、rosdep metadata、原模型资产安装/恢复；使用同一个 .venv，不控制设备/桌面。
- `verification/`：Stage 9 control/stream/dynamic/model/audio/DDS/launch 回归与源审计。结果集中 models/minisegway/stage9/results/architecture；fake/replay/fixture 不作硬件通过。
- `build_minisegway_plant.py` / `extract_minisegway_twip.py`：原 plant/source helper，保留 provenance；日常启动不重建冻结 plant。

原 scenario/算法在 ros2_ws/src 对应 owner。当前命令见 [RUNNING](../models/minisegway/stage9/RUNNING.md)。audit --snapshot 只用于首次整理前，不覆盖已有快照。
