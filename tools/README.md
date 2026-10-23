# 工具用途

没有正式交互入口；唯一用户 Demo 是 scripts/demo_stage8_interaction.py。

- `environment/`：venv namespace hook、官方音频 deb 本地 fallback、rosdep metadata、原模型资产安装/恢复；使用同一个 .venv，不控制设备/桌面。
- `verification/verify_runtime_ros.py`：当前实际 Graph / 正式 launch / DDS / namespace / 参数 / 身份 / 异常退出验收；执行 Fake/replay，不访问硬件或 GUI。
- `verification/smoke_stage9_perception_ros.py`：明确选择 fixture track 1 的完整感知 DDS 诊断，非正式 Demo；旧正式 ROS smoke 已归档 stage9/reference/pre_runtime_architecture。
- control/stream/dynamic/models 复现 wrapper 保留历史场景名称，均要求显式 `--output-dir`；成果写本工程独立会话目录，禁止覆盖冻结证据。fake/replay/fixture 不作硬件通过。
- `audit_architecture.py` 只重审旧源码搬迁；不要在新工程调用它覆盖旧结果。当前工程的 SHA / 文档检查由 `audit_runtime_architecture.py` 执行。
- `build_minisegway_plant.py` / `extract_minisegway_twip.py`：原 plant/source helper，保留 provenance；日常启动不重建冻结 plant。
- `embedded/`：官方ST源/ARM本地工具链获取与版本核对、冻结参数导出、ioc/MDK模板生成；第三方代码不vendor进仓库。
- `verification/verify_cross_language.py`、`compare_cross_language.py`、`probe_native_roundoff.py`：显式C/Python闭环验收、完整差异记录和单项运算顺序ablation；`verify_embedded_bridge.py`为真实rclcpp/DDS＋PTY软件验证；`benchmark_embedded_perception.py`可在购板后复测当前三个模型。统一使用显式本工程output-dir，不作硬件成功声明。说明见 [跨语言合同](../docs/CROSS_LANGUAGE_EMBEDDED.md)。
- `verification/stress_embedded_uart.py`：有界双向8N1限速的实际bridge/DDS/PTY＋C协议压力；`serial_fault_inject.c`仅用于该测试的TTY partial/EAGAIN注入。`analyze_embedded_timing.py`读取真实协议、ARM ELF/反汇编/stack文件，输出预算与未知项，不宣称WCET。固件host逻辑测试在`firmware/test`，不进入ARM生产构建。

原 scenario/算法在 ros2_ws/src 对应 owner。当前入口、生命周期、接口与调试命令见 [运行架构](../docs/RUNTIME_ARCHITECTURE.md)。新完整工程按改动内容命名，不为每个小调整生成报告。

空间工程验收：`verification/export_spatial_geometry.py --check` 校验冻结 MJCF 派生几何；`verify_spatial_foundation.py --output-dir <本工程会话>` 执行五场景物理/GT 后验与被动消融；`verify_spatial_ros.py --output-dir <本工程会话>` 执行实际 DDS、source-time TF、失效及生命周期验收。均为诊断，不是正式交互 Demo。

`verify_spatial_protocol.py --output-dir <本工程会话>` 使用独立 typed DDS 软件夹具，检查时钟/epoch/frame/geometry/无效数据与重复源拒绝；不驱动 plant 或实体。

SLAM 实际 DDS/物理验收：`verification/verify_slam_integration.py`；地图/定位独立离线评测：`evaluate_slam_map.py` + 使用已安装 Karto API 的 `inspect_slam_pose_graph.cpp`；标准地图/图加载与隔离故障退出：`verify_slam_lifecycle.py`。这些是验证工具，不是正式交互或控制入口，见 [SLAM 合同](../docs/SLAM_MAPPING.md)。
