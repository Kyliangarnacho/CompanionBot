# Stage 7 最终收尾报告

## 完成结果与验收范围

Stage 7 现在提供一个仅显示 RGB 的操作入口：`scripts/demo_stage7_perception.py`。它串起已验证的相机输入、YOLO26n、ByteTrack、手动选择 Master、OSNet 待确认参考特征与重获、独立运行的 YOLO26n-depth、同帧 torso 深度、相机坐标 XYZ，以及 provisional 机器人相对观测。Stage 6 仍未接入。自动化 smoke 只验证软件链路，不代表 GUI 真人验收已完成，也不代表真实安装几何或身份判断准确性已经验证。

## 运行时职责与接口约定

| 组件 | 职责与输出 |
| --- | --- |
| Stage 7.2 source | 唯一的 OpenCV 图像采集所有者；输出 BGR、`uint8`、H×W×3 的 `ColorFrame`，并带有 source ID、序号、实际分辨率和 host 读帧完成时间 |
| Detector worker | 在原始 source frame 像素坐标中生成 YOLO 人体框 → ByteTrack tracklet → Master/ReID；不会等待 depth |
| `MasterReIDSession` | 按顺序调用现有 Stage 7.6 API；状态由 worker 持有；GUI 只发送选择 ID / 清除命令 |
| Depth worker | 独立的单槽 latest-frame 输入；输出与准确 source frame 对齐的 `DepthFrame`，深度为 `float32` 米，缺失值为 NaN |
| 结果配对 | 有界保留 16 个结果；要求 source ID、序号、host 时间和尺寸一致；不会把新的 bbox 与旧 depth map 配在一起 |
| `master_depth.py` | 仅处理当前可见且 LOCKED 的 Master；使用中央 40% 宽高 ROI，至少需要 10 个有限正深度样本并取中位数；先对 bbox 中心 anchor 做去畸变，再反投影 |
| `robot_geometry.py` | 使用经过验证的刚性 camera extrinsic 和 source time 对应的 pitch，输出 `RobotRelativeObservation`；不具有跟随器或执行器控制权限 |

最终 demo 修复了此前一个集成缺口：较早的 Stage 7.7 `concurrent` 模式使用立即手动锁定，没有运行 ReID。该 benchmark 模式仍保留；最终 `full` 模式采用 Stage 7.6 的确认与重获行为。每次真实 tracker update 都会在 detector worker 中经过 Master/ReID，即使 preview/result 配对跳过了已完成的结果帧也如此。YOLO、ByteTrack、`track_buffer`、OSNet、crop 规则和 Master/ReID 底层算法均未改变。RGB 几何 overlay 使用匹配的 source-frame snapshot；若 Master 状态/绑定发生变化，或 depth-ready snapshot 已超过 1 秒，就丢弃该 overlay。这只是显示限制，不是 Stage 6 的 stale-observation 安全策略。

待锁定阶段需要同一 track 的三个严格同帧 512D embedding，时间偏移至少为 0、0.15、0.30 秒；随后求平均并做 L2 归一化。LOST candidate 必须连续经过三个 tracker update。分数 ≥0.68 时接受；分数在 [0.50, 0.68) 时约每 0.20 秒重试；更低分数停止重试，直到该 track 消失。这些是 provisional engineering thresholds，目前没有不同人员的负样本校准。最终 runner 读取现有 Stage 7.6 policy asset，不另建一份阈值配置。

## 坐标系、外参与 pitch 接口

配置文件：[config/final_demo.json](config/final_demo.json)。

- 相机光学坐标系 C：+X 向右、+Y 向下、+Z 向前；单位为米。
- 车体坐标系 B：+X 向前、+Y 向左、+Z 向上；原点在左右轮轴中点。
- 调平坐标系 L：与 B 共用物理原点和机器人朝向，并移除 pitch；它不是世界/地图坐标系，也不代表地面交点。
- `P_B = R_BC @ P_C + t_BC`。配置保存合法的 3×3 旋转矩阵和以米为单位的平移。旋转校验会拒绝反射或非正交矩阵，不会静默修正。
- `P_L = R_y(theta_at_source_time) @ P_B`。正 theta 表示绕 +Y 的右手旋转，即车头向下。平移后的点绕共同的轮轴原点旋转。`x_forward_m=P_L[0]`、`y_left_m=P_L[1]`；横向负值有效。Z-up 保留作诊断。
- 暂定安装位置：相机中心在轮轴前方 **0.08 m、左侧 0 m、轮轴上方 0.35 m**，朝下 **10°**。这是模拟安装参数，不是测量所得的真实安装位置，也不是相机离地高度。
- 当前 `ConstantPitchProvider` 输出 **0°**，并明确标记为 `constant_simulated` 和 provisional。替换接口为 `PitchProvider.at_source_time(t)`。实测实现必须转换时钟、按 t 插值历史数据、转换符号/单位，并在历史数据未覆盖 t 时返回 None。`PitchSample.source_time_s` 回显请求时间；可选的 `pitch_time_s` 和 `capture_time_mapped_to_host` basis 用于明确标识曝光时间映射到 host clock 的语义。当前 provider 使用的就是接收时间本身。不能把最新 pitch 值伪装成请求时间对应的值。runner 支持注入 `pitch_provider_override`，无需改动几何/业务链。

这些轴约定遵循 [Open Robotics 坐标约定](https://reps.openrobotics.org/rep-0103/)，但没有引入 ROS。旋转计算使用 [SciPy Rotation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.apply.html)。现有 MiniSegway MJCF 使用不同的车体局部轴约定（前向为 -Y，pitch 绕局部 X）；未来 robot-state provider 必须显式转换符号和轴向。这里不会把 Stage 6 的 pitch 原值直接别名接入。

## 时间与深度语义

当前所有 source timestamp 都表示 **host `perf_counter` 时钟域中的读帧完成时间**。该时间会原样贯穿 detection、tracking、depth 和 robot observation；它不表示曝光时刻。`infer` 是 depth adapter 的耗时；`age` 是 depth result-ready 时间减去 host 读帧完成时间。机器人观测还记录自身的 result-ready 时间和实际 pitch 对齐时间/basis。匹配的 pitch 必须回显相同的 source 请求时间并使用同一时钟域；只有明确标注了对齐 basis，才允许使用映射过来的较早曝光时间。无时间覆盖时观测不可用，source 时间或时钟不匹配时会拒绝。要实现真实曝光时间补偿，还需要设备时间戳，或经过验证的采集延迟模型加 Pi/MCU 时钟映射。

继承的 C920e K/D 标定仍严格对应 1280×720；没有缩放 K。该标定 asset 的设备身份是否与当前枚举到的 C920 完全一致，尚未独立验证。Detector 的 `imgsz=640` 与 depth 的 `imgsz=768` 都只是网络内部尺寸；bbox、ReID crops 和 depth maps 仍对应原始 source frame 像素。

YOLO26 的全局 axial-Z 与 Euclidean-range 约定仍未确认。[Hypersim 转换说明](https://docs.ultralytics.com/datasets/depth/hypersim/)明确把沿视线的距离转换为平面深度，这支持该训练数据源使用 axial Z；但 [depth task 文档](https://docs.ultralytics.com/tasks/depth/)并没有说明混合数据集 checkpoint 的全局约定。配置、preview 和输出中都把 `axial_z` 明确标为 provisional 假设。helper 为 axial 和 range 分别实现了公式。几何单测不能证明真实距离精度。

## 最终验证

- 项目环境：Python 3.11.9、NumPy 2.2.6、SciPy 1.14.1、OpenCV 5.0.0；收尾期间没有安装或升级任何包。
- 当前完整测试集：**150 passed**，通过 `.venv\Scripts\python.exe -m pytest -q` 运行。`pytest.ini` 指定 `tests/` 为当前测试集；已归档的 Stage 5.1 测试快照仍引用过时 API，保留为历史材料。
- 新增测试覆盖：相机向右对应 robot-Y 负方向、平移与 pitch 符号、pitch 两个方向、固定 leveled point 的不变恢复、pitch 缺失或时间错误、明确的 capture-to-host 映射 metadata、非法旋转、asset 分辨率、仅 RGB 输出，以及 final worker 中 pending/cancel/LOST/新 ID reacquire/clear 的接线。
- 真实 C920 **headless 全链路 smoke**：MSMF，requested/reported 1280×720@30，实际 1280×720，共 300 source frames，实测 source 27.09 Hz，正常退出。得到 **124 个严格同帧配对**、**104 个 camera-depth 与 robot observations**、一个由三次采样创建的 Master reference，以及一个 `MASTER_REID_REACQUIRED` 事件（ID 1 → ID 2，分数 **0.8963**）。这确认真实模型和事件链路确实运行；不能独立证明身份判断正确。

| 最终 smoke 通道 | 稳态 Hz | 推理耗时中位数 / p95 | 结果 age 中位数 / p95 |
| --- | ---: | ---: | ---: |
| Depth | 14.03 | 68.59 / 85.04 ms | 91.55 / 129.14 ms |
| Detector + tracker + Master/ReID | 27.23 | detector 27.48 / 33.96 ms | 31.60 / 49.75 ms |
| 最终验证运行：depth | 12.04 | 82.19 / 86.75 ms | 100.16 / 132.26 ms |
| 最终验证运行：detector + tracker + Master/ReID | 25.60 | detector 30.86 / 36.02 ms | 34.71 / 56.22 ms |

已排除两个 warmup 结果。`full` 模式 detector 通道的 age 包含 Master/ReID 耗时；detector inference timing 只统计 detector 调用本身。这次短时 smoke 不是新的公平性能消融，也不是 Raspberry Pi benchmark。数据见 [results/final_live_smoke/final_live_smoke_report.md](results/final_live_smoke/final_live_smoke_report.md)，同目录还保存本地 CSV、JSON 和事件日志。较早的 depth 配对消融见 [Stage 7.7 baseline 报告](stage7_7/results/STAGE7_7_DEPTH_BASELINE_REPORT.md)。

明确 pitch 对齐时间 metadata 后，又进行了第二次 **180-frame** headless 运行并正常退出：**56 个配对、54 个 robot observations**、一个 Master reference，较短场景中没有 reacquire 事件。输出确认 source/ready/pitch-time 字段分离，并使用 `host_read_complete` 对齐 basis。该次运行比第一次慢；两次结果都已保留，没有调 inference 参数，也不声称这是受控对比。最新数据见 [results/final_verified/final_verified_report.md](results/final_verified/final_verified_report.md)。

## 目录整理与审查结论

- 七个旧 `models/minisegway/stage7_N/` 目录已移到 `models/minisegway/stage7/stage7_N/`。56 个文件全部保留，包括被 ignore 的原始结果。`MIGRATION_MANIFEST.json` 记录原名称和 hashes。文本路径引用已迁移；测量数据和二进制 asset 保持原样。最终审查确认 56 个文件都在：34 个逐字节一致，22 个文本文件因路径迁移/归档说明而调整。没有修改原始 CSV 或二进制 asset。
- 回滚保险 patch 原样移到 `stage7_3/reference/`。其中的旧路径作为历史快照内容有意保留。
- 现有 image/stream/tracking/Master/depth benchmark 入口保留文件名，并改为写入新目录。最终 launcher 是同一 depth runner 的薄入口，没有创建第二套 camera acquisition。当前和兼容旧名称的七个 demo CLI help 检查均通过。Markdown 本地链接有效；除了不可变 patch 外，没有遗留旧 stage 目录引用。
- 历史报告保留原始测量和阶段决定。被后续运行时行为取代的内容加了简短归档说明。learning log 记录了 timestamp 修正、环境错位、用户撤回的 camera timeout 判断、ReID 阈值变化、summary NameError 和几何语义不确定性。
- 本地 detector weights 处于 ignore 状态；没有新 vendoring 模型或第三方源码。用户原有未提交 Stage 7 工作及其 `AGENTS.md` 修改均予以保留。

## 真人验收与剩余硬件工作

```powershell
.\.venv\Scripts\python.exe scripts\demo_stage7_perception.py --camera-device 1 --preview
```

点击一个 person box，观察 `PENDING_LOCK → LOCKED`，再前后及左右移动。RGB 面板显示 Master 状态、ROI/anchor、depth、camera XYZ、provisional robot XY，以及简洁的 Hz/infer/age。相机 +X 朝图像右侧增加，因此 robot 的 Y-left 应减小。短暂丢失及 LOST/ReID 行为沿用 Stage 7.6 语义。按 `c` 清除；按 `q`/Escape 退出。像素坐标对应未镜像的原始图像；OpenCV 窗口可以缩放显示画面，但不会改变 inference 或 bbox 坐标。

用于实体跟随前还需：确认 K/D 对应的物理相机和安装外参 T；验证 depth convention 与 metric accuracy，以及不同人员的 ReID 负样本表现；提供包含符号、时钟和延迟对齐的真实 pitch；测量 Pi 5 算力和真实 MCU transport；定义 Stage 7→6 边界的 stale/lost 行为。Stage 6 的 synthetic observation cadence 不构成 detector FPS 要求。冻结的 Stage 6 Governor/KF/yaw/controller 参数以及已知制动距离风险都保持不变。
