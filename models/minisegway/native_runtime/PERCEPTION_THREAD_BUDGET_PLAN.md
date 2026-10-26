# CPU 视觉线程预算有限对照方案

本附件属于同一 Native Runtime 工程，不是新的正式 Demo 或独立报告。当前仅准备工具/固定输入；实际测量结论以 `results/session_20261023/thread_budget/` 各 fresh-process summary 和 comparison 为准。

## 假设及成熟机制

当前 `.venv` 中 Ultralytics 的 `utils/torch_utils.py:245–365` 显示：CPU `select_device` 最终调用公开 `torch.set_num_threads(NUM_THREADS)`；`utils/__init__.py:70` 的库预算是 `min(8,max(1,cpu_count−1))`。实际 `engine/predictor.py:454–478` 在首次 predictor `setup_model` 调用 `select_device`，再进入 `AutoBackend`；后续同 predictor `stream_inference` 在 model 已有时不重复 setup。因此仅设环境 `OMP_NUM_THREADS=1` 不能证明最终 worker 仍用一线程，必须记录实际 `torch.get_num_threads()`。

[PyTorch2.14 的 set_num_threads 官方接口](https://docs.pytorch.org/docs/2.14/generated/torch.set_num_threads.html) 管理 CPU intra-op 预算；官方建议在执行计算前设置。[当前版本 ParallelOpenMP 实现](https://raw.githubusercontent.com/pytorch/pytorch/v2.14.1/aten/src/ATen/ParallelOpenMP.cpp) 的 setter 更新 OpenMP/MKL 线程值并清理 MKLDNN computation cache，thread-local OpenMP 状态使每个实际 worker 的数值都必须检查。正式实现不会修改第三方 `NUM_THREADS`、monkeypatch 库或重写执行框架。

[set_num_interop_threads 官方文档](https://docs.pytorch.org/docs/2.14/generated/torch.set_num_interop_threads.html) 说明它只可在 inter-op 工作开始前设置一次。本次只研究 intra-op，interop 与 OpenCV 设置保持库原行为，全部记录，不把两个参数同时改变来混淆因果。

## 预先固定的实验

- 固定12张已经保存的真实 MuJoCo Bookstore/Gazebo 左RGB，取已有 `gazebo_640_fast_depth_eval/*heldout_split/rgb.png` 排序后前12张；不重新渲染、不挑检测成功帧。路径、PNG尺寸与SHA256锁在 `thread_budget/input_manifest.json`。
- 原 `PersonDetector` 加原 `YoloDepthAdapter`，各独立 model/worker。模型、检测640、depth768、confidence0.25读取原冻结 `visual_depth_protocol_gazebo_640_fast.json`，权重哈希固定。图像不缩小、模型和精度/安全门限不改变。
- 预算 `default/1/2/4` 各一个全新进程；所有进程启动前均固定 `OMP_NUM_THREADS=1`，以 `default` 观察库接管后的真实预算。其它环境变量原样记录；不对NUM_THREADS做补丁。
- 首次以真实RGB完成两个模型初始化/预测，记录 setter 前后实际线程数。随后每个 worker 在所有接受/测量样本之前用公开setter设置相同请求预算；两个setter串行完成，防止一边推理一边清库cache。`default` 保持原库值。再以同一真实RGB各预热一次。两阶段 warmup 都明确排除测量和安全接受样本。
- 两个独立worker按每张图共同开始、共同完成的barrier测量，固定并发竞争条件。这个有限微对照不冒充正式异步 latest-slot 吞吐、覆盖率或实时 source-age 验收；正式接入仍需要独立系统实测。
- 每个测量图通过真实 `cv2.imread` 读入，两个worker收到相同 ColorFrame，时间戳是本次离线读完成的真实单调时钟。旧渲染 exposure/sim时间不伪造成当前相机时间；原图 provenance 保留在manifest。

## 输出和判定

保存每调用 wall/thread CPU、P50/P95/P99/max，实际每次推理前后预算、native TID，配对 wall/process CPU、整窗口 process CPU、RSS/peak、上下文切换及 `/proc/self/task` CPU census。worker CPU不含其OpenMP helper，process CPU包含所有线程；`/proc` TID CPU粒度明确记录，不能误称纳秒级。

各预算保存完整depth和person boxes/confidence数组，比较 exact equality、形状/finite mask、最大及P99绝对误差、最大相对误差。这里不事后选择一个更宽的“相等门限”，也不修改原视觉/控制安全要求。只有一次短run的结果是布局选择线索；不承诺尾延迟置信区间、端到端跟随性能或Pi5成绩。默认与各预算的失败/退化同样保留。

运行工具：`tools/verification/benchmark_perception_thread_budget.py`。根 agent 完成功能回归并明确空闲测量窗口后才开始模型run。所有结果 `hardware_execution_ready=false`。
