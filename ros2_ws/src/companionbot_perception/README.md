# companionbot_perception

感知算法和 worker 的唯一归属，不产生运动命令。

- `core/`：Camera、YOLO、ByteTrack、Master/OSNet ReID、depth、same-source pairing、calibration/geometry，原逻辑与模型保持。
- `runtime/session.py`：原 full perception；`stream.py` 为原 detector stream。正式交互复用 worker，只有一个 Camera owner。
- `runtime/owner.py`：正式 app 的参数/observer adapter，复用原完整感知 session，不导入 behavior/ROS。
- `ros/node.py`：PerceptionNode，与 interaction 同进程、独立 executor；三个有界 latest 槽承接 observer，有消费者才调用 `ros/publishers.py` 序列化。无第二相机 owner，不把本地消费绕回 DDS。
- `ros/camera_tf.py`：可选 executable camera_tf，node /companionbot/provisional_camera_tf，只显式 opt-in 发布原未实测 mount。
- `diagnostics/`：camera/image/tracking/master-lock；`benchmarks/`：tracker/ReID，均非正式 Demo。
- `test/` + setup.cfg：color wire 与原 Stage 7 软件回归。

依赖 interfaces，不依赖 behavior/control/follow/simulation。诊断：`python -m companionbot_perception.runtime.session --mode full …`，GUI 由用户手动执行。RGB/Master 的本地消费者是 behavior buffers；relative 只有诊断消费者。read-complete 不是曝光时间；1280×720 K 不缩放，geometry 仍 provisional，stage6_connected=false。参见 [运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)。
