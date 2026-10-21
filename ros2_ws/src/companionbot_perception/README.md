# companionbot_perception

感知算法和 worker 的唯一归属，不产生运动命令。

- `core/`：Camera、YOLO、ByteTrack、Master/OSNet ReID、depth、same-source pairing、calibration/geometry，原逻辑与模型保持。
- `runtime/session.py`：原 full perception；`stream.py` 为原 detector stream。正式交互复用 worker，只有一个 Camera owner。
- `ros/publishers.py`：PerceptionPublishers 复用 behavior interaction node/context；不是第二相机节点。
- `ros/camera_tf.py`：可选 executable camera_tf，node /companionbot/provisional_camera_tf，只显式 opt-in 发布原未实测 mount。
- `diagnostics/`：camera/image/tracking/master-lock；`benchmarks/`：tracker/ReID，均非正式 Demo。
- `test/` + setup.cfg：color wire 与原 Stage 7 软件回归。

依赖 interfaces，不依赖 behavior/control/follow/simulation。诊断：`python -m companionbot_perception.runtime.session --mode full …`，GUI 由用户手动执行。read-complete 不是曝光时间；1280×720 K 不缩放，geometry 仍 provisional，stage6_connected=false。参见 [运行](../../../models/minisegway/stage9/RUNNING.md)、[架构](../../../models/minisegway/stage9/ARCHITECTURE.md)。
