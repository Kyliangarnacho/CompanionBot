# companionbot_perception

感知算法和 worker 的唯一归属，不产生运动命令。Python保留成熟AI编排；可选米制双目图像处理由普通C++库承担。

- `core/`：Camera、YOLO、ByteTrack、Master/OSNet ReID、独立depth、same-source pairing、calibration/geometry，原算法与模型保持。
- `runtime/session.py`、`stream.py`、`owner.py`：原感知session/正式app adapter，复用唯一Camera owner，不导入behavior/ROS。
- `runtime/rendered.py`：仿真图像低频adapter，保留完整acquisition/read-complete/ready、sensorized pitch历史、原模型与严格same-frame pair；显式选择唯一metric来源，未验证或失效时motion invalid。
- `native/`、`runtime/stereo.py`：真实同snapshot左右RGB→官方OpenCV4.6.0 `StereoSGBM`/校准、source、纹理与LR检查→axial-Z/诊断；一次批量pybind11调用，不进入500Hz控制tick。无GT输入、前帧深度替用或单目fallback。
- `ros/node.py`：与interaction同进程、独立executor；有界latest槽在有消费者时调用`ros/publishers.py`，不把本地消费绕回DDS。`ros/camera_tf.py`仍是显式opt-in的provisional mount TF owner。
- `diagnostics/`、`benchmarks/`、`test/`：诊断/原Stage7回归；正式交互入口仍由behavior app承担。

构建owner已迁为`ament_cmake`＋`ament_cmake_python`，同时安装原Python包、C++共享库与`_native_stereo`。精确链接system OpenCV4.6.0 core/imgproc/calib3d；Python cv2 wheel只通过NumPy像素边界使用，不共享`cv::Mat` ABI。沿用根`scripts/build_ros.sh`和`scripts/env.sh`、同一`.venv/bin/python`构建/运行。原`camera_tf` executable名称、模块main与安装位置保持；无import时编译或Python原生替代实现。原setup.py/setup.cfg作为Python元数据/历史配置保留，正式Stage7 test注册由CMake显式维护。

新增`models/minisegway/native_runtime/config/visual_stereo_runtime.json`只是显式可选仿真配置：`metric_depth_source=native_stereo`、`stereo_validation=null`。当前真实固定grid的精度/覆盖及时效未通过原门禁，不能授权FOLLOW；原YOLODepth仍独立执行并严格配对。验证文件必须绑定camera/protocol/asset、实际source与5个已载ELF、detector/depth/ReID模型SHA、全部distinct heldout及原精度门限；失败文件被拒绝，不回退到单目输出。

依赖interfaces，不依赖behavior/control/follow/simulation。RGB/Master本地消费者是behavior buffers；显式仿真adapter可把有效性结果交给既有C++FollowController，运动权限仍属于follow/MCU，当前未认证深度不会获准。物理C920 mount/距离语义仍provisional；1280×720标定K不自动缩放，read-complete不是曝光时间。`hardware_execution_ready=false`。GUI诊断由用户手动执行。参见[运行架构](../../../docs/RUNTIME_ARCHITECTURE.md)、[图像合同与实际边界](../../../models/minisegway/native_runtime/visual_contract.md)。
