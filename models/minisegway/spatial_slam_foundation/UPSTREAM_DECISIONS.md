# 空间状态与 SLAM 数据基座：开工审查与复用决策

审查日期：2026-10-21。以本地脏工作区为准；开工文件 SHA256 与 Git 状态保存在 `results/session_20261021/workspace_before.json`、`git_status_before.txt`。已有跨语言工程、冻结算法和原始成功/失败证据不覆盖。

## 实际审查范围

本地：根 AGENTS/CURRENT_STATE/README，运行与跨语言架构、最近两项工程报告及 learning log；MuJoCo 两份 plant、virtual IMU/encoder、longitudinal estimator、yaw estimator/allocator、Stage3B runner 的初始化与逐 tick 调度、Stage5 frozen resolver、MCU C/HAL 与串口协议合同、perception geometry/source stamp、相机 TF、bringup 和既有回归入口。历史 Stage3–6 的 slip、payload、slope、timing 与失败结论约束本次设计，不能因新增 ROS 输出而重新定义。

指定 upstream 均实际读取实现，不只审查 README。三个项目当前均为 Apache-2.0；本工程借鉴职责和接口，**不复制或 vendoring upstream 源码、mesh**。

| upstream / 锁定 revision | 实际审查的实现 | Borrow / Adapt / Reject |
| --- | --- | --- |
| [linorobot2](https://github.com/linorobot/linorobot2/tree/f6b0bd0350c4a9d6b815ef4bb5c701a1e5129711)（jazzy） | `linorobot2_description/urdf/{robots/2wd,mech/base,sensors/imu,sensors/generic_laser,controllers/diff_drive}.urdf.xacro`；`linorobot2_base/config/ekf.yaml`；bringup 与 gazebo launch | **Borrow** description / sensor / bringup 分工与标准消息、唯一 odom authority。**Adapt** footprint 作为导航投影。**Reject** fixed footprint→body、caster、DiffDrive actuator plugin 以及不含 pitch 的轮速模型。其 EKF two_d_mode 不能用于压平物理 body。 |
| [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware/tree/1ed605af61a806f6bb593709da8bc8f9cec0cbdf)（jazzy） | `firmware/src/firmware.ino` 的 executor/control/publish/time-sync；`lib/{kinematics,odometry}/*.cpp`、`lib/imu/imu_interface.h` | **Borrow** 原始传感器与标准 odom 的接口分离。**Adapt** 传感器轴与 sample time 必须显式。**Reject** 50 Hz executor 驱动电机、RPM PID、publish-time 同盖 IMU/odom、fake IMU、gyro deadband、固定 covariance 的直接照搬。CompanionBot 保留 MCU 本地 500 Hz / ReferenceBlock / 物理 gate。 |
| [rbot](https://github.com/rlxai/rbot/tree/47af3fc3780bd582837f48ce3785d3d68834a1b7)（main） | description base/lidar 与 gazebo_sensors；control controllers.yaml；localization ekf.yaml；simulation launch / ros_gz_bridge.yaml；顶层 simulation bringup | **Borrow** GT topic 隔离、joint_states 单 owner、EKF 与底盘 TF 互斥、map authority 互斥。**Adapt** 传感器挂载和 headless 配置。**Reject** 普通 diff_drive / fixed footprint、提前引入全部导航与可选传感器、重复融合同源 pose/twist、把未验证的 Gazebo 能力当本项目验收。 |

配置事实按上述 revision 审查，不保证未来 upstream 分支不变。文件 blob SHA 与来源索引另存本次 `upstream_review.json`；代码仍留在上游。

## 标准和官方实现

- [REP-103 源文](https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst)：SI、右手 x前/y左/z上，optical x右/y下/z前。本地 MJ→ROS 是绕 Z 的 proper rotation，不能用反射矩阵让历史轮名看似正确。
- [REP-105 源文](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)：连续但会漂移的 odom；map→odom 由真正定位模块负责。本次不发布 map，也不发布 identity map→odom。
- [robot_localization 配置原文](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst)：二维模式及同源信息重复融合的风险。当前已有 encoder＋pitch 和 gyro 估计，先用透明的被动积分 baseline；未出现独立融合源或量化失败前不再叠 EKF。未来可替换唯一 odom authority，不能同时发布。
- [ROS 时间设计](https://design.ros2.org/articles/clock_and_time.html)、[标准消息源码](https://github.com/ros2/common_interfaces/tree/jazzy)：空间 Header 使用 source/horizon time；原有业务消息 publication-time 合同保持。IMU 无 orientation 时 covariance[0]=-1；Odometry pose/twist 分别属于 parent/child frame；LaserScan stamp 为首束 acquisition time。
- [MuJoCo 3.2 API](https://mujoco.readthedocs.io/en/3.2.0/APIreference/APIfunctions.html#ray-casting)：本地实际安装 3.2.0，有 `mj_multiRay`，支持无 GPU/GUI 的真实场景几何射线。优先扩展现有 plant。当前不需要 Gazebo 迁移；对旋转扫描时间、反射材质、多径、真实器件噪声和大规模性能不作保证。

部分 docs.ros.org / ros.org 页面本次返回 Anubis Access Denied，改读同一官方项目的 GitHub 原文，并核对已安装消息定义；未把拒绝页当技术资料。

## 自平衡专属推导与最小复杂度

`v_ROS = [-v_MJ.y, v_MJ.x, v_MJ.z]`，det=+1。历史 `left_wheel` 位于 ROS -y（右轮），`right_wheel` 位于 ROS +y（左轮）；只在空间 adapter 显式映射，保持所有历史 actuator/encoder 通道。

无滑滚动时 `ds = r*(mean(dphi_relative)+dtheta_body)`；不能只积分 encoder 或把 theta_eq 当姿态。已有 longitudinal estimator 已实现这项修正，空间 odometry 复用其 traveled-distance estimate，不另建一条轮速路径。该 s 是轨迹弧长，不能转弯后继续当 odom.x。

Euler `Rz(yaw) Ry(pitch) Rx(roll)` 的平面航向速率为 `(gyro_y*sin(roll)+gyro_z*cos(roll))/cos(pitch)`。固定 10° pitch、1 rad/s yaw 的解析反例中，直接积分 body gyro-z 少 1.5192%；纯 pitch、固定轮轴时，普通差速里程计会错误移动 `-r*dtheta`。这两个可重复反例足以支持 **仅在被动空间层** 增加轴映射与 Euler-rate 投影；不修改冻结 pitch/yaw 控制。

空间 roll 使用现有 complementary baseline（同一 tau / dt）补足未被控制器估计的自由度；空间 yaw 复用 timestamp/age-aware `GyroRelativeYawEstimator`，输入改为正确的导航 yaw rate。它与控制器的 body-z integral 是不同物理量，不能彼此偷偷替代。无磁航向、绝对位置或滑移修正；平面支持和小 roll/pitch 仍是能力边界，须报告漂移与动态误差。

## AGENTS 整理参考

实际读取 [MonoTeach AGENTS](https://github.com/Kyliangarnacho/MonoTeach/blob/main/AGENTS.md)（blob `6acf11900d58b1962e944b47bcb35c91d26f4e70`）。采用 canonical data、兼容协议、有限 blocker 调查与职责原则；不采用机械臂路径、设备状态或 GUI 例外。CompanionBot 的禁用桌面自动化、冻结控制、安全与 Git 权限继续有效。
