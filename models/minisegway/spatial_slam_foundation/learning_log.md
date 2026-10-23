# 空间状态与 SLAM 数据基座：可复用结论

## 相同数字不代表相同状态

轮体相对角、绝对滚动角、轮轴弧长与世界x各有含义。自平衡的纯俯仰即可构造普通差速公式的反例，已有 longitudinal estimator 的 pitch 修正应复用。转弯后的弧长必须经航向积分，不能换topic名字就声称是平面位置。

历史MJ左右轮名与ROS几何左右相反，正确做法是一个proper rotation加显式通道索引，不改冻结actuator命名或用反射。JointState保留相对轮角，body TF保留物理pitch；二者组合才描述轮的空间姿态。

body gyro-z积分与Euler导航yaw不同。10°pitch下直接body-z积分航向会少1.5192%，足以支撑被动空间层的Euler rate投影；没有理由为此改冻结控制yaw。共用数值核可以兼容不同物理量，前提是名称、输入与职责明确。

## 初始化必须有可观测信息

释放后近自由落体的加速度不足以初始化重力方向。首次roll冒烟失败并非需要滤波调参；原控制器已有静止持有样本，读取同一已消费measurement即可，不增加采样、不扰动RNG也不借GT。初始化与动态精度要分别分析。

## 时间来源比topic齐全重要

source sample、available、estimate horizon与ROS发布/到达时间不能互换。IMU packet比控制horizon早1ms是物理链路事实；首个source-time TF查询缺覆盖应显式报告，不能重盖now或补identity历史。

暂停clock不等于source超时；clock前进且source停止才构成仿真source timeout。epoch/映射变化不能静默续接旧odom。设备到ROS的mapping id只是一条协议声明，必须另有实测映射算法和不确定性证据。

## authority核验依赖真实运行API

Jazzy rclpy MessageInfo只暴露时间戳和sequence，不提供publisher GID；Python对象/dict访问形式都不能创造缺失字段。应查官方实现后，把actual GID检查放在现有C++基础设施层，不能从topic完整性或预期节点名猜测authority。保持唯一运行实现，删除不再被运行使用的平行Python审计器。

单父、单owner和static/dynamic通道是不同约束。TF audit失败应有可识别的GID/Graph证据并结束所依赖的有限图，而不是另起fallback发布者。Graph发现具有异步性，真实失败中已有GID但node name仍是unknown；用有界pending和WARN等身份解析，不批准unknown、不刷新等待起点，超过期限仍失败。不能把一次启动成功当作不存在故障。

## 配置一致性需要贯穿消费链

源码config与install share即使当前数值相等，修改后也可能分叉。launch应把同一选定geometry/LiDAR路径传给RSP、传感器生成和空间消费者。非零xyz/rpy的真实MuJoCo+Xacro对照比只测默认零外参有信息增益。

几何由冻结MJCF派生，带来源hash和一致性检查；可选camera只引用已有provisional外参，不再写一份平行数值。实体尚未标定的事实不能通过URDF存在而变成已标定。

## 合理仿真不需要oracle估计

MuJoCo物理site和碰撞几何是生成虚拟射线的必要输入，允许留在sensor simulator；pose/hit-id/scene模型不能输出给估计器。真实倾斜扫描命中地面是正确测量，不是需要GT leveling修复的噪点。是否适配二维SLAM应由后续传感器/算法验证决定。

瞬时360束应明确time_increment=0，不能伪装成机械扫描。简单独立测距噪声不代表材质、多径和真实器件模型；当前能力足够就扩展原MuJoCo，不为工具偏好换仿真器。

## 保留透明baseline与失败证据

已有encoder/pitch/gyro同源信息不足以证明需要再叠EKF。先提供透明、有漂移和未标定协方差声明的被动积分；未来有独立信息源或量化不足时再替换唯一odom authority。

finite输入不能保证finite派生结果，差分/矩阵传播可溢出。输出有限性gate与fault latch比随意增加物理限速阈值更直接，标称控制历史也能保持完全一致。

被动tap应以逐tick控制历史消融证明未改变旧链，而不仅比较最终误差。7000行控制历史完全一致、199300个native仲裁tick维持旧parity，提供系统回归证据。主机计时与arrival统计可变，要保留完整差异，不能把功能history一致夸大为全部文件exact。

验收工具也会出错：symlink-install执行权限、Jazzy字节常量、best-effort独立传输、元数据字段都应在真实运行中检查。原失败日志与修复后结果一起保存；沙箱禁止本机socket的失败不能被藏掉，也不应导致改机器人算法。

重复SIGINT可来自终端进程组与launch转发，清理阶段仍接受它会打断worker.join和manifest落盘。进入有界清理后忽略重复SIGINT、保留SIGTERM，实际进程组中断比只给parent发信号更能检验退出路径。传感器断流fixture还应保证消费者已获得启动观察窗口，不能把DDS尚未发现而没有观测误认作断流抑制成功。
