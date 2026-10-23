# 已退休的迁移期 ROS smoke

`smoke_stage9_ros.py` 保留迁移期的诊断实现及固定输出路径，供历史审查，不能再次执行覆盖旧证据。当前验收由 [verify_runtime_ros.py](../../../../../tools/verification/verify_runtime_ros.py) 负责，要求显式成果目录并检查真实 Graph、namespace/参数、source→feedback 身份、异常和退出。没有保留运行时兼容 wrapper 或第二正式入口。
