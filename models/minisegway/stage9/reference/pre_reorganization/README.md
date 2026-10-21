# 整理前参考，非运行入口

当前 [架构](../../ARCHITECTURE.md)与[运行](../../RUNNING.md)在上两层。本目录带 COLCON_IGNORE，不作为 active workspace 构建。

- `companionbot_ros/`：整理前通用 ROS adapter/package、launch、17 项测试；由新 owner 替代，保留接口/迁移对照价值。
- `demo_stage7_perception.py`：只固定 full mode 的重复 wrapper；诊断直接用 perception/runtime/session。
- `audit_stage9_boundary.py`：初次迁移的原路径/字节审计；由前后快照、完整定义 AST/alias、历史 hash 和运行数值对照替代。
- `RUNNING.md` / `INTERFACES.md` / `STAGE9_REPORT.md`：初次两包迁移文档快照，仅为历史。

原代码位置快照/搬迁表见 ../../results/architecture/before_sources.zip 与 relocation.json。Stage 3–8 的冻结成果、失败证据、provenance 均留在原处。本轮只归档明确已替代的运行/审计文件。
