# CompanionBot 开发约束

## 开工原则

- 开始新的控制器、辨识算法、传感器、硬件接口或大型模块前，先确认标准术语，并搜索论文、官方文档和成熟开源 upstream；先做 Borrow / Adapt / Reject 判断，避免重复造轮子。
- 优先复用成熟数值库和实现，例如 MuJoCo、SciPy ZOH/DARE/least-squares；只自行实现 CompanionBot 特有的接口、状态语义和安全边界。
- 只有现有简单 baseline 出现可量化失败时，才允许升级复杂度。

## 实验纪律

- 一次只验证一个核心假设，使用公平 ablation；明确区分 estimator 学习、supervisor 接管和 actuator 实际输出。
- PID、LQR、Q/R、plant、payload、摩擦和 torque limit 一旦作为 baseline 冻结，不得在比较中暗改。
- 不得为了让新算法“赢”而挑场景、缩小扰动、隐藏失败、改变 truth/tolerance，或丢弃不利工况。
- Controller/estimator 禁止读取仅供验收的 GT/oracle 信息，包括 payload 真实位置/速度、接触状态、loaded reference model 和事后 theta_eq。
- 日志必须区分 requested command、软件限幅和实际 actuator command；保留 saturation、fallback、rejection 和 failure 原因。
- 分析失败时先区分 plant/model change、matched disturbance、hidden-state、unmatched/contact dynamics 和 authority 不足，再决定改什么。

## 工程范围

- MuJoCo physics step 固定 1 ms；V1 controller update 固定 2 ms，除非新阶段明确重新定义实验。
- Moving-payload V1 主链是 control-time sensorized state + 固定 nominal ID-LQR + 单一 2 Hz Q-filter matched disturbance compensation。Auto Probe/RLS 只用于诊断，不得未经新证据重新接入 actuator。
- 验收脚本中的 payload GT observer 必须保持 post-hoc-only 数据边界。
- 第三方 upstream 和本地派生 CAD mesh 不进入本仓库；许可证和来源记录不得删除。
- 不为了“显得完整”增加空目录、万能抽象或未验证模块。

## Git 与交付

- 用户检查前不要 commit、push、创建 GitHub repository 或添加 remote。
- 保留用户已有改动；删除或迁移文件前确认目标范围。
- 每轮结束只报告真实修改、验证结果、已知限制和未解决问题。

## 电脑与 GUI 操作

- 禁止 Codex 调用 Computer Use、桌面自动化或其他方式控制用户电脑。
- 需要 viewer、IDE、浏览器或任何 GUI 人工确认时，只向用户提供简短明确的操作步骤，由用户亲自执行并反馈结果。
- 不得以节省步骤、自动截图或验证方便为理由绕过此限制。
