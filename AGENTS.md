# CompanionBot 开发约束

## 开工原则

- 开始新的控制器、辨识算法、传感器、硬件接口或大型模块前，先确认标准术语，并搜索论文、官方文档和成熟开源 upstream；先做 Borrow / Adapt / Reject 判断，避免重复造轮子。
- 优先复用成熟数值库和实现，例如 MuJoCo、SciPy ZOH/DARE/least-squares；只自行实现 CompanionBot 特有的接口、状态语义和安全边界。
- 只有现有简单 baseline 出现可量化失败时，才允许升级复杂度。

## 实验纪律

- 一次只验证一个核心假设，使用公平 ablation；明确区分 estimator 学习、supervisor 接管和 actuator 实际输出。
- 每个阶段产生的配置、结果、history、图表和摘要必须保存到该阶段对应的成果目录中，不得再次散落到共享模型目录或其他阶段目录。
- PID、LQR、Q/R、plant、payload、摩擦和 torque limit 一旦作为 baseline 冻结，不得在比较中暗改。
- 不得为了让新算法“赢”而挑场景、缩小扰动、隐藏失败、改变 truth/tolerance，或丢弃不利工况。
- Controller/estimator 禁止读取仅供验收的 GT/oracle 信息，包括 payload 真实位置/速度、接触状态、loaded reference model 和事后 theta_eq。
- 日志必须区分 requested command、软件限幅和实际 actuator command；保留 saturation、fallback、rejection 和 failure 原因。
- 分析失败时先区分 plant/model change、matched disturbance、hidden-state、unmatched/contact dynamics 和 authority 不足，再决定改什么。

## 工程范围

- MuJoCo physics step 固定 1 ms；V1 controller update 固定 2 ms，除非新阶段明确重新定义实验。
- Moving-payload V1 主链是 control-time sensorized state + 固定 nominal ID-LQR；Q observer 保留用于诊断，冻结 Stage 3 baseline 的 actuator augmentation 为 OFF。Stage 4 slope 仅把 Q ON 作为显式 ablation candidate。Auto Probe/RLS 只用于诊断，不得未经新证据重新接入 actuator。
- 验收脚本中的 payload GT observer 必须保持 post-hoc-only 数据边界。
- 第三方 upstream 和本地派生 CAD mesh 不进入本仓库；许可证和来源记录不得删除。
- 不为了“显得完整”增加空目录、万能抽象或未验证模块。

## Python 环境与执行

- 当前主开发环境是 WSL2 Ubuntu 24.04 + ROS 2 Jazzy，统一使用仓库 `.venv/bin/python`（基于 `/usr/bin/python3` 3.12、`--system-site-packages` 以复用 Jazzy deb）。Windows 维护时使用 `D:\project\CompanionBot\.venv\Scripts\python.exe`；两个系统必须各建 `.venv`，不得复制复用或因缺包改用系统 Python运行项目。
- 每次涉及依赖安装或测试前，先确认当前 Python 可执行文件路径、Python 版本，以及 `python -m pip --version` 显示的 pip 所属环境。
- Windows 下优先显式调用 `D:\project\CompanionBot\.venv\Scripts\python.exe`，并用同一解释器执行 `-m pip`、`-m pytest`、脚本和依赖检查，确保安装环境、测试环境与运行环境一致。
- 虚拟环境缺少所需包时，直接在该 `.venv` 中安装；禁止在一个环境安装、另一个环境验证。
- 仅当 `.venv` 已损坏或存在明确兼容性问题时才考虑切换解释器，并在执行前说明具体原因和影响。

## Git 与交付

- 用户检查前不要 commit、push、创建 GitHub repository 或添加 remote。
- 保留用户已有改动；删除或迁移文件前确认目标范围。
- 每轮结束只报告真实修改、验证结果、已知限制和未解决问题。

## 阶段收尾与文档分工

- 完成全部实现与验收后，清理本阶段已失效的临时算法产物、测试缓存及重复中间输出；删除前核对绝对路径、实际依赖和文档引用，不误删冻结成果、用户文件或仍支撑结论的原始成功/失败证据。
- `CURRENT_STATE.md` 负责当前工程状态、阶段完成项、验证结果和待办；每次阶段收尾做简短、准确的更新。
- README 只在原有结构上微调项目能力、限制与当前正式 Demo 启动命令，不加入“Stage X 又完成了什么”、测试流水账或阶段报告等冗余内容；这些属于 CURRENT_STATE 和对应阶段文档。
- 保持唯一正式交互入口 `scripts/demo_stage8_interaction.py`；诊断和自动验收脚本须明确标注用途，不另立正式 Demo。

## 临时产物、报告与交付

- 每个大阶段的文字交付统一放在 `C:\Users\29603\Desktop\CompanionBot\stage report\<阶段名称>\`（WSL：`/mnt/c/Users/29603/Desktop/CompanionBot/stage report/<阶段名称>/`）。同一阶段只维护一份 `final_report.md` 和一份 `learning_log.md`，后续迭代更新原文件，不为每个 Step/实验另建报告；Stage 9 使用 `Stage9_Linux_ROS2/`。
- `final_report.md` 包含 modified files、tests、deviations、remaining issues，并说明重要新增或调整类/函数的输入、处理与输出；区分已实现、计划和实际验收。`learning_log.md` 只记录可复用结论、失败机制与取舍，不复制测试流水账。
- 原始配置、JSON、history、日志和图表仍保存在仓库对应阶段成果目录；长期文档只同步当前合同、能力边界及报告索引。既有历史报告与冻结证据保留，不因统一报告格式追溯删除。
- 临时实验输出集中到本阶段成果目录下的独立会话目录；同类完整临时 run 默认最多保留最近 3 份，仅在结果安全落盘、Owner 与引用核对后清理。正式成果、失败证据、未知 Owner 路径、有效 workspace build/install/log 和用户文件不按缓存自动删除，不引入其他项目的清理器。
- 指定目录无法访问时明确报告，不静默改用其他长期目录。交付使用独立文件副本并核对 SHA256；若沿用已授权的仓库/桌面双位置保存，明确来源并核验副本一致性。纯文档交付只做内容、链接与副本检查，不默认重跑机器人仿真。

## 电脑与 GUI 操作

- 禁止 Codex 调用 Computer Use、桌面自动化或其他方式控制用户电脑。
- 需要 viewer、IDE、浏览器或任何 GUI 人工确认时，只向用户提供简短明确的操作步骤，由用户亲自执行并反馈结果。
- 不得以节省步骤、自动截图或验证方便为理由绕过此限制。
