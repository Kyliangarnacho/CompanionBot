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

- CompanionBot 的安装、测试、脚本运行和依赖检查统一优先使用仓库虚拟环境 `D:\project\CompanionBot\.venv`；不得因为虚拟环境缺包而切换到系统 Python。
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

## 电脑与 GUI 操作

- 禁止 Codex 调用 Computer Use、桌面自动化或其他方式控制用户电脑。
- 需要 viewer、IDE、浏览器或任何 GUI 人工确认时，只向用户提供简短明确的操作步骤，由用户亲自执行并反馈结果。
- 不得以节省步骤、自动截图或验证方便为理由绕过此限制。
