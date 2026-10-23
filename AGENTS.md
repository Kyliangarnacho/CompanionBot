# CompanionBot 工程规范

本文件规定长期工程原则与权限；动态状态、验收结果及待办见 [CURRENT_STATE.md](CURRENT_STATE.md)，接口细节由 [运行架构](docs/RUNTIME_ARCHITECTURE.md)、[跨语言控制架构](docs/CROSS_LANGUAGE_EMBEDDED.md) 和 [空间状态合同](docs/SPATIAL_STATE.md) 维护。先读与任务相关的真实实现及历史证据，以本地工作区为准。

## 系统分析与复用

- 全局最优优先。修改前追踪上下游依赖、数据流、事实来源、模块职责、性能及安全影响，先确定问题应由谁解决，避免局部修补把问题转移给邻接模块。
- 新控制器、辨识算法、传感器、硬件接口或大型模块开工前，确认标准术语，阅读论文、官方资料和成熟 upstream 的相关实现，形成具体 Borrow / Adapt / Reject 判断；不能只读 README。
- 优先复用本仓库已有能力与成熟库，如 MuJoCo、SciPy ZOH/DARE/least-squares。只有简单 baseline 出现可量化不足，才增加复杂度；不为尚无证据的未来需求建立空模块、万能抽象或提前锁定算法与器件组合。
- 一个事实只有一个 canonical source；派生配置须有来源与一致性检查。保持接口和协议兼容，变更版本或语义须显式说明生产者、消费者与迁移范围，不建立平行状态机、重复配置或隐藏 fallback。
- Python 负责适合它的 AI/视觉、规划、实验与高层组合；C++ 承担需要它的通信和系统运行责任；MCU C/HAL 承担本地确定性控制和硬件边界。按实际收益分工，不为语言比例拆分；ROS 不调度高频闭环。
- 第三方 upstream 与本地派生 CAD mesh 不进入本仓库；保留来源、许可证和版本记录。

## 控制、空间与权限边界

- 冻结主链保持 MuJoCo 1 ms physics、V1/MCU 2 ms（500 Hz）control、ReferenceBlock 与既有 Pi/MCU 调度合同。重新定义周期必须是显式新实验，不能改写原 baseline 的比较条件。
- Moving-payload V1 使用 control-time sensorized state 与固定 nominal ID-LQR。冻结生产 baseline 的 Q actuator augmentation 为 OFF；Q observer、Auto Probe/RLS 保留诊断职责，不得未经新证据重新接入 actuator。历史显式 Q-ON ablation 不代表生产授权。
- Controller/estimator 不读取验收专用 GT/oracle，包括 payload 真值、接触状态、loaded reference model 和事后 theta_eq。GT observer 保持 post-hoc-only；仿真器可用物理状态生成传感器，但不得将真值伪装成估计或经传感器接口泄漏。
- 保留物理姿态、轮体相对角、轮轴运动和导航投影的不同语义。遵循右手坐标与 SI 单位；source time、estimate horizon、publication/arrival time、clock domain、source epoch 与 reference epoch 不可混用。跨设备时间须有明确映射与不确定性，不能直接相减。
- 每条 TF 有唯一 authority；没有定位就不生成 map→odom，没有传感器/时钟覆盖就报告 unavailable。不得通过清零 pitch、伪造传感器、补 identity TF 或复制 requested 为 actual 来制造完整链路。
- Agent、Supervisor、reference producer、MCU safety 与物理 arm 各自权限保持。ACCEPT 不等于完成，Fake 不等于实体执行；ROS、Agent 或通信 bridge 不越权恢复运动、代发 heartbeat 或绕过本地失联/超时/物理输出 gate。
- 日志区分 requested、软件限幅、allocator 输出及 actual actuator feedback，保留 saturation、fallback、rejection、故障与取消原因；未知实际输出不能记作成功或零值。

## 实验、故障与回归

- 一次实验验证一个核心假设，采用公平 ablation；区分 estimator 学习、supervisor 接管与实际 actuator 输出。PID、LQR、Q/R、plant、payload、摩擦、torque limit、truth/tolerance 和已冻结场景不得在比较中暗改。
- 不为展示新功能降低既有验收标准，或挑场景、缩小扰动、隐藏失败、丢弃不利工况。原始成功与失败证据一起保存，软件夹具、仿真和实物结果分开陈述。
- 故障先查数据、时间、坐标、权限与模块交互，再区分 plant/model change、matched disturbance、hidden-state、unmatched/contact dynamics 和 authority 不足；据此决定是否改算法，避免“修 A1 引发 A2”的循环。
- 对同一 blocker 制定有限、能排除假设的调查与停止条件；没有信息增益时记录边界并继续不依赖它的工作，不无限调参或用更大抽象掩盖根因。
- 根据影响面执行构建、自动回归和真实数据流检查。只有实测才能称为通过；对没有设备、没有 GUI 人工反馈或未覆盖的工况明确列出限制。

## 环境与电脑操作

- 主开发环境为 WSL2 Ubuntu 24.04 + ROS 2 Jazzy，项目统一使用 `.venv/bin/python`（`/usr/bin/python3` 3.12、`--system-site-packages`，复用 Jazzy deb）。Windows 使用 `D:\project\CompanionBot\.venv\Scripts\python.exe`；两系统各建 venv，不复制复用。
- 依赖安装或测试前核对解释器绝对路径、Python 版本与同一解释器执行 `-m pip --version` 的归属；安装、`-m pytest`、脚本及验证用同一 venv。缺包就在该 venv 安装，不因缺包改用系统 Python。仅有损坏或明确兼容性证据时才考虑切换，并先说明原因和影响。
- 使用现有环境脚本与构建流程；源码、安装态模块、ROS overlay 及配置必须对应。工程输出和 ROS 日志显式指向本工程成果目录，避免写入旧工程目录。
- 禁止 Codex 调用 Computer Use、桌面自动化或其他方式控制用户电脑。需要 viewer、IDE、浏览器或任意 GUI 人工确认时，只提供简短步骤，由用户亲自执行；不能以截图、验证方便或节省步骤绕过。

## Git、成果保护与入口

- 用户检查前不 commit、push、创建 GitHub repository 或添加 remote。保留用户已有改动；删除或迁移前核对目标绝对路径、实际依赖、归属及引用范围，不覆盖冻结成果和用户文件。
- 唯一正式交互入口为 `scripts/demo_stage8_interaction.py`；正式 ROS bringup 委托同一 app。诊断、测试和实验入口明确标注用途，不另立正式 Demo。
- 新工程按主要内容命名，不机械增加 Stage 编号。历史编号、入口文件名、报告和冻结目录保留，不为命名整齐追溯迁移或删除。
- 完成实现与验收后清理本工程失效临时产物、测试缓存和重复中间输出。临时实验集中在本工程成果目录的独立会话中；同类完整临时 run 默认最多留最近 3 份，仅在结果安全落盘、Owner 与引用核对后清理。
- 正式成果、支撑结论的成功/失败证据、未知 Owner 路径、有效 workspace build/install/log 与用户文件不按缓存自动删除，不引入其他项目的清理器。

## 文档与交付

- `AGENTS.md` 维护稳定原则；`CURRENT_STATE.md` 简短更新当前能力、结果、限制与待办；架构文档维护接口合同。README 只微调能力、限制及正式入口命令，不堆工程历史、测试流水账或报告正文。
- 只有足够规模的完整工程单独维护一份 `final_report.md` 和一份 `learning_log.md`；后续迭代更新原文件，小修不另开报告。final_report 包含 modified files、tests、deviations、remaining issues，以及重要类/函数的输入、处理、输出；learning_log 只记可复用结论、失败机制与取舍。
- 完整工程文字交付目录为 `C:\Users\29603\Desktop\CompanionBot\stage report\<工程名称>\`（WSL：`/mnt/c/Users/29603/Desktop/CompanionBot/stage report/<工程名称>/`）；既有工程沿用原目录，历史索引由 CURRENT_STATE 维护。
- 原始配置、JSON、history、日志、图表保存在仓库本工程成果目录，不散落共享模型或旧工程目录。长期文档只保留合同、边界与证据/报告索引，不复制原始流水账。
- 指定交付目录不可访问时明确报告，不静默换长期目录。交付使用独立文件副本并核对 SHA256；沿用已授权的仓库/桌面双位置保存时明确来源并验证一致，不产生两份独立演进报告。
- 每轮只报告真实修改、验证、偏差、限制和未解决问题，区分已实现、计划与已验收。纯文档调整只做内容、链接和副本检查，不默认重跑机器人仿真。
