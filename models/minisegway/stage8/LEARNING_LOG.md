# Stage 8 learning and debugging log

本日志沿袭项目的“观察问题 → 查证 → 有限修正 → 验证与边界”记录方式。这里的门限、旧入口和测试数属于当时调试状态；当前 baseline 统一以 [最终报告](STAGE8_REPORT.md)、[agent.json](config/agent.json) 和 [RUNNING.md](RUNNING.md) 为准。完整连续记录保存在 [reference 历史快照](reference/stage8_development_history.md)，成功和失败证据继续留在原 `results/`。

## Stage 8 V1 — 单 Agent 与确定性行为边界

- 采用 PydanticAI Slim + Qwen，借用官方 Function Calling、structured output、streaming/events、usage limits 和 cancellation；拒绝迁入旧 agent-core、多 Agent 路由或 MCP 平台。CompanionBot 只保留本地唤醒、帧契约和高层行为 Supervisor。
- 第一次真实请求遇到401。查证不是模型能力问题，而是密钥解析正则截断含点的新格式；修正解析并加回归，未打印密钥或通过换模型掩盖失败。[401证据](results/qwen_63323d6f750c40389e72544548568215/summary.json)保留，Token usage未知而非证明零消费。
- 普通问候、FOLLOW Function Calling、JSON Object/Pydantic校验和合成视觉均通过；四类成功验证合计5 requests、4364 input / 147 output tokens。计入上述失败为6 network attempts。它们只是初版API兼容性证据，不包含真实相机/麦克风验收。
- Windows pipe输入与测试UTF-8不一致，CLI统一标准输入/输出编码。取消竞争曾重复记录CANCEL，清理改为只撤销本轮仍active的任务；丢ACK保持执行不确定性，不能谎报未执行。V1完整回归为204项，原150项冻结工程测试保留。

## Stage 8.1 — 真实相机共享与早期音频

- 在Stage7既有capture/fanout上加非消费式tap，先给detector/depth slots供帧，再发布独立只读副本；Agent不打开第二个CameraStream。来源、sequence、read-complete时间、源尺寸、ROI和JPEG编码尺寸分别保留，source重启须clear。
- 初期相机设备/后端出现失败，保留[后端诊断](results/camera_backend_451d0d4f57034e3689a6e6027cd12cbb.json)。用户插入C920后实际链路才通过，不把缺设备时的视频fixture当作实机证据。
- 早期用C920麦克风跑过PCM/SAPI检查；这是当时的设备bring-up，不是最终输入选择。用户随后指定内置Realtek，正式入口改为按名称选阵列并校验格式，失败不回退C920。
- 同一Scene不受控的tap OFF/ON各360帧短测有360 calls / 0 failures，copy median0.948 ms。detector/source吞吐在ON更高不能证明tap提速；CPU/scene变化未控制，因此只报告执行与复制开销，不给因果结论。
- 真实关键帧Qwen检查1 request、1854 input / 26 output tokens，首文本1.276 s、完成1.755 s。host read-complete从未改名为曝光时间，raw RGB valid也不等于geometry或运动许可。

## 最终交互集成 — Streaming、设备和可见反馈

- 用户指定原生麦克风后，Realtek安静短采样曾近零。只读检查权限、endpoint静音/电平、格式和PCM，并保留失败/低电平证据；未操作Windows设置，不因安静采样就宣布硬件损坏或改用C920。[Realtek诊断](results/realtek_5710f1f2602f4d04b8fb2fef9d862ad7.json)。
- 用户反馈看不到问题是否被accept。原正式入口增加窗口、input source/ACCEPT/REJECT/IGNORED、具体gate原因、Master/task、ASR/RMS和工具call/return；首帧之后才启用音频，模型加载和PERCEPTION_READY另有提示。
- 使用原生Agent事件流逐段显示，并将稳定中文句段送入独立SAPI FIFO。行为轮等Supervisor ACK和最终结果，避免提前播报成功；generation取消清待播并停止当前语音。
- 最初只等待最后一条语音future，可能最后排队项已取消但当前播放尚未停。改为等待本generation的全部future，工具失败也purge；idle从最后播报结束后重新计时。[真实取消检查](results/native_cancel_final_9ae6898349414dd788d5882b7937b3d7.json)。
- MSMF硬件transform启动很慢，组合入口使用OpenCV官方process-local flag并在cv2导入前设置。后来一个验证脚本先import cv2再import入口，flag太晚，仅1帧后失败；[失败](results/speech_native_cc0b71d2935648078fe06fd7fa7af5ce.json)保留，修正验证脚本的import顺序后成功，Stage7独立入口/算法未改。

## 首轮人工验收 — 唤醒、视觉和再唤醒拒绝

- 用户报告叫不醒、“你前面有什么”被拒绝、休眠再唤醒后的“你好”被拒绝，以及怀疑相机未加载模型。实际[首轮日志](results/interaction_a9864d9c009843d696f6fc61d4bee900.jsonl)有Realtek peak11106 / RMS208.80、4条transcript、8次唤醒确认拒绝；麦克风确实有声音，但未保存逐句原文/PCM，不能补造每次失败的确定原因。
- 两路Vosk曾要求自由问句与受限grammar整句等价，带问句尾部无法吻合；改为确认唯一唤醒前缀、保留自由问句尾部。受限grammar不能单独以高confidence制造唤醒。原confidence最低词不改，新增句级分数与epoch，旧状态音频不得进入新会话。
- 视觉hint只枚举少数词，遗漏“前面”和实物品牌/功效表达。按真实失败升级为同一Agent原生`capture_view`/多模态ToolReturn，取图后禁止行为；显式`/look[-roi]`保留。无需额外分类LLM。[语义视觉联合验证](results/semantic_integration_a468f77d85fb4e09918e12b72365e26a.json)。
- 自然缄默由typed output tool `enter_sleep {action:SLEEP}`结束，不维护“退下/滚/闭嘴”的关键词路由；引用/翻译负例正常答复，图片轮不允许休眠授权。
- 初次照最新文档写output prepare构造参数，在本机PydanticAI2.53.0产生TypeError；[失败回归](results/semantic_fix_tests_eb86e425103f44409c99cfb558ea5956.json)保留。核对安装版本后改用该版本官方PrepareOutputTools capability，未升级Runtime。
- 拒绝提示曾混淆键盘与语音，新增来源/具体confidence、age、epoch、播放原因；休眠→文字唤醒→键盘你好回归通过。相机[metrics](results/interaction_a9864d9c009843d696f6fc61d4bee900_metrics.json)实际有5860帧、5407 detector /2188 depth，首次推理约2.64/2.72 s；启动快不代表漏掉模型。
- 该轮真实Qwen自然视觉、引用负例和3种自然缄默合计6 requests /6 attempts、8392 input /191 output tokens，0重试；自然视觉首文本2.070 s、首SAPI提交2.340 s、模型完成2.410 s。语义休眠零语音段；这些是有限smoke，不是商品事实准确率benchmark。

## 深度人工验收 — ASR、容错、口头取消和idle

- 用户看到“你好小七/小琪”仍叫不醒。[深度运行](results/interaction_a2bfbb34585348898ee9641da17a0442.jsonl)有peak13452 /RMS257.90，候选句级0.64187/0.46825被当时0.80挡住。按授权保留“你好小”前缀，在ASR边界接受qi同音名字，并把普通唤醒句级门槛设为0.35；不是任意编辑距离，也不是所有行为门控降到0.35。
- Vosk small是轻量唤醒baseline，但普通问句错字/断句已有实际失败。Borrow官方sherpa-onnx SenseVoice int8+Silero VAD，保留同一个麦克风capture；0.9 s停顿整句提交。不训练模型、不自研ASR/VAD、不增加云端请求。[资产来源/revision/SHA256](results/speech_assets_dfcbbc9df68f4395b08363fe80ad3882.json)保留。
- SenseVoice不提供可用confidence，因此以null/unavailable和VAD标志明确表达；不能假装高分。无confidence的语音行为须带名字前缀，PCM16k mono s16le、callback receipt和epoch原义不变，其他rate不隐式转换。
- 用相同8条合成PCM作公平对照，71字符中Vosk错1、SenseVoice错0，median decode0.844/0.093 s。旧模型把“打断回答”识别为“打转回答”；普通简单合成句大多也正确，不能据此宣称真人准确率显著提升。[完整对照](results/speech_comparison_08543321578246e289b773a3b90f7680/comparison.json)。
- 播放普通问句仍保护，只开放完整名字+明确打断口令；用近期自身TTS文字作veto。合成口令经真实SenseVoice/Silero、Fake streaming和原生SAPI验证model/speech CANCEL、pending0。[取消证据](results/speech_interrupt_406b209ea223485c99559629079002d0.json)。这不是AEC，扬声器重叠声仍可能漏打断。
- 一个fake声卡fixture名字不含Microphone/阵列，被选卡逻辑正确拒绝，验证脚本却只等callback而误记Timeout；[失败](results/speech_interrupt_12c334edc43a4d46b23797341da2b0bb.json)保留。修正fixture和异常监控，不修改产品选卡规则来通过测试。
- idle最终为30 s，生成/播放/VAD用户语音保护计时；到期本地休眠只提示一次“小柒先走啦”，主动“闭嘴”不告别。[真实设备检查](results/speech_native_e54e5f34033048d6b1e3cdc8b6156b9a.json)通过，但无人受控说话，不把0 transcripts当识别性能。

## 白盒审查与阶段收口

- 用户随后反馈基本功能正常，仅底部按钮裁切。窗口改为聊天区可收缩、输入/按钮/提示保留行空间、按钮均分宽度；不改变交互逻辑，无桌面自动操作。
- 白盒审查明确SLEEP/ACTIVE是真实session状态，LISTENING/THINKING/SPEAKING是派生视图；睡眠时tap继续复制帧，零Agent读帧/上传不等于camera停采。发现RobotSnapshot缺显式clock域、导航缺pose/progress映射、ACK需有限deadline/幂等、wire-dispatch前无再次freshness校验，写入最终报告，未为了形式完整新增算法。
- 整理前报告把不同阶段门限/设备/测试数叠在一起，容易误读。现拆为最终报告、运行说明和本Learning Log；原完整连续记录归入reference，保留失败上下文，原results路径不动。
- 现行Runtime从未调用的`vision_requested`关键词helper及其10个参数化断言退出active；旧“小伴→小半”专用映射和1个兼容断言归档，当前仍支持一般可配置唤醒，现行“小柒”容错/门限不变。Fake Model中的简化关键词只是离线fixture，仍有用途，不当作云端语义实现删掉。
- 早期`smoke_stage8_interaction.py`归档：它仍走旧Vosk音频、没有当前真实Master前提，不能代表最终正式Demo。`demo_stage8_interaction.py`保留唯一交互入口，`run_stage8_agent.py`保留无设备文字诊断，`smoke_stage8_qwen.py`保留有限API兼容性检查。显式Vosk/严格半双工选项仍有效，不因“旧”字删除。
- 收口完整active回归**260 passed in13.98 s**，归档断言**11 passed in2.14 s**，pip check通过，4个入口`--help`通过。此前270→260是退休hint用例退出，语义视觉、安全、取消竞争回归保留；[收口验证](results/stage8_closeout_verification.json)记录命令/stdout/stderr。
- 通过[保留清单](results/closeout_preservation_manifest.json)核对Stage3–7工程与用户成果、Stage8config和既有results。未改门限/依赖，不新增模型或硬件请求，不操作GUI、不commit/push。完整性检查见[收口检查](results/stage8_closeout_integrity.json)。

历史内容位置：[`stage8_development_history.md`](reference/stage8_development_history.md)为完整报告快照；[`legacy_interaction.py`](reference/legacy_interaction.py)与[`legacy_cases.py`](reference/legacy_cases.py)是退休helper/断言；[`smoke_stage8_interaction.py`](reference/smoke_stage8_interaction.py)是早期设备诊断。reference不被当前入口导入，默认pytest只收集`tests/`。
