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


## Stage 8.2 — Agent Dataflow / Interface Finalization（2026-10-04）

- 开工读取AGENTS并运行sin17-radar：catalog retrieval/entity resolution用PydanticAI typed tools + RapidFuzz小型召回；参考ROS Actions状态/取消语义但不安装ROS。Reject向量数据库、多Agent、万能Envelope。原Stage8 baseline 110项通过。
- 真实缺口：lookup使用字符串包含并把所有目的地交给模型；正式入口behavior_requested/strict gate可能漏自然表达；input_id与turn_id未关联，ACK无强ID检查，事件与task历史无上界。补充数据驱动品牌/分类/属性/区域关系与turn-bound resolution，Supervisor复核合法ID、任务/时钟/序号和取消，不改Stage3–7。
- 第一次集成104 pass / 6 fail。失败包括旧FAILED断言（现在结果未知为UNKNOWN）、隐藏工具假设、旧知识JSON结构、unscored语音关键词拒绝，以及Streaming时机。原始计数与具体原因留在results/stage82_initial_verification.json；语音安全测试改为检查低行动置信度/无前缀不派发，而非继续断言关键词路由。
- Streaming取舍：所有文字回合可语义选择工具后，不能提前知道文本是否是行为前言。保持官方Agent.run/events，含行为权限的响应暂存到工具选择明确；ACK后的最终回答和取图后的视觉回答继续流式播报。纯文字首响应可能延后，不宣称旧首句重叠性能完全保留。故意错误前言测试仍禁止ACK前播报。
- 第一轮真实Qwen失败：nested attributes被编码为JSON string，Pydantic validation以UnexpectedModelBehavior失败。只加有界JSON对象解码后继续严格校验，没有放开多余字段或ID。一次试图给BehaviorRequest加Annotated解析改变框架单模型参数展平schema，模型却继续发平铺intent/destination_id；读安装的PydanticAI upstream后恢复原展平形式，保留v1包裹兼容，并新增针对三个调用形态的schema测试。
- 无匹配时重复检索导致UsageLimitExceeded；后续限制两次检索且只允许一次模式修正，NO_MATCH/REJECT后结束。新鲜类别命中会优先专区，避免模型误标product后按SKU数量追问；实体ID/SKU只精确匹配，禁止近邻自动纠错。
- 完整真实批次stage82_qwen_fe7b658709574d2cb0cd019862fd7ccb曾21/21应用路径正确，40 requests / 83829 input / 2243 output tokens。随后复验保留了失败：Qwen偶尔使用flavor/packaging/brand键、或额外filters，另有模式自我修正。把字段/值别名放入attribute_definitions数据；未知参数仍严格校验，lookup仅允许一次PydanticAI schema修正。已有失败证明有时需要第四次模型请求，因此上限从3→4；正常链路仍2–3，不增加第二Agent或SDK重试。
- 模型在真实NO_MATCH/REJECT/CLARIFY后的解释仍可能报schema错误。应用基于已确认的工具结果生成固定拒绝/澄清，answer_source=local_catalog_fallback；保留模型FAILED、error_type和usage不完整，验收只把预期拒绝的这种路径标为应用正确，不伪装模型成功。
- 异步任务补充RUNNING/progress、CANCEL_REQUESTED、UNKNOWN、2s cooperative deadline、300s task deadline、request_id幂等与v2 metadata；模型结束后通过独立callback/poll处理完成/取消，迟到/乱序/错ID/协议降级不能覆盖新任务。限定内存ledger/事件/历史/GUI与输入队列。Backend重启协调和磁盘日志轮转仍由未来adapter/运维负责，不虚构持久化恢复。
- Frame/Master仅在Stage8 adapter增加source_epoch/接收时间/可选production时钟；不把host_read_complete改成曝光时间。相同帧Master选择/清除合法，所以不盲目拒绝所有相同sequence。ASR新增typed provenance，不变PCM/confidence语义；旧recognition_epoch拒绝前不提交source watermark，防止污染重连状态。
- 正式入口--headless/--no-preview C920真实检验通过：Qwen主动capture_view，源帧46、1280×720、age0.0258s，2requests/4914input/38outputtokens、3.202s。未启动GUI，结果在stage82_native_camera_first.txt和对应interaction日志，未存用户图像。
- 本地音频复验首次错误地假设历史合成fixture为16kHz，断言失败在stage82_native_audio.txt保留。实际原fixture22050Hz；验收侧显式用SciPy resample_poly转16kHz，ASR adapter仍坚持16kHz且没有隐藏重采样。SenseVoice/Silero识别“你前面有什么？”，Realtek读取无错误、原生Huihui SAPI COMPLETED 3.082s/pending0。证据stage82_native_audio.json；没有受控真人语音/扬声器AEC准确率结论。
- 最新全仓回归、真实Qwen交付批次、环境/pip与冻结路径检查统一记录在results/stage82_delivery_verification.json。此前失败批次和旧实验结果保留；不commit/push、不操作桌面GUI。

- 最终交付：全仓294项通过；最新真实批次stage82_qwen_67195502b5d74ed2ac9d4be99c2d9050为21/21应用路径正确，17模型轮中1次过期补充的解释UnexpectedModelBehavior，使用有事实依据的本地拒绝且保留FAILED。41network attempts、已报告108048/2372 tokens；不是21次模型全成功，也不是可靠率保证。补查ROI兼容时保留legacy缺省epoch=None的原同帧校验，只有明确填epoch的adapter才获得重启隔离保证。

## Stage 8.2 语义检索微调（2026-10-05）

- 先读AGENTS与现行三份文档，sin17-radar确认 query expansion / multi-query fusion；Borrow现有PydanticAI typed tool，Adapt有界并集与来源证据，参考RRF但当前无需排序平台/向量库。开工Stage8回归144项通过。扩展在第一次Qwen构造Tool Call时完成，不加改写Agent、独立请求或新依赖。
- 原目录把“快食面”预存为方便面alias，不能据旧验收推断模型语义理解。本轮去掉该分类的口语补丁、品牌错字及“那个…”指示词；保留真实业务名称、英文品牌、分类和属性单位。加同品牌茶饮料与饮料父类验证跨品类，没有新词库、目的地或坐标。
- CatalogQuery增加默认空、最多3项且每项1–160字符的query_variants。Provider共同应用全部结构化过滤，各查询召回再按实体ID合并；query_index→queries保留来源，分数不能直接确认意图。精确SKU/ID忽略扩展。商品/分类/目的地分别最多30项，保持revision/count/truncated；未来Provider不用遵守JSON扫描实现。
- 第一真实批次 `stage82_qwen_24915e5f68784099b4d99cda473f00bf` 两条扩展OFF为NO_MATCH、ON为RESOLVED，首次实时扩展成功；但验收脚本碰到首调用target_kind=sku（非法枚举）后在对照解析中异常中止。模型实际已按框架一次修正为product；修复验收脚本，保留最早非法参数与validation_error，用实际合法调用对照，不篡改首调用记录。
- 第二批次 `stage82_qwen_91192e840b6f4706bab4bc07f56018ef` 暴露词边界去重错误：连写原query与空格分词variant归一化相同，被误去重；另有“袋装方便面”组合字段只得到fuzzy召回。修正查询identity保留分词差别，并对当前候选自己的真实字段做≤160字符的字面组合覆盖，不新增业务映射、不降低可信门槛。
- “苹果汁”扩展为“果汁”不应把具体商品提升成无导航位置的父分类。Resolver区分原查询直接实体命中与扩展分类假设；不同区域/实体冲突澄清，补充时原entity/category/destination集合均受交集约束，旧扩展不会自动带入下一句。
- 后续批次 `stage82_qwen_07c31942442a49f193460af73eb8144f`、`stage82_qwen_a203cfbdfe764b75958696b5336b0873`、`stage82_qwen_7b80daea654e43f3887bc0c8de83fe7f` 保留错误凭据/回合顺序导致的UsageLimitExceeded：包括entity.id、revision和生成UUID冒充resolution_id。Supervisor均拒绝，已派发后模型失败的任务按既有逻辑取消；不放宽ID或Token预算。精简来源元数据，查询只存一份，另加强工具字段职责说明；候选输出省略typed可恢复的可选默认/null字段，保留所有实际证据。失败不能被解释为“零执行”或模型成功。
- 本轮正式入口 headless / no-preview / no-mic / no-tts 复验在camera-device1打开失败（MSMF），没有首帧/模型请求，原始输出在 `semantic_refinement_formal_camera.txt`。未操作GUI或系统设备设置，也未用其他camera冒充C920；原10-04成功设备证据仍保留，当前硬件复验未通过。
- 用户追加收尾规则：清理stale临时产物/缓存，CURRENT_STATE负责阶段状态，README只微调能力和正式命令。已写入仓库AGENTS；最终结论、全仓回归、完整live批次与清理清单集中在 `results/semantic_refinement_delivery.json`，不新建阶段报告体系。
- 候选输出省略可恢复默认/null字段后，最后语义批次 `stage82_qwen_e4cc12e7fcb4460fb0688a13b78445d0` 11/11通过、24请求、74674/1646已报告tokens、wall48.875s；两条语义表达首次扩展OFF为NO_MATCH、ON为RESOLVED。其他可字面检索的用例不伪称扩展增益。
- 独立privacy首次 `stage82_qwen_821551066ce945e59c96dea02d451084` 中Qwen编造enter_behavior工具名，UnexpectedModelBehavior；未触发SLEEP，依赖该前提的零请求用例也记失败。补强“使用API工具名，不按意图创造名称”说明；同时单独验证本地/sleep，使语义休眠失败不会掩盖sleep gate本身的契约。最终privacy批次f12a2612513a4e3885b72fd5ac112b5a为3/3、1模型请求；controls批次082db98c2e6946b0a4e0dd6956674e2c为5/5、7请求。原失败记录保留，不增加通用输出重试、工具别名路由或关键词休眠fallback。
- 交付审查额外发现规模边界：不能把“商品列表截断”自动当成“明确品类专区有歧义”。Provider增加可选region_entity_conflict（null=完整性未知），检查截断前并集；100件同类依旧解析专区，第101件隐藏的跨区扩展命中则CLARIFY。没有把全表扫描写进Protocol要求，外部Provider缺完整性证明时保守处理。最终回归计数与清理清单以delivery JSON为准；只删无引用的成功中间测试日志及可再生缓存，不删原始模型/设备失败。

- 用户重新接上摄像头后，同一正式入口、同一camera-device1无GUI复验成功：1280×720、Qwen自主capture_view、源帧41/age0.00284s，2请求、6450/50tokens、wall3.400s、Streaming正常；证据semantic_refinement_camera_reconnected.txt与interaction_187d648c44444ba3a4d3efe577205f94日志/metrics。先前打开失败记录保留，本次关闭麦克风/TTS，不混作真人语音验证。最终全仓302项通过（13.02s），pip check通过。
