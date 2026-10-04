"""Standalone text/optional offline microphone Agent; all robot execution is fake."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
import threading
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("PYDANTIC_AI_NO_BANNER", "1")

from embodied_agent.behavior import BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend
from embodied_agent.config import AgentConfig, STAGE_DIR
from embodied_agent.frames import Stage7FrameBuffer
from embodied_agent.interaction import AgentSession, normalized, CONTROLS
from embodied_agent.runtime import AgentRuntime, fake_model, qwen_runtime
from embodied_agent.ui import input_feedback
from embodied_agent.catalog import JsonCatalogProvider


async def main(args, *, frames=None, camera_runner=None, camera_ready=None,
               master_status=None, ui=None) -> None:
    config = AgentConfig.load(args.config)
    if getattr(args, "idle_timeout", None) is not None:
        config = AgentConfig.model_validate(config.model_dump() | {"idle_timeout_s": args.idle_timeout})
    catalog = JsonCatalogProvider.load(STAGE_DIR / "config/knowledge.json")
    if master_status:
        from embodied_agent.perception import PerceptionAwareFakeRobot
        robot = PerceptionAwareFakeRobot(master_status)
    else:
        robot = FakeRobotBackend()
    navigation = FakeNavigationBackend(catalog.destination_ids())
    supervisor = BehaviorSupervisor(robot, navigation, max_state_age_s=config.robot_max_age_s)
    run_id = "interaction_" + uuid4().hex
    telemetry = STAGE_DIR / "results" / (run_id + ".jsonl")
    runtime = (AgentRuntime(fake_model(), supervisor, config, telemetry=telemetry, master_status=master_status, catalog=catalog) if args.fake
               else qwen_runtime(supervisor, config, telemetry=telemetry, master_status=master_status, catalog=catalog))

    def emit(kind, data):
        if ui:
            ui.emit(kind, data)

    def runtime_event(kind, data):
        if kind == "vision":
            emit("feedback", "Agent capture_view · " + json.dumps(data, ensure_ascii=False))
        elif kind == "tool":
            emit("feedback", "Agent 工具 · " + str(data["name"]) + " · " + data["phase"])
    runtime.on_event = runtime_event

    def feedback(result):
        runtime.record_event("behavior", result.model_dump(mode="json"))
        print("\n[任务事件] " + result.model_dump_json(), flush=True)
        emit("feedback", "Supervisor · " + result.model_dump_json())
    supervisor.on_feedback = feedback

    speech = None
    spoken_turn = None
    drains = set()

    def begin_turn():
        nonlocal spoken_turn
        if speech:
            from embodied_agent.audio import StreamingSpeech
            spoken_turn = StreamingSpeech(speech)

    def text(fragment):
        print(fragment, end="", flush=True)
        emit("stream", fragment)

    def speech_fragment(fragment):
        if spoken_turn:
            try:
                spoken_turn.push(fragment)
            except Exception as error:
                runtime.record_event("tts_failed", {"error_type": type(error).__name__})
                speech.interrupt()

    def speech_metrics():
        return {"first_speech_s": spoken_turn.first_speech_s if spoken_turn else None}

    async def finish_speech(turn, outcome):
        timing = await turn.finish((outcome.status == "COMPLETED" or outcome.metrics.get("answer_source") == "local_catalog_fallback")
                                   and not outcome.metrics.get("interaction_action"))
        timing.update(model_wall_s=outcome.metrics["wall_s"], status=outcome.status,
                      turn_id=outcome.metrics["turn_id"],
                      interaction_action=outcome.metrics.get("interaction_action"),
                      first_token_s=outcome.metrics["first_token_s"],
                      first_text_s=outcome.metrics["first_text_s"],
                      requests=outcome.metrics["requests"],
                      input_tokens=outcome.metrics["input_tokens"], output_tokens=outcome.metrics["output_tokens"])
        runtime.record_event("interaction_complete", timing)
        emit("metrics", {"phase": "speech_complete", "data": timing})

    def speech_event(kind, data):
        runtime.record_event(kind, data)
        if kind == "tts_started" and spoken_turn:
            spoken_turn.started(data)

    def speak(text):
        if speech:
            try:
                speech.say(text)
            except Exception as error:
                runtime.record_event("tts_failed", {"error_type": type(error).__name__})
                print("TTS 失败；回答保留在文字输出中。")

    def complete(outcome):
        failed_without_fallback = outcome.status != "COMPLETED" and outcome.metrics.get("answer_source") != "local_catalog_fallback"
        print("\n" + outcome.text if failed_without_fallback else "")
        print(json.dumps(outcome.metrics, ensure_ascii=False))
        emit("metrics", {"phase": "model_complete", "data": outcome.metrics})
        if failed_without_fallback:
            emit("response", outcome.text)
        if outcome.metrics.get("interaction_action") == "SLEEP":
            emit("response", 'Agent 指令 {"action":"SLEEP"} 已执行；停止播报、清空历史并休眠。')
        if spoken_turn:
            task = asyncio.create_task(finish_speech(spoken_turn, outcome))
            drains.add(task)
            task.add_done_callback(drains.discard)

    buffer = frames if frames is not None else Stage7FrameBuffer()
    session = AgentSession(runtime, frames=buffer,
                           on_text=text, on_complete=complete, on_speech=speech_fragment,
                           on_run_start=begin_turn, speech_metrics=speech_metrics,
                           preempt_on_input=True,
                           on_silence=lambda: speech.interrupt() if speech else None)
    print(f"SLEEP | 唤醒短语：{config.wake_phrase} | robot/navigation=fake")
    print("/interrupt /cancel /stop /wait /sleep /status /complete /look 问题 /quit")
    print("自然视觉问题自动选帧；/look 问题；/look-roi x1 y1 x2 y2 问题（原图坐标）。")
    print("播放中可说“" + config.wake_phrase + "，打断回答”；只接收明确打断口令，没有 AEC。文字/空格始终可用。")
    emit("response", "C920 持续感知；Agent SLEEP。麦克风仅本地唤醒监听，执行后端为 fake。")
    queue = asyncio.Queue(maxsize=64)
    consumed = threading.Event()
    input_stop = threading.Event()
    camera_stop = threading.Event()
    loop = asyncio.get_running_loop()

    def enqueue(item):
        def put():
            text = item[0] if isinstance(item, tuple) else item
            command = normalized(text)
            prefix = normalized(config.wake_phrase)
            if command.startswith(prefix):
                command = command[len(prefix):]
            if command in {normalized(key) for key in CONTROLS} or command == normalized("/quit"):
                while not queue.empty():
                    queue.get_nowait()
            if queue.full():
                # New input supersedes queued old intent; keep a bounded inbox.
                queue.get_nowait()
                runtime.record_event("input_queue", {"reason": "oldest_dropped_capacity", "capacity": 64})
            queue.put_nowait(item)
        try:
            loop.call_soon_threadsafe(put)
        except RuntimeError:
            pass

    def read_text():
        # A blocking console read must not hold asyncio's executor open on preview
        # quit/Ctrl+C. This daemon only reads user stdin; no keyboard automation.
        while not input_stop.is_set():
            line = sys.stdin.readline()
            enqueue(line)
            if not line:
                return
            consumed.wait()
            consumed.clear()

    async def voice_callback(text, **kwargs):
        enqueue((text, kwargs))

    reader = threading.Thread(target=read_text, name="agent-console-input", daemon=True)
    audio = None
    camera = None
    output_device = "文字输出"
    input_device = "本地麦克风初始化中"
    audio_level = ""
    last_status = None
    async def monitor_tasks():
        while True:
            await supervisor.poll()
            await asyncio.sleep(0.5)
    monitor = asyncio.create_task(monitor_tasks())
    def show_status():
        nonlocal last_status
        generating = bool(session.task and not session.task.done())
        status = ("SLEEP" if session.state.value == "SLEEP" else
                  "SPEAKING" if session.playback_active else "THINKING" if generating else "LISTENING")
        master = runtime.master_status()
        master_text = master.get("state", "UNAVAILABLE") if master.get("available") else "UNAVAILABLE"
        current = next((e for e in reversed(supervisor.events) if e.task_id == supervisor.active_task), None) if supervisor.active_task else None
        task_text = (f"{current.intent.value if current.intent else '-'}:{current.status}"
                     if current else "NONE")
        line = (f"{status} | 生成={'是' if generating else '否'} | Master={master_text} "
                f"ID={master.get('track_id')} | task={task_text} | 模拟执行 fake")
        if line != last_status:
            last_status = line
            print("\n[状态] " + line)
            emit("state", line)
        if ui:
            ui.preview_line = f"Agent {status} | Master {master_text} | task {task_text} | execution=fake"
    try:
        if camera_runner:
            print("先打开 C920 与 Stage 7 感知链…")
            emit("state", "CAMERA_STARTING · 正在打开 C920；Agent SLEEP")
            camera = asyncio.create_task(asyncio.to_thread(camera_runner, camera_stop, enqueue, run_id))
            if camera_ready is not None:
                deadline = loop.time() + 45
                while not camera_ready.is_set():
                    if camera.done():
                        camera.result()
                        raise RuntimeError("Camera stopped before a valid frame")
                    if loop.time() >= deadline:
                        raise TimeoutError("Camera first-frame deadline")
                    if ui and not ui.commands.empty():
                        command = ui.commands.get_nowait()
                        if command == "/quit":
                            return
                        enqueue(command)
                    await asyncio.sleep(.05)
                emit("response", "C920 首帧就绪，感知链持续运行；正在启用本地唤醒监听。")
        if getattr(args, "tts", False):
            from embodied_agent.audio import SpeechOutput
            speech = SpeechOutput(session.set_playback, on_event=speech_event)
            try:
                details = await speech.start()
                runtime.record_event("tts_ready", details)
                print("本地 TTS：" + details["voice"])
                output_device = details["output"]
                emit("device", "TTS：" + details["voice"] + "；输出：" + details["output"])
            except Exception as error:
                runtime.record_event("tts_failed", {"error_type": type(error).__name__})
                await speech.close()
                speech = None
                print("本地 TTS 不可用；文字交互继续。")
                emit("error", "TTS 不可用；文字交互继续。")
        if args.vosk_model:
            from embodied_agent.audio import microphone_events
            from embodied_agent.devices import microphone_diagnostics
            emit("device", "相机已就绪，正在检查内置麦克风权限、静音和采样…")
            diagnostic = await asyncio.to_thread(microphone_diagnostics, args.audio_device,
                                                samplerate=args.audio_samplerate, seconds=1)
            diagnostic_path = STAGE_DIR / "results" / (run_id + "_microphone.json")
            diagnostic_path.parent.mkdir(parents=True, exist_ok=True)
            diagnostic_path.write_text(json.dumps(diagnostic, ensure_ascii=False, indent=2), encoding="utf-8")
            if diagnostic.get("error_type"):
                emit("error", "内置麦克风诊断失败；将报告音频错误并保留文字输入，不切换 C920。")
            def audio_event(kind, details):
                nonlocal input_device, audio_level
                if kind not in ("audio_level", "user_speech"):
                    runtime.record_event(kind, {k: v for k, v in details.items() if k not in ("free_text", "grammar_text")})
                if kind == "user_speech":
                    if session.state.value == "ACTIVE" and not session.voice_blocked():
                        session.last_activity_s = time.perf_counter()
                if kind == "asr_decoding":
                    mode = "播放中的打断口令" if details.get("interrupt_only") else "整句"
                    emit("asr", f"本地 SenseVoice 正在识别{mode}，音频 {details['duration_s']:.1f}s…")
                if kind == "question_asr_ready":
                    emit("asr", f"普通问句：SenseVoice + Silero VAD，本地识别；停顿 {details['silence_s']:.1f}s 后整句提交")
                if kind == "asr_result":
                    meanings = {"question_candidate": "普通问句候选", "wake_prefix": "识别到唯一唤醒前缀",
                                "wake_prefix_absent": "没有匹配唤醒前缀", "wake_corroborated": "两路确认唤醒前缀",
                                "wake_decoder_mismatch": "两路唤醒识别不一致",
                                "grammar_only_uncorroborated": "唤醒解码有候选，通用解码未确认",
                                "sensevoice_question": "SenseVoice 整句结果（不提供 confidence）",
                                "sensevoice_interrupt": "播放中仅匹配明确打断口令，其他内容忽略"}
                    def score(value):
                        return f"{value:.2f}" if isinstance(value, (float, int)) else "不提供"
                    emit("asr", f"ASR {'问句' if details['active'] else '睡眠监听'}：通用『{details['free_text'] or '-'}』/ 唤醒『{details['grammar_text'] or '-'}』 · {meanings[details['decision']]} · "
                                f"最低词 {score(details['confidence'])} / 句级 {score(details['utterance_confidence'])}")
                if kind == "voice_interrupt_rejected":
                    emit("asr", "打断候选与机器人自己的播报相符，按回声保护忽略。")
                if kind == "audio_ready":
                    print(f"麦克风就绪：{details['samplerate']} Hz；唤醒 Vosk，问句 {details['question_backend']}；等待“" + config.wake_phrase + "”。")
                    input_device = f"{details['name']} ({details['hostapi']}) / {details['samplerate']} Hz / 问句 {details['question_backend']}"
                if kind == "audio_level":
                    audio_level = f"最近一秒 RMS {details['pcm_rms']:.1f} / peak {details['pcm_peak']} · {details['mode']}"
                if kind in ("audio_ready", "audio_level"):
                    emit("device" if kind == "audio_ready" else "level",
                         f"输入：{input_device}；输出：{output_device}；{audio_level}")
                if kind == "audio_near_silent":
                    print("麦克风当前输入电平很低；请说话检查是否变化，若持续不变请检查静音与 Windows 权限。")
                    emit("response", "内置麦克风当前电平很低。请说话观察 RMS；持续不变时检查硬件静音/权限。未切换 C920 麦克风。")
            audio = asyncio.create_task(microphone_events(args.vosk_model, voice_callback,
                                        device=args.audio_device, is_blocked=session.voice_blocked,
                                        is_active=lambda: session.state.value == "ACTIVE",
                                        wake_phrase=config.wake_phrase,
                                        wake_asr_phrase=config.wake_asr_phrase,
                                        samplerate=getattr(args, "audio_samplerate", 16000),
                                        on_event=audio_event, state_epoch=lambda: session.asr_epoch,
                                        fuzzy_wake=config.wake_fuzzy,
                                        question_model_dir=(args.asr_model_dir if getattr(args, "asr_backend", "vosk") == "sensevoice" else None),
                                        silence_s=config.asr_silence_s,
                                        voice_interrupt_enabled=getattr(args, "voice_interrupt", True),
                                        is_self_echo=lambda text: speech.is_self_echo(text) if speech else False))
        if not ui:
            reader.start()
        while True:
            if ui:
                while not ui.commands.empty():
                    enqueue(ui.commands.get_nowait())
            show_status()
            if audio and audio.done():
                try:
                    audio.result()
                except Exception as error:
                    runtime.record_event("audio_failed", {"error_type": type(error).__name__})
                    print("麦克风失败：" + type(error).__name__ + "；文字输入仍可用。")
                    emit("error", "麦克风失败：" + type(error).__name__ + "；保留文字入口，未切换 C920 麦克风。")
                audio = None
            if camera and camera.done():
                camera.result()
                print("Stage 7 相机已结束。")
                break
            try:
                item = await asyncio.wait_for(queue.get(), timeout=0.2)
            except TimeoutError:
                if await session.expire_idle():
                    emit("response", f"空闲 {config.idle_timeout_s:g} 秒，已休眠。" + config.idle_sleep_message)
                    runtime.record_event("idle_sleep_notice", {"tts_requested": bool(speech and config.idle_sleep_message)})
                    if speech and config.idle_sleep_message:
                        speak(config.idle_sleep_message)
                continue
            voice = isinstance(item, tuple)
            line, kwargs = item if voice else (item.strip(), {})
            if voice:
                score = kwargs.get('confidence')
                label = f"{score:.2f}" if isinstance(score, (float, int)) else "confidence 未提供"
                print(f"本地识别 [{label}]：{line}")
            if not voice and item == "":
                await session.wait()
                break
            if not voice and not line:
                consumed.set()
                continue
            if line == "/quit":
                break
            command = normalized(line)
            if command.startswith(normalized(config.wake_phrase)):
                command = command[len(normalized(config.wake_phrase)):]
            if not voice and speech and command in {normalized(key) for key in CONTROLS}:
                speech.interrupt()
            if line == "/status":
                response = json.dumps({"interaction": session.state.value, "master": runtime.master_status(),
                                       "hardware_execution_ready": False,
                                       "task": (await supervisor.status()).model_dump(mode="json")}, ensure_ascii=False)
                print(response)
                emit("response", response)
                emit("input", input_feedback(line, state=session.state.value, local=True))
            elif line == "/master":
                emit("response", json.dumps(runtime.master_status(), ensure_ascii=False))
                print(json.dumps(runtime.master_status(), ensure_ascii=False))
                emit("input", input_feedback(line, state=session.state.value, local=True))
            elif line == "/complete":
                if supervisor.active_task in navigation.tasks:
                    task_id = supervisor.active_task
                    navigation.complete(task_id)
                    print((await supervisor.status(task_id)).model_dump(mode="json"))
                else:
                    print("没有可完成的 fake navigation 任务。")
                    emit("response", "没有可完成的 fake navigation 任务。")
                emit("input", input_feedback(line, state=session.state.value, local=True))
            else:
                roi = None
                if line.startswith("/look-roi "):
                    parts = line.split(maxsplit=5)
                    try:
                        roi = tuple(int(p) for p in parts[1:5])
                        if len(roi) != 4 or len(parts) != 6:
                            raise ValueError()
                    except ValueError:
                        decision = {"text": line, "status": "REJECT",
                                    "reason": "ROI 格式错误；用法：/look-roi x1 y1 x2 y2 问题"}
                        print(decision["reason"])
                        emit("input", decision)
                        runtime.record_event("input_decision", {key: value for key, value in decision.items() if key != "text"})
                        consumed.set()
                        continue
                    line, vision = parts[5], True
                else:
                    # Semantic vision selection is a native Agent tool. /look
                    # remains the explicit one-request path, with no classifier.
                    vision = line.startswith("/look ")
                    if line.startswith("/look "):
                        line = line[6:]
                previous = session.task
                if not voice and speech and session.state.value != "SLEEP":
                    speech.interrupt()
                input_id = uuid4().hex
                response = await session.receive(line, vision=vision, roi_xyxy=roi, input_id=input_id, **kwargs)
                valid_voice = session.last_decision != "voice_rejected"
                decision = input_feedback(line, state=session.state.value, valid_voice=valid_voice,
                                          new_turn=session.task is not previous, response=response,
                                          local=session.last_decision == "local_control",
                                          source="voice" if voice else "text", gate=session.last_gate)
                decision["input_id"] = input_id
                decision["turn_id"] = session.last_gate.get("turn_id")
                print("\n[输入] " + decision["status"] + " · " + decision["reason"])
                emit("input", decision)
                if session.task is not previous:
                    emit("stream", "Agent：")
                runtime.record_event("input_decision", {key: value for key, value in decision.items() if key != "text"})
                if response:
                    print(response)
                    emit("response", response)
                    if speech:
                        spoken = response
                        if response.startswith("{"):
                            feedback = json.loads(response)
                            spoken = "行为反馈 " + feedback["status"] + "。当前使用模拟后端。"
                        if session.state.value != "SLEEP" and session.last_decision not in ("busy", "vision_rejected"):
                            speak(spoken)
            if not voice:
                consumed.set()
    finally:
        monitor.cancel()
        await asyncio.gather(monitor, return_exceptions=True)
        input_stop.set()
        consumed.set()
        camera_stop.set()
        if speech:
            speech.interrupt()
        if audio:
            audio.cancel()
            await asyncio.gather(audio, return_exceptions=True)
        await session.close()
        if speech:
            await speech.close()
        if drains:
            await asyncio.gather(*drains, return_exceptions=True)
        if camera:
            await camera
        buffer.clear()
    print(f"阶段日志：{telemetry}")
    emit("response", f"阶段日志：{telemetry}")


def parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fake", action="store_true", help="Offline FunctionModel, no API key needed")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--vosk-model", type=Path, help="Local unpacked Chinese Vosk model directory")
    parser.add_argument("--asr-backend", choices=["vosk", "sensevoice"], default="vosk", help="Question ASR; wake remains local Vosk")
    parser.add_argument("--asr-model-dir", type=Path, default=ROOT / ".venv/models/sensevoice")
    parser.add_argument("--no-voice-interrupt", action="store_false", dest="voice_interrupt", default=True,
                        help="Strict half duplex; disable addressed voice interrupt channel")
    parser.add_argument("--idle-timeout", type=float, help="Seconds of idle before local sleep announcement")
    parser.add_argument("--audio-device", help="Input index or device-name substring; default built-in Realtek")
    parser.add_argument("--audio-samplerate", type=int, choices=[16000, 22050, 44100, 48000], default=16000,
                        help="Native PCM sample rate, also passed to Vosk; no hidden resampling")
    parser.add_argument("--tts", action="store_true", help="Offline Windows Chinese SAPI5 output")
    return parser


def run_cli(args, **kwargs):
    for output in (sys.stdin, sys.stdout, sys.stderr):
        if hasattr(output, "reconfigure"):
            output.reconfigure(encoding="utf-8")
    try:
        asyncio.run(main(args, **kwargs))
    except KeyboardInterrupt:
        pass
    except Exception as error:
        # Do not dump credential-bearing SDK exceptions or locals.
        print(f"启动/音频错误：{type(error).__name__}", file=sys.stderr)
        if kwargs.get("ui"):
            kwargs["ui"].emit("error", f"启动/运行错误：{type(error).__name__}。请核对 C920、内置 Realtek、Vosk 模型及阶段日志。")
        sys.exit(1)


if __name__ == "__main__":
    run_cli(parser().parse_args())
