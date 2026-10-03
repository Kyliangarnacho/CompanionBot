"""Real-interaction adapter boundaries; no cloud, microphone or speaker required."""
import asyncio
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from embodied_agent.audio import SpeechOutput, microphone_events, wake_grammar, confirmed_wake
from embodied_agent.behavior import BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend
from embodied_agent.config import AgentConfig
from embodied_agent.frames import Stage7FrameBuffer, FrameSnapshot, encode_keyframe
from embodied_agent.interaction import AgentSession
from embodied_agent.runtime import AgentRuntime, fake_model
from perception.camera import ColorFrame
from perception.detection_stream import LatestFrameSlot
from scripts.demo_yolo26n_depth import _FrameFanout


def color(seq=5):
    return ColorFrame(np.zeros((24, 32, 3), np.uint8), seq, "camera:test", 32, 24, 10 + seq * .01)


def test_frame_tap_never_consumes_inference_and_isolates_failure():
    slots = (LatestFrameSlot(), LatestFrameSlot())
    frames = Stage7FrameBuffer()
    tap = _FrameFanout(slots, frame_observer=frames.publish)
    raw = color()
    tap.put(raw)
    snapshot = frames.latest()
    assert frames.latest() is snapshot
    assert all(slot.get(timeout_s=0) is raw for slot in slots)
    raw.bgr[:] = 255
    assert not snapshot.frame.bgr.any() and not snapshot.frame.bgr.flags.writeable
    tap.frame_observer = lambda _: (_ for _ in ()).throw(ValueError("tap unavailable"))
    raw = color(6)
    tap.put(raw)
    assert all(slot.get(timeout_s=0) is raw for slot in slots)
    assert tap.observer_calls == 2 and tap.observer_failures == 1
    tap.close()


def test_roi_uses_one_selected_source_frame_and_sleep_never_reads_it():
    async def run():
        clock = lambda: 10.06
        frames = Stage7FrameBuffer()
        frames.publish(color())
        reads = 0
        class AdvancingProvider:
            def latest(self):
                nonlocal reads
                reads += 1
                snapshot = frames.latest()
                frames.publish(color(6))
                return snapshot
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(clock),
                               FakeNavigationBackend(set()), clock=clock), AgentConfig())
        session = AgentSession(runtime, frames=AdvancingProvider(), clock=clock)
        await session.receive("眼前有什么", vision=True, roi_xyxy=(2, 3, 10, 15))
        assert reads == 0 and not runtime.outcomes
        await session.receive("你好小柒")
        await session.receive("眼前有什么", vision=True, roi_xyxy=(2, 3, 10, 15))
        outcome = await session.wait()
        assert outcome.metrics["frame"]["source_sequence_id"] == 5
        assert outcome.metrics["frame"]["roi_xyxy"] == [2, 3, 10, 15] and reads == 1
        await session.close()
    asyncio.run(run())


class FakeSpeechEngine:
    def __init__(self):
        self.owner = threading.get_ident()
        self.until = 0
        self.spoken = []
        self.voice_info = {"voice": "test Chinese", "backend": "fake"}
    def check(self):
        assert threading.get_ident() == self.owner
    def close(self): self.check()
    def pump(self): self.check()
    def say(self, text):
        self.check()
        self.spoken.append(text)
        self.until = time.perf_counter() + (2 if text == "long" else .03)
    def is_busy(self):
        self.check()
        return time.perf_counter() < self.until
    def stop(self):
        self.check()
        self.until = 0


def test_speech_interrupt_keeps_async_controls_responsive_and_replaces_backlog():
    async def run():
        gates, engines = [], []
        def factory():
            engine = FakeSpeechEngine()
            engines.append(engine)
            return engine
        speech = SpeechOutput(gates.append, engine_factory=factory)
        await speech.start()
        first = speech.say("long")
        await asyncio.sleep(.03)
        # Model/text cancellation is free to run while COM is busy.
        started = time.perf_counter()
        speech.interrupt()
        assert (await asyncio.wait_for(first, .4))["status"] == "CANCEL"
        assert time.perf_counter() - started < .4 and gates[-1] is False
        old = speech.say("long")
        new = speech.say("new")
        assert (await old)["status"] == "CANCEL"
        assert gates[-1] is True  # Old completion cannot open the new playback gate.
        assert (await new)["status"] == "COMPLETED"
        assert gates[-1] is False
        await speech.close()
        assert not speech.thread.is_alive()
    asyncio.run(run())


@pytest.mark.parametrize("free_conf", [.99, .6])
@pytest.mark.parametrize("samplerate", [16000, 48000])
def test_pcm_playback_is_discarded_and_wake_requires_local_corroboration(monkeypatch, tmp_path, free_conf, samplerate):
    async def run():
        blocked, received, recognizers, events = [False], [], [], []
        loop = asyncio.get_running_loop()
        class Recognizer:
            def __init__(self, _model, _rate, grammar=None):
                self.grammar, self.inputs, self.resets, self.rate = grammar, [], 0, _rate
                recognizers.append(self)
            def SetWords(self, _): pass
            def Reset(self): self.resets += 1
            def AcceptWaveform(self, data):
                self.inputs.append(data)
                return True
            def Result(self):
                return json.dumps({"text": "你 好 小 七", "result": [{"conf": .99 if self.grammar else free_conf}]})
        class Stream:
            def __init__(self, **kwargs):
                self.callback = kwargs["callback"]
                assert kwargs["samplerate"] == samplerate and kwargs["blocksize"] == samplerate // 10
                assert kwargs["channels"] == 1 and kwargs["dtype"] == "int16"
            def __enter__(self):
                def emit(data): self.callback(data, 1, None, False)
                # Simulated playback blocks raw PCM. Subsequent valid wake is local.
                blocked[0] = True
                loop.call_later(.01, emit, b"echo")
                loop.call_later(.03, lambda: blocked.__setitem__(0, False))
                loop.call_later(.04, emit, b"wake")
                return self
            def __exit__(self, *_): pass
        monkeypatch.setitem(sys.modules, "sounddevice", SimpleNamespace(
            RawInputStream=Stream, check_input_settings=lambda **_: None))
        sys.modules["sounddevice"].query_devices = lambda: [{"index": 0, "name": "Microphone Array Realtek", "hostapi": 0, "max_input_channels": 1}]
        sys.modules["sounddevice"].query_hostapis = lambda: [{"name": "MME"}]
        monkeypatch.setitem(sys.modules, "vosk", SimpleNamespace(
            Model=lambda _: object(), KaldiRecognizer=Recognizer, SetLogLevel=lambda _: None))
        ready = asyncio.Event()
        async def callback(text, **kwargs):
            received.append((text, kwargs))
            ready.set()
        task = asyncio.create_task(microphone_events(tmp_path, callback,
                    is_blocked=lambda: blocked[0], is_active=lambda: False,
                    samplerate=samplerate,
                    on_event=lambda kind, data: events.append((kind, data))))
        await asyncio.wait_for(ready.wait(), 1)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        assert recognizers[0].inputs == [b"wake"]  # Local corroboration, never cloud ASR.
        assert recognizers[1].inputs == [b"wake"]
        assert all(rec.rate == samplerate for rec in recognizers)
        assert all(rec.resets for rec in recognizers)
        assert received[0][1]["confidence"] == free_conf
        assert events[-1][1]["blocked_chunks"] == 1
    asyncio.run(run())


def test_wake_grammar_includes_unknown_and_exact_controls():
    grammar, accepted = wake_grammar("你好小柒")
    assert "[unk]" in grammar and "你好小七暂停机器人" in accepted
    assert "你 好 小 七" in grammar
    assert "暂停机器人" not in accepted and "他刚才说你好小柒" not in accepted


@pytest.mark.parametrize("free_text,expected", [
    ("你好 小柒", True), ("你好 小七", True), ("你好 小气", False),
    ("他刚才说你好小柒", False), ("你好 [unk]", False), ("你好 小伴", False)])
def test_closed_grammar_cannot_force_near_word_wake(free_text, expected):
    assert confirmed_wake({"text": free_text}, {"text": "你 好 小 七"}, "你好小柒") == expected


def test_frame_formats_are_explicit_and_foreign_clock_is_rejected():
    snapshot = FrameSnapshot(color())
    image, metadata = encode_keyframe(snapshot, now_s=10.06, max_age_s=1)
    assert metadata["frame_contract_version"] == 1
    assert metadata["pixel_format"] == "bgr8" and metadata["image_encoding"] == "jpeg"
    assert metadata["clock_domain"] == "host_perf_counter" and metadata["roi_space"] == "source_pixels"
    assert metadata["encoded_width"] == 32 and metadata["encoded_height"] == 24
    assert metadata["valid"] is True and metadata["validity_scope"] == "raw_rgb_contract"
    assert cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR).shape == (24, 32, 3)
    with pytest.raises(ValueError, match="frame_invalid_or_clock_unsupported"):
        encode_keyframe(FrameSnapshot(color(), clock_domain="mcu_ticks"), now_s=10.06, max_age_s=1)
    with pytest.raises(ValueError, match="frame_contract_mismatch"):
        encode_keyframe({"frame": color()}, now_s=10.06, max_age_s=1, roi_xyxy=(0, 0, 10, 10))
    with pytest.raises(ValueError, match="frame_invalid_or_clock_unsupported"):
        encode_keyframe(FrameSnapshot(color(), valid="false"), now_s=10.06, max_age_s=1)
    _, cropped = encode_keyframe(snapshot, now_s=10.06, max_age_s=1, roi_xyxy=(2, 3, 10, 15))
    assert (cropped["width"], cropped["height"]) == (32, 24)
    assert (cropped["encoded_width"], cropped["encoded_height"]) == (8, 12)


def test_custom_wake_remains_configurable_without_legacy_name_mapping():
    from embodied_agent.interaction import canonical_voice_wake
    assert AgentConfig().wake_phrase == "你好小柒"
    assert confirmed_wake({"text": "你好助手"}, {"text": "你好助手"}, "你好助手")
    assert canonical_voice_wake("你好助手，介绍一下", "你好助手") == "你好助手，介绍一下"
    assert canonical_voice_wake("你好小半，介绍一下", "你好助手") == "你好小半，介绍一下"


def test_invalid_voice_transport_types_never_wake_or_call_model():
    async def run():
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(),
                               FakeNavigationBackend(set())), AgentConfig())
        session = AgentSession(runtime)
        for kwargs in [{"confidence": float("inf")}, {"confidence": "1"}, {"confidence": True},
                       {"final": "true"}, {"captured_at_s": "10"}, {"captured_at_s": float("inf")}]:
            assert await session.receive("你好小柒", source="voice", **kwargs) is None
            assert session.state.value == "SLEEP"
        with pytest.raises(ValueError, match="Unicode"):
            await session.receive("你好小柒".encode("utf-8"))
        assert not runtime.outcomes
        await session.close()
    asyncio.run(run())


def test_builtin_name_selection_never_falls_back_to_webcam():
    from embodied_agent.audio import select_input_device
    devices = [{"index": 1, "name": "Microphone C920", "hostapi": 0, "max_input_channels": 2},
               {"index": 8, "name": "Microphone Array Realtek", "hostapi": 1, "max_input_channels": 2},
               {"index": 3, "name": "Microphone Array Realtek", "hostapi": 0, "max_input_channels": 2}]
    sd = SimpleNamespace(query_devices=lambda: devices, query_hostapis=lambda: [{"name": "MME"}, {"name": "WASAPI"}],
                         check_input_settings=lambda **_: None)
    assert select_input_device(sd)["index"] == 3
    assert select_input_device(sd, "realtek")["selection"] == "override"
    assert select_input_device(sd, "1")["index"] == 1  # Explicit override only.
    devices[:] = devices[:1]
    with pytest.raises(RuntimeError, match="built-in"):
        select_input_device(sd)


def test_microphone_unsupported_format_is_explicit():
    from embodied_agent.audio import select_input_device
    def incompatible(**_): raise RuntimeError("unsupported rate")
    sd = SimpleNamespace(query_devices=lambda: [{"index": 3, "name": "Microphone Realtek", "hostapi": 0, "max_input_channels": 2}],
                         query_hostapis=lambda: [{"name": "MME"}], check_input_settings=incompatible)
    with pytest.raises(RuntimeError):
        select_input_device(sd)
    with pytest.raises(RuntimeError):
        select_input_device(sd, "AirPods")


def test_sentence_buffer_chinese_boundaries_and_partial_tail():
    from embodied_agent.audio import SentenceBuffer
    buffer = SentenceBuffer()
    assert buffer.push("你好，欢迎参观") == []
    assert buffer.push("。接下来") == ["你好，欢迎参观。"]
    assert buffer.push("我们看展品！") == ["接下来我们看展品！"]
    assert buffer.push("还没结束") == []
    assert buffer.push("", final=True) == ["还没结束"]
    assert buffer.push("长" * 97) == ["长" * 80]


def test_stream_queue_order_interruption_and_no_old_tail():
    from embodied_agent.audio import StreamingSpeech
    async def run():
        engines, gates = [], []
        def factory():
            class LongFake(FakeSpeechEngine):
                def say(self, text):
                    super().say(text)
                    if text.startswith("long"):
                        self.until = time.perf_counter() + 2
            engine = LongFake()
            engines.append(engine)
            return engine
        speech = SpeechOutput(gates.append, engine_factory=factory)
        await speech.start()
        first = StreamingSpeech(speech)
        first.push("long。旧句二。旧句三。")
        await asyncio.sleep(.04)
        speech.interrupt()
        first.push("这个取消后的尾巴不得播放。")
        timing = await first.finish(False)
        assert timing["speech_status"] == "CANCEL"
        fresh = StreamingSpeech(speech)
        fresh.push("new。下一句。")
        assert (await fresh.finish())["speech_status"] == "COMPLETED"
        assert "旧句二。" not in engines[0].spoken and "旧句三。" not in engines[0].spoken
        assert engines[0].spoken[-2:] == ["new。", "下一句。"]
        assert not speech.pending and not speech.stream_open and gates[-1] is False
        await speech.close()
    asyncio.run(run())


def test_live_master_metadata_clock_and_staleness():
    from embodied_agent.perception import MasterStatusBuffer
    from perception.master_selection import MasterTrackingFrame, MasterState
    now = [10.0]
    master = MasterStatusBuffer(clock=lambda: now[0])
    assert not master.status()["available"]
    value = MasterTrackingFrame("c920", 9, 9.8, 1280, 720, MasterState.UNSELECTED, None, None)
    master.publish(value)
    result = master.status()
    assert result["available"] and result["source_sequence_id"] == 9
    assert result["clock_domain"] == "host_perf_counter" and result["hardware_execution_ready"] is False
    now[0] = 12.0
    assert not master.status()["available"]
    master.clear()
    with pytest.raises(ValueError):
        master.publish({"state": "LOCKED"})


def test_ui_feedback_separates_received_accepted_sleep_and_rejection():
    from embodied_agent.ui import DemoBridge, input_feedback
    bridge = DemoBridge()
    bridge.submit(" 你好小柒 ")
    assert bridge.commands.get_nowait() == "你好小柒"
    assert input_feedback("问题", state="SLEEP")["status"] == "IGNORED"
    assert input_feedback("问题", state="ACTIVE", new_turn=True)["status"] == "ACCEPT"
    assert input_feedback("问题", state="ACTIVE", valid_voice=False)["status"] == "REJECT"
    assert input_feedback("问题", state="ACTIVE", response="视觉请求未发送：frame_stale")["status"] == "REJECT"
    assert input_feedback("/wait", state="SLEEP", local=True)["status"] == "ACCEPT"
    from embodied_agent.ui import format_metrics
    line = format_metrics({"phase": "model_complete", "data": {"first_text_s": .8, "first_speech_s": None, "wall_s": 1.2}})
    assert "0.80s" in line and "待确认" in line and "1.20s" in line


def test_official_stream_can_speak_before_generation_finishes():
    from pydantic_ai.models.function import FunctionModel
    from embodied_agent.audio import StreamingSpeech
    async def run():
        holders, engines, events = {}, [], []
        def factory():
            engine = FakeSpeechEngine()
            engines.append(engine)
            return engine
        def event(kind, data):
            if kind == "tts_started": holders["turn"].started(data)
            events.append(kind)
        speech = SpeechOutput(lambda _: None, engine_factory=factory, on_event=event)
        await speech.start()
        async def stream(_messages, info):
            assert "request_behavior" not in {tool.name for tool in info.function_tools}
            yield "第一句已稳定。"
            deadline = asyncio.get_running_loop().time() + 1
            while "tts_started" not in events:
                assert asyncio.get_running_loop().time() < deadline
                await asyncio.sleep(.005)
            assert not session.task.done()
            yield "第二句继续生成。"
        session = AgentSession(AgentRuntime(FunctionModel(stream_function=stream),
            BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig()),
            strict_behavior_intent=True, on_run_start=lambda: holders.update(turn=StreamingSpeech(speech)),
            on_speech=lambda fragment: holders["turn"].push(fragment))
        await session.receive("你好小柒介绍一下")
        outcome = await session.wait()
        timing = await holders["turn"].finish()
        assert outcome.status == "COMPLETED" and outcome.metrics["text_stream_events"] >= 2
        assert timing["first_speech_s"] < outcome.metrics["wall_s"]
        assert engines[0].spoken == ["第一句已稳定。", "第二句继续生成。"]
        await session.close()
        await speech.close()
    asyncio.run(run())


def test_behavior_preamble_is_not_displayed_or_spoken_before_supervisor_ack():
    from pydantic_ai.models.function import FunctionModel, DeltaToolCall
    async def run():
        robot = FakeRobotBackend()
        supervisor = BehaviorSupervisor(robot, FakeNavigationBackend(set()))
        delivered = []
        async def stream(messages, _info):
            if any(p.part_kind == "tool-return" for p in messages[-1].parts):
                yield "模拟 FOLLOW 已接收，没有实体运动。"
            else:
                yield "已经在实体跟随。"  # Deliberate unsafe preamble before ACK.
                yield {0: DeltaToolCall(name="request_behavior", tool_call_id="follow",
                                       json_args='{"request":{"intent":"FOLLOW"}}')}
        def output(text):
            delivered.append((text, [r.status for r in supervisor.events]))
        runtime = AgentRuntime(FunctionModel(stream_function=stream), supervisor, AgentConfig())
        session = AgentSession(runtime, strict_behavior_intent=True, on_text=output, on_speech=output)
        await session.receive("你好小柒跟随我")
        outcome = await session.wait()
        assert outcome.status == "COMPLETED" and outcome.metrics["speech_deferred_for_behavior_ack"]
        assert delivered and all("ACCEPT" in statuses for _, statuses in delivered)
        assert all("已经在实体跟随" not in text for text, _ in delivered)
        await session.close()
    asyncio.run(run())


def test_single_entry_camera_precedes_local_listening_and_input_acceptance(tmp_path):
    from argparse import Namespace
    from scripts.run_stage8_agent import main
    from embodied_agent.ui import DemoBridge
    import threading
    async def run():
        bridge, ready = DemoBridge(), threading.Event()
        order = []
        def camera(stop, _enqueue, _run_id):
            order.append("camera")
            ready.set()
            stop.wait(5)
            return 0
        args = Namespace(config=None, fake=True, tts=False, vosk_model=None,
                         audio_device=None, audio_samplerate=16000)
        bridge.submit("/look-roi bad input")
        bridge.submit("你好小柒")
        bridge.submit("普通对话")
        task = asyncio.create_task(main(args, ui=bridge, camera_runner=camera, camera_ready=ready))
        accepted = rejected = False
        deadline = asyncio.get_running_loop().time() + 5
        while True:
            while not bridge.updates.empty():
                kind, data = bridge.updates.get_nowait()
                accepted |= kind == "input" and data["status"] == "ACCEPT"
                rejected |= kind == "input" and data["status"] == "REJECT" and "ROI" in data["reason"]
                if kind == "metrics":
                    assert order == ["camera"] and accepted and rejected
                    bridge.submit("/quit")
                    await asyncio.wait_for(task, 2)
                    return
            assert asyncio.get_running_loop().time() < deadline
            await asyncio.sleep(.02)
    asyncio.run(run())


def test_live_master_is_follow_prerequisite_even_for_fake_execution():
    from embodied_agent.perception import PerceptionAwareFakeRobot
    from embodied_agent.behavior import BehaviorRequest
    async def run():
        value = {"available": True, "state": "UNSELECTED", "visible": False}
        robot = PerceptionAwareFakeRobot(lambda: value)
        supervisor = BehaviorSupervisor(robot, FakeNavigationBackend(set()))
        assert (await supervisor.submit(BehaviorRequest(intent="FOLLOW"))).status == "REJECT"
        value.update(state="LOCKED", visible=True)
        result = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert result.status == "ACCEPT" and result.backend == "fake"
        await supervisor.cancel()
        value["available"] = False
        assert (await supervisor.submit(BehaviorRequest(intent="FOLLOW"))).status == "REJECT"
    asyncio.run(run())


def test_stream_generation_cancel_also_clears_speech_queue():
    from pydantic_ai.models.function import FunctionModel
    from embodied_agent.audio import StreamingSpeech
    async def run():
        engines, holders = [], {}
        def factory():
            class Slow(FakeSpeechEngine):
                def say(self, text):
                    super().say(text)
                    self.until = time.perf_counter() + 2
            engine = Slow()
            engines.append(engine)
            return engine
        speech = SpeechOutput(lambda _: None, engine_factory=factory)
        await speech.start()
        produced = asyncio.Event()
        async def stream(_messages, _info):
            yield "第一句播放。待播的第二句。待播的第三句。"
            produced.set()
            await asyncio.Event().wait()
        session = AgentSession(AgentRuntime(FunctionModel(stream_function=stream),
            BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig()),
            strict_behavior_intent=True, on_run_start=lambda: holders.update(turn=StreamingSpeech(speech)),
            on_speech=lambda text: holders["turn"].push(text))
        await session.receive("你好小柒长回答")
        await produced.wait()
        await asyncio.sleep(.04)
        speech.interrupt()  # Main entry's local control first purges speech.
        await session.receive("/interrupt")
        assert (await session.wait()).status == "CANCEL"
        timing = await holders["turn"].finish(False)
        assert timing["speech_status"] == "CANCEL" and not speech.pending
        assert len(engines[0].spoken) <= 1
        await speech.close()
        await session.close()
    asyncio.run(run())


def test_stream_tool_failure_purges_native_renderer_backlog():
    from pydantic_ai.models.function import FunctionModel, DeltaToolCall
    from embodied_agent.audio import StreamingSpeech
    async def run():
        holders = {}
        speech = SpeechOutput(lambda _: None, engine_factory=FakeSpeechEngine)
        await speech.start()
        class FailedRobot(FakeRobotBackend):
            async def latest_state(self): raise RuntimeError("offline")
        async def stream(_messages, _info):
            yield "先说明当前问题。后续句子。"
            yield {0: DeltaToolCall(name="robot_status", tool_call_id="bad", json_args="{}")}
        session = AgentSession(AgentRuntime(FunctionModel(stream_function=stream),
            BehaviorSupervisor(FailedRobot(), FakeNavigationBackend(set())), AgentConfig()),
            strict_behavior_intent=True, on_run_start=lambda: holders.update(turn=StreamingSpeech(speech)),
            on_speech=lambda text: holders["turn"].push(text))
        await session.receive("你好小柒查询状态")
        outcome = await session.wait()
        assert outcome.status == "FAILED"
        await holders["turn"].finish(False)
        assert not speech.pending and not speech.stream_open
        await speech.close()
        await session.close()
    asyncio.run(run())


def test_readonly_knowledge_and_master_tools_use_local_providers():
    from pydantic_ai.models.function import FunctionModel, DeltaToolCall
    async def run():
        returns = []
        async def stream(messages, info):
            tool_return = [p for p in messages[-1].parts if p.part_kind == "tool-return"]
            if tool_return:
                returns.extend(p.content for p in tool_return)
                yield "仅演示知识；真实 Master 来自 Stage 7。"
            else:
                assert "request_behavior" not in {t.name for t in info.function_tools}
                yield {0: DeltaToolCall(name="lookup_knowledge", tool_call_id="knowledge", json_args='{"query":"纪念杯"}'),
                       1: DeltaToolCall(name="master_status", tool_call_id="master", json_args="{}")}
        runtime = AgentRuntime(FunctionModel(stream_function=stream),
            BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig(),
            master_status=lambda: {"available": True, "state": "UNSELECTED", "source_id": "actual_stage7"})
        session = AgentSession(runtime, strict_behavior_intent=True)
        await session.receive("你好小柒纪念杯和当前Master状态")
        outcome = await session.wait()
        assert outcome.status == "COMPLETED" and outcome.metrics["tool_calls"] == 2
        assert any(r.get("source_id") == "actual_stage7" for r in returns)
        assert any(r.get("items", [{}])[0].get("id") == "demo_cup" for r in returns if r.get("items"))
        await session.close()
    asyncio.run(run())


def test_idle_sleep_waits_for_speech_end_then_starts_a_fresh_window():
    from embodied_agent.interaction import InteractionState
    async def run():
        now = [10.0]
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())),
                               AgentConfig(idle_timeout_s=1))
        session = AgentSession(runtime, clock=lambda: now[0])
        await session.receive("你好小柒")
        session.set_playback(True)
        now[0] = 20
        await session.expire_idle()
        assert session.state == InteractionState.ACTIVE
        session.set_playback(False)
        now[0] = 20.5
        await session.expire_idle()
        assert session.state == InteractionState.ACTIVE
        now[0] = 21.1
        await session.expire_idle()
        assert session.state == InteractionState.SLEEP
        await session.close()
    asyncio.run(run())


def test_stale_grammar_endpoint_does_not_erase_newer_free_speech(monkeypatch, tmp_path):
    async def run():
        loop = asyncio.get_running_loop()
        class Recognizer:
            def __init__(self, _model, _rate, grammar=None): self.grammar = grammar
            def SetWords(self, _): pass
            def Reset(self): pass
            def AcceptWaveform(self, data):
                return data == (b"oldw" if self.grammar else b"free")
            def Result(self):
                return json.dumps({"text": "[unk]" if self.grammar else "请跟随我", "result": [{"conf": .95}]})
        class Stream:
            def __init__(self, **kwargs): self.callback = kwargs["callback"]
            def __enter__(self):
                for delay, data in [(.01, b"oldw"), (.65, b"free"), (1.25, b"tick")]:
                    loop.call_later(delay, self.callback, data, 2, None, False)
                return self
            def __exit__(self, *_): pass
        monkeypatch.setitem(sys.modules, "sounddevice", SimpleNamespace(
            query_devices=lambda: [{"index": 0, "name": "Microphone Array Realtek", "hostapi": 0, "max_input_channels": 1}],
            query_hostapis=lambda: [{"name": "MME"}],
            RawInputStream=Stream, check_input_settings=lambda **_: None))
        monkeypatch.setitem(sys.modules, "vosk", SimpleNamespace(
            Model=lambda _: object(), KaldiRecognizer=Recognizer, SetLogLevel=lambda _: None))
        received = asyncio.Future()
        async def callback(text, **kwargs): received.set_result(text)
        task = asyncio.create_task(microphone_events(tmp_path, callback, is_active=lambda: True))
        try:
            assert await asyncio.wait_for(received, 2) == "请跟随我"
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
    asyncio.run(run())


@pytest.mark.parametrize("phrase", ["你退下吧", "你滚吧", "闭嘴"])
def test_semantic_sleep_is_one_output_call_and_silent(phrase):
    async def run():
        silenced=[]
        runtime=AgentRuntime(fake_model(),BehaviorSupervisor(FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig())
        session=AgentSession(runtime,on_silence=lambda:silenced.append(True))
        await session.receive("你好小柒")
        await session.receive(phrase)
        outcome=await session.wait()
        assert outcome.status=="COMPLETED",outcome.metrics
        assert outcome.metrics["interaction_action"]=="SLEEP" and outcome.metrics["requests"]==1
        assert session.state.value=="SLEEP" and not runtime.history_turns and silenced
        assert outcome.text==""
        await session.receive("这个是什么品牌")
        assert len(runtime.outcomes)==1
        await session.close()
    asyncio.run(run())


@pytest.mark.parametrize("phrase", ["你前面有什么", "这个是什么品牌", "这个有什么功效"])
def test_semantic_capture_tool_uses_fresh_single_frame_and_two_requests(phrase):
    from pydantic_ai.models.function import FunctionModel,DeltaToolCall
    from pydantic_ai.messages import BinaryContent,UserPromptPart
    async def run():
        reads=[]
        class Provider:
            def latest(self):reads.append(True);return FrameSnapshot(color())
        async def stream(messages,info):
            returns=[p for p in messages[-1].parts if p.part_kind=="tool-return"]
            if not returns:
                assert "capture_view" in {t.name for t in info.function_tools}
                yield {0:DeltaToolCall(name="capture_view",tool_call_id="image",json_args="{}")}
            else:
                assert "enter_sleep" not in {t.name for t in info.output_tools}
                assert any(isinstance(p,UserPromptPart) and isinstance(p.content,list)
                           and any(isinstance(c,BinaryContent) for c in p.content) for p in messages[-1].parts)
                yield "依据当前图像回答，不猜测不可见的功效。"
        runtime=AgentRuntime(FunctionModel(stream_function=stream),BehaviorSupervisor(FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig())
        events=[]
        runtime.on_event=lambda kind,data:events.append((kind,data))
        session=AgentSession(runtime,frames=Provider(),clock=lambda:10.06,strict_behavior_intent=True)
        await session.receive(phrase)
        assert not reads and not runtime.outcomes
        await session.receive("你好小柒")
        await session.receive(phrase)
        outcome=await session.wait()
        assert outcome.status=="COMPLETED",outcome.metrics
        assert len(reads)==1 and outcome.metrics["requests"]==2
        assert outcome.metrics["frame"]["source_sequence_id"]==5
        assert any(k=="tool" and d["name"]=="capture_view" and d["phase"]=="call" for k,d in events)
        assert all(not any(isinstance(c,BinaryContent) for c in p.content)
                   for turn in runtime.history_turns for m in turn for p in m.parts
                   if isinstance(p,UserPromptPart) and isinstance(p.content,list))
        await session.close()
    asyncio.run(run())


def test_semantic_capture_rejects_stale_frame_and_motion_after_image():
    from pydantic_ai.models.function import FunctionModel,DeltaToolCall
    async def run():
        for now in (10.06,20.0):
            robot=FakeRobotBackend()
            async def stream(messages,_):
                returns=[p for p in messages[-1].parts if p.part_kind=="tool-return"]
                if not returns:
                    yield {0:DeltaToolCall(name="capture_view",tool_call_id="image",json_args="{}")}
                elif returns[0].tool_name=="capture_view":
                    yield {0:DeltaToolCall(name="request_behavior",tool_call_id="bad",json_args='{"request":{"intent":"FOLLOW"}}')}
                else:yield "行为被拒绝。"
            runtime=AgentRuntime(FunctionModel(stream_function=stream),BehaviorSupervisor(robot,FakeNavigationBackend(set())),AgentConfig())
            frames=Stage7FrameBuffer();frames.publish(color())
            session=AgentSession(runtime,frames=frames,clock=lambda:now)
            await session.receive("你好小柒请看画面并跟随")
            outcome=await session.wait()
            assert outcome.status=="COMPLETED",outcome.metrics
            assert not robot.requests and outcome.metrics["vision_attempted"]
            assert (outcome.metrics["frame"] is None)==(now==20.0)
            await session.close()
    asyncio.run(run())


def test_new_text_dismissal_preempts_old_generation_then_sleeps():
    from pydantic_ai.models.function import FunctionModel,DeltaToolCall
    async def run():
        producing=asyncio.Event();silent=[]
        async def stream(messages,_):
            prompt=messages[-1].parts[0].content
            if prompt=="请安静休息":
                yield {0:DeltaToolCall(name="enter_sleep",tool_call_id="bye",json_args='{"action":"SLEEP"}')}
            else:
                yield "长回答开头。";producing.set();await asyncio.Event().wait()
        runtime=AgentRuntime(FunctionModel(stream_function=stream),BehaviorSupervisor(FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig())
        session=AgentSession(runtime,preempt_on_input=True,on_silence=lambda:silent.append(True))
        await session.receive("你好小柒长回答")
        await producing.wait()
        await session.receive("请安静休息")
        assert (await session.wait()).metrics["interaction_action"]=="SLEEP"
        assert runtime.outcomes[0].status=="CANCEL" and session.state.value=="SLEEP" and len(silent)>=2
        await session.close()
    asyncio.run(run())


def test_sleep_output_preempts_coemitted_motion_tool_and_cancels_existing_task():
    from pydantic_ai.models.function import FunctionModel,DeltaToolCall
    from embodied_agent.behavior import BehaviorRequest
    async def run():
        robot=FakeRobotBackend();supervisor=BehaviorSupervisor(robot,FakeNavigationBackend(set()))
        existing=await supervisor.submit(BehaviorRequest(intent="WAIT"))
        async def stream(_messages,_):
            yield {0:DeltaToolCall(name="request_behavior",tool_call_id="bad",json_args='{"request":{"intent":"FOLLOW"}}'),
                   1:DeltaToolCall(name="enter_sleep",tool_call_id="sleep",json_args='{"action":"SLEEP"}')}
        session=AgentSession(AgentRuntime(FunctionModel(stream_function=stream),supervisor,AgentConfig()))
        await session.receive("你好小柒退下")
        assert (await session.wait()).metrics["interaction_action"]=="SLEEP"
        assert len(robot.requests)==1 and not supervisor.active_task
        assert robot.tasks[existing.task_id].status=="CANCEL"
        await session.close()
    asyncio.run(run())


def test_native_utterance_score_is_additive_and_voice_rejections_are_specific():
    from embodied_agent.audio import asr_scores
    assert asr_scores({"result":[{"conf":.3,"start":0,"end":.1},{"conf":.95,"start":.1,"end":1}]})==pytest.approx((.3,.885))
    async def run():
        runtime=AgentRuntime(fake_model(),BehaviorSupervisor(FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig())
        session=AgentSession(runtime,clock=lambda:10)
        await session.receive("你好小柒",source="voice",confidence=.7,utterance_confidence=.9)
        assert session.state.value=="ACTIVE"
        await session.receive("你好",source="voice",confidence=.3,utterance_confidence=.88)
        assert (await session.wait()).status=="COMPLETED"
        await session.receive("你好",source="voice",confidence=.1,utterance_confidence=.2)
        assert session.last_gate["reason"]=="confidence_below_threshold"
        await session.receive("你好",source="voice",captured_at_s=7)
        assert session.last_gate["reason"]=="audio_stale_or_future"
        await session.receive("你好",source="voice",recognition_epoch=0)
        assert session.last_gate["reason"]=="recognition_state_changed"
        session.set_playback(True)
        await session.receive("你好",source="voice")
        assert session.last_gate["reason"]=="playback_active"
        await session.receive("键盘你好")
        assert (await session.wait()).status=="COMPLETED"
        session.set_playback(False)
        await session.receive("/sleep")
        await session.receive("你好小柒")
        await session.receive("你好",source="voice",confidence=.5,utterance_confidence=.9)
        assert session.last_gate["reason"]=="echo_guard_active"
        await session.close()
    asyncio.run(run())


def test_wake_prefix_corroboration_keeps_question_tail_without_accepting_near_words():
    assert confirmed_wake({"text":"你好 小七 这个是什么品牌"},{"text":"你 好 小 七"},"你好小柒")
    assert not confirmed_wake({"text":"你好 小气"},{"text":"你 好 小 七"},"你好小柒")
    from embodied_agent.ui import input_feedback
    data=input_feedback("你好",state="ACTIVE",valid_voice=False,source="voice",
                        gate={"reason":"confidence_below_threshold","score":.2,"threshold":.6})
    assert data["source"]=="voice" and "0.20 < 0.60" in data["reason"]


@pytest.mark.parametrize("name", ["七", "琪", "棋", "琦", "气"])
def test_requested_phonetic_wake_tolerance_accepts_measured_low_scores(name):
    from embodied_agent.interaction import canonical_voice_wake
    async def run():
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig())
        session = AgentSession(runtime)
        assert canonical_voice_wake(f"你好 小{name}，这个是什么？", "你好小柒", fuzzy=True) == "你好小柒，这个是什么？"
        await session.receive(f"你好小{name}", source="voice", confidence=.2, utterance_confidence=.468)
        assert session.state.value == "ACTIVE" and not runtime.outcomes
        await session.receive("/sleep")
        await session.receive(f"他刚才说你好小{name}", source="voice", confidence=.9, utterance_confidence=.9)
        assert session.state.value == "SLEEP"
        await session.receive("你好小伴", source="voice", confidence=.9, utterance_confidence=.9)
        assert session.state.value == "SLEEP"
        await session.close()
    asyncio.run(run())


def test_playback_interrupt_exemption_is_only_for_complete_addressed_cancel():
    async def run():
        stopped = []
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig())
        session = AgentSession(runtime, on_silence=lambda: stopped.append(True))
        await session.receive("你好小柒")
        session.set_playback(True)
        await session.receive("你好小琪打断回答", source="voice", confidence=.2,
                              utterance_confidence=.75, playback_control=True)
        assert session.last_decision == "local_control" and stopped and not runtime.outcomes
        await session.receive("你好小柒跟随我", source="voice", confidence=.99,
                              utterance_confidence=.99, playback_control=True)
        assert session.last_gate["reason"] == "invalid_playback_control"
        await session.receive("别的客人在聊天", source="voice", confidence=.99, utterance_confidence=.99)
        assert session.last_gate["reason"] == "playback_active"
        await session.close()
    asyncio.run(run())


def test_unscored_asr_keeps_missing_confidence_and_requires_vad_and_addressed_behavior():
    async def run():
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig())
        session = AgentSession(runtime, strict_behavior_intent=True)
        await session.receive("你好小柒")
        data = dict(source="voice", confidence=None, confidence_kind="unavailable",
                    asr_backend="sensevoice", vad_validated=True)
        await session.receive("这个是什么品牌？", **data)
        assert (await session.wait()).status == "COMPLETED"
        assert session.last_gate["score"] is None and session.last_gate["confidence"] is None
        await session.receive("请跟随我", **data)
        assert session.last_gate["reason"] == "unscored_behavior_requires_wake_prefix"
        await session.receive("你好", **(data | {"vad_validated": False}))
        assert session.last_gate["reason"] == "invalid_voice_format"
        await session.receive("你好", **(data | {"recognition_epoch": -1}))
        assert session.last_gate["reason"] == "recognition_state_changed"
        await session.close()
    asyncio.run(run())


def test_idle_transition_notifies_once_and_explicit_sleep_remains_silent():
    async def run():
        now = [10.0]
        runtime = AgentRuntime(fake_model(), BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend(set())), AgentConfig(idle_timeout_s=1))
        session = AgentSession(runtime, clock=lambda: now[0])
        await session.receive("你好小柒")
        now[0] = 12
        assert await session.expire_idle() is True
        assert await session.expire_idle() is False
        assert session.state.value == "SLEEP" and not runtime.outcomes
        await session.receive("你好小柒")
        await session.receive("/sleep")
        assert await session.expire_idle() is False
        await session.close()
    asyncio.run(run())


def test_echo_veto_matches_command_inside_tts_and_does_not_invent_aec():
    from embodied_agent.audio import SpeechOutput
    from embodied_agent.interaction import voice_interrupt
    speech = SpeechOutput(lambda _: None)
    speech._echo_text = "操作提示：你好小七，打断回答。"
    speech._echo_until_s = time.perf_counter() + 1
    assert speech.is_self_echo("你好小琪，打断回答")
    assert not speech.is_self_echo("你好小柒，停一下")
    assert voice_interrupt("你好小棋，停一下", "你好小柒", fuzzy=True) == "你好小柒，打断回答"
    assert voice_interrupt("有人说你好小柒打断回答", "你好小柒", fuzzy=True) is None
    assert voice_interrupt("你好小柒打断回答然后跟随我", "你好小柒", fuzzy=True) is None


def test_sensevoice_adapter_is_explicit_about_pcm_rate_and_silence_clipping(tmp_path):
    from embodied_agent.speech_recognition import SenseVoiceInput, SpeechSegment, segment_quality
    with pytest.raises(ValueError, match="16000 Hz"):
        SenseVoiceInput(tmp_path, samplerate=48000)
    with pytest.raises(ValueError, match="assets missing"):
        SenseVoiceInput(tmp_path)
    assert not segment_quality(SpeechSegment(np.zeros(16000), 1, 0, 0))
    assert not segment_quality(SpeechSegment(np.ones(16000), 1, 32768, 1))
    assert segment_quality(SpeechSegment(np.ones(16000)*.1, 1, 3276, 0))


def test_playback_pcm_only_dispatches_interrupt_and_never_normal_question(monkeypatch, tmp_path):
    from types import SimpleNamespace
    import sys
    async def run():
        received, events = [], []
        loop = asyncio.get_running_loop()
        class Recognizer:
            def __init__(self, *_): self.tag = 0
            def SetWords(self, _): pass
            def Reset(self): pass
            def AcceptWaveform(self, data): self.tag = int(np.frombuffer(data, np.int16)[0]); return True
            def Result(self):
                text = {1:"普通顾客正在交谈", 2:"你好小棋停一下"}[self.tag]
                return json.dumps({"text":text,"result":[{"conf":.75}]})
        class Stream:
            def __init__(self, **kwargs): self.callback = kwargs["callback"]
            def __enter__(self):
                for delay, tag in [(.01,1),(.03,2)]:
                    loop.call_later(delay, self.callback, np.full(1600,tag,dtype=np.int16).tobytes(),1600,None,False)
                return self
            def __exit__(self,*_): pass
        sd=SimpleNamespace(RawInputStream=Stream,check_input_settings=lambda **_:None,
                           query_devices=lambda:[{"index":0,"name":"Realtek Microphone Array","hostapi":0,"max_input_channels":1}],
                           query_hostapis=lambda:[{"name":"MME"}])
        monkeypatch.setitem(sys.modules,"sounddevice",sd)
        monkeypatch.setitem(sys.modules,"vosk",SimpleNamespace(Model=lambda _:object(),KaldiRecognizer=Recognizer,SetLogLevel=lambda _:None))
        ready=asyncio.Event()
        async def callback(text,**kwargs): received.append((text,kwargs));ready.set()
        task=asyncio.create_task(microphone_events(tmp_path,callback,is_blocked=lambda:True,
                    is_active=lambda:True,fuzzy_wake=True,voice_interrupt_enabled=True,
                    on_event=lambda kind,data:events.append((kind,data))))
        try: await asyncio.wait_for(ready.wait(),1)
        finally: task.cancel();await asyncio.gather(task,return_exceptions=True)
        assert len(received)==1 and received[0][0]=="你好小柒，打断回答"
        assert received[0][1]["playback_control"] is True
        assert events[-1][1]["blocked_chunks"]==2
    asyncio.run(run())


@pytest.mark.parametrize("blocked", [False, True])
def test_question_asr_preserves_unscored_event_and_rejects_old_epoch(monkeypatch, tmp_path, blocked):
    from embodied_agent.speech_recognition import SpeechSegment
    import embodied_agent.speech_recognition as recognition
    async def run():
        loop=asyncio.get_running_loop();received=[];epoch=[0]
        class QuestionASR:
            speaking=False
            def __init__(self,*_,**kwargs): assert kwargs['samplerate']==16000
            def reset(self): pass
            def feed(self,_): return [SpeechSegment(np.full(16000,.1),1,3276,0)]
            def decode(self,_):
                epoch[0]=1  # A sleep/wake boundary occurs during recognition.
                return '你好小琪，停一下。' if blocked else '这个是什么品牌？'
        class Recognizer:
            def __init__(self,*_): pass
            def SetWords(self,_): pass
            def Reset(self): pass
            def AcceptWaveform(self,_): raise AssertionError('Question must use SenseVoice')
        class Stream:
            def __init__(self,**kwargs): self.callback=kwargs['callback']
            def __enter__(self):
                loop.call_later(.01,self.callback,np.full(1600,1000,dtype=np.int16).tobytes(),1600,None,False)
                return self
            def __exit__(self,*_):pass
        monkeypatch.setattr(recognition,'SenseVoiceInput',QuestionASR)
        monkeypatch.setitem(sys.modules,'vosk',SimpleNamespace(Model=lambda _:object(),KaldiRecognizer=Recognizer,SetLogLevel=lambda _:None))
        monkeypatch.setitem(sys.modules,'sounddevice',SimpleNamespace(RawInputStream=Stream,check_input_settings=lambda **_:None,
                    query_devices=lambda:[{'index':0,'name':'Realtek Microphone Array','hostapi':0,'max_input_channels':1}],
                    query_hostapis=lambda:[{'name':'MME'}]))
        ready=asyncio.Event()
        async def callback(text,**kwargs):received.append((text,kwargs));ready.set()
        task=asyncio.create_task(microphone_events(tmp_path,callback,question_model_dir=tmp_path,state_epoch=lambda:epoch[0],
                    fuzzy_wake=True,voice_interrupt_enabled=True,is_blocked=lambda:blocked))
        try:await asyncio.wait_for(ready.wait(),1)
        finally:task.cancel();await asyncio.gather(task,return_exceptions=True)
        text,data=received[0]
        assert text==('你好小柒，打断回答' if blocked else '这个是什么品牌？')
        assert data['confidence'] is None and data['confidence_kind']=='unavailable'
        assert data['playback_control'] is blocked
        assert data['recognition_epoch']==0 and data['vad_validated'] is True
        runtime=AgentRuntime(fake_model(),BehaviorSupervisor(FakeRobotBackend(),FakeNavigationBackend(set())),AgentConfig())
        session=AgentSession(runtime);await session.receive('你好小柒')
        await session.receive(text,**data)
        assert session.last_gate['reason']=='recognition_state_changed' and not runtime.outcomes
        await session.close()
    asyncio.run(run())
