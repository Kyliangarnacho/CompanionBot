"""Stage 8 safety/privacy boundaries tested without network or real robots."""
import asyncio
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from pydantic import ValidationError
from pydantic_ai import CancellationToken
from pydantic_ai.messages import ModelRequest, UserPromptPart
from pydantic_ai.models.function import DeltaToolCall, FunctionModel
from pydantic_ai.profiles.openai import OpenAIModelProfile
import pytest

from embodied_agent.behavior import (
    BehaviorRequest, BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend, Intent,
)
from embodied_agent.config import AgentConfig, load_api_key
from embodied_agent.frames import FrameROI, FrameSnapshot, Stage7FrameBuffer, encode_keyframe
from embodied_agent.interaction import AgentSession, InteractionState
from embodied_agent.runtime import AgentRuntime, fake_model
from perception.camera import ColorFrame


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    monkeypatch.setattr("pydantic_ai.models.ALLOW_MODEL_REQUESTS", False)


def setup(config=None, model=None, clock=None):
    clock = clock or (lambda: 10.0)
    robot = FakeRobotBackend(clock)
    nav = FakeNavigationBackend({"service_desk", "robot_exhibit"}, clock=clock)
    supervisor = BehaviorSupervisor(robot, nav, clock=clock)
    runtime = AgentRuntime(model or fake_model(), supervisor, config or AgentConfig())
    session = AgentSession(runtime, clock=clock)
    return session, robot, nav, supervisor


def frame(seq=42, source="stage7:test", time_s=10.0):
    return ColorFrame(np.zeros((32, 64, 3), np.uint8), seq, source, 64, 32, time_s)


def test_sleep_no_model_no_frame_and_one_request_answer():
    async def run():
        session, robot, _, _ = setup()
        class ForbiddenFrames:
            def latest(self):
                raise AssertionError("Sleep must not read frames")
        session.frames = ForbiddenFrames()
        for text in ["商场很吵", "他刚才说你好小柒", "眼前有什么？"]:
            assert await session.receive(text, vision=True) is None
        assert not session.runtime.outcomes and not robot.requests
        assert session.state == InteractionState.SLEEP
        assert await session.receive("你好小柒") == "已唤醒。"
        assert not session.runtime.outcomes
        await session.receive("你好")
        result = await session.wait()
        assert result.status == "COMPLETED" and result.metrics["requests"] == 1
        assert result.metrics["first_text_s"] is not None
        assert result.metrics["network_attempts"] == 0
        await session.close()
    asyncio.run(run())


def test_function_tool_follow_accept_then_cancel():
    async def run():
        session, robot, _, supervisor = setup()
        await session.receive("你好小柒，跟着我")
        result = await session.wait()
        assert result.status == "COMPLETED", result.metrics
        assert result.metrics["requests"] == 2 and result.metrics["tool_calls"] == 1
        assert robot.requests[0].intent == Intent.FOLLOW
        task = supervisor.active_task
        await session.receive("/cancel")
        assert (await supervisor.status(task)).status == "CANCEL"
        assert len(session.runtime.outcomes) == 1
        await session.close()
    asyncio.run(run())


def test_explicit_follow_can_resume_from_wait_after_latest_state_check():
    async def run():
        _, robot, _, supervisor = setup()
        wait = await supervisor.submit(BehaviorRequest(intent="WAIT"))
        follow = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert follow.status == "ACCEPT"
        assert (await supervisor.status(wait.task_id)).status == "CANCEL"
        robot.safety_ok = False
        await supervisor.submit(BehaviorRequest(intent="WAIT"))
        assert (await supervisor.submit(BehaviorRequest(intent="FOLLOW"))).reason == "robot_not_ready"
    asyncio.run(run())


def test_cancel_reports_revoked_action_during_model_response():
    async def run():
        response_started = asyncio.Event()
        async def stream(messages, _info):
            if any(part.part_kind == "tool-return" for part in messages[-1].parts):
                yield "反馈中"
                response_started.set()
                await asyncio.Event().wait()
            else:
                yield {0: DeltaToolCall(name="request_behavior", tool_call_id="follow",
                                       json_args='{"request":{"intent":"FOLLOW"}}')}
        session, _, _, supervisor = setup(model=FunctionModel(stream_function=stream))
        await session.receive("你好小柒跟随我")
        await response_started.wait()
        response = await session.receive("/cancel")
        assert json.loads(response)["status"] == "CANCEL"
        assert supervisor.active_task is None
        await session.close()
    asyncio.run(run())


def test_no_motion_if_wait_cancellation_is_unconfirmed():
    async def run():
        _, robot, _, supervisor = setup()
        await supervisor.submit(BehaviorRequest(intent="WAIT"))
        async def failed(_task_id):
            raise RuntimeError("cancel transport offline")
        robot.cancel = failed
        result = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert result.reason == "previous_task_cancel_unconfirmed"
        assert [request.intent for request in robot.requests] == [Intent.WAIT]
    asyncio.run(run())


def test_robot_is_rechecked_after_previous_task_status_await():
    async def run():
        _, robot, _, supervisor = setup()
        await supervisor.submit(BehaviorRequest(intent="WAIT"))
        original = robot.status
        async def change_state(task_id):
            robot.master_locked = False
            return await original(task_id)
        robot.status = change_state
        result = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert result.reason == "master_not_locked"
        assert [request.intent for request in robot.requests] == [Intent.WAIT]
    asyncio.run(run())


def test_configurable_wake_phrase_and_invalid_punctuation():
    with pytest.raises(ValidationError):
        AgentConfig(wake_phrase="，！/")
    async def run():
        session, _, _, _ = setup(config=AgentConfig(wake_phrase="小伴醒醒"))
        await session.receive("你好小柒")
        assert session.state == InteractionState.SLEEP
        assert await session.receive("小伴醒醒") == "已唤醒。"
        await session.close()
    asyncio.run(run())


@pytest.mark.parametrize("attribute,value,reason", [
    ("state_age_s", 1, "stale_robot_state"), ("state_age_s", -1, "stale_robot_state"),
    ("connected", False, "robot_not_ready"), ("execution_ready", False, "robot_not_ready"),
    ("safety_ok", False, "robot_not_ready"), ("master_locked", False, "master_not_locked"),
])
def test_behavior_revalidates_latest_robot_state(attribute, value, reason):
    async def run():
        _, robot, _, supervisor = setup()
        setattr(robot, attribute, value)
        result = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert result.status == "REJECT" and result.reason == reason
        assert not robot.requests
    asyncio.run(run())


def test_navigation_busy_unknown_complete_and_cancel():
    async def run():
        _, robot, nav, supervisor = setup()
        bad = await supervisor.submit(BehaviorRequest(intent="GUIDE_TO", destination="unknown"))
        assert bad.status == "REJECT" and bad.reason == "unknown_destination"
        accepted = await supervisor.submit(BehaviorRequest(intent="GUIDE_TO", destination="service_desk"))
        assert accepted.status == "ACCEPT" and not robot.requests
        busy = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert busy.reason == "task_busy_cancel_first"
        nav.complete(accepted.task_id)
        assert (await supervisor.status()).status == "COMPLETED"
        assert supervisor.active_task is None
        again = await supervisor.submit(BehaviorRequest(intent="GUIDE_TO", destination="service_desk"))
        assert (await supervisor.cancel()).status == "CANCEL"
        assert (await nav.status(again.task_id)).status == "CANCEL"
    asyncio.run(run())


@pytest.mark.parametrize("data", [
    {"intent": "VELOCITY", "speed": 0.4}, {"intent": "GUIDE_TO"},
    {"intent": "GUIDE_TO", "destination": " "}, {"intent": "FOLLOW", "destination": "service_desk"},
    {"intent": "FOLLOW", "torque": 0.5},
])
def test_invalid_or_low_level_intents_are_not_an_api(data):
    with pytest.raises(ValidationError):
        BehaviorRequest.model_validate(data)


def test_cancel_during_state_fetch_prevents_action():
    async def run():
        _, robot, _, supervisor = setup()
        called = asyncio.Event()
        resume = asyncio.Event()
        original = robot.latest_state
        async def delayed():
            called.set()
            await resume.wait()
            return await original()
        robot.latest_state = delayed
        token = CancellationToken()
        task = asyncio.create_task(supervisor.submit(BehaviorRequest(intent="FOLLOW"),
                                                     authorized=lambda: not token.cancelled))
        await called.wait()
        token.cancel()
        resume.set()
        assert (await task).reason == "interaction_cancelled"
        assert not robot.requests
    asyncio.run(run())


def test_native_cancellation_during_backend_accept_revokes_action():
    async def run():
        session, robot, _, supervisor = setup()
        dispatched = asyncio.Event()
        acknowledge = asyncio.Event()
        original = robot.request
        async def delayed(task_id, request):
            dispatched.set()
            await acknowledge.wait()
            return await original(task_id, request)
        robot.request = delayed
        await session.receive("你好小柒跟随我")
        await dispatched.wait()
        interrupt = asyncio.create_task(session.receive("/interrupt"))
        await asyncio.sleep(0)
        acknowledge.set()
        await asyncio.wait_for(interrupt, 1)
        assert (await session.wait()).status == "CANCEL"
        assert [event.status for event in supervisor.events] == ["ACCEPT", "CANCEL"]
        assert supervisor.active_task is None
        assert next(iter(robot.tasks.values())).status == "CANCEL"
        await session.close()
    asyncio.run(run())


def test_timeout_during_backend_accept_does_not_orphan_action():
    async def run():
        session, robot, _, supervisor = setup(config=AgentConfig(run_timeout_s=0.05))
        original = robot.request
        async def delayed(task_id, request):
            await asyncio.sleep(0.12)
            return await original(task_id, request)
        robot.request = delayed
        await session.receive("你好小柒跟随我")
        result = await session.wait()
        assert result.status == "FAILED" and result.metrics["error_type"] == "TimeoutError"
        assert supervisor.active_task is None
        assert next(iter(robot.tasks.values())).status == "CANCEL"
        await session.close()
    asyncio.run(run())


def test_backend_ack_failure_is_uncertain_not_claimed_rejected():
    async def run():
        _, robot, _, supervisor = setup()
        async def unknown(_task_id, _request):
            raise RuntimeError("acknowledgement lost")
        robot.request = unknown
        result = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert result.status == "UNKNOWN" and result.reason == "backend_ack_unavailable"
        assert supervisor.active_task == result.task_id
    asyncio.run(run())


def test_stop_still_dispatches_when_navigation_cancel_fails_and_blocks_new_motion():
    async def run():
        _, robot, nav, supervisor = setup()
        accepted = await supervisor.submit(BehaviorRequest(intent="GUIDE_TO", destination="service_desk"))
        async def unavailable(_task_id):
            raise RuntimeError("transport failure")
        nav.cancel = unavailable
        result = await supervisor.submit(BehaviorRequest(intent="STOP_REQUEST"))
        assert result.status == "ACCEPT" and robot.requests[-1].intent == Intent.STOP_REQUEST
        assert supervisor.active_task == accepted.task_id
        result = await supervisor.submit(BehaviorRequest(intent="FOLLOW"))
        assert result.status == "REJECT" and result.reason == "task_busy_cancel_first"
    asyncio.run(run())


def test_stop_bypasses_model_and_unavailable_state():
    async def run():
        session, robot, _, _ = setup()
        async def unavailable():
            raise RuntimeError("state offline")
        robot.latest_state = unavailable
        response = await session.receive("/stop")
        assert json.loads(response)["status"] == "ACCEPT"
        assert robot.requests[-1].intent == Intent.STOP_REQUEST
        assert not session.runtime.outcomes
        await session.close()
    asyncio.run(run())


def test_active_stream_interrupt_and_pause_are_immediate():
    async def run():
        streamed = asyncio.Event()
        async def stream(_messages, _info):
            yield "正在回答"
            streamed.set()
            await asyncio.Event().wait()
        model = FunctionModel(stream_function=stream)
        session, robot, _, _ = setup(model=model)
        await session.receive("你好小柒讲解一下")
        await streamed.wait()
        await asyncio.wait_for(session.receive("/wait"), 1)
        result = await session.wait()
        assert result.status == "CANCEL" and not result.metrics["usage_complete"]
        assert robot.requests[-1].intent == Intent.WAIT
        assert session.state == InteractionState.ACTIVE
        await session.close()
    asyncio.run(run())


def test_model_timeout_and_failure_are_logged_without_raw_errors():
    async def run():
        async def stream(_messages, _info):
            await asyncio.Event().wait()
            yield "unreachable"
        session, _, _, _ = setup(config=AgentConfig(run_timeout_s=0.03),
                                  model=FunctionModel(stream_function=stream))
        await session.receive("你好小柒测试")
        result = await session.wait()
        assert result.status == "FAILED" and result.metrics["error_type"] == "TimeoutError"
        assert result.metrics["wall_s"] < 1
        await session.close()
    asyncio.run(run())


def test_usage_request_limit_keeps_first_behavior_but_cancels_on_failed_run():
    async def run():
        session, robot, _, supervisor = setup(config=AgentConfig(request_limit=1))
        await session.receive("你好小柒跟随我")
        result = await session.wait()
        assert result.status == "FAILED" and result.metrics["error_type"] == "UsageLimitExceeded"
        assert result.metrics["requests"] == 1 and len(robot.requests) == 1
        assert supervisor.active_task is None
        assert any(event.status == "CANCEL" for event in supervisor.events)
        await session.close()
    asyncio.run(run())


@pytest.mark.parametrize("voice_args", [
    {"final": False}, {"confidence": 0.5}, {"confidence": float("nan")},
    {"captured_at_s": 8}, {"captured_at_s": 11},
])
def test_bad_voice_does_not_wake(voice_args):
    async def run():
        session, _, _, _ = setup()
        assert await session.receive("你好小柒", source="voice", **voice_args) is None
        assert session.state == InteractionState.SLEEP
    asyncio.run(run())


def test_voice_control_requires_address_and_playback_guard():
    async def run():
        clock = [10.0]
        session, robot, _, _ = setup(clock=lambda: clock[0])
        session.set_playback(True)
        await session.receive("你好小柒", source="voice")
        assert session.state == InteractionState.SLEEP
        session.set_playback(False)
        await session.receive("你好小柒", source="voice")
        assert session.state == InteractionState.SLEEP
        clock[0] += 1
        assert await session.receive("你好 小柒", source="voice") == "已唤醒。"
        await session.receive("停止机器人", source="voice")
        assert not robot.requests
        await session.receive("你好小柒，停止机器人", source="voice")
        assert robot.requests[-1].intent == Intent.STOP_REQUEST
        await session.close()
    asyncio.run(run())


def test_idle_sleep_clears_history_and_cancels_navigation():
    async def run():
        clock = [10.0]
        session, _, _, supervisor = setup(clock=lambda: clock[0])
        await session.receive("你好小柒带我去服务台")
        await session.wait()
        assert supervisor.active_task is not None and session.runtime.history_turns
        clock[0] += 61
        await session.expire_idle()
        assert session.state == InteractionState.SLEEP
        assert supervisor.active_task is None and not session.runtime.history_turns
        await session.close()
    asyncio.run(run())


def test_frame_buffer_does_not_consume_or_mutate_stage7():
    buffer = Stage7FrameBuffer()
    original = frame()
    buffer.publish(original)
    original.bgr[:] = 255
    assert buffer.latest().frame.bgr.max() == 0
    assert buffer.latest() is buffer.latest()
    with pytest.raises(ValueError):
        buffer.publish(frame())
    with pytest.raises(ValueError):
        buffer.publish(frame(43, "other", 10.1))
    buffer.clear()
    buffer.publish(frame(0, "other", 10.1), valid=False, reason="capture_failure")
    assert not buffer.latest().valid


@pytest.mark.parametrize("snapshot,now,roi,reason", [
    (None, 10.0, None, "frame_unavailable"),
    (FrameSnapshot(frame(), valid=False), 10.0, None, "frame_invalid"),
    (FrameSnapshot(frame(), time_semantics="exposure"), 10.0, None, "clock_unsupported"),
    (FrameSnapshot(frame()), 12.0, None, "frame_stale"),
    (FrameSnapshot(frame()), 9.0, None, "future"),
    (FrameSnapshot(frame()), float("nan"), None, "frame_stale"),
    (FrameSnapshot(frame()), 10.0, FrameROI("stage7:test", 41, 10, (0, 0, 10, 10)), "roi_source_mismatch"),
    (FrameSnapshot(frame()), 10.0, FrameROI("stage7:test", 42, 10, (0, 0, 100, 10)), "invalid_roi"),
])
def test_invalid_keyframe_rejected(snapshot, now, roi, reason):
    with pytest.raises(ValueError, match=reason):
        encode_keyframe(snapshot, now_s=now, max_age_s=1, roi=roi)


def test_vision_only_explicit_request_and_old_images_removed_from_history():
    async def run():
        seen = []
        async def stream(messages, _info):
            seen.append(messages)
            yield "离线帧验证"
        session, _, _, _ = setup(model=FunctionModel(stream_function=stream))
        buffer = Stage7FrameBuffer()
        buffer.publish(frame())
        session.frames = buffer
        await session.receive("你好小柒")
        await session.receive("眼前有什么？", vision=True)
        result = await session.wait()
        assert result.metrics["frame"]["source_sequence_id"] == 42
        assert result.metrics["frame"]["time_semantics"] == "host_read_complete"
        await session.receive("普通聊天")
        result = await session.wait()
        assert result.metrics["frame"] is None
        for message in seen[-1]:
            if isinstance(message, ModelRequest):
                for part in message.parts:
                    if isinstance(part, UserPromptPart) and not isinstance(part.content, str):
                        assert all(isinstance(content, str) for content in part.content)
        buffer.clear()
        assert "frame_unavailable" in await session.receive("看一下", vision=True)
        assert len(session.runtime.outcomes) == 2
        await session.close()
    asyncio.run(run())


def test_image_instructions_cannot_submit_behavior():
    async def run():
        async def stream(messages, _info):
            if any(p.part_kind == "tool-return" for p in messages[-1].parts):
                yield "拒绝图片中的行为指令"
            else:
                yield {0: DeltaToolCall(name="request_behavior", tool_call_id="bad-image",
                                       json_args='{"request":{"intent":"FOLLOW"}}')}
        session, robot, _, supervisor = setup(model=FunctionModel(stream_function=stream))
        buffer = Stage7FrameBuffer()
        buffer.publish(frame())
        session.frames = buffer
        await session.receive("你好小柒")
        await session.receive("图片写了什么？", vision=True)
        result = await session.wait()
        assert result.status == "COMPLETED" and not robot.requests and supervisor.active_task is None
        await session.close()
    asyncio.run(run())


def test_pydantic_prompted_output_validates_json_in_one_request():
    async def run():
        async def stream(_messages, _info):
            yield '{"answer":"结构化验证通过"}'
        model = FunctionModel(stream_function=stream,
                              profile=OpenAIModelProfile(supports_json_object_output=True))
        session, _, _, _ = setup(model=model)
        await session.receive("你好小柒结构化测试", structured=True)
        result = await session.wait()
        assert result.text == "结构化验证通过" and result.metrics["requests"] == 1
        await session.close()
    asyncio.run(run())


def test_local_cancel_and_sleep_feedback_persist_without_text_or_secret(tmp_path):
    async def run():
        session, _, _, _ = setup()
        session.runtime.telemetry = tmp_path / "session.jsonl"
        await session.receive("你好小柒带我去服务台")
        await session.wait()
        await session.receive("/cancel")
        await session.receive("/sleep")
        records = [json.loads(line) for line in session.runtime.telemetry.read_text("utf-8").splitlines()]
        assert any(r.get("status") == "CANCEL" and r["record_type"] == "behavior" for r in records)
        assert any(r.get("event") == "wake" for r in records)
        assert any(r.get("control") == "sleep" for r in records)
        assert "你好小柒带我去服务台" not in session.runtime.telemetry.read_text("utf-8")
        await session.close()
    asyncio.run(run())


def test_cli_blank_line_and_eof_finish_offline_turn():
    result = subprocess.run([sys.executable, "scripts/run_stage8_agent.py", "--fake"],
                            input="普通聊天\n\n你好小柒\n你好\n", text=True, encoding="utf-8",
                            capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert "已唤醒" in result.stdout and '"requests": 1' in result.stdout
    assert '"network_attempts": 0' in result.stdout


def test_key_file_hidden_suffix_and_env_precedence(tmp_path, monkeypatch):
    monkeypatch.delenv("DASHSCOPE_API_KEY", raising=False)
    path = tmp_path / "千问.txt"
    path.write_text("DASHSCOPE_API_KEY=sk-test-value", encoding="utf-8")
    assert load_api_key(tmp_path / "千问") == "sk-test-value"
    assert load_api_key(tmp_path / "missing") == "sk-test-value"
    monkeypatch.delenv("DASHSCOPE_API_KEY")
    path.write_text("sk-one sk-two", encoding="utf-8")
    with pytest.raises(ValueError, match="exactly one"):
        load_api_key(path)


def test_versioned_dot_separated_key_is_not_truncated(tmp_path, monkeypatch):
    monkeypatch.delenv("DASHSCOPE_API_KEY", raising=False)
    path = tmp_path / "key.txt"
    value = "sk-ma-1.region.test-workspace.random.signature_test"
    path.write_text(value, encoding="utf-8")
    assert load_api_key(path) == value


@pytest.mark.parametrize("url", ["http://dashscope.aliyuncs.com/compatible-mode/v1",
                                 "https://attacker.example/compatible-mode/v1",
                                 "https://secret@dashscope.aliyuncs.com/compatible-mode/v1"])
def test_key_not_sent_to_arbitrary_endpoints(url):
    with pytest.raises(ValidationError):
        AgentConfig(base_url=url)
