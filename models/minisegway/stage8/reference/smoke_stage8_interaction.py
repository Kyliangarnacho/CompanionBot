"""Historical Stage 8.1 diagnostic; not the current Demo or its acceptance runner.

Results remain under Stage 8. No GUI, actuator, extra capture owner or ASR cloud.
"""
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

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("PYDANTIC_AI_NO_BANNER", "1")

from embodied_agent.audio import SpeechOutput, microphone_events
from embodied_agent.behavior import BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend
from embodied_agent.config import AgentConfig, STAGE_DIR
from embodied_agent.frames import FrameROI, Stage7FrameBuffer, encode_keyframe
from embodied_agent.interaction import AgentSession
from embodied_agent.runtime import AgentRuntime, fake_model, qwen_runtime
from scripts import demo_yolo26n_depth as stage7


async def main(args):
    directory = STAGE_DIR / "results" / ("devices_" + uuid4().hex)
    directory.mkdir(parents=True)
    config = AgentConfig.load()
    summary = {"cloud_requested": args.qwen, "camera": {}, "checks": {}, "audio_events": [], "tts": [],
               "network_attempts": 0, "provider_input_tokens": 0, "provider_output_tokens": 0}
    frames = Stage7FrameBuffer()
    reads = 0
    class Provider:
        def latest(self):
            nonlocal reads
            reads += 1
            return frames.latest()
    supervisor = BehaviorSupervisor(FakeRobotBackend(), FakeNavigationBackend({"service_desk", "robot_exhibit"}))
    runtime = AgentRuntime(fake_model(), supervisor, config, telemetry=directory / "fake.jsonl")
    session = AgentSession(runtime, frames=Provider())
    stop = threading.Event()
    camera = audio = speech = cloud = None
    passed = False
    try:
        source = (["--video", str(args.video), "--realtime-playback"] if args.video
                  else ["--camera-device", str(args.camera_device), "--opencv-backend", str(stage7.cv2.CAP_MSMF)])
        summary["source_kind"] = "video_fixture" if args.video else "live_camera"
        camera_argv = ["--mode", "full", *source,
                       "--max-source-frames", str(args.frames), "--output-dir", str(directory)]
        # Same fixed models/profile in both runs. The live scene is uncontrolled;
        # rates are observations, not a statistical no-regression claim.
        await asyncio.to_thread(stage7.main, camera_argv + ["--run-name", "baseline"], write_report=False)
        camera = asyncio.create_task(asyncio.to_thread(stage7.main,
                    camera_argv + ["--run-name", "tapped"], frame_observer=frames.publish,
                    shutdown_event=stop, write_report=False))
        deadline = time.perf_counter() + 90
        while frames.latest() is None:
            if camera.done():
                camera.result()
                raise RuntimeError("Camera ended before publishing")
            if time.perf_counter() > deadline:
                raise TimeoutError("Camera startup deadline")
            await asyncio.sleep(.05)
        await session.receive("眼前有什么", vision=True)
        summary["checks"]["sleep_no_model_no_selection"] = not runtime.outcomes and reads == 0
        snapshot = frames.latest()
        frame = snapshot.frame
        roi = FrameROI(frame.source_id, frame.sequence_id, frame.host_receive_time_s,
                       (0, 0, frame.width // 2, frame.height // 2))
        image, meta = encode_keyframe(snapshot, now_s=time.perf_counter(), max_age_s=1, roi=roi)
        summary["camera"]["keyframe"] = {**meta, "jpeg_bytes": len(image.data)}
        try:
            encode_keyframe(snapshot, now_s=frame.host_receive_time_s + 1.1, max_age_s=1)
        except ValueError as error:
            summary["checks"]["stale_frame_rejected"] = str(error) == "frame_stale_or_future"
        wrong_roi = FrameROI(frame.source_id, frame.sequence_id - 1, frame.host_receive_time_s, roi.xyxy)
        try:
            encode_keyframe(snapshot, now_s=time.perf_counter(), max_age_s=1, roi=wrong_roi)
        except ValueError as error:
            summary["checks"]["old_roi_rejected"] = str(error) == "roi_source_mismatch"
        if args.qwen:
            cloud = AgentSession(qwen_runtime(BehaviorSupervisor(FakeRobotBackend(),
                                   FakeNavigationBackend(set())), config, telemetry=directory / "qwen.jsonl"), frames=frames)
            await cloud.receive("你好小柒")
            response = await cloud.receive("请用一句话描述镜头中的主要物体；看不清请说明，不推测身份。", vision=True)
            if response:
                raise RuntimeError("Live visual request rejected locally")
            outcome = await cloud.wait()
            summary["qwen"] = {"answer": outcome.text, "metrics": outcome.metrics}
            summary["network_attempts"] = cloud.runtime.network_counter["attempts"]
            summary["provider_input_tokens"] = outcome.metrics["input_tokens"]
            summary["provider_output_tokens"] = outcome.metrics["output_tokens"]
            summary["checks"]["live_qwen_vision_completed"] = outcome.status == "COMPLETED"
        if not args.no_audio:
            ready = asyncio.Event()
            def audio_event(kind, details):
                summary["audio_events"].append({"kind": kind, "details": details})
                if kind == "audio_ready": ready.set()
            async def transcript(text, **kwargs):
                # Observe recognition metadata only; don't let background speech
                # change this scripted smoke's wake/behavior sequence.
                summary["audio_events"].append({"kind": "transcript", "details": {
                    "confidence": kwargs["confidence"], "text_length": len(text)}})
            audio = asyncio.create_task(microphone_events(args.vosk_model, transcript,
                        device=args.audio_device, is_blocked=session.voice_blocked,
                        is_active=lambda: session.state.value == "ACTIVE",
                        wake_phrase=config.wake_phrase, wake_asr_phrase=config.wake_asr_phrase,
                        on_event=audio_event))
            async def wait_ready():
                wait = asyncio.create_task(ready.wait())
                done, _ = await asyncio.wait([wait, audio], timeout=15, return_when=asyncio.FIRST_COMPLETED)
                if audio in done: audio.result()
                if not ready.is_set():
                    wait.cancel()
                    raise TimeoutError("Microphone readiness deadline")
            await wait_ready()
            speech = SpeechOutput(session.set_playback,
                        on_event=lambda kind, data: summary["tts"].append({"kind": kind, **data}))
            summary["speech_voice"] = await speech.start()
            first = speech.say("小伴语音输出测试。" * 8)
            await asyncio.sleep(.25)
            started = time.perf_counter()
            speech.interrupt()
            cancelled = await first
            summary["checks"]["tts_cancelled"] = cancelled["status"] == "CANCEL"
            summary["tts_cancel_wall_s"] = time.perf_counter() - started
            completed = await speech.say("小伴语音测试完成。")
            summary["checks"]["tts_completed"] = completed["status"] == "COMPLETED"
            await asyncio.sleep(2)
            audio.cancel()
            await asyncio.gather(audio, return_exceptions=True)
            audio = None
            audio_stats = next(e["details"] for e in reversed(summary["audio_events"]) if e["kind"] == "audio_closed")
            summary["checks"]["real_pcm_acquired"] = audio_stats["pcm_samples"] > 0
            summary["checks"]["playback_pcm_blocked"] = audio_stats["blocked_chunks"] > 0
            await speech.close()
            speech = None
        await session.receive("你好小柒")
        for prompt, intent in [("跟随我", "FOLLOW"), ("带我去服务台", "GUIDE_TO")]:
            await session.receive(prompt)
            result = await session.wait()
            summary["checks"][intent + "_accepted"] = any(e["status"] == "ACCEPT" and e["intent"] == intent
                                                      for e in result.metrics["behavior_events"])
            cancelled = json.loads(await session.receive("/cancel"))
            summary["checks"][intent + "_cancelled"] = cancelled["status"] == "CANCEL"
        wait = json.loads(await session.receive("/wait"))
        summary["checks"]["WAIT_accepted"] = wait["status"] == "ACCEPT"
        await session.receive("/sleep")
        before = len(runtime.outcomes)
        await session.receive("你看到了什么", vision=True)
        summary["checks"]["sleep_restored"] = len(runtime.outcomes) == before and reads == 0
        await camera
        for name in ("baseline", "tapped"):
            summary["camera"][name] = json.loads((directory / (name + "_metrics.json")).read_text("utf-8"))
        summary["checks"]["tap_success"] = summary["camera"]["tapped"]["frame_observer"]["failures"] == 0
        passed = all(summary["checks"].values())
    except Exception as error:
        summary["error_type"] = type(error).__name__
    finally:
        stop.set()
        if audio:
            audio.cancel()
            await asyncio.gather(audio, return_exceptions=True)
        if speech: await speech.close()
        if cloud:
            summary["network_attempts"] = cloud.runtime.network_counter["attempts"]
            await cloud.close()
        await session.close()
        if camera: await asyncio.gather(camera, return_exceptions=True)
        frames.clear()
        summary["passed"] = passed
        (directory / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), "utf-8")
        print(json.dumps({"saved": str(directory), "passed": passed, "checks": summary["checks"],
                          "network_attempts": summary["network_attempts"], "error_type": summary.get("error_type")}, ensure_ascii=False))
    return passed


if __name__ == "__main__":
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"): stream.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    sources = parser.add_mutually_exclusive_group(required=True)
    sources.add_argument("--camera-device", type=int)
    sources.add_argument("--video", type=Path, help="Explicit fixture alternative when C920 is unavailable")
    parser.add_argument("--frames", type=int, default=360)
    parser.add_argument("--audio-device", help="Input index/name override; default built-in Realtek")
    parser.add_argument("--vosk-model", type=Path, default=ROOT / ".venv/models/vosk-model-small-cn-0.22")
    parser.add_argument("--no-audio", action="store_true")
    parser.add_argument("--qwen", action="store_true", help="Exactly one on-demand live camera Qwen vision turn")
    sys.exit(0 if asyncio.run(main(parser.parse_args())) else 1)
