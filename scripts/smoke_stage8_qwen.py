"""Bounded live API verification; fake robot and a synthetic visual fixture only."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
os.environ.setdefault("PYDANTIC_AI_NO_BANNER", "1")

import cv2
import numpy as np

from embodied_agent.behavior import BehaviorSupervisor, FakeNavigationBackend, FakeRobotBackend
from embodied_agent.config import AgentConfig, STAGE_DIR
from embodied_agent.frames import Stage7FrameBuffer
from embodied_agent.interaction import AgentSession
from embodied_agent.runtime import qwen_runtime
from perception.camera import ColorFrame


async def main(args) -> bool:
    config = AgentConfig.load(args.config)
    navigation = FakeNavigationBackend({"service_desk", "robot_exhibit"})
    supervisor = BehaviorSupervisor(FakeRobotBackend(), navigation, max_state_age_s=config.robot_max_age_s)
    run_dir = STAGE_DIR / "results" / ("qwen_" + uuid4().hex)
    runtime = qwen_runtime(supervisor, config, telemetry=run_dir / "requests.jsonl")
    frames = Stage7FrameBuffer()
    session = AgentSession(runtime, frames=frames)
    records = []
    try:
        # Readiness/privacy invariant: even an explicit visual question stays silent asleep.
        await session.receive("眼前有什么？", vision=True)
        assert not runtime.outcomes and runtime.network_counter["attempts"] == 0
        await session.receive(config.wake_phrase)
        cases = ["text", "tools", "structured", "vision"] if args.case == "all" else [args.case]
        for case in cases:
            runtime.clear_history()
            if case == "text":
                prompt = "用不超过十五个汉字打个招呼。"
            elif case == "tools":
                prompt = "请调用 request_behavior 提交 FOLLOW 跟随请求。说明实际后端和接收结果。"
            elif case == "structured":
                prompt = "请返回结构化回答，answer 为：结构化验证通过。"
            else:
                pixels = np.full((240, 320, 3), 255, np.uint8)
                cv2.rectangle(pixels, (30, 65), (130, 165), (0, 0, 255), -1)
                cv2.circle(pixels, (230, 115), 50, (255, 0, 0), -1)
                run_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(run_dir / "synthetic_vision.png"), pixels)
                frames.publish(ColorFrame(pixels, 0, "stage8:synthetic_smoke", 320, 240, time.perf_counter()))
                prompt = "图片有哪两种有色图形？用一句话说颜色和形状。"
            await session.receive(prompt, vision=case == "vision", structured=case == "structured")
            outcome = await session.wait()
            valid = outcome is not None and outcome.status == "COMPLETED"
            if valid and case == "tools":
                valid = any(event["status"] == "ACCEPT" and event["intent"] == "FOLLOW"
                            for event in outcome.metrics["behavior_events"])
            elif valid and case == "structured":
                valid = outcome.text == "结构化验证通过。" or outcome.text == "结构化验证通过"
            elif valid and case == "vision":
                valid = all(word in outcome.text for word in ("红", "蓝")) and any(
                    word in outcome.text for word in ("方", "正方", "矩形")) and "圆" in outcome.text
            record = {"case": case, "validated": valid,
                      "answer": outcome.text if outcome else "local_gate_rejected",
                      "metrics": outcome.metrics if outcome else None}
            records.append(record)
            print(json.dumps(record, ensure_ascii=False), flush=True)
            await supervisor.cancel()
            # One failing request stops this run. No hidden network/model fallback.
            if not valid:
                break
    finally:
        await session.close()
    summary = {"model": config.model, "base_url": config.base_url,
               "network_attempts": runtime.network_counter["attempts"],
               "provider_input_tokens": sum(r["metrics"]["input_tokens"] for r in records if r["metrics"]),
               "provider_output_tokens": sum(r["metrics"]["output_tokens"] for r in records if r["metrics"]),
               "cases": records}
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), "utf-8")
    print(f"Saved: {run_dir}")
    return bool(records) and all(r["validated"] for r in records)


if __name__ == "__main__":
    for output in (sys.stdout, sys.stderr):
        if hasattr(output, "reconfigure"):
            output.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=["text", "tools", "structured", "vision", "all"], default="text")
    parser.add_argument("--config", type=Path)
    try:
        sys.exit(0 if asyncio.run(main(parser.parse_args())) else 1)
    except Exception as error:
        print(f"Smoke setup failed: {type(error).__name__}", file=sys.stderr)
        sys.exit(1)
