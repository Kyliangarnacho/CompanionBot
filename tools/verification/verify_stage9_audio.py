"""Native Linux TTS process/FIFO smoke; acoustic onset/intelligibility unverified."""
from companionbot_contracts.paths import repository_root

import asyncio
import json
from pathlib import Path
import sys

ROOT = repository_root()



async def main():
    from companionbot_behavior.core.audio import SpeechOutput
    states, events = [], []
    speech = SpeechOutput(states.append, on_event=lambda kind, data: events.append((kind, data)))
    try:
        backend = await asyncio.wait_for(speech.start(), 5)
        completed = await asyncio.wait_for(speech.say("这是 CompanionBot 的 Linux 语音检查。"), 15)
        assert completed["status"] == "COMPLETED"
        interrupted = speech.say("这是一段用于检查取消的较长语音。" * 8)
        await asyncio.sleep(.1)
        speech.interrupt()
        cancelled = await asyncio.wait_for(interrupted, 5)
        assert cancelled["status"] == "CANCEL"
    finally:
        await speech.close()
    assert not speech.pending and states[-1] is False and not speech.thread.is_alive()
    report = {"backend": backend, "completed": completed, "cancelled": cancelled,
              "pending": len(speech.pending), "worker_stopped": True,
              "events": events,
              "acoustic_output_verified": False, "passed": True}
    output = ROOT / "models/minisegway/stage9/results/architecture/audio/tts.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    asyncio.run(main())
