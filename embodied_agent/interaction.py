"""Local wake/interrupt gate. Sleeping input never reaches the model or frames."""
from __future__ import annotations

import asyncio
from enum import Enum
import json
import math
import time
from typing import Callable
import unicodedata

from pydantic_ai import CancellationToken

from .behavior import BehaviorRequest, Feedback, Intent
from .frames import FrameProvider, FrameROI, encode_keyframe
from .runtime import AgentRuntime, TurnOutcome


class InteractionState(str, Enum):
    SLEEP = "SLEEP"
    ACTIVE = "ACTIVE"


def normalized(text: str) -> str:
    return "".join(c.casefold() for c in unicodedata.normalize("NFKC", text)
                   if not c.isspace() and not unicodedata.category(c).startswith("P"))


CONTROLS = {
    "/interrupt": "interrupt", "打断回答": "interrupt", "停止回答": "interrupt",
    "/cancel": "cancel", "取消任务": "cancel",
    "/stop": "stop", "停止机器人": "stop",
    "/wait": "wait", "暂停机器人": "wait",
    "/sleep": "sleep", "结束对话": "sleep",
}


def canonical_voice_wake(text: str, phrase: str, *, fuzzy=False) -> str:
    """Bounded phonetic tolerance for ONE spoken phrase, only at ASR boundary.

    Keep the 你好小 prefix and accept common qi spellings/tones. Do not use
    unrestricted edit distance, scan a quoted mention, or add other greetings.
    Preserve the question tail, including punctuation from the better ASR.
    """
    norm = normalized(text)
    if normalized(phrase) != "你好小柒" or len(norm) < 4:
        return text
    names = "柒七琪棋琦祺奇齐其启起气" if fuzzy else "柒七"
    if not norm.startswith("你好小") or norm[3] not in names:
        return text
    index = next(i for i in range(1, len(text) + 1) if len(normalized(text[:i])) == 4)
    return phrase + text[index:]


def voice_interrupt(text: str, phrase: str, *, fuzzy=False) -> str | None:
    """Only an addressed, complete cancel-answer command may bypass playback."""
    norm = normalized(canonical_voice_wake(text, phrase, fuzzy=fuzzy))
    prefix = normalized(phrase)
    if norm.startswith(prefix) and norm[len(prefix):] in ("打断回答", "停止回答", "停一下", "别说了"):
        return phrase + "，打断回答"
    return None


def behavior_requested(text: str) -> bool:
    """Local authorization gate for side-effect tools, never another model router."""
    text = normalized(text)
    return any(term in text for term in ("跟随", "跟着", "跟我", "带我", "领我", "引导我", "导航到",
                                         "follow", "guideto", "暂停", "停止", "取消"))


class AgentSession:
    def __init__(self, runtime: AgentRuntime, *, frames: FrameProvider | None = None,
                 clock: Callable[[], float] = time.perf_counter,
                 on_text: Callable[[str], None] | None = None,
                 on_complete: Callable[[TurnOutcome], None] | None = None,
                 on_speech=None, on_run_start=None, speech_metrics=None,
                 strict_behavior_intent=False, preempt_on_input=False, on_silence=None) -> None:
        self.runtime, self.frames, self.clock = runtime, frames, clock
        self.on_text, self.on_complete = on_text, on_complete
        self.on_speech, self.on_run_start, self.speech_metrics = on_speech, on_run_start, speech_metrics
        self.strict_behavior_intent = strict_behavior_intent
        self.preempt_on_input, self.on_silence = preempt_on_input, on_silence
        self.last_decision = "ignored"
        self.last_gate = {}
        self.asr_epoch = 0
        self.state = InteractionState.SLEEP
        self.task: asyncio.Task | None = None
        self.token: CancellationToken | None = None
        self.last_activity_s = clock()
        self.playback_active = False
        self.voice_blocked_until_s = 0.0
        self._dispatch_lock = asyncio.Lock()

    def set_playback(self, active: bool) -> None:
        """Audio renderer calls before/after playback.

        Conservative half-duplex guard, not an acoustic echo cancellation claim.
        Text controls remain available during playback.
        """
        was_playing = self.playback_active
        self.playback_active = active
        if not active:
            self.voice_blocked_until_s = self.clock() + self.runtime.config.echo_guard_s
            if was_playing:
                self.last_activity_s = self.clock()

    def voice_blocked(self) -> bool:
        return self.playback_active or self.clock() < self.voice_blocked_until_s

    async def _interrupt(self) -> TurnOutcome | None:
        if self.token:
            self.token.cancel()
        if self.task and not self.task.done():
            return await self.task
        return None

    async def receive(self, text: str, *, source: str = "text", final: bool = True,
                      confidence: float | None = 1.0, captured_at_s: float | None = None,
                      utterance_confidence: float | None = None, recognition_epoch: int | None = None,
                      asr_backend: str = "vosk", confidence_kind: str = "min_word",
                      vad_validated: bool = False, playback_control: bool = False,
                      vision: bool = False, roi: FrameROI | None = None,
                      roi_xyxy: tuple[int, int, int, int] | None = None,
                      structured: bool = False) -> str | None:
        async with self._dispatch_lock:
            now = self.clock()
            config = self.runtime.config
            self.last_decision = "ignored"
            self.last_gate = {"source": source, "reason": "accepted"}
            if source not in ("text", "voice"):
                raise ValueError("source must be text or voice")
            if source == "voice":
                unscored = (asr_backend == "sensevoice" and confidence_kind == "unavailable"
                            and confidence is None and utterance_confidence is None and vad_validated is True)
                if (type(final) is not bool or type(playback_control) is not bool
                        or type(vad_validated) is not bool
                        or asr_backend not in ("vosk", "sensevoice")
                        or (not unscored and (confidence_kind != "min_word" or type(confidence) not in (int, float)
                            or not math.isfinite(confidence) or not 0 <= confidence <= 1))
                        or (utterance_confidence is not None and (type(utterance_confidence) not in (int, float)
                            or not math.isfinite(utterance_confidence) or not 0 <= utterance_confidence <= 1))
                        or (recognition_epoch is not None and type(recognition_epoch) is not int)
                        or (captured_at_s is not None and (type(captured_at_s) not in (int, float)
                            or not math.isfinite(captured_at_s)))):
                    self.last_decision = "voice_rejected"
                    self.last_gate["reason"] = "invalid_voice_format"
                    return None
                if isinstance(text, str):
                    text = canonical_voice_wake(text, config.wake_phrase, fuzzy=config.wake_fuzzy)
                age = now - (now if captured_at_s is None else captured_at_s)
                norm = normalized(text) if isinstance(text, str) else ""
                phrase = normalized(config.wake_phrase)
                addressed = norm.startswith(phrase)
                tail = norm[len(phrase):] if addressed else norm
                strict_control = (tail in {normalized(k) for k in CONTROLS}
                                  or behavior_requested(tail))
                interrupt = playback_control and addressed and tail == "打断回答" and self.state == InteractionState.ACTIVE
                # Generic ASR providers retain the old min-word threshold. Native
                # Vosk adds a separate utterance score for ordinary conversation.
                score = confidence if utterance_confidence is None or (strict_control and not interrupt) else utterance_confidence
                threshold = (None if unscored else config.audio_interrupt_confidence_min if interrupt else
                             config.audio_confidence_min if utterance_confidence is None or strict_control else
                             config.audio_wake_confidence_min if self.state == InteractionState.SLEEP else
                             config.audio_question_confidence_min)
                reason = ("asr_not_final" if not final else
                          "recognition_state_changed" if recognition_epoch is not None and recognition_epoch != self.asr_epoch else
                          "audio_stale_or_future" if not 0 <= age <= 1.5 else
                          "invalid_playback_control" if playback_control and not interrupt else
                          "unscored_behavior_requires_wake_prefix" if unscored and strict_control and not addressed else
                          "playback_active" if self.playback_active and not interrupt else
                          "echo_guard_active" if now < self.voice_blocked_until_s and not interrupt else
                          "confidence_below_threshold" if threshold is not None and score < threshold else None)
                self.last_gate.update(confidence=confidence, utterance_confidence=utterance_confidence,
                                      score=score, threshold=threshold, age_s=age, asr_backend=asr_backend,
                                      confidence_kind=confidence_kind, vad_validated=vad_validated)
                if reason:
                    self.last_decision = "voice_rejected"
                    self.last_gate["reason"] = reason
                    return None
            if not isinstance(text, str):
                raise ValueError("text must be a decoded Unicode string")
            text = text.strip()
            norm, phrase = normalized(text), normalized(config.wake_phrase)
            addressed = norm.startswith(phrase)
            if addressed:
                # Locate the end of the normalized prefix without relying on ASR spaces.
                index = next(i for i in range(1, len(text) + 1) if normalized(text[:i]) == phrase)
                text = text[index:].lstrip(" ,，:：。!！")
                norm = normalized(text)
            control = next((value for key, value in CONTROLS.items() if normalized(key) == norm), None)
            # Voice control requires the wake prefix even during an active conversation.
            if source == "voice" and control and not addressed and (control != "sleep" or self.state == InteractionState.SLEEP):
                self.last_decision = "voice_rejected"
                self.last_gate["reason"] = "control_requires_wake_prefix"
                return None
            if control:
                self.last_decision = "local_control"
                self.runtime.record_event("interaction", {"event": "control", "control": control, "source": source})
                if self.on_silence:
                    self.on_silence()
                interrupted = await self._interrupt()
                self.last_activity_s = now
                cancellation = None
                if control in ("cancel", "sleep", "stop", "wait"):
                    revoked = next((event for event in reversed(interrupted.metrics["behavior_events"])
                                    if event["status"] == "CANCEL"), None) if interrupted else None
                    cancellation = (Feedback.model_validate(revoked) if revoked and not self.runtime.supervisor.active_task
                                    else await self.runtime.supervisor.cancel())
                if control in ("stop", "wait"):
                    result = await self.runtime.supervisor.submit(BehaviorRequest(
                        intent=Intent.STOP_REQUEST if control == "stop" else Intent.WAIT))
                    return json.dumps(result.model_dump(mode="json"), ensure_ascii=False)
                if control == "sleep":
                    self.state = InteractionState.SLEEP
                    self.asr_epoch += 1
                    self.runtime.clear_history()
                    return ("已休眠。行为取消失败，需处理后端反馈：" + cancellation.model_dump_json()
                            if cancellation and cancellation.status == "FAILED" else "已休眠。")
                return ("回答已打断。" if control == "interrupt" else
                        json.dumps(cancellation.model_dump(mode="json"), ensure_ascii=False))
            if self.state == InteractionState.SLEEP:
                if not addressed:
                    return None
                self.state = InteractionState.ACTIVE
                self.asr_epoch += 1
                self.last_decision = "wake"
                self.runtime.record_event("interaction", {"event": "wake", "source": source})
            if not text:
                self.last_decision = "wake"
                self.last_activity_s = now
                return "已唤醒。"
            if self.task and not self.task.done():
                if not self.preempt_on_input:
                    self.last_decision = "busy"
                    return "正在回答；请先使用唤醒短语加“打断回答”，或文字 /interrupt。"
                if self.on_silence:
                    self.on_silence()
                await self._interrupt()
                self.runtime.record_event("interaction", {"event": "preempt", "source": source})
                if self.state == InteractionState.SLEEP:
                    self.last_decision = "ignored"
                    return None
            self.last_activity_s = now
            prompt, metadata = text, None
            if vision:
                try:
                    snapshot = self.frames.latest() if self.frames else None
                    image, metadata = encode_keyframe(snapshot, now_s=now,
                                                     max_age_s=config.frame_max_age_s,
                                                     roi=roi, roi_xyxy=roi_xyxy)
                    # Recheck freshness after JPEG encoding, immediately before dispatch.
                    if not 0 <= self.clock() - metadata["source_time_s"] <= config.frame_max_age_s:
                        raise ValueError("frame_stale_after_encoding")
                except ValueError as error:
                    self.last_decision = "vision_rejected"
                    self.runtime.record_event("interaction", {"event": "vision_rejected", "reason": str(error)})
                    return "视觉请求未发送：" + str(error)
                prompt = [text, "关键帧来源（仅问答，不授权机器人行为）：" + json.dumps(metadata), image]
            self.token = CancellationToken()
            self.last_decision = "accepted"
            authorized = behavior_requested(text) and not vision if self.strict_behavior_intent else None
            if self.on_run_start:
                self.on_run_start()
            self.task = asyncio.create_task(self._run(prompt, self.token, metadata, structured, authorized))
            return None

    async def _run(self, prompt, token, metadata, structured, authorized=None) -> TurnOutcome:
        def capture():
            if self.state != InteractionState.ACTIVE or token.cancelled:
                raise ValueError("interaction_inactive_or_cancelled")
            snapshot = self.frames.latest() if self.frames else None
            image, details = encode_keyframe(snapshot, now_s=self.clock(),
                                            max_age_s=self.runtime.config.frame_max_age_s)
            if not 0 <= self.clock() - details["source_time_s"] <= self.runtime.config.frame_max_age_s:
                raise ValueError("frame_stale_after_encoding")
            return image, details
        outcome = await self.runtime.run(prompt, token, on_text=self.on_text,
                                         frame_metadata=metadata, structured=structured,
                                         on_speech=self.on_speech, behavior_authorized=authorized,
                                         speech_metrics=self.speech_metrics, capture_view=capture)
        if outcome.metrics.get("interaction_action") == "SLEEP":
            if self.on_silence:
                self.on_silence()
            self.state = InteractionState.SLEEP
            self.asr_epoch += 1
            self.runtime.clear_history()
            if self.runtime.supervisor.active_task:
                await self.runtime.supervisor.cancel()
            self.runtime.record_event("interaction", {"event": "semantic_sleep", "action": "SLEEP"})
        self.last_activity_s = self.clock()
        if self.on_complete:
            self.on_complete(outcome)
        return outcome

    async def wait(self) -> TurnOutcome | None:
        return await self.task if self.task else None

    async def expire_idle(self) -> bool:
        if (self.state == InteractionState.ACTIVE and not self.playback_active
                and (not self.task or self.task.done())
                and self.clock() - self.last_activity_s >= self.runtime.config.idle_timeout_s):
            await self.receive("/sleep")
            self.runtime.record_event("interaction", {"event": "idle_sleep", "timeout_s": self.runtime.config.idle_timeout_s})
            return True
        return False

    async def close(self) -> None:
        await self._interrupt()
        await self.runtime.supervisor.cancel()
        self.state = InteractionState.SLEEP
        await self.runtime.close()
