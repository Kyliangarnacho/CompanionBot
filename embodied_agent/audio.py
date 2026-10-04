"""Optional offline Vosk + sounddevice input; no cloud ASR and no custom DSP.

Model assets stay in an ignored local cache. Recognition and SAPI speech are
local. Playback gates ordinary questions; optional addressed cancel commands
use a restricted local ASR path. This is not acoustic echo cancellation.
"""
from __future__ import annotations

import asyncio
import json
from pathlib import Path
import queue as thread_queue
import threading
import time
import re
from collections import deque
from uuid import uuid4

import numpy as np

from .interaction import CONTROLS, normalized, canonical_voice_wake, voice_interrupt
from .inputs import ASRMetadata


def select_input_device(sd, override=None, *, samplerate=16000) -> dict:
    """Select a physical input by name; never silently fall back to a webcam.

    Prefer MME for the existing mono 16 kHz contract. WASAPI often only accepts
    the endpoint's native 48 kHz rate; a CLI override remains explicit.
    """
    devices, apis = list(sd.query_devices()), list(sd.query_hostapis())
    if override is not None and str(override).lstrip("-").isdigit():
        candidates = [d for d in devices if d["index"] == int(override)]
    else:
        query = str(override or "").casefold()
        candidates = [d for d in devices if d["max_input_channels"] > 0 and (
            all(token in (d["name"] + " " + apis[d["hostapi"]]["name"]).casefold()
                for token in query.split()) if query else
            (any(word in d["name"].casefold() for word in ("realtek", "内置麦克风", "internal microphone"))
             and any(word in d["name"].casefold() for word in ("麦克风", "microphone", "mic input"))))]
    candidates = [d for d in candidates if d["max_input_channels"] > 0]
    if override is None:
        arrays = [d for d in candidates if any(word in d["name"].casefold()
                  for word in ("array", "阵列", "internal", "内置"))]
        if arrays:
            candidates = arrays
    candidates.sort(key=lambda d: (apis[d["hostapi"]]["name"] != "MME",
                                  "WDM-KS" in apis[d["hostapi"]]["name"], d["index"]))
    errors = []
    for device in candidates:
        try:
            sd.check_input_settings(device=device["index"], samplerate=samplerate, channels=1, dtype="int16")
            return {"index": device["index"], "name": device["name"],
                    "hostapi": apis[device["hostapi"]]["name"], "samplerate": samplerate,
                    "channels": 1, "format": "pcm_s16le", "selection": "override" if override is not None else "built_in_name"}
        except Exception as error:
            errors.append(type(error).__name__)
    raise RuntimeError("No compatible built-in microphone" if override is None else
                       "Requested microphone unavailable or PCM format unsupported")


def asr_wake_phrase(wake_phrase: str, override: str | None = None) -> str:
    # The measured small-cn transcript uses the common numeral 七 for 柒.
    return normalized(override or wake_phrase).replace("柒", "七")


def wake_grammar(wake_phrase: str, wake_asr_phrase: str | None = None) -> tuple[list[str], set[str]]:
    # Chinese names are not necessarily words in the small-cn vocabulary. Use
    # individual characters (Vosk's standard dynamic grammar, no model training).
    phrase = asr_wake_phrase(wake_phrase, wake_asr_phrase)
    spoken = [phrase]
    spoken += [phrase + normalized(key) for key in CONTROLS if not key.startswith("/")]
    return [" ".join(value) for value in spoken] + ["[unk]"], set(spoken)


def confirmed_wake(free_result: dict, wake_result: dict, wake_phrase: str,
                   wake_asr_phrase: str | None = None, *, fuzzy=False) -> bool:
    """Closed grammar must agree with an independent open-vocabulary decode.

    Without corroboration, 小白 was forced to 小伴 with confidence 1.0.
    Canonicalize only the configured wake prefix's bounded phonetic variants;
    the closed grammar cannot manufacture a wake prefix on its own.
    """
    free_text = normalized(canonical_voice_wake(free_result.get("text", ""), wake_phrase, fuzzy=fuzzy))
    # Corroborate the ONE wake prefix. Free speech may include a question after
    # it while the constrained decoder ends at the wake word; whole-utterance
    # equality incorrectly rejected that normal interaction.
    prefix = asr_wake_phrase(wake_phrase, wake_asr_phrase)
    return (asr_wake_phrase(free_text).startswith(prefix)
            and normalized(wake_result.get("text", "")).startswith(prefix))


def asr_scores(result: dict) -> tuple[float, float]:
    """Keep the legacy minimum score; add a separate duration-weighted score.

    Vosk confidence is not a calibrated probability. Missing/invalid scores
    fail closed; no text/PCM format or clock semantics are changed.
    """
    words = result.get("result", [])
    if not words:
        return 0.0, 0.0
    pairs = []
    for word in words:
        value = word.get("conf", 0)
        if type(value) not in (int, float) or not np.isfinite(value) or not 0 <= value <= 1:
            return 0.0, 0.0
        start, end = word.get("start", 0), word.get("end", 0)
        if any(type(v) not in (int, float) or not np.isfinite(v) for v in (start, end)):
            return 0.0, 0.0
        duration = end - start
        pairs.append((float(value), max(.01, float(duration))))
    return min(v for v, _ in pairs), sum(v * w for v, w in pairs) / sum(w for _, w in pairs)


async def microphone_events(model_path: Path, callback, *, device=None,
                            is_blocked=lambda: False, is_active=lambda: True,
                            wake_phrase="你好小柒", wake_asr_phrase=None, samplerate=16000,
                            on_event=None, state_epoch=lambda: 0, fuzzy_wake=False,
                            question_model_dir: Path | None = None, silence_s=0.9,
                            voice_interrupt_enabled=False, is_self_echo=lambda text: False) -> None:
    if type(samplerate) is not int or not 8000 <= samplerate <= 96000:
        raise ValueError("Unsupported PCM sample rate")
    try:
        import sounddevice as sd
        from vosk import KaldiRecognizer, Model, SetLogLevel
    except ImportError:
        raise RuntimeError("Install requirements-agent-audio.txt in the project .venv") from None
    if not model_path.is_dir():
        raise ValueError("Vosk model directory is missing")
    SetLogLevel(-1)
    selected = select_input_device(sd, device, samplerate=samplerate)
    device = selected["index"]
    model = await asyncio.to_thread(Model, str(model_path))
    recognizer = KaldiRecognizer(model, samplerate)
    recognizer.SetWords(True)
    grammar, wake_commands = wake_grammar(wake_phrase, wake_asr_phrase)
    asr_phrase = asr_wake_phrase(wake_phrase, wake_asr_phrase)
    wake = KaldiRecognizer(model, samplerate, json.dumps(grammar, ensure_ascii=False))
    wake.SetWords(True)
    question_asr = None
    if question_model_dir is not None:
        from .speech_recognition import SenseVoiceInput
        started = time.perf_counter()
        question_asr = await asyncio.to_thread(SenseVoiceInput, question_model_dir,
                                             samplerate=samplerate, silence_s=silence_s)
        if on_event:
            on_event("question_asr_ready", {"backend": "sensevoice", "confidence_kind": "unavailable",
                                            "vad": "silero", "silence_s": silence_s,
                                            "load_s": time.perf_counter() - started})
    loop = asyncio.get_running_loop()
    source_epoch = uuid4().hex
    def provenance(captured_at):
        return ASRMetadata(source_id="microphone:" + str(selected["index"]), source_epoch=source_epoch,
                           sequence=stats["final_transcripts"], received_at_s=captured_at)
    queue: asyncio.Queue = asyncio.Queue(maxsize=8)
    reset_needed = False
    stats = {"chunks": 0, "blocked_chunks": 0, "stale_chunks": 0,
             "overflows": 0, "final_transcripts": 0, "state_resets": 0, "pcm_samples": 0,
             "pcm_peak": 0, "pcm_rms": None, "wake_confirmation_rejections": 0,
             "voice_interrupts": 0, "voice_echo_rejections": 0, "question_segments": 0,
             "interrupt_segments": 0}
    last_blocked = False
    sum_squares, warned_quiet = 0.0, False
    last_level_s = 0.0
    level_count, level_squares, level_peak = 0, 0.0, 0

    def enqueue(data, captured_at, overflow, blocked, epoch):
        nonlocal reset_needed
        if queue.full():
            queue.get_nowait()
            reset_needed = True
            stats["overflows"] += 1
        if overflow:
            reset_needed = True
            stats["overflows"] += 1
        queue.put_nowait((data, captured_at, blocked, epoch))

    def audio_callback(indata, _frames, _time, status):
        try:
            loop.call_soon_threadsafe(enqueue, bytes(indata), time.perf_counter(),
                                     bool(status), is_blocked(), state_epoch())
        except RuntimeError:  # Event loop already closed during shutdown.
            pass

    pending = {}
    last_epoch = state_epoch()

    def diagnostic(free, candidate, decision):
        if on_event:
            minimum, score = asr_scores(free)
            loop.call_soon_threadsafe(on_event, "asr_result", {
                "free_text": free.get("text", ""), "grammar_text": candidate.get("text", ""),
                "decision": decision, "confidence": minimum, "utterance_confidence": score,
                "active": is_active()})

    def free_input(result, active):
        # An exact open-vocabulary wake prefix is also a valid local baseline;
        # it still passes the session's confidence/freshness gates. Closed-grammar
        # output alone can never manufacture the prefix.
        allowed = active or asr_wake_phrase(canonical_voice_wake(
            result.get("text", ""), wake_phrase, fuzzy=fuzzy_wake)).startswith(asr_phrase)
        diagnostic(result, {}, "question_candidate" if active else "wake_prefix" if allowed else "wake_prefix_absent")
        return result if allowed else None

    def recognize(data, active, captured_at):
        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            if result.get("text"):
                pending["free"] = (result, captured_at)
        if wake.AcceptWaveform(data):
            result = json.loads(wake.Result())
            if result.get("text"):
                pending["wake"] = (result, captured_at)
        free, candidate = pending.get("free"), pending.get("wake")
        if free and candidate and abs(free[1] - candidate[1]) <= .5:
            pending.clear()
            if normalized(candidate[0].get("text", "")) in wake_commands:
                if confirmed_wake(free[0], candidate[0], wake_phrase, wake_asr_phrase, fuzzy=fuzzy_wake):
                    # Convert ASR lexicon spelling to the configured display name
                    # at the audio adapter boundary, not in the Agent Runtime.
                    candidate[0]["text"] = canonical_voice_wake(free[0]["text"], wake_phrase, fuzzy=fuzzy_wake)
                    # A constrained grammar can report 1.0 for a wrong word.
                    # Keep the minimum confidence from both local decoders.
                    candidate[0]["result"] = free[0].get("result", []) + candidate[0].get("result", [])
                    diagnostic(free[0], candidate[0], "wake_corroborated")
                    # Use free decoding for the utterance score; constrained
                    # grammar's optimistic scores cannot raise it.
                    candidate[0]["utterance_confidence"] = asr_scores(free[0])[1]
                    return candidate[0]
                stats["wake_confirmation_rejections"] += 1
                diagnostic(free[0], candidate[0], "wake_decoder_mismatch")
            return free_input(free[0], active)
        # Endpoints may differ. Never pair transcripts from separate utterances.
        if free and captured_at - free[1] > .5:
            pending.clear()
            return free_input(free[0], active)
        if candidate and captured_at - candidate[1] > .5:
            # Expiring an older grammar result must not erase a newer free result.
            pending.pop("wake", None)
            diagnostic({}, candidate[0], "grammar_only_uncorroborated")
        return None

    try:
        sd.check_input_settings(device=device, samplerate=samplerate, channels=1, dtype="int16")
        with sd.RawInputStream(samplerate=samplerate, blocksize=samplerate // 10, device=device,
                               dtype="int16", channels=1, callback=audio_callback):
            if on_event:
                on_event("audio_ready", {**selected, "device": device, "samplerate": samplerate, "channels": 1,
                         "format": "pcm_s16le", "clock_domain": "host_perf_counter",
                         "timestamp_semantics": "audio_callback_host_receipt",
                         "asr_wake_phrase": asr_phrase,
                         "question_backend": "sensevoice" if question_asr else "vosk",
                         "voice_interrupt_enabled": voice_interrupt_enabled})
            while True:
                data, captured_at, captured_blocked, epoch = await queue.get()
                stats["chunks"] += 1
                samples = np.frombuffer(data, dtype=np.int16).astype(np.float64)
                stats["pcm_samples"] += len(samples)
                stats["pcm_peak"] = max(stats["pcm_peak"], int(np.max(np.abs(samples), initial=0)))
                sum_squares += float(np.dot(samples, samples))
                level_count += len(samples)
                level_squares += float(np.dot(samples, samples))
                level_peak = max(level_peak, int(np.max(np.abs(samples), initial=0)))
                stats["pcm_rms"] = (sum_squares / max(1, stats["pcm_samples"])) ** .5
                if on_event and captured_at - last_level_s >= 1:
                    last_level_s = captured_at
                    on_event("audio_level", {"pcm_peak": level_peak,
                             "pcm_rms": (level_squares / max(1, level_count)) ** .5,
                             "mode": ("local_interrupt_only" if voice_interrupt_enabled and is_active() else "playback_blocked") if captured_blocked or is_blocked() else
                                     "question_listening" if is_active() else "local_wake_only"})
                    level_count, level_squares, level_peak = 0, 0.0, 0
                if (stats["pcm_samples"] >= 2 * samplerate and stats["pcm_peak"] <= 8
                        and stats["pcm_rms"] < 1 and not warned_quiet):
                    warned_quiet = True
                    if on_event:
                        on_event("audio_near_silent", {"device": device, "pcm_peak": stats["pcm_peak"]})
                blocked = captured_blocked or is_blocked()
                stale = not 0 <= time.perf_counter() - captured_at <= 1.5
                if state_epoch() != last_epoch:
                    last_epoch = state_epoch()
                    reset_needed = True
                    stats["state_resets"] += 1
                stale = stale or epoch != last_epoch
                if reset_needed or stale or blocked != last_blocked:
                    # Discard recognizer state too: playback must not survive into
                    # a later final transcript after the echo guard has expired.
                    recognizer.Reset()
                    wake.Reset()
                    pending.clear()
                    if question_asr:
                        question_asr.reset()
                    reset_needed = False
                    last_blocked = blocked
                    stats["stale_chunks"] += int(stale)
                    if stale:
                        continue
                if blocked:
                    # Ordinary playback PCM never reaches the question ASR or
                    # Agent. Only an addressed complete interrupt may pass.
                    stats["blocked_chunks"] += 1
                    if not voice_interrupt_enabled or not is_active():
                        continue
                if question_asr and is_active():
                    from .speech_recognition import segment_quality
                    segments = question_asr.feed(data)
                    if question_asr.speaking and on_event and not blocked:
                        on_event("user_speech", {"captured_at_s": captured_at})
                    for segment in segments:
                        if not segment_quality(segment):
                            if on_event:
                                on_event("asr_audio_rejected", {"reason": "silence_clipping_or_duration",
                                                                 "duration_s": segment.duration_s, "rms": segment.rms})
                            continue
                        stats["interrupt_segments" if blocked else "question_segments"] += 1
                        started = time.perf_counter()
                        if on_event:
                            on_event("asr_decoding", {"backend": "sensevoice", "duration_s": segment.duration_s,
                                                      "interrupt_only": blocked})
                        text = await asyncio.to_thread(question_asr.decode, segment)
                        if on_event:
                            on_event("asr_result", {"free_text": text, "grammar_text": "", "decision": "sensevoice_interrupt" if blocked else "sensevoice_question",
                                                     "confidence": None, "utterance_confidence": None, "active": True,
                                                     "backend": "sensevoice", "confidence_kind": "unavailable",
                                                     "duration_s": segment.duration_s, "rms": segment.rms,
                                                     "decode_s": time.perf_counter() - started})
                        if text:
                            interrupt = voice_interrupt(text, wake_phrase, fuzzy=fuzzy_wake) if voice_interrupt_enabled else None
                            if blocked and not interrupt:
                                continue
                            if interrupt and is_self_echo(text):
                                stats["voice_echo_rejections"] += 1
                                if on_event:
                                    on_event("voice_interrupt_rejected", {"reason": "matches_own_tts_text"})
                                continue
                            if interrupt:
                                stats["voice_interrupts"] += 1
                            stats["final_transcripts"] += 1
                            await callback(interrupt or canonical_voice_wake(text, wake_phrase, fuzzy=fuzzy_wake),
                                           source="voice", final=True, confidence=None, confidence_kind="unavailable",
                                           asr_backend="sensevoice", vad_validated=True,
                                           playback_control=bool(interrupt), recognition_epoch=epoch, captured_at_s=captured_at,
                                           asr_metadata=provenance(captured_at))
                    continue
                result = await asyncio.to_thread(recognize, data, is_active(), captured_at)
                if result:
                    confidence, score = asr_scores(result)
                    score = result.get("utterance_confidence", score)
                    if result.get("text"):
                        text = canonical_voice_wake(result["text"], wake_phrase, fuzzy=fuzzy_wake)
                        canonical = normalized(text)
                        if canonical.startswith(asr_phrase):
                            text = wake_phrase + canonical[len(asr_phrase):]
                        interrupt = voice_interrupt(text, wake_phrase, fuzzy=fuzzy_wake) if voice_interrupt_enabled else None
                        if blocked:
                            if not interrupt:
                                continue
                            if is_self_echo(text):
                                stats["voice_echo_rejections"] += 1
                                if on_event:
                                    on_event("voice_interrupt_rejected", {"reason": "matches_own_tts_text"})
                                continue
                        if interrupt:
                            stats["voice_interrupts"] += 1
                        stats["final_transcripts"] += 1
                        await callback(interrupt or text, source="voice", final=True,
                                       confidence=confidence, utterance_confidence=score,
                                       playback_control=bool(interrupt),
                                       recognition_epoch=epoch, captured_at_s=captured_at,
                                       asr_metadata=provenance(captured_at))
    finally:
        if on_event:
            on_event("audio_closed", stats)


class _WindowsSpeech:
    """Thin adapter over the installed Windows speech engine, not a synthesizer."""
    def __init__(self):
        import win32com.client
        self.voice = win32com.client.Dispatch("SAPI.SpVoice")
        tokens = self.voice.GetVoices()
        token = next((tokens.Item(i) for i in range(tokens.Count)
                      if "804" in tokens.Item(i).GetAttribute("Language").lower().split(";")), None)
        if token is None:
            raise RuntimeError("Install a Windows Simplified Chinese SAPI voice")
        self.voice.Voice = token
        self.voice.Rate = 0
        self.voice_info = {"voice": token.GetDescription(), "backend": "windows_sapi5",
                           "output": self.voice.AudioOutput.GetDescription()}

    def say(self, text):
        # Async + literal text (SVSFIsNotXML): model output is never parsed as SSML.
        self.voice.Speak(text, 1 | 16)

    def is_busy(self):
        return not self.voice.WaitUntilDone(0)

    def stop(self):
        self.voice.Speak("", 1 | 2 | 16)  # Async purge.
        if not self.voice.WaitUntilDone(1000):
            raise TimeoutError("SAPI purge deadline")

    def pump(self):
        import pythoncom
        pythoncom.PumpWaitingMessages()

    def close(self):
        self.stop()
        self.voice = None


class SpeechOutput:
    """Optional native SAPI5 renderer with its COM loop on one worker thread.

    Stable text segments share one generation and play in order. The asyncio
    thread remains free for text/preview-key controls.
    """
    def __init__(self, set_playback, *, on_event=None, engine_factory=None) -> None:
        self.set_playback, self.on_event = set_playback, on_event
        self.engine_factory = engine_factory
        self.jobs = thread_queue.Queue()
        self.generation = 0
        self.latest = None
        self.thread = None
        self.closed = False
        self.pending = set()
        self.stream_open = False
        self._echo_text = ""
        self._echo_until_s = 0.0
        self._spoken_history = deque(maxlen=24)

    def is_self_echo(self, text: str) -> bool:
        canonical = re.sub(r"你好小[柒七琪棋琦祺奇齐其启起气]", "你好小柒", normalized(text))
        recent = "".join(t for when, t in self._spoken_history if time.perf_counter() - when <= 20)
        spoken = re.sub(r"你好小[柒七琪棋琦祺奇齐其启起气]", "你好小柒", normalized(recent + self._echo_text))
        # Text provenance veto, not acoustic echo cancellation. In headphones
        # the channel works well; overlapping speaker echo still needs AEC.
        return bool(canonical and (recent or time.perf_counter() <= self._echo_until_s) and canonical in spoken)

    async def start(self) -> dict:
        self.loop = asyncio.get_running_loop()
        ready = self.loop.create_future()
        self.thread = threading.Thread(target=self._worker, args=(ready,),
                                       name="local-sapi-speech", daemon=True)
        self.thread.start()
        return await ready

    def begin_stream(self) -> int:
        self.interrupt()
        self.stream_open = True
        return self.generation

    def enqueue(self, text: str, generation: int):
        if self.closed or self.thread is None or not self.thread.is_alive():
            raise RuntimeError("Speech output is not running")
        if generation != self.generation or not text.strip():
            return None
        future = self.loop.create_future()
        self.latest = future
        self.pending.add(future)
        self.set_playback(True)
        self.jobs.put((generation, text, future))
        return future

    def end_stream(self, generation: int) -> None:
        if generation == self.generation:
            self.stream_open = False
            if not self.pending:
                self.set_playback(False)

    def say(self, text: str):
        generation = self.begin_stream()  # Local acknowledgement replaces old speech.
        future = self.enqueue(text, generation)
        self.end_stream(generation)
        return future

    def interrupt(self) -> None:
        self.generation += 1
        self.stream_open = False
        while True:
            try:
                job = self.jobs.get_nowait()
            except thread_queue.Empty:
                break
            if job:
                self._finish(job[2], {"status": "CANCEL", "generation": job[0], "queued": True})

    async def wait(self):
        return await self.latest if self.latest else None

    def _finish(self, future, result) -> None:
        if not future.done():
            future.set_result(result)
        self.pending.discard(future)
        if not self.pending and not self.stream_open:
            self.set_playback(False)
        if self.on_event:
            self.on_event("tts_finished", result)

    def _ready(self, future, result=None, error=None):
        if not future.done():
            if error:
                future.set_exception(RuntimeError("Local TTS initialization failed: " + error))
            else:
                future.set_result(result)

    def _worker(self, ready) -> None:
        engine, com = None, None
        try:
            if self.engine_factory is None:
                import pythoncom
                com = pythoncom
                com.CoInitialize()
                engine = _WindowsSpeech()
            else:
                engine = self.engine_factory()
            self.loop.call_soon_threadsafe(self._ready, ready, engine.voice_info)
            while True:
                try:
                    job = self.jobs.get(timeout=0.03)
                except thread_queue.Empty:
                    engine.pump()
                    continue
                if job is None:
                    break
                generation, text, future = job
                started, status = time.perf_counter(), "CANCEL"
                try:
                    if generation == self.generation:
                        self._echo_text, self._echo_until_s = text, float("inf")
                        self._spoken_history.append((time.perf_counter(), text))
                        engine.say(text)
                        if self.on_event:
                            self.loop.call_soon_threadsafe(self.on_event, "tts_started",
                                                          {"generation": generation, "started_at_s": time.perf_counter()})
                        status = "COMPLETED"
                        while engine.is_busy():
                            engine.pump()
                            if generation != self.generation or time.perf_counter() - started > 90:
                                engine.stop()
                                status = "CANCEL" if generation != self.generation else "TIMEOUT"
                                break
                            time.sleep(0.01)
                        engine.pump()
                except Exception as error:
                    status = "FAILED:" + type(error).__name__
                self._echo_until_s = time.perf_counter() + 1.0
                self.loop.call_soon_threadsafe(self._finish, future,
                                               {"status": status, "generation": generation,
                                                "wall_s": time.perf_counter() - started})
        except Exception as error:
            self.loop.call_soon_threadsafe(self._ready, ready, None, type(error).__name__)
            # Preserve text interaction and release the microphone gate on failure.
            while True:
                try:
                    job = self.jobs.get_nowait()
                except thread_queue.Empty:
                    break
                if job:
                    self.loop.call_soon_threadsafe(self._finish, job[2], {"status": "FAILED"})
        finally:
            if engine:
                engine.close()
            if com:
                com.CoUninitialize()

    async def close(self) -> None:
        self.closed = True
        self.interrupt()
        self.jobs.put(None)
        if self.thread:
            await asyncio.to_thread(self.thread.join, 5)
            if self.thread.is_alive():
                raise RuntimeError("Local speech worker did not stop")
        self.set_playback(False)


class SentenceBuffer:
    """Small Chinese sentence/phrase buffer; synthesis stays in native SAPI."""
    def __init__(self):
        self.text = ""

    def push(self, fragment: str, *, final=False) -> list[str]:
        self.text += fragment
        result = []
        while self.text:
            match = re.search(r"[。！？!?；;\n]", self.text)
            end = match.end() if match else 0
            if not end and len(self.text) >= 32:
                clause = re.search(r"[，,：:]", self.text[16:80])
                end = 16 + clause.end() if clause else 0
            if not end and (final or len(self.text) >= 96):
                end = len(self.text) if final else 80
            if not end:
                break
            segment, self.text = self.text[:end], self.text[end:]
            segment = re.sub(r"[*#`]+", "", segment).strip()
            if segment:
                result.append(segment)
        return result


class StreamingSpeech:
    """One answer's segmentation and timing; no model/runtime ownership."""
    def __init__(self, speech: SpeechOutput):
        self.speech = speech
        self.generation = speech.begin_stream()
        self.started_at_s = time.perf_counter()
        self.buffer = SentenceBuffer()
        self.last = None
        self.futures = []
        self.first_speech_s = None
        self.segments = 0

    def push(self, fragment: str, *, final=False):
        if self.generation != self.speech.generation:
            return
        for segment in self.buffer.push(fragment, final=final):
            self.last = self.speech.enqueue(segment, self.generation)
            if self.last:
                self.futures.append(self.last)
            self.segments += 1

    def started(self, event):
        if event.get("generation") == self.generation and self.first_speech_s is None:
            self.first_speech_s = event["started_at_s"] - self.started_at_s

    async def finish(self, completed=True) -> dict:
        if completed:
            self.push("", final=True)
            self.speech.end_stream(self.generation)
        elif self.generation == self.speech.generation:
            self.speech.interrupt()
        # Purging queued jobs completes their futures before the worker confirms
        # that the currently playing segment stopped. Await every segment in this
        # generation, never just the last queued future.
        results = await asyncio.gather(*self.futures) if self.futures else []
        result = results[-1] if results else {"status": "NO_SPEECH"}
        return {"first_speech_s": self.first_speech_s, "speech_segments": self.segments,
                "interaction_wall_s": time.perf_counter() - self.started_at_s,
                "speech_status": result["status"], "speech_timing_semantics": "sapi_async_command_not_acoustic_onset"}
