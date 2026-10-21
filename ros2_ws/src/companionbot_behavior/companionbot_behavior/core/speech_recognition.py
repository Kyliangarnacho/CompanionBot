"""Local SenseVoice/Silero adapter; capture and Agent ownership stay unchanged.

Adapted from sherpa-onnx's official vad-with-non-streaming-asr example
(Apache-2.0); SenseVoice and Silero model assets are MIT, kept in .venv.
No network calls, model downloads, microphone opening or fabricated confidence.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np


@dataclass
class SpeechSegment:
    samples: np.ndarray
    duration_s: float
    rms: float
    clipped_fraction: float


class SenseVoiceInput:
    def __init__(self, model_dir: Path, *, samplerate=16000, silence_s=0.9):
        if samplerate != 16000:
            raise ValueError("SenseVoice/Silero adapter requires explicit mono 16000 Hz PCM")
        paths = {name: model_dir / name for name in ("model.int8.onnx", "tokens.txt", "silero_vad.onnx")}
        if any(not path.is_file() for path in paths.values()):
            raise ValueError("SenseVoice assets missing; install locally or explicitly select --asr-backend vosk")
        import sherpa_onnx
        self.recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=str(paths["model.int8.onnx"]), tokens=str(paths["tokens.txt"]),
            num_threads=2, language="zh", use_itn=True, provider="cpu")
        config = sherpa_onnx.VadModelConfig()
        config.sample_rate = samplerate
        config.num_threads = 1
        config.silero_vad.model = str(paths["silero_vad.onnx"])
        config.silero_vad.threshold = 0.5
        config.silero_vad.min_speech_duration = 0.2
        config.silero_vad.min_silence_duration = silence_s
        config.silero_vad.max_speech_duration = 15
        self.vad = sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=35)
        self.pending = np.empty(0, dtype=np.float32)

    def reset(self):
        self.vad.reset()
        self.pending = np.empty(0, dtype=np.float32)

    @property
    def speaking(self):
        return self.vad.is_speech_detected()

    def feed(self, pcm: bytes) -> list[SpeechSegment]:
        if len(pcm) % 2:
            raise ValueError("PCM s16le must contain whole samples")
        samples = np.frombuffer(pcm, dtype="<i2").astype(np.float32) / 32768.0
        self.pending = np.concatenate((self.pending, samples))
        window = 512
        while len(self.pending) >= window:
            self.vad.accept_waveform(self.pending[:window])
            self.pending = self.pending[window:]
        segments = []
        while not self.vad.empty():
            # front is invalidated by pop, so copy before releasing it.
            values = np.array(self.vad.front.samples, dtype=np.float32, copy=True)
            self.vad.pop()
            segments.append(SpeechSegment(values, len(values) / 16000,
                                          float(np.sqrt(np.mean(values ** 2))) * 32768,
                                          float(np.mean(np.abs(values) >= .999))))
        return segments

    def decode(self, segment: SpeechSegment) -> str:
        stream = self.recognizer.create_stream()
        stream.accept_waveform(16000, segment.samples)
        self.recognizer.decode_stream(stream)
        return re.sub(r"<\|[^|]*\|>", "", stream.result.text).strip()


def segment_quality(segment: SpeechSegment) -> bool:
    """Reject silence/clipping, without treating audio level as ASR confidence."""
    return (0.2 <= segment.duration_s <= 16 and np.isfinite(segment.rms)
            and segment.rms >= 8 and segment.clipped_fraction < .2)
