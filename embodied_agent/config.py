"""Non-secret configuration and process-local DashScope credentials."""
from __future__ import annotations

import os
from pathlib import Path
import re
from urllib.parse import urlparse

from pydantic import BaseModel, ConfigDict, Field, field_validator


STAGE_DIR = Path(__file__).resolve().parents[1] / "models/minisegway/stage8"


class AgentConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)
    model: str = "qwen3.8-flash"
    base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    wake_phrase: str = "你好小柒"
    # Optional lexicon spelling for local ASR; user-facing phrase remains above.
    wake_asr_phrase: str | None = None
    idle_timeout_s: float = Field(default=30, gt=0, le=3600)
    idle_sleep_message: str = "小柒先走啦。"
    wake_fuzzy: bool = True
    run_timeout_s: float = Field(default=30, gt=0, le=120)
    frame_max_age_s: float = Field(default=1, gt=0, le=5)
    robot_max_age_s: float = Field(default=0.5, gt=0, le=2)
    request_limit: int = Field(default=3, ge=1, le=6)
    tool_calls_limit: int = Field(default=4, ge=1, le=8)
    total_tokens_limit: int = Field(default=12000, ge=100, le=50000)
    max_output_tokens: int = Field(default=512, ge=32, le=2048)
    max_history_turns: int = Field(default=4, ge=0, le=20)
    audio_confidence_min: float = Field(default=0.85, ge=0, le=1)
    # Additive scores: legacy confidence remains minimum word confidence.
    audio_question_confidence_min: float = Field(default=0.60, ge=0, le=1)
    audio_wake_confidence_min: float = Field(default=0.35, ge=0, le=1)
    audio_interrupt_confidence_min: float = Field(default=0.50, ge=0, le=1)
    asr_silence_s: float = Field(default=0.9, ge=0.3, le=2)
    echo_guard_s: float = Field(default=0.6, ge=0, le=5)

    @field_validator("base_url")
    @classmethod
    def validate_url(cls, value: str) -> str:
        parsed = urlparse(value)
        if (parsed.scheme != "https" or not parsed.hostname
                or not parsed.hostname.endswith(".aliyuncs.com")
                or parsed.username or parsed.password or parsed.query or parsed.fragment
                or not parsed.path.rstrip("/").endswith("/compatible-mode/v1")):
            raise ValueError("base_url must be an HTTPS Alibaba compatible-mode endpoint")
        return value.rstrip("/")

    @field_validator("model", "wake_phrase")
    @classmethod
    def nonempty(cls, value: str) -> str:
        if not value.strip():
            raise ValueError("value must not be empty")
        return value.strip()

    @field_validator("wake_phrase")
    @classmethod
    def spoken_phrase(cls, value: str) -> str:
        if not any(c.isalnum() for c in value):
            raise ValueError("wake_phrase must contain spoken characters")
        return value

    @field_validator("wake_asr_phrase")
    @classmethod
    def valid_asr_phrase(cls, value: str | None) -> str | None:
        if value is not None and not any(c.isalnum() for c in value):
            raise ValueError("wake_asr_phrase must contain spoken characters")
        return value.strip() if value is not None else None

    @classmethod
    def load(cls, path: Path | None = None) -> "AgentConfig":
        config = cls.model_validate_json((path or STAGE_DIR / "config/agent.json").read_text("utf-8"))
        overrides = {}
        for env, field in (("QWEN_MODEL", "model"), ("QWEN_BASE_URL", "base_url")):
            if value := os.environ.get(env):
                overrides[field] = value
        return cls.model_validate(config.model_dump() | overrides)


def load_api_key(path: Path | None = None) -> str:
    """Prefer DASHSCOPE_API_KEY; never persist or display the credential.

    The desktop file may have a hidden .txt suffix or a single labelled key.
    Ambiguous files are rejected rather than guessing or dumping their contents.
    """
    if value := os.environ.get("DASHSCOPE_API_KEY"):
        return value.strip()
    location = path or Path(os.environ.get("QWEN_API_KEY_FILE", str(Path.home() / "Desktop/千问")))
    if not location.exists() and not location.suffix:
        location = location.with_suffix(".txt")
    if location.is_dir():
        candidates = [p for p in location.iterdir() if p.is_file() and p.suffix in ("", ".txt", ".env")]
        if len(candidates) != 1:
            raise ValueError("Select the key file with QWEN_API_KEY_FILE")
        location = candidates[0]
    try:
        content = location.read_text(encoding="utf-8-sig")
    except (OSError, UnicodeError):
        raise ValueError("Cannot read credential file; set DASHSCOPE_API_KEY or QWEN_API_KEY_FILE") from None
    # Current Model Studio keys may be versioned/dot-separated, not just sk-hex.
    # Boundaries also keep an ASCII-looking prefix of a malformed key from matching.
    keys = set(re.findall(r"(?<![A-Za-z0-9_.-])sk-[A-Za-z0-9_.-]+(?![A-Za-z0-9_.-])", content))
    if len(keys) != 1:
        raise ValueError("Credential file must contain exactly one DashScope key")
    key = keys.pop()
    os.environ["DASHSCOPE_API_KEY"] = key
    return key
