"""Historical wake spelling and keyword vision hints; not imported by the baseline."""
from embodied_agent.interaction import normalized
from embodied_agent.audio import asr_wake_phrase

def canonical_voice_wake(text: str, phrase: str, *, fuzzy=False) -> str:
    """Bounded phonetic tolerance for ONE spoken phrase, only at ASR boundary.

    Keep the 你好小 prefix and accept common qi spellings/tones. Do not use
    unrestricted edit distance, scan a quoted mention, or add other greetings.
    Preserve the question tail, including punctuation from the better ASR.
    """
    norm = normalized(text)
    if normalized(phrase) == "你好小伴" and norm.startswith("你好小半"):
        index = next(i for i in range(1, len(text) + 1) if len(normalized(text[:i])) == 4)
        return phrase + text[index:]
    if normalized(phrase) != "你好小柒" or len(norm) < 4:
        return text
    names = "柒七琪棋琦祺奇齐其启起气" if fuzzy else "柒七"
    if not norm.startswith("你好小") or norm[3] not in names:
        return text
    index = next(i for i in range(1, len(text) + 1) if len(normalized(text[:i])) == 4)
    return phrase + text[index:]


def vision_requested(text: str) -> bool:
    """Legacy caller hint only; the final demo uses the Agent's capture_view tool."""
    text = normalized(text)
    return (any(term in text for term in ("你看到了什么", "你看到什么", "前面有什么", "面前有什么", "眼前有什么",
                                          "这是什么", "这个是什么", "帮我看看", "请看看"))
            or (any(term in text for term in ("画面", "镜头", "摄像头", "这个商品", "这个展品"))
                and any(term in text for term in ("什么", "介绍", "识别", "描述", "看到", "看一下"))))


def confirmed_wake(free_result: dict, wake_result: dict, wake_phrase: str,
                   wake_asr_phrase: str | None = None, *, fuzzy=False) -> bool:
    """Closed grammar must agree with an independent open-vocabulary decode.

    Without corroboration, 小白 was forced to 小伴 with confidence 1.0.
    The legacy 小伴 profile spells /xiao ban/ as 小半; only that observed
    homophone is canonicalized, never arbitrary edit-distance/fuzzy matches.
    """
    free_text = normalized(canonical_voice_wake(free_result.get("text", ""), wake_phrase, fuzzy=fuzzy))
    if normalized(wake_phrase) == "你好小伴" and free_text.startswith("你好小半"):
        free_text = "你好小伴" + free_text[len("你好小半"):]
    # Corroborate the ONE wake prefix. Free speech may include a question after
    # it while the constrained decoder ends at the wake word; whole-utterance
    # equality incorrectly rejected that normal interaction.
    prefix = asr_wake_phrase(wake_phrase, wake_asr_phrase)
    return (asr_wake_phrase(free_text).startswith(prefix)
            and normalized(wake_result.get("text", "")).startswith(prefix))

