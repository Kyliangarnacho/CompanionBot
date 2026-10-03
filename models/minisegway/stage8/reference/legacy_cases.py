"""Archived assertions for removed helpers; excluded from the active tests/ suite."""
import pytest
from embodied_agent.config import AgentConfig
from models.minisegway.stage8.reference.legacy_interaction import vision_requested, confirmed_wake, canonical_voice_wake

@pytest.mark.parametrize("text", ["你面前有什么？", "你好小柒，请介绍这个展品", "帮我看看这个商品", "镜头里是什么"])
def test_explicit_natural_visual_questions(text):
    assert vision_requested(text)


@pytest.mark.parametrize("text", ["你好", "请跟随我", "带我去服务台", "比较轻行杯和随身灯", "什么是机器人", "介绍博物馆历史"])
def test_ordinary_questions_do_not_select_camera(text):
    assert not vision_requested(text)


def test_legacy_custom_wake_remains_configurable_but_is_not_the_default():
    assert AgentConfig().wake_phrase == "你好小柒"
    assert confirmed_wake({"text": "你好小半"}, {"text": "你好小伴"}, "你好小伴")
    assert canonical_voice_wake("你好小半，介绍一下", "你好小伴") == "你好小伴，介绍一下"

