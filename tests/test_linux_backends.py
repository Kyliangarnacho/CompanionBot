"""Platform contract tests with device fixtures, not physical audio certification."""
from types import SimpleNamespace
import pytest
from companionbot_behavior.core.audio import select_input_device


def linux_device(name="default", default=0):
    return SimpleNamespace(query_devices=lambda: [dict(index=0, name=name, hostapi=0, max_input_channels=2)],
        query_hostapis=lambda: [{"name": "ALSA"}], default=SimpleNamespace(device=(default, 0)),
        check_input_settings=lambda **kwargs: None)


def test_linux_uses_only_configured_input(monkeypatch):
    monkeypatch.setattr("companionbot_behavior.core.audio.sys.platform", "linux")
    selected = select_input_device(linux_device())
    assert selected["selection"] == "configured_linux_default"
    assert selected["samplerate"] == 16000 and selected["format"] == "pcm_s16le"
    with pytest.raises(RuntimeError):
        select_input_device(linux_device(default=-1))


@pytest.mark.parametrize("name", ["Microphone C920", "webcam", "RDPSink.monitor"])
def test_linux_never_silently_uses_camera_or_monitor(monkeypatch, name):
    monkeypatch.setattr("companionbot_behavior.core.audio.sys.platform", "linux")
    with pytest.raises(RuntimeError):
        select_input_device(linux_device(name))
    assert select_input_device(linux_device(name), "0")["selection"] == "override"


def test_windows_name_policy_is_preserved(monkeypatch):
    monkeypatch.setattr("companionbot_behavior.core.audio.sys.platform", "win32")
    with pytest.raises(RuntimeError):
        select_input_device(linux_device())


def speech_processes(monkeypatch, pulse=True):
    from companionbot_behavior.core import audio
    processes = []
    class Pipe:
        def __init__(self):
            self.data, self.closed = b"", False
        def write(self, data):
            self.data += data
        def close(self):
            self.closed = True
    class Process:
        def __init__(self, command, **kwargs):
            self.command, self.options = command, kwargs
            self.stdin, self.stdout, self.code = Pipe(), Pipe(), None
            self.terminated = False
            processes.append(self)
        def poll(self):
            return self.code
        def terminate(self):
            self.terminated, self.code = True, -15
        def wait(self, timeout):
            return self.code
    monkeypatch.setattr(audio.shutil, "which", lambda name: "/fixture/" + name)
    monkeypatch.setattr(audio.subprocess, "Popen", Process)
    if pulse:
        monkeypatch.setenv("PULSE_SERVER", "fixture")
    else:
        monkeypatch.delenv("PULSE_SERVER", raising=False)
    return audio._LinuxSpeech(), processes


def test_linux_speech_cancels_both_pulse_pipeline_children(monkeypatch):
    engine, processes = speech_processes(monkeypatch)
    text = "取消检查 $(must_remain_text)"
    engine.say(text)
    assert len(processes) == 2 and "--stdout" in processes[0].command
    assert processes[0].stdin.data == text.encode("utf-8") and processes[0].stdin.closed
    assert all("shell" not in p.options for p in processes)
    assert engine.is_busy()
    engine.stop()
    assert all(p.terminated for p in processes)
    assert not engine.is_busy()


def test_linux_speech_child_failure_is_not_success(monkeypatch):
    engine, processes = speech_processes(monkeypatch)
    engine.say("诊断")
    processes[0].code = 1
    with pytest.raises(RuntimeError, match="TTS process failed"):
        engine.is_busy()
    engine.close()
    assert processes[1].terminated


def test_linux_without_pulse_retains_native_audio_backend(monkeypatch):
    engine, processes = speech_processes(monkeypatch, pulse=False)
    engine.say("诊断")
    assert len(processes) == 1 and "--stdout" not in processes[0].command
    assert engine.voice_info["output"] == "espeak_native_audio"
    processes[0].code = 0
    assert not engine.is_busy()
    engine.close()
