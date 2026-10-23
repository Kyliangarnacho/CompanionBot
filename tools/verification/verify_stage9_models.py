"""Hardware-free Linux inference/fixture diagnostics, not accuracy certification."""
from companionbot_contracts.paths import repository_root

import json
import argparse
from pathlib import Path
import sys
import time
import wave

ROOT = repository_root()

OUTPUT: Path  # Assigned from the required --output-dir before any execution.


def main():
    global OUTPUT
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir", type=Path, required=True)
    OUTPUT = cli.parse_args().output_dir.resolve()
    import cv2
    import numpy as np
    from companionbot_perception.core.camera import ColorFrame
    from companionbot_perception.core.person_detector import PersonDetector, PersonDetectorConfig
    from companionbot_perception.core.person_reid import PersonReIdentifier, TorchReIDOSNetBackend, default_osnet_weights_path
    from companionbot_perception.core.depth import YoloDepthAdapter, YoloDepthConfig
    from companionbot_behavior.core.speech_recognition import SenseVoiceInput, SpeechSegment
    from vosk import Model, KaldiRecognizer
    OUTPUT.mkdir(parents=True, exist_ok=True)
    report = {"kind": "software_fixture_only", "real_devices_tested": False}
    original = cv2.imread(str(ROOT / "models/minisegway/stage7/stage7_1/results/person_detection_preview.jpg"))
    # Deliberate fixture construction, not calibration/image-coordinate scaling.
    # The canvas is not an image from the C920; no physical distance is assessed.
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
    scaled = cv2.resize(original, (432, 576))
    canvas[72:648, 424:856] = scaled
    frame = ColorFrame(canvas, 1, "fixture:stage9_canvas", 1280, 720, time.perf_counter())
    detections = PersonDetector(PersonDetectorConfig(model_path=ROOT / "yolo26n.pt", device="cpu")).process(frame)
    assert detections
    assert all(d.source_sequence_id == frame.sequence_id and d.source_time_s == frame.host_receive_time_s for d in detections)
    report["detector"] = {"persons": len(detections), "source_metadata_preserved": True}
    reid = PersonReIdentifier(TorchReIDOSNetBackend(default_osnet_weights_path()))
    feature = reid.embed(canvas, detections[0].bbox_xyxy_px, source_id=frame.source_id,
                         source_sequence_id=frame.sequence_id, track_id=1)
    assert feature.dimension == 512 and np.isfinite(feature.vector).all()
    assert abs(float(np.linalg.norm(feature.vector)) - 1) < 1e-5
    report["osnet"] = {"dimension": feature.dimension, "unit_norm": float(np.linalg.norm(feature.vector))}
    depth = YoloDepthAdapter(YoloDepthConfig(model_path=ROOT / ".venv/models/yolo26n-depth.pt", device="cpu")).process(frame)
    assert depth.depth_m.shape == (720, 1280) and np.any(np.isfinite(depth.depth_m))
    report["depth"] = {"shape": list(depth.depth_m.shape), "finite_pixels": int(np.isfinite(depth.depth_m).sum()),
                        "distance_accuracy_verified": False}
    video = OUTPUT / "canvas_replay.avi"
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 10., (1280, 720))
    assert writer.isOpened()
    try:
        for _ in range(120):
            writer.write(canvas)
    finally:
        writer.release()
    report["replay"] = {"path": str(video.relative_to(ROOT)), "frames": 120, "fps": 10,
                        "source": "resized annotated historical preview on synthetic canvas; no camera calibration claim"}
    model_dir = ROOT / ".venv/models/sensevoice"
    sense = SenseVoiceInput(model_dir)
    with wave.open(str(model_dir / "upstream_zh.wav"), "rb") as wav:
        assert wav.getframerate() == 16000 and wav.getnchannels() == 1 and wav.getsampwidth() == 2
        samples = np.frombuffer(wav.readframes(wav.getnframes()), dtype="<i2").astype(np.float32) / 32768
    text = sense.decode(SpeechSegment(samples, len(samples)/16000, float(np.sqrt(np.mean(samples**2)))*32768, 0.))
    assert text
    assert sense.feed(np.zeros(32000, dtype="<i2").tobytes()) == []
    report["sensevoice"] = {"upstream_fixture_text": text, "silero_silence_rejected": True}
    wake = KaldiRecognizer(Model(str(ROOT / ".venv/models/vosk-model-small-cn-0.22")), 16000)
    wake.AcceptWaveform(np.zeros(32000, dtype="<i2").tobytes())
    assert json.loads(wake.FinalResult()).get("text", "") == ""
    report["vosk"] = {"model_loaded": True, "silence_text_empty": True, "real_wake_verified": False}
    report["passed"] = True
    (OUTPUT / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
