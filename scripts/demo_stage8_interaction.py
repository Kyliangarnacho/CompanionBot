"""Real Stage 7 C920 + local microphone/TTS, with the unchanged Stage 8 Agent."""
from pathlib import Path
import sys
import os
import threading
import json
from uuid import uuid4

# OpenCV's documented Windows option avoids slow MSMF hardware-transform startup.
# Process-local and overridable; the Stage 7 standalone entry/config is untouched.
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from embodied_agent.frames import Stage7FrameBuffer
from embodied_agent.config import STAGE_DIR
from scripts import demo_yolo26n_depth as stage7
from scripts.run_stage8_agent import parser, run_cli
from embodied_agent.perception import MasterStatusBuffer
from embodied_agent.ui import DemoBridge, launch_panel


def camera_runner(args, frames, shutdown, enqueue, run_id, ready=None, masters=None, ui=None):
    def publish(frame):
        try:
            frames.publish(frame)
            if ready:
                ready.set()
        except Exception:
            # Failed publication cannot leave a previous snapshot apparently valid.
            frames.clear()
            raise

    def key(value):
        command = {32: "/interrupt", ord("p"): "/wait", ord("s"): "/sleep",
                   ord("q"): "/quit", 27: "/quit"}.get(value)
        if command:
            enqueue(command)

    argv = ["--mode", "full", "--camera-device", str(args.camera_device),
            "--opencv-backend", str({"auto": stage7.cv2.CAP_ANY,
                                     "msmf": stage7.cv2.CAP_MSMF,
                                     "dshow": stage7.cv2.CAP_DSHOW}[args.camera_backend]),
            "--output-dir", str(STAGE_DIR / "results"),
            "--run-name", run_id]
    if args.preview:
        argv += ["--preview"]
    if args.max_source_frames:
        argv += ["--max-source-frames", str(args.max_source_frames)]
    def publish_master(result):
        if masters:
            masters.publish(result)
    try:
        return stage7.main(argv, frame_observer=publish, shutdown_event=shutdown,
                           on_key=key, write_report=False, log_progress=False,
                           perception_observer=publish_master,
                           perception_status=(lambda text: ui.emit("perception", text)) if ui else None,
                           preview_status=(lambda: ui.preview_line) if ui else None)
    finally:
        frames.clear()
        if masters:
            masters.clear()


if __name__ == "__main__":
    os.chdir(ROOT)  # One absolute launch command also resolves existing model caches.
    cli = parser()
    cli.description = __doc__
    cli.add_argument("--camera-device", type=int, required=True)
    cli.add_argument("--camera-backend", choices=["auto", "msmf", "dshow"], default="msmf")
    cli.add_argument("--preview", action="store_true", default=True, help="User-operated Stage 7 preview; Space interrupts speech")
    cli.add_argument("--no-preview", action="store_false", dest="preview")
    cli.add_argument("--headless", action="store_true", help="Console only; opens no GUI")
    cli.add_argument("--max-source-frames", type=int, help="Finite headless diagnostic run")
    cli.add_argument("--diagnose-audio", action="store_true", help="Read-only built-in mic/permission/PCM check; exits without a GUI/model")
    cli.set_defaults(tts=True, vosk_model=ROOT / ".venv/models/vosk-model-small-cn-0.22", asr_backend="sensevoice")
    cli.add_argument("--no-tts", action="store_false", dest="tts")
    cli.add_argument("--no-mic", action="store_const", const=None, dest="vosk_model")
    args = cli.parse_args()
    if args.diagnose_audio:
        from embodied_agent.devices import microphone_diagnostics
        result = microphone_diagnostics(args.audio_device, samplerate=args.audio_samplerate)
        path = STAGE_DIR / "results" / ("microphone_" + uuid4().hex + ".json")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"selected": result.get("selected"), "pcm": result.get("pcm"),
                          "permissions": result["permissions"], "endpoints": result["endpoints"],
                          "error_type": result.get("error_type"), "result": str(path)}, ensure_ascii=False))
        sys.exit(1 if result.get("error_type") else 0)
    if args.headless:
        args.preview = False
    frames = Stage7FrameBuffer()
    ready, masters = threading.Event(), MasterStatusBuffer()
    ui = None if args.headless else DemoBridge()
    def run():
        run_cli(args, frames=frames, camera_ready=ready, ui=ui, master_status=masters.status,
                camera_runner=lambda stop, enqueue, run_id: camera_runner(args, frames, stop, enqueue, run_id, ready, masters, ui))
    if ui:
        launch_panel(ui, run)
    else:
        run()
