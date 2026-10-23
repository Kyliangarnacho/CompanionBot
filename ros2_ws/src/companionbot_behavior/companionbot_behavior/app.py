"""Sole formal interaction: existing Stage 7 source, local audio and Stage 8 Agent."""
from companionbot_contracts.paths import repository_root

from pathlib import Path
import sys
import os
import threading
import json
import math
from uuid import uuid4

# OpenCV's documented Windows option avoids slow MSMF hardware-transform startup.
# Process-local and overridable; the Stage 7 standalone entry/config is untouched.
if sys.platform == "win32":
    os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")

ROOT = repository_root()


from companionbot_behavior.core.frames import Stage7FrameBuffer
from companionbot_behavior.core.config import STAGE_DIR
from companionbot_behavior.runtime.session import parser, run_cli
from companionbot_behavior.core.perception import MasterStatusBuffer
from companionbot_behavior.core.ui import DemoBridge, launch_panel


def camera_runner(args, frames, shutdown, enqueue, run_id, ready=None, masters=None, ui=None):
    from companionbot_perception.runtime.owner import run_perception
    ros = getattr(args, "ros_runtime", None)
    perception = ros.perception if ros else None
    if perception:
        # One capture run has one source epoch across local and ROS observers.
        frames.source_epoch = perception.epoch
        if masters:
            masters.source_epoch = perception.epoch
    def publish(frame):
        try:
            frames.publish(frame)
            if perception:
                # Reuse the immutable Agent snapshot; no extra RGB copy or DDS in capture.
                perception.publish_color(frames.latest().frame)
            if ready:
                ready.set()
        except Exception as error:
            # Failed publication cannot leave a previous snapshot apparently valid.
            frames.clear()
            if perception:
                perception.set_state("ERROR", type(error).__name__)
            raise

    def key(value):
        command = {32: "/interrupt", ord("p"): "/wait", ord("s"): "/sleep",
                   ord("q"): "/quit", 27: "/quit"}.get(value)
        if command:
            enqueue(command)

    def publish_master(result):
        if masters:
            masters.publish(result)
        if perception:
            perception.publish_master(result)
    def status(text):
        if perception and text.startswith("PERCEPTION_READY"):
            perception.set_state("RUNNING")
        if ui:
            ui.emit("perception", text)
    try:
        return run_perception(args, shutdown, run_id, on_color=publish, on_master=publish_master,
                              on_observation=perception.publish_observation if perception else None,
                              on_key=key, status=status,
                              preview_status=(lambda: ui.preview_line) if ui else None)
    finally:
        frames.clear()
        if masters:
            masters.clear()


def main(argv=None):
    os.chdir(ROOT)  # One absolute launch command also resolves existing model caches.
    cli = parser()
    cli.description = __doc__
    sources = cli.add_mutually_exclusive_group(required=True)
    sources.add_argument("--camera-device", type=lambda s: int(s) if s.isdecimal() else s)
    sources.add_argument("--video", type=Path, help="Replay through the unchanged Stage 7 full chain")
    sources.add_argument("--no-camera", action="store_true", help="Device-unavailable text mode; Master remains unavailable")
    cli.add_argument("--camera-backend", choices=["auto", "msmf", "dshow", "v4l2"],
                     default="msmf" if sys.platform == "win32" else "auto")
    cli.add_argument("--ros", action="store_true", help="Expose this same runtime through ROS 2 Jazzy")
    cli.add_argument("--exit-after", type=float, help="Finite diagnostic lifetime in host steady seconds")
    cli.add_argument("--preview", action="store_true", default=True, help="User-operated Stage 7 preview; Space interrupts speech")
    cli.add_argument("--no-preview", action="store_false", dest="preview")
    cli.add_argument("--headless", action="store_true", help="Console only; opens no GUI")
    cli.add_argument("--max-source-frames", type=int, help="Finite headless diagnostic run")
    cli.add_argument("--diagnose-audio", action="store_true", help="Read-only built-in mic/permission/PCM check; exits without a GUI/model")
    cli.set_defaults(tts=True, vosk_model=ROOT / ".venv/models/vosk-model-small-cn-0.22", asr_backend="sensevoice")
    cli.add_argument("--no-tts", action="store_false", dest="tts")
    cli.add_argument("--no-mic", action="store_const", const=None, dest="vosk_model")
    raw_args = list(sys.argv[1:] if argv is None else argv)
    if "--ros" in raw_args:
        from rclpy.utilities import remove_ros_args
        args = cli.parse_args(remove_ros_args(args=["interaction", *raw_args])[1:])
        args.ros_args = ["interaction", *raw_args]
    else:
        args = cli.parse_args(raw_args)
    if sys.platform != "win32" and args.camera_backend in ("msmf", "dshow"):
        cli.error("MSMF/DShow are Windows-only; use auto or v4l2 on Linux")
    if args.exit_after is not None and (not math.isfinite(args.exit_after) or args.exit_after <= 0):
        cli.error("--exit-after must be finite and positive")
    if args.video is not None and not args.video.is_file():
        cli.error("video file does not exist: " + str(args.video))
    if args.output_dir is None:
        args.output_dir = ROOT / "models/minisegway/runtime_architecture/results/sessions" / uuid4().hex
    args.output_dir = args.output_dir.resolve()
    if args.diagnose_audio:
        from companionbot_behavior.core.devices import microphone_diagnostics
        result = microphone_diagnostics(args.audio_device, samplerate=args.audio_samplerate)
        path = (args.output_dir or STAGE_DIR / "results") / ("microphone_" + uuid4().hex + ".json")
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
    masters.source_epoch = frames.source_epoch
    ui = None if args.headless else DemoBridge()
    def run():
        run_cli(args, frames=frames, camera_ready=ready, ui=ui, master_status=masters.status,
                camera_runner=(None if args.no_camera else lambda stop, enqueue, run_id:
                    camera_runner(args, frames, stop, enqueue, run_id, ready, masters, ui)))
    if ui:
        launch_panel(ui, run)
    else:
        run()


if __name__ == "__main__":
    main()
