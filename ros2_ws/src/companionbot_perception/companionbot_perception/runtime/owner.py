"""Formal app adapter to the existing full perception session, without ROS/UI ownership."""
from . import session


def run_perception(args, shutdown, run_id, *, on_color, on_master=None, on_observation=None,
                   on_key=None, status=None, preview_status=None):
    source = (["--video", str(args.video), "--realtime-playback"] if args.video
              else ["--camera-device", str(args.camera_device)])
    backend = {"auto": session.cv2.CAP_ANY, "msmf": session.cv2.CAP_MSMF,
               "dshow": session.cv2.CAP_DSHOW, "v4l2": session.cv2.CAP_V4L2}[args.camera_backend]
    argv = ["--mode", "full", *source, "--opencv-backend", str(backend),
            "--output-dir", str(args.output_dir), "--run-name", run_id]
    if args.preview:
        argv.append("--preview")
    if args.max_source_frames:
        argv += ["--max-source-frames", str(args.max_source_frames)]
    try:
        return session.main(argv, frame_observer=on_color, shutdown_event=shutdown,
                            on_key=on_key, write_report=False, log_progress=False,
                            perception_observer=on_master, robot_observation_observer=on_observation,
                            perception_status=status, preview_status=preview_status)
    except SystemExit as error:
        # argparse is a CLI concern, never a process-exiting exception in a worker Task.
        raise ValueError("perception_arguments_rejected:" + str(error.code)) from None
