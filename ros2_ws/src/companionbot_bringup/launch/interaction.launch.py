"""Runs the sole formal Demo via its ROS wrapper. Offline/device-free by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return value(name).lower() == "true"
    args = ["--output-dir", value("output_dir")]
    if enabled("no_camera"):
        args.append("--no-camera")
    elif value("video"):
        args += ["--video", value("video")]
    else:
        args += ["--camera-device", value("camera_device"), "--camera-backend", value("camera_backend")]
    for name, flag in (("fake", "--fake"), ("headless", "--headless"),
                       ("no_mic", "--no-mic"), ("no_tts", "--no-tts")):
        if enabled(name):
            args.append(flag)
    if value("audio_device"):
        args += ["--audio-device", value("audio_device")]
    if value("exit_after"):
        args += ["--exit-after", value("exit_after")]
    interaction = Node(package="companionbot_behavior", executable="interaction", output="screen", arguments=args)
    nodes = [interaction, RegisterEventHandler(OnProcessExit(target_action=interaction,
             on_exit=[EmitEvent(event=Shutdown(reason="formal interaction ended"))]))]
    if enabled("provisional_camera_tf"):
        nodes.append(Node(package="companionbot_perception", executable="camera_tf", output="screen"))
    return nodes


def generate_launch_description():
    import os
    from pathlib import Path
    root = Path(os.environ["COMPANIONBOT_ROOT"])
    defaults = {"no_camera": "true", "video": "", "camera_device": "0", "camera_backend": "auto",
                "fake": "true", "headless": "true", "no_mic": "true", "no_tts": "true",
                "audio_device": "", "exit_after": "", "provisional_camera_tf": "false",
                "output_dir": str(root / "models/minisegway/stage9/results/architecture/interaction")}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=start)])
