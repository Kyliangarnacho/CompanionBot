"""Runs the sole formal Demo via its ROS wrapper. Offline/device-free by default."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from companionbot_contracts.paths import repository_root
from companionbot_bringup.processes import exit_with_runtime


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        raw = value(name).lower()
        if raw not in ("true", "false"):
            raise ValueError(name + " must be true or false")
        return raw == "true"
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
    if value("agent_config"):
        args += ["--config", value("agent_config")]
    args += ["--navigation-mode", value("navigation_mode")]
    for name, flag in (("navigation_bindings", "--navigation-bindings"), ("catalog_file", "--catalog-file")):
        if value(name):
            args += [flag, value(name)]
    if value("max_source_frames"):
        args += ["--max-source-frames", value("max_source_frames")]
    interaction = Node(package="companionbot_behavior", executable="interaction", output="screen", arguments=args,
                       namespace=value("namespace"), parameters=[value("params_file")],
                       sigterm_timeout="80", sigkill_timeout="5")
    nodes = [interaction, RegisterEventHandler(OnProcessExit(target_action=interaction, on_exit=exit_with_runtime))]
    if enabled("provisional_camera_tf"):
        camera_tf = Node(package="companionbot_perception", executable="camera_tf", output="screen",
                         namespace=value("namespace"))
        nodes += [camera_tf, RegisterEventHandler(OnProcessExit(target_action=camera_tf, on_exit=exit_with_runtime))]
    return nodes


def generate_launch_description():
    from pathlib import Path
    from uuid import uuid4
    root = repository_root()
    defaults = {"no_camera": "true", "video": "", "camera_device": "0", "camera_backend": "auto",
                "fake": "true", "headless": "true", "no_mic": "true", "no_tts": "true",
                "audio_device": "", "exit_after": "", "provisional_camera_tf": "false",
                "namespace": "companionbot", "agent_config": "", "max_source_frames": "",
                "navigation_mode": "fake", "navigation_bindings": "", "catalog_file": "",
                "params_file": str(Path(get_package_share_directory("companionbot_bringup")) / "config/interaction.yaml"),
                "output_dir": str(root / "models/minisegway/runtime_architecture/results/sessions" / uuid4().hex)}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=start)])
