"""Diagnostic only: synthetic replay -> original Stage 7 -> typed ROS DDS.

Explicitly selects fixture track 1 using the existing diagnostic option. This
is neither a second interactive Demo nor real-person/physical-distance proof.
"""
from companionbot_contracts.paths import repository_root

import json
import argparse
from collections import deque
import os
from pathlib import Path
import sys
import threading
import time

ROOT = repository_root()



def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir", type=Path, required=True)
    cli.add_argument("--video", type=Path, default=ROOT / "models/minisegway/stage9/results/architecture/models/canvas_replay.avi")
    args = cli.parse_args()
    os.environ["ROS_DOMAIN_ID"] = "98"
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from companionbot_interfaces.msg import ColorFrame, MasterStatus, RelativeObservation
    from companionbot_perception.ros.node import PerceptionNode
    import companionbot_perception.runtime.session as stage7
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    context = Context()
    rclpy.init(context=context)
    bridge = PerceptionNode(context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(bridge)
    client = Node("perception_fixture_subscriber", context=context)
    executor.add_node(client)
    colors, masters, observations = (deque(maxlen=512) for _ in range(3))
    client.create_subscription(ColorFrame, "/companionbot/camera/color", lambda m: colors.append(
        {"meta": m.meta, "width": m.image.width, "height": m.image.height}), qos_profile_sensor_data)
    client.create_subscription(MasterStatus, "/companionbot/perception/master", lambda m: masters.append(m), qos_profile_sensor_data)
    client.create_subscription(RelativeObservation, "/companionbot/perception/relative", lambda m: observations.append(m), qos_profile_sensor_data)
    result, errors, stop = [], [], threading.Event()
    def run():
        try:
            result.append(stage7.main(["--mode", "full", "--video", str(args.video.resolve()),
                "--realtime-playback", "--select-track-id", "1", "--device", "cpu",
                "--output-dir", str(output), "--run-name", "fixture"],
                frame_observer=bridge.publish_color, perception_observer=bridge.publish_master,
                robot_observation_observer=bridge.publish_observation, shutdown_event=stop,
                perception_status=lambda text: bridge.set_state("RUNNING") if text.startswith("PERCEPTION_READY") else None,
                log_progress=False, write_report=False))
        except Exception as error:
            errors.append(error)
    worker = threading.Thread(target=run, name="stage9-fixture")
    worker.start()
    try:
        deadline = time.perf_counter() + 80
        while worker.is_alive() and time.perf_counter() < deadline:
            executor.spin_once(timeout_sec=.01)
        stop.set()
        worker.join(timeout=35)
        assert not worker.is_alive(), "fixture workers did not stop"
        if errors:
            raise errors[0]
        assert colors and masters and observations
        assert any(m.state == "LOCKED" and m.visible for m in masters)
        assert all(m["meta"].time_semantics == "host_read_complete" and m["width"] == 1280 and m["height"] == 720 for m in colors)
        assert all(m.meta.clock_domain == "host_perf_counter" and m.meta.has_produced_time
                   and m.meta.produced_at_s >= m.meta.source_time_s and not m.stage6_connected for m in observations)
        assert all(m.header.frame_id == "robot_leveled_axle" and m.extrinsic_provisional
                   and m.pitch_provisional and m.depth_convention_provisional for m in observations)
        summary = {"colors_received": len(colors), "master_updates_received": len(masters),
                   "relative_observations_received": len(observations),
                   "source_time_s": observations[0].meta.source_time_s,
                   "ros_publication_sec": observations[0].header.stamp.sec,
                   "stage6_connected": False, "physical_distance_verified": False,
                   "fixture_track_selected": 1, "passed": True}
        summary["adapter"] = bridge.snapshot()
        (output / "summary.json").write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary))
    finally:
        stop.set()
        worker.join(timeout=35)
        bridge.set_state("STOPPING")
        executor.shutdown()
        client.destroy_node()
        bridge.destroy_node()
        context.try_shutdown()


if __name__ == "__main__":
    main()
