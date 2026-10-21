"""Existing Stage 9 ROS contract regression, relocated to its owner."""
from types import SimpleNamespace
import numpy as np
from rclpy.serialization import serialize_message, deserialize_message
from companionbot_interfaces.msg import ColorFrame
from companionbot_perception.core.camera import ColorFrame as CoreColorFrame
from companionbot_perception.ros.publishers import PerceptionPublishers

def test_noncontiguous_bgr_wire_preserves_source_and_separates_ros_header():
    saved = []
    bridge = SimpleNamespace(epoch="camera-epoch", color=SimpleNamespace(publish=saved.append),
                             _header=lambda header, frame: setattr(header, "frame_id", frame))
    image = np.arange(6*8*3, dtype=np.uint8).reshape(6,8,3)[:,::2,:]
    frame = CoreColorFrame(image, 7, "/dev/video/by-id/c920", 4, 6, 123.45)
    PerceptionPublishers.publish_color(bridge, frame)
    wire = deserialize_message(serialize_message(saved[0]), ColorFrame)
    restored = np.frombuffer(bytes(wire.image.data), dtype=np.uint8).reshape(6,4,3)
    assert np.array_equal(image, restored)
    assert wire.image.step == 12 and wire.image.encoding == "bgr8" and wire.image.is_bigendian == 0
    assert wire.meta.source_time_s == 123.45 and wire.meta.sequence == 7
    assert wire.meta.time_semantics == "host_read_complete" and not wire.meta.has_produced_time
    assert wire.image.header.frame_id == "camera_optical_frame"
    assert wire.image.header.stamp.sec == 0  # independent of source perf_counter


