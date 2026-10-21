"""ROS wire contracts; run after colcon build. No device/GUI use."""
from types import SimpleNamespace
import numpy as np
import pytest
from rclpy.serialization import serialize_message, deserialize_message
from companionbot_interfaces.msg import ColorFrame, PiMcuPacket
from companionbot_interfaces.srv import UserInput
from companionbot_ros.contracts import InputGuard, source_stamp
from companionbot_ros.interaction_bridge import InteractionBridge
from companionbot_ros.packets import encode_packet, decode_packet
from perception.camera import ColorFrame as CoreColorFrame
from control.pi_mcu_packets import (ReferenceBlockPacket, RobotStatePacket, YawCommandPacket,
    HeartbeatPacket, SafetyCommandPacket, SafetyStatusPacket, TargetObservationPacket,
    ResumeReferenceCommandPacket)


def input_stamp(seq=1, t=10.0):
    return source_stamp("test:text", "epoch-a", seq, t, time_semantics="text_submitted")


def test_unmapped_device_clock_never_inherits_host_clock_identity():
    from companionbot_ros.contracts import HOST_CLOCK_ID
    stamp = source_stamp("mcu", "epoch", 1, .2, clock_domain="mcu_ticks",
                         produced_at_s=10., produced_clock_domain="host_perf_counter")
    assert stamp.clock_id == ""
    assert stamp.received_clock_id == stamp.produced_clock_id == HOST_CLOCK_ID
    assert stamp.source_time_s == .2 and stamp.produced_at_s == 10.


def test_freshness_order_and_epoch_are_not_ros_time():
    guard = InputGuard(clock=lambda: 10.1)
    stamp = input_stamp()
    assert guard.accept(stamp) is None
    assert guard.accept(stamp) == "duplicate_or_out_of_order"
    stamp = input_stamp(2, 10.05)
    stamp.clock_domain = "ros_time"
    assert guard.accept(stamp) == "unmapped_clock_or_time_semantics"
    stamp = input_stamp(2, 10.05)
    stamp.source_epoch = "restart"
    assert guard.accept(stamp) == "source_restart_requires_rebind"
    assert guard.accept(input_stamp(2, 10.05)) is None


@pytest.mark.parametrize("mutate,reason", [
    (lambda m: setattr(m, "clock_id", "another-machine"), "unmapped_clock_or_time_semantics"),
    (lambda m: setattr(m, "source_time_s", 12.0), "stale_or_future_input"),
    (lambda m: setattr(m, "source_time_s", 8.0), "stale_or_future_input"),
    (lambda m: setattr(m, "source_time_s", float("nan")), "stale_or_future_input"),
    (lambda m: setattr(m, "has_sequence", False), "invalid_source_contract")])
def test_rejected_input_does_not_poison_watermark(mutate, reason):
    guard = InputGuard(clock=lambda: 10.1)
    stamp = input_stamp()
    mutate(stamp)
    assert guard.accept(stamp) == reason
    assert guard.accept(input_stamp()) is None


def test_ros_input_only_queues_and_cannot_bypass_session():
    queued = []
    bridge = SimpleNamespace(guard=InputGuard(clock=lambda: 10.1), enqueue=queued.append)
    request = UserInput.Request(meta=input_stamp(), text="你好小柒")
    response = InteractionBridge._input(bridge, request, UserInput.Response())
    assert response.queued and "not_execution_ack" in response.reason and queued == ["你好小柒"]
    assert not InteractionBridge._input(bridge, request, UserInput.Response()).queued


def test_noncontiguous_bgr_wire_preserves_source_and_separates_ros_header():
    saved = []
    bridge = SimpleNamespace(epoch="camera-epoch", color=SimpleNamespace(publish=saved.append),
                             _header=lambda header, frame: setattr(header, "frame_id", frame))
    image = np.arange(6*8*3, dtype=np.uint8).reshape(6,8,3)[:,::2,:]
    frame = CoreColorFrame(image, 7, "/dev/video/by-id/c920", 4, 6, 123.45)
    InteractionBridge.publish_color(bridge, frame)
    wire = deserialize_message(serialize_message(saved[0]), ColorFrame)
    restored = np.frombuffer(bytes(wire.image.data), dtype=np.uint8).reshape(6,4,3)
    assert np.array_equal(image, restored)
    assert wire.image.step == 12 and wire.image.encoding == "bgr8" and wire.image.is_bigendian == 0
    assert wire.meta.source_time_s == 123.45 and wire.meta.sequence == 7
    assert wire.meta.time_semantics == "host_read_complete" and not wire.meta.has_produced_time
    assert wire.image.header.frame_id == "camera_optical_frame"
    assert wire.image.header.stamp.sec == 0  # independent of source perf_counter


def packet_cases():
    return [("reference", ReferenceBlockPacket.from_rows(reference_epoch=2,sequence_id=7,
                start_control_tick=50,generated_time_s=.08,dt_s=.002,yaw_rate_target_rad_s=0,
                rows=[(.1,.2,.3,.4,.5,.6)])),
            ("state", RobotStatePacket(1,7,.08,.2,.3,.4,"NORMAL")),
            ("yaw", YawCommandPacket(1,7,.08,.2)), ("heartbeat", HeartbeatPacket(1,7,.08)),
            ("safety", SafetyCommandPacket(1,7,.08,"HARD_STOP","fixture")),
            ("status", SafetyStatusPacket(1,7,.08,"NORMAL","fixture",0)),
            ("observation", TargetObservationPacket(1,7,.08,1.,.2,True,.9)),
            ("resume", ResumeReferenceCommandPacket(1,7,.08,2,"HARD"))]


@pytest.mark.parametrize("channel,packet", packet_cases())
def test_existing_pi_mcu_bytes_roundtrip_without_retiming(channel, packet):
    message = encode_packet(channel,packet,source_id="fixture",source_epoch="epoch",source_time_s=.08,clock_id="sim:epoch")
    wire = deserialize_message(serialize_message(message), PiMcuPacket)
    assert bytes(wire.payload) == packet.to_bytes()
    restored = decode_packet(wire,expected_clock_id="sim:epoch",expected_source_epoch="epoch")
    assert restored.to_bytes() == packet.to_bytes()
    assert wire.meta.source_time_s == .08 and wire.meta.clock_domain == "mujoco_sim"
    with pytest.raises(ValueError,match="clock_or_epoch"):
        decode_packet(wire,expected_clock_id="host:other",expected_source_epoch="epoch")
    wire.meta.sequence = 6
    with pytest.raises(ValueError,match="sequence"):
        decode_packet(wire,expected_clock_id="sim:epoch",expected_source_epoch="epoch")
