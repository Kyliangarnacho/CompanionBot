"""Existing Stage 9 ROS contract regression, relocated to its owner."""
import pytest
from rclpy.serialization import serialize_message, deserialize_message
from companionbot_interfaces.msg import PiMcuPacket
from companionbot_control.ros.packets import encode_packet, decode_packet
from companionbot_contracts.packets import (ReferenceBlockPacket, RobotStatePacket, YawCommandPacket, HeartbeatPacket, SafetyCommandPacket, SafetyStatusPacket, TargetObservationPacket, ResumeReferenceCommandPacket)

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


