"""Lossless adapter of Stage 6 codecs. It does not dispatch to an actuator."""
from companionbot_interfaces.msg import PiMcuPacket
from companionbot_contracts.packets import ReferenceBlockPacket, YawCommandPacket, HeartbeatPacket, SafetyCommandPacket, SafetyStatusPacket, RobotStatePacket, TargetObservationPacket, ResumeReferenceCommandPacket
from companionbot_contracts.stamps import source_stamp


CODECS = {"reference": ("mcu", ReferenceBlockPacket), "yaw": ("mcu", YawCommandPacket),
          "heartbeat": ("mcu", HeartbeatPacket), "safety": ("mcu", SafetyCommandPacket),
          "status": ("pi", SafetyStatusPacket), "state": ("pi", RobotStatePacket),
          "observation": ("pi", TargetObservationPacket), "resume": ("mcu", ResumeReferenceCommandPacket)}


def encode_packet(channel, packet, *, source_id, source_epoch, source_time_s, clock_id):
    direction, codec = CODECS[channel]
    if not isinstance(packet, codec):
        raise ValueError("packet_channel_mismatch")
    message = PiMcuPacket()
    message.meta = source_stamp(source_id, source_epoch, packet.sequence_id, source_time_s,
                               clock_domain="mujoco_sim", clock_id=clock_id,
                               time_semantics="sim_packet_generated")
    message.direction, message.channel = direction, channel
    message.payload = packet.to_bytes()
    message.hardware_execution_ready = False
    return message


def decode_packet(message, *, expected_clock_id, expected_source_epoch):
    if message.channel not in CODECS or len(message.payload) > 65536:
        raise ValueError("invalid_packet_channel_or_size")
    direction, codec = CODECS[message.channel]
    meta = message.meta
    if message.direction != direction or message.hardware_execution_ready:
        raise ValueError("invalid_packet_direction_or_authority")
    if (meta.contract_version != 2 or meta.clock_domain != "mujoco_sim"
            or meta.clock_id != expected_clock_id or meta.source_epoch != expected_source_epoch
            or meta.time_semantics != "sim_packet_generated" or not meta.has_sequence):
        raise ValueError("unmapped_packet_clock_or_epoch")
    packet = codec.from_bytes(bytes(message.payload))
    if packet.sequence_id != meta.sequence:
        raise ValueError("packet_sequence_mismatch")
    return packet
