"""Explicit v1 simulation packet -> v2 embedded command boundary.

This does not generate plans, heartbeat liveness, arm requests or authorization.
Callers own their packets and deadlines; the C++ bridge and MCU revalidate both.
"""
from dataclasses import dataclass
import ctypes as C
import math
import struct
from companionbot_contracts.packets import (
    ReferenceBlockPacket, HeartbeatPacket, YawCommandPacket, SafetyCommandPacket,
    ResumeReferenceCommandPacket)
from .native import Frame, Parser, library

def uint32(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xffffffff:
        raise ValueError("embedded integer must fit uint32; reconnect before exhaustion")
    return value

@dataclass(frozen=True)
class EmbeddedFrame:
    kind: int
    session_id: int
    sequence_id: int
    control_tick: int
    payload: bytes = b""
    host_deadline_monotonic_ns: int = 0
    def __post_init__(self):
        for value in (self.session_id, self.sequence_id, self.control_tick): uint32(value)
        if self.session_id == 0 or not 0 <= self.kind <= 255 or len(self.payload) > 672:
            raise ValueError("invalid embedded frame")
        if self.host_deadline_monotonic_ns < 0:
            raise ValueError("host monotonic deadline must be nonnegative")
    def c_frame(self):
        frame = Frame(); frame.type = self.kind; frame.session = self.session_id
        frame.sequence = self.sequence_id; frame.tick = self.control_tick; frame.length = len(self.payload)
        frame.payload[:len(self.payload)] = self.payload
        return frame
    def to_wire(self):
        frame, data = self.c_frame(), (C.c_uint8 * 696)()
        length = library().cb_frame_encode(C.byref(frame), data, len(data))
        if not length: raise ValueError("could not encode embedded frame")
        return bytes(data[:length])
    def to_ros(self):
        from companionbot_interfaces.msg import McuFrame
        from companionbot_contracts.stamps import HOST_CLOCK_ID
        msg = McuFrame(); msg.kind = self.kind; msg.session_id = self.session_id
        msg.sequence_id = self.sequence_id; msg.control_tick = self.control_tick
        msg.host_deadline_monotonic_ns = self.host_deadline_monotonic_ns
        msg.host_clock_id = HOST_CLOCK_ID
        msg.payload = list(self.payload)
        return msg

class EmbeddedParser:
    def __init__(self): self.state = Parser()
    def feed(self, data):
        frames = []; frame = Frame()
        for byte in data:
            if library().cb_parser_feed(C.byref(self.state), byte, C.byref(frame)) == 1:
                frames.append(EmbeddedFrame(frame.type, frame.session, frame.sequence,
                                           frame.tick, bytes(frame.payload[:frame.length])))
        return frames

def command_from_packet(channel, payload, *, session_id, valid_until_tick,
                        host_deadline_monotonic_ns, safety_reason_code=1):
    classes = {"reference": ReferenceBlockPacket, "heartbeat": HeartbeatPacket,
        "yaw": YawCommandPacket, "safety": SafetyCommandPacket, "resume": ResumeReferenceCommandPacket}
    if channel not in classes: raise ValueError("unsupported embedded TX channel")
    packet = classes[channel].from_bytes(payload)
    if channel == "reference":
        kind, body = 1, packet.to_bytes()
        if packet.sample_count != 25 or abs(packet.dt_s - .002) > 1e-12:
            raise ValueError("embedded V1 uses exactly 25 samples at 2 ms")
    elif channel == "heartbeat": kind, body = 2, b""
    elif channel == "yaw":
        # The old JSON roundtrip preserved binary64 yaw; there is no measured
        # bandwidth need to reduce its precision on the hardware link.
        kind, body = 3, struct.pack("<d", packet.yaw_rate_target_rad_s)
    elif channel == "safety":
        if not 1 <= safety_reason_code <= 65535: raise ValueError("invalid safety reason code")
        kind, body = 4, struct.pack("<BH", int(packet.action == "LONGITUDINAL_STOP"), safety_reason_code)
    else:
        kind, body = 5, struct.pack("<IB", uint32(packet.reference_epoch), int(packet.authority == "LONGITUDINAL"))
    return EmbeddedFrame(kind, session_id, packet.sequence_id, valid_until_tick, body,
                         host_deadline_monotonic_ns)

def decode_status(frame):
    if frame.kind != 128 or len(frame.payload) != 144: raise ValueError("expected v2 MCU status")
    integers = struct.unpack_from("<8I", frame.payload)
    values = struct.unpack_from("<20f", frame.payload, 32)
    return {"session_id": frame.session_id, "control_tick": frame.control_tick,
        "reference_epoch": integers[0], "safety_state": integers[1], "reason": integers[2],
        "faults": integers[3], "last_rejection": integers[4], "rejections": integers[5],
        "deadline_overruns": integers[6], "actual_valid": bool(integers[7] & 1),
        "armed": bool(integers[7] & 2), "estimator_valid": bool(integers[7] & 4),
        "sum_clipped": bool(integers[7] & 8), "differential_clipped": bool(integers[7] & 16),
        "allocator_guard_clipped": bool(integers[7] & 32),
        "estimated_state": values[:6], "reference": values[6:12],
        "sum_requested_nm": values[12], "sum_limited_nm": values[13],
        "diff_requested_nm": values[14], "diff_used_nm": values[15],
        "allocated_wheels_nm": values[16:18], "actual_wheels_nm": values[18:20],
        "rejection_reason_counts": dict(zip(("ok", "invalid", "epoch_or_session", "stale",
            "capacity", "sequence", "underrun", "state"), struct.unpack_from("<8I", frame.payload, 112)))}
