"""Numerical and authority regression of the C core against frozen Python.

Requires the native library from build_ros.sh; no devices or simulator oracle.
"""
from dataclasses import asdict
import ctypes as C
import math
import struct
import numpy as np
import pytest
from companionbot_control.runtime import native as n
from companionbot_control.core.safety_brake import SafetyBrakeConfig, SafetyBrakePrimitive, SafetyReference
from companionbot_control.core.yaw_control import GyroRelativeYawEstimator, ParallelYawPD, allocate_longitudinal_priority
from companionbot_control.estimation.longitudinal_estimation import (
    LongitudinalEstimator, LongitudinalAccelerationCompensationConfig, load_longitudinal_estimator_config)
from companionbot_control.sensors.encoder import load_encoder_profile
from companionbot_control.runtime.mcu import MCUReferenceBlockReceiver, MCUSafetyArbiter
from companionbot_control.runtime.yaw_reference import MCULatestYawSource
from companionbot_contracts.packets import (
    ReferenceBlockPacket, HeartbeatPacket, SafetyCommandPacket, ResumeReferenceCommandPacket, YawCommandPacket)

@pytest.mark.parametrize("compensated", [False, True])
def test_estimator_frozen_sensor_trace(compensated):
    config, encoder = load_longitudinal_estimator_config(), load_encoder_profile()
    compensation = LongitudinalAccelerationCompensationConfig() if compensated else None
    py, c = [cls(config, encoder, 0.042, acceleration_compensation=compensation)
             for cls in (LongitudinalEstimator, n.NativeLongitudinalEstimator)]
    rng = np.random.default_rng(20261021)
    counts = np.zeros(2, dtype=np.int64)
    for tick in range(4000):
        angle = 0.12 * math.sin(tick * .009)
        accel = np.array([0, 9.81 * math.sin(angle), 9.81 * math.cos(angle)]) + rng.normal(0, .01, 3)
        gyro = rng.normal(0, .1, 3)
        counts += rng.integers(-3, 4, 2)
        kwargs = dict(measurement_age_s=(tick % 4) * .001,
                      allow_kinematic_extrapolation=tick % 13 != 0)
        a = (py.reset if tick == 0 else py.update)(accel, gyro, counts, **kwargs)
        b = (c.reset if tick == 0 else c.update)(accel, gyro, counts, **kwargs)
        for key, value in asdict(a).items():
            np.testing.assert_allclose(getattr(b, key), value, rtol=0, atol=3e-12, err_msg=f"tick={tick}, {key}")
        for key, value in py.acceleration_compensation_log_fields().items():
            np.testing.assert_allclose(c.acceleration_compensation_log_fields()[key], value,
                                       rtol=0, atol=3e-12, err_msg=key)

@pytest.mark.parametrize("velocity", [-.6, -.1, 0., .4, .6])
def test_brake_continuity_jerk_and_handoff(velocity):
    config = SafetyBrakeConfig(.5, .8, 1, .002)
    initial = SafetyReference(.4, velocity, .04, .03, -.1, .2)
    py, c = SafetyBrakePrimitive(config, initial), n.NativeBrake(config, initial)
    for tick in range(3000):
        a, b = py.step(), c.step()
        np.testing.assert_allclose(list(asdict(b).values()), list(asdict(a).values()), rtol=0, atol=1e-12)
        assert py.hidden_handoff_done == c.hidden_handoff_done
        assert py.velocity_settled == c.velocity_settled

def test_lqr_yaw_allocator_randomized():
    rng = np.random.default_rng(91)
    py, c = ParallelYawPD(.55, .20), n.NativeYawPD(.55, .20)
    for _ in range(2000):
        gain, error = rng.normal(size=(2, 4))
        assert n.feedback(gain, error) == pytest.approx(-float(gain @ error), abs=4e-15)
        args = rng.uniform(-8, 8, 4)
        np.testing.assert_allclose(list(asdict(c.command(*args)).values()), list(asdict(py.command(*args)).values()), atol=1e-14, rtol=0)
        args = (rng.uniform(-1.26, 1.26), rng.uniform(-2, 2), .63)
        assert n.allocate(*args) == allocate_longitudinal_priority(*args)
    with pytest.raises(ValueError): n.feedback([1]*4, [0, 0, math.nan, 0])
    with pytest.raises(ValueError): n.allocate(2, 0, .63)

def test_yaw_sample_time_and_stale_semantics():
    py, c = GyroRelativeYawEstimator(), n.NativeYawEstimator()
    assert py.reset(.1, 0, .003, allow_extrapolation=True) == c.reset(.1, 0, .003, allow_extrapolation=True)
    for tick in range(1, 1000):
        args = (.4 * math.sin(tick * .01), (tick // 3) * .006, .002)
        kwargs = dict(valid_new_sample=tick % 19 != 0, allow_extrapolation=tick % 29 != 0)
        assert py.update(*args, **kwargs) == c.update(*args, **kwargs)
    with pytest.raises(ValueError): c.update(.1, -1, 0, valid_new_sample=True, allow_extrapolation=True)

def test_encoder_conversion_preserves_frozen_operation_order():
    # Regression for the measured 50 ms governor event shift: factoring
    # (tau * counts) / cpr into counts * (tau / cpr) changed feedback roundoff.
    config, encoder = load_longitudinal_estimator_config(), load_encoder_profile()
    py = LongitudinalEstimator(config, encoder, .042)
    c = n.NativeLongitudinalEstimator(config, encoder, .042)
    for counts in ([101, -14729], [1000003, -48342], [12345, 54321]):
        for estimator in (py, c): estimator.reset([0, 0, 9.81], [0, 0, 0], counts)
        for step in range(5):
            current = [counts[0] + step, counts[1] - 2 * step]
            a, b = [estimator.update([0, 0, 9.81], [0, 0, 0], current) for estimator in (py, c)]
            np.testing.assert_array_equal(a.wheel_relative_angle_hat_rad, b.wheel_relative_angle_hat_rad)
            np.testing.assert_array_equal(a.wheel_relative_velocity_hat_rad_s, b.wheel_relative_velocity_hat_rad_s)

def make_pair():
    receiver = MCUReferenceBlockReceiver(dt_s=.002, block_samples=25)
    yaw = MCULatestYawSource(dt_s=.002, timeout_s=.2, decay_rate_rad_s2=.8)
    py = MCUSafetyArbiter(receiver, SafetyBrakeConfig(.5, .8, 1, .002),
        heartbeat_timeout_s=.15, starvation_margin_s=.012, nominal_pitch_rad=0,
        status_sink=lambda *_: None, yaw_source=yaw)
    c = n.NativeMCU(n.McuConfig(n.BrakeConfig(.002, .5, .8, 1), .15, .012, 0, .2, .8))
    return py, c

def block(epoch, sequence, start, position=0., velocity=.2):
    return ReferenceBlockPacket.from_rows(reference_epoch=epoch, sequence_id=sequence,
        start_control_tick=start, generated_time_s=0., dt_s=.002, yaw_rate_target_rad_s=0,
        rows=[(position + i * .002 * velocity, velocity, 0, .01, 0, .02) for i in range(25)])

@pytest.mark.parametrize("mode", ["longitudinal", "hard", "escalate", "heartbeat", "starvation"])
def test_mcu_packet_safety_and_resume_trace(mode):
    py, c = make_pair()
    seq = 0
    def receive(channel, packet, tick):
        payload = packet.to_bytes()
        py.receive(channel, payload, tick * .002)
        return c.receive(channel, payload, tick)
    receive("reference", block(0, 0, 0), 0)
    receive("reference", block(0, 1, 25, .01), 0)
    states = set()
    resumed = False
    maximum_delta = 0.
    for tick in range(2300):
        if tick % 20 == 0 and not (mode == "heartbeat" and 100 < tick < 1400):
            receive("heartbeat", HeartbeatPacket(1, tick, tick * .002), tick)
        if tick % 10 == 0:
            receive("yaw", YawCommandPacket(1, tick, tick * .002, .3), tick)
        if tick == 100 and mode in ("longitudinal", "hard", "escalate"):
            receive("safety", SafetyCommandPacket(1, 0, tick * .002,
                "HARD_STOP" if mode == "hard" else "LONGITUDINAL_STOP", "TEST"), tick)
        if tick == 150 and mode == "escalate":
            receive("safety", SafetyCommandPacket(1, 1, tick * .002, "HARD_STOP", "TEST"), tick)
        if tick == 1600 and py.state in ("SAFE_HOLD", "LONGITUDINAL_STOP"):
            receive("reference", block(py.receiver.reference_epoch, 0, tick + 10,
                py.last_applied.reference_state[0], 0), tick)
            receive("resume", ResumeReferenceCommandPacket(1, 0, tick * .002,
                py.receiver.reference_epoch, "LONGITUDINAL" if py.state == "LONGITUDINAL_STOP" else "HARD"), tick)
            resumed = True
        py.set_local_state(v_hat_m_s=0, pitch_hat_rad=0, pitch_rate_hat_rad_s=0)
        a, b = py.command(tick * .002), c.step(tick, 0, 0, 0)
        states.add(py.state)
        values = [a.reference_state[0], a.reference_state[1], a.reference_acceleration_m_s2,
                  a.reference_state[2], a.reference_state[3], a.u_ff_after_lifecycle_nm, a.yaw_rate_target_rad_s]
        actual = [b.reference.p, b.reference.v, b.reference.a, b.reference.theta,
                  b.reference.theta_dot, b.reference.u_ff, b.yaw_rate]
        delta = float(np.max(np.abs(np.asarray(values) - actual)))
        maximum_delta = max(maximum_delta, delta)
        assert delta < 1e-12, (tick, py.state, delta)
        assert ["NORMAL", "BRAKING", "LONGITUDINAL_STOP", "SAFE_HOLD"][b.safety_state] == py.state
        assert b.epoch == py.receiver.reference_epoch
        if py.state == "NORMAL" and py.receiver.next is None and not (mode == "starvation" and 100 < tick < 1600):
            seq += 1
            start = py.receiver.active.start_control_tick + 25
            receive("reference", block(py.receiver.reference_epoch, seq + 1, start,
                a.reference_state[0] + .002 * .2, .2), tick)
    assert "BRAKING" in states and "NORMAL" in states and resumed
    assert ("LONGITUDINAL_STOP" if mode == "longitudinal" else "SAFE_HOLD") in states

def test_mcu_rejections_and_missing_reference_fail_closed():
    _, c = make_pair()
    assert c.receive("reference", block(1, 0, 0).to_bytes(), 0) == 2
    assert c.receive("reference", block(0, 0, 0).to_bytes(), 1) == 3
    assert c.receive("reference", b"short", 0) == 1
    assert c.receive("heartbeat", HeartbeatPacket(1, 2, 0).to_bytes(), 0) == 0
    assert c.receive("heartbeat", HeartbeatPacket(1, 2, 0).to_bytes(), 0) == 5
    output = c.step(0, 0, 0, 0)
    assert output.safety_state == 1 and output.reason == 4 and output.epoch == 1
    assert c.receive("reference", block(0, 0, 25).to_bytes(), 0) == 2
    assert c.receive("resume", ResumeReferenceCommandPacket(1, 0, 0, 1).to_bytes(), 0) == 7

def test_wire_crc_cobs_and_session_deadline():
    import zlib
    lib = n.library()
    frame = n.Frame(); frame.type = 2; frame.session = 0x11223344; frame.sequence = 7; frame.tick = 25
    data = (C.c_uint8 * 696)()
    length = lib.cb_frame_encode(C.byref(frame), data, len(data))
    wire = bytes(data[:length])
    # Independent standard COBS decoder and Python zlib CRC, not a roundtrip alone.
    raw = bytearray(); index = 0
    while index < len(wire)-1:
        code = wire[index]; index += 1
        raw.extend(wire[index:index+code-1]); index += code-1
        if code != 255 and index < len(wire)-1: raw.append(0)
    assert raw[:16] == struct.pack("<BBHIII", 2, 2, 0, frame.session, 7, 25)
    assert struct.unpack("<I", raw[-4:])[0] == zlib.crc32(raw[:-4])
    parser, decoded = n.Parser(), n.Frame()
    results = [lib.cb_parser_feed(C.byref(parser), b, C.byref(decoded)) for b in wire]
    assert results[-1] == 1 and decoded.session == frame.session
    _, c = make_pair()
    assert lib.cb_mcu_dispatch(c.state, C.byref(decoded), 123, 0) == 2
    assert lib.cb_mcu_dispatch(c.state, C.byref(decoded), frame.session, 26) == 3
    assert lib.cb_mcu_dispatch(c.state, C.byref(decoded), frame.session, 0) == 0

def test_embedded_reference_and_yaw_end_to_end_dispatch():
    from companionbot_control.runtime.embedded_link import command_from_packet, EmbeddedParser
    _, c = make_pair()
    parser = EmbeddedParser()
    def dispatch(channel, packet):
        frame = command_from_packet(channel, packet.to_bytes(), session_id=77,
            valid_until_tick=100, host_deadline_monotonic_ns=1)
        decoded, = parser.feed(frame.to_wire())
        wire = decoded.c_frame()
        assert n.library().cb_mcu_dispatch(c.state, C.byref(wire), 77, 0) == 0
        return frame
    reference = block(0, 0, 0)
    frame = dispatch("reference", reference)
    assert len(frame.payload) == 640 and frame.payload == reference.to_bytes()
    dispatch("heartbeat", HeartbeatPacket(1, 0, 0))
    target = .3141592653589793
    yaw = dispatch("yaw", YawCommandPacket(1, 0, 0, target))
    assert struct.unpack("<d", yaw.payload)[0] == target
    output = c.step(0, 0, 0, 0)
    assert output.yaw_rate == target
    np.testing.assert_array_equal([output.reference.p, output.reference.v, output.reference.a,
        output.reference.theta, output.reference.theta_dot, output.reference.u_ff], reference.sample(0))

def test_status_keeps_requested_limited_actual_and_rejection_reasons_separate():
    from companionbot_control.runtime.embedded_link import EmbeddedFrame, decode_status
    integers = (7, 3, 2, 32, 5, 10, 0, 4 | 8 | 16)
    values = [0.] * 12 + [2., 1.26, .8, 0., .63, .63, math.nan, math.nan]
    histogram = (0, 2, 1, 3, 0, 4, 0, 0)
    status = decode_status(EmbeddedFrame(128, 77, 1, 100, struct.pack("<8I20f8I", *integers, *values, *histogram)))
    assert status["sum_requested_nm"] == 2
    assert status["sum_limited_nm"] == pytest.approx(1.26)
    assert status["actual_valid"] is False and all(math.isnan(v) for v in status["actual_wheels_nm"])
    assert status["estimator_valid"] and status["sum_clipped"] and status["differential_clipped"]
    assert status["allocator_guard_clipped"] is False
    assert status["rejection_reason_counts"]["sequence"] == 4
