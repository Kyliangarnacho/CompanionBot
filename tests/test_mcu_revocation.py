"""Accepted/rejected safety matrix and old-generation race parity.

These are host software MCU tests. Neither oracle reads plant truth.
"""
import numpy as np
import pytest

from companionbot_contracts.packets import (
    HeartbeatPacket, ReferenceBlockPacket, ResumeReferenceCommandPacket,
    SafetyCommandPacket, YawCommandPacket,
)
from companionbot_control.core.safety_brake import SafetyBrakeConfig
from companionbot_control.runtime import native as n
from companionbot_control.runtime.mcu import MCUReferenceBlockReceiver, MCUSafetyArbiter
from companionbot_control.runtime.yaw_reference import MCULatestYawSource


class Pair:
    def __init__(self):
        self.py = MCUSafetyArbiter(
            MCUReferenceBlockReceiver(dt_s=.002, block_samples=25),
            SafetyBrakeConfig(.5, .8, 1., .002), heartbeat_timeout_s=.15,
            starvation_margin_s=.012, nominal_pitch_rad=0.,
            status_sink=lambda *_: None,
            yaw_source=MCULatestYawSource(dt_s=.002, timeout_s=.2, decay_rate_rad_s2=.8),
        )
        self.native = n.NativeMCU(n.McuConfig(n.BrakeConfig(.002, .5, .8, 1.), .15, .012, 0., .2, .8))
        self.maximum_delta = 0.

    def send(self, channel, packet, tick):
        self.py.receive(channel, packet.to_bytes(), tick * .002)
        return self.native.receive(channel, packet.to_bytes(), tick)

    def reference(self, epoch, start, tick, sequence=0):
        return self.send("reference", ReferenceBlockPacket.from_rows(
            reference_epoch=epoch, sequence_id=sequence, start_control_tick=start,
            generated_time_s=tick * .002, dt_s=.002, yaw_rate_target_rad_s=0.,
            rows=[(0., 0., 0., 0., 0., 0.)] * 25), tick)

    def safety(self, sequence, authority, tick, reason="EXTERNAL_STOP"):
        return self.send("safety", SafetyCommandPacket(1, sequence, tick * .002,
            "HARD_STOP" if authority == "HARD" else "LONGITUDINAL_STOP", reason), tick)

    def resume(self, sequence, epoch, tick, authority="HARD"):
        return self.send("resume", ResumeReferenceCommandPacket(1, sequence, tick * .002, epoch, authority), tick)

    def yaw(self, tick):
        return self.send("yaw", YawCommandPacket(1, tick, tick * .002, 0.), tick)

    def heartbeat(self, tick):
        return self.send("heartbeat", HeartbeatPacket(1, tick, tick * .002), tick)

    def step(self, tick):
        py, c = self.py.command(tick * .002), self.native.step(tick, 0., 0., 0.)
        expected = np.array([*py.reference_state, py.reference_acceleration_m_s2,
                             py.u_ff_after_lifecycle_nm, py.yaw_rate_target_rad_s])
        actual = np.array([c.reference.p, c.reference.v, c.reference.theta,
                           c.reference.theta_dot, c.reference.a, c.reference.u_ff, c.yaw_rate])
        self.maximum_delta = max(self.maximum_delta, float(np.max(np.abs(expected - actual))))
        np.testing.assert_allclose(actual, expected, rtol=0., atol=2e-12)
        assert ("NORMAL", "BRAKING", "LONGITUDINAL_STOP", "SAFE_HOLD")[c.safety_state] == self.py.state
        assert c.epoch == self.py.receiver.reference_epoch
        return c

    def prepare(self, phase):
        self.reference(0, 0, 0)
        self.reference(0, 25, 0, 1)
        self.heartbeat(0)
        self.yaw(0)
        self.step(0)
        authority = "LONGITUDINAL" if phase in ("NORMAL", "LONGITUDINAL_STOP") else "HARD"
        assert self.safety(5, authority, 1) == 0
        if phase == "NORMAL":
            return 1
        end = 1 if phase == "BRAKING" else 600
        for tick in range(1, end + 1):
            if tick % 20 == 0:
                self.heartbeat(tick)
            self.step(tick)
        assert self.py.state == phase
        return end + 1


@pytest.mark.parametrize("phase", ["NORMAL", "BRAKING", "LONGITUDINAL_STOP", "SAFE_HOLD"])
@pytest.mark.parametrize("authority", ["HARD", "LONGITUDINAL"])
@pytest.mark.parametrize("sequence", [4, 5, 6])
def test_state_sequence_authority_matrix(phase, authority, sequence):
    pair = Pair()
    tick = pair.prepare(phase)
    epoch = pair.py.receiver.reference_epoch
    expected = 5 if sequence <= 5 else (7 if authority == "LONGITUDINAL" and phase != "NORMAL" else 0)
    assert pair.safety(sequence, authority, tick) == expected
    accepted_sequence = sequence if expected == 0 else 5
    assert pair.py.last_safety_sequence_id == accepted_sequence
    assert pair.py.receiver.reference_epoch == epoch + int(expected == 0 and authority == "HARD")
    pair.step(tick)


@pytest.mark.parametrize("armed", [False, True])
@pytest.mark.parametrize("initial_authority", ["HARD", "LONGITUDINAL"])
def test_hard_stop_revokes_pending_or_armed_resume(armed, initial_authority):
    pair = Pair()
    pair.prepare("SAFE_HOLD" if initial_authority == "HARD" else "LONGITUDINAL_STOP")
    epoch = pair.py.receiver.reference_epoch
    pair.yaw(601)
    assert pair.reference(epoch, 610, 601) == 0
    assert pair.resume(0, epoch, 601, initial_authority) == 0
    if armed:
        pair.step(601)
        assert pair.py.resume_start_tick == 610
    assert pair.safety(6, "HARD", 602) == 0
    assert pair.py.pending_resume_epoch is None
    assert pair.py.pending_resume_authority is None
    assert pair.py.resume_start_tick is None
    assert pair.py.receiver.next is None
    assert pair.py.receiver.reference_epoch == epoch + 1
    assert pair.reference(epoch, 620, 603, 1) == 2
    assert pair.resume(1, epoch, 603, initial_authority) == 7
    for tick in range(603, 621):
        pair.step(tick)
    assert pair.py.state == "SAFE_HOLD"
    pair.yaw(621)
    assert pair.reference(epoch + 1, 630, 621) == 0
    assert pair.resume(2, epoch + 1, 621) == 0
    for tick in range(621, 631):
        pair.step(tick)
    assert pair.py.state == "NORMAL"


def test_heartbeat_loss_revokes_armed_longitudinal_resume_and_needs_new_authorization():
    pair = Pair()
    pair.prepare("LONGITUDINAL_STOP")
    epoch = pair.py.receiver.reference_epoch
    assert pair.reference(epoch, 700, 601) == 0
    assert pair.resume(0, epoch, 601, "LONGITUDINAL") == 0
    pair.step(601)
    for tick in range(602, 701):
        pair.step(tick)
    assert pair.py.state == "SAFE_HOLD" and pair.py.health == "PI_LOST"
    assert pair.py.reason == "HEARTBEAT_TIMEOUT"
    assert pair.py.receiver.reference_epoch == epoch + 1
    assert pair.py.pending_resume_epoch is None and pair.py.receiver.next is None
    pair.yaw(701)
    assert pair.resume(1, epoch + 1, 701) == 7
    pair.heartbeat(702)
    assert pair.py.reason == "HEARTBEAT_TIMEOUT"
    assert pair.safety(6, "HARD", 702) == 0
    assert pair.py.reason == "HEARTBEAT_TIMEOUT"
    epoch += 1  # A fresh STOP establishes a second independent revocation barrier.
    pair.yaw(702)
    assert pair.resume(2, epoch + 1, 702) == 7  # Same-tick yaw cannot rearm HARD.
    pair.yaw(703)
    assert pair.reference(epoch + 1, 710, 703) == 0
    assert pair.resume(3, epoch + 1, 703) == 0
    for tick in range(703, 711):
        pair.step(tick)
    assert pair.py.state == "NORMAL"


def test_new_hard_stop_preserves_independent_starvation_reason_and_brake_progress():
    pair = Pair()
    pair.reference(0, 0, 0)
    pair.heartbeat(0)
    pair.yaw(0)
    for tick in range(601):
        if tick % 20 == 0:
            pair.heartbeat(tick)
        pair.step(tick)
    assert pair.py.state == "SAFE_HOLD"
    assert pair.py.reason == "REFERENCE_STARVATION_IMMINENT"
    epoch, brake_step = pair.py.receiver.reference_epoch, pair.py.brake.step_index
    assert pair.safety(0, "HARD", 601) == 0
    assert pair.py.reason == "REFERENCE_STARVATION_IMMINENT"
    assert pair.py.brake.step_index == brake_step
    result = pair.step(601)
    assert result.reason == 3 and result.epoch == epoch + 1
