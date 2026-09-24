"""Focused contracts for independent yaw freshness and split safety authority."""

import unittest

from control.pi_mcu_packets import (PROTOCOL_VERSION, HeartbeatPacket,
    ReferenceBlockPacket, ResumeReferenceCommandPacket, SafetyCommandPacket,
    YawCommandPacket)
from control.safety_brake import SafetyBrakeConfig
from control.stage6_pi_mcu_link import MCUSafetyArbiter, MCUReferenceBlockReceiver
from control.stage6_yaw_servo import BearingYawServo, MCULatestYawSource


def block(sequence, start, epoch=0):
    return ReferenceBlockPacket.from_rows(
        reference_epoch=epoch, sequence_id=sequence,
        start_control_tick=start, generated_time_s=0.0, dt_s=0.002,
        yaw_rate_target_rad_s=-0.4,
        rows=[(0, 0, 0, 0, 0, 0)]*25)


class SplitAuthorityTest(unittest.TestCase):
    def test_latest_yaw_overrides_block_and_stale_decays(self):
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        receiver.receive(block(0, 0).to_bytes(), 0.0)
        receiver.receive(block(1, 25).to_bytes(), 0.0)
        yaw = MCULatestYawSource(dt_s=0.002, timeout_s=0.01,
                                 decay_rate_rad_s2=1.0)
        arbiter = MCUSafetyArbiter(receiver, SafetyBrakeConfig(0.5, 0.8, 0.002, 0.002),
            heartbeat_timeout_s=0.15, starvation_margin_s=0.002,
            nominal_pitch_rad=0.0, status_sink=lambda *_: None,
            yaw_source=yaw)
        arbiter.receive("heartbeat", HeartbeatPacket(
            PROTOCOL_VERSION, 0, 0.0).to_bytes(), 0.0)
        arbiter.receive("yaw", YawCommandPacket(
            PROTOCOL_VERSION, 0, 0.0, 0.3).to_bytes(), 0.0)
        self.assertAlmostEqual(arbiter.command(0.0).yaw_rate_target_rad_s, 0.3)
        arbiter.receive("yaw", YawCommandPacket(
            PROTOCOL_VERSION, 1, 0.002, -0.2).to_bytes(), 0.002)
        self.assertAlmostEqual(arbiter.command(0.002).yaw_rate_target_rad_s, -0.2)
        for tick in range(2, 7):
            arbiter.command(tick*0.002)
        self.assertAlmostEqual(arbiter.command(0.014).yaw_rate_target_rad_s, -0.198)
        self.assertEqual(yaw.source, "LOCAL_YAW_TO_ZERO")

    def test_longitudinal_stop_keeps_yaw_hard_stop_disarms_it(self):
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        receiver.receive(block(0, 0).to_bytes(), 0.0)
        receiver.receive(block(1, 25).to_bytes(), 0.0)
        yaw = MCULatestYawSource(dt_s=0.002, timeout_s=0.2,
                                 decay_rate_rad_s2=1.0)
        arbiter = MCUSafetyArbiter(receiver, SafetyBrakeConfig(0.5, 0.8, 0.002, 0.002),
            heartbeat_timeout_s=0.15, starvation_margin_s=0.002,
            nominal_pitch_rad=0.0, status_sink=lambda *_: None,
            yaw_source=yaw)
        arbiter.receive("heartbeat", HeartbeatPacket(
            PROTOCOL_VERSION, 0, 0.0).to_bytes(), 0.0)
        arbiter.receive("yaw", YawCommandPacket(
            PROTOCOL_VERSION, 0, 0.0, 0.3).to_bytes(), 0.0)
        arbiter.command(0.0)
        arbiter.receive("safety", SafetyCommandPacket(
            PROTOCOL_VERSION, 0, 0.002, "LONGITUDINAL_STOP", "NEAR"
        ).to_bytes(), 0.002)
        self.assertAlmostEqual(arbiter.command(0.002).yaw_rate_target_rad_s, 0.3)
        self.assertEqual(receiver.reference_epoch, 1)
        self.assertIsNone(receiver.active)
        arbiter.receive("safety", SafetyCommandPacket(
            PROTOCOL_VERSION, 1, 0.004, "HARD_STOP", "FAULT"
        ).to_bytes(), 0.004)
        self.assertAlmostEqual(arbiter.command(0.004).yaw_rate_target_rad_s, 0.298)
        self.assertEqual(yaw.source, "LOCAL_YAW_TO_ZERO")
        arbiter.state = "SAFE_HOLD"
        arbiter.receive("resume", ResumeReferenceCommandPacket(
            PROTOCOL_VERSION, 0, 0.006, 1, "HARD"
        ).to_bytes(), 0.006)
        self.assertEqual(arbiter.events[-1]["event"], "resume_rejected")
        arbiter.receive("yaw", YawCommandPacket(
            PROTOCOL_VERSION, 1, 0.006, 0.1).to_bytes(), 0.006)
        arbiter.receive("resume", ResumeReferenceCommandPacket(
            PROTOCOL_VERSION, 1, 0.006, 1, "HARD"
        ).to_bytes(), 0.006)
        self.assertEqual(arbiter.events[-1]["event"], "resume_requested")

    def test_positive_bearing_commands_positive_yaw(self):
        beta, rate = BearingYawServo(gain_s_inv=1.0,
                                     max_rate_rad_s=0.5).command(1.0, 0.5)
        self.assertGreater(beta, 0)
        self.assertGreater(rate, 0)


if __name__ == "__main__":
    unittest.main()
