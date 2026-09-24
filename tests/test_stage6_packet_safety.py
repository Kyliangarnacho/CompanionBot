"""Packet isolation and takeover invariants for the Stage 6 MCU boundary."""

import math
import struct
import unittest

from control.pi_mcu_packets import (
    PROTOCOL_VERSION, HeartbeatPacket, ReferenceBlockPacket,
    ResumeReferenceCommandPacket, RobotStatePacket, SafetyCommandPacket,
    SafetyStatusPacket,
    TargetObservationPacket,
)
from control.safety_brake import (
    SafetyBrakeConfig, SafetyBrakePrimitive, SafetyReference,
)
from control.stage6_pi_mcu_link import (
    InMemoryPacketTransport, MCUSafetyArbiter, MCUReferenceBlockReceiver,
    Stage6PacketLink,
)


def block(sequence_id: int, start_tick: int,
          epoch: int = 0) -> ReferenceBlockPacket:
    return ReferenceBlockPacket.from_rows(
        reference_epoch=epoch, sequence_id=sequence_id,
        start_control_tick=start_tick, generated_time_s=0.0,
        dt_s=0.002, yaw_rate_target_rad_s=0.0,
        rows=[(1.0, 0.2, 0.1, 0.12, 0.02, 0.05)] * 25,
    )


class Stage6PacketSafetyTest(unittest.TestCase):
    def test_reference_packet_is_float32_and_consumer_has_no_producer(self):
        packet = block(0, 0)
        encoded = bytearray(packet.to_bytes())
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        receiver.receive(bytes(encoded), 0.0)
        encoded[-1] ^= 0xFF
        sample = receiver.command(0)
        self.assertAlmostEqual(sample.reference_state[1], 0.2, places=6)
        self.assertFalse(hasattr(receiver, "producer"))
        self.assertEqual(packet.sample_count, 25)
        receiver.reference_epoch = 1
        receiver.receive(block(1, 25).to_bytes(), 0.05)
        self.assertEqual(receiver.events[-1]["event"], "invalid_epoch_rejected")
        corrupted = bytearray(packet.to_bytes())
        struct.pack_into("<f", corrupted, len(corrupted) - 24, math.nan)
        with self.assertRaisesRegex(ValueError, "non-finite reference"):
            ReferenceBlockPacket.from_bytes(bytes(corrupted))

    def test_target_packet_has_only_relative_measurement(self):
        packet = TargetObservationPacket(
            PROTOCOL_VERSION, 3, 0.15, 1.4, 0.0, True, 1.0
        )
        decoded = TargetObservationPacket.from_bytes(packet.to_bytes())
        self.assertEqual(decoded, packet)
        self.assertNotIn(b"master_velocity", packet.to_bytes())
        self.assertNotIn(b"world", packet.to_bytes())

    def test_local_brake_first_sample_is_continuous_and_jerk_bounded(self):
        config = SafetyBrakeConfig(0.5, 0.8, 1.0, 0.002)
        initial = SafetyReference(1.0, 0.3, 0.2, 0.12, 0.02, 0.05)
        brake = SafetyBrakePrimitive(config, initial)
        self.assertEqual(brake.step(), initial)
        previous = initial
        for _ in range(4000):
            current = brake.step()
            self.assertLessEqual(abs(current.a_m_s2 - previous.a_m_s2),
                                 0.8 * 0.002 + 1e-10)
            self.assertLessEqual(abs(current.a_m_s2), 0.5 + 1e-10)
            previous = current
        self.assertTrue(brake.hidden_handoff_done)
        self.assertTrue(brake.velocity_settled)
        self.assertAlmostEqual(previous.theta_rad, 0.0, places=9)
        self.assertAlmostEqual(previous.theta_dot_rad_s, 0.0, places=9)
        self.assertAlmostEqual(previous.u_ff_nm, 0.0, places=9)

    def test_safety_packet_preempts_cached_reference_without_replan(self):
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        receiver.receive(block(0, 0).to_bytes(), 0.0)
        receiver.receive(block(1, 25).to_bytes(), 0.0)
        statuses = []
        arbiter = MCUSafetyArbiter(
            receiver, SafetyBrakeConfig(0.5, 0.8, 1.0, 0.002),
            heartbeat_timeout_s=0.15, starvation_margin_s=0.012,
            nominal_pitch_rad=0.0,
            status_sink=lambda payload, time_s: statuses.append((time_s, payload)),
        )
        arbiter.receive("heartbeat", HeartbeatPacket(
            PROTOCOL_VERSION, 0, 0.0).to_bytes(), 0.0)
        normal = arbiter.command(0.0)
        arbiter.receive("safety", SafetyCommandPacket(
            PROTOCOL_VERSION, 0, 0.002, "BRAKE", "OBSTACLE"
        ).to_bytes(), 0.002)
        safe = arbiter.command(0.002)
        self.assertEqual(arbiter.state, "BRAKING")
        self.assertEqual(receiver.reference_epoch, 1)
        self.assertIsNone(receiver.active)
        self.assertIsNone(receiver.next)
        self.assertEqual(safe.planning_path, "SAFETY")
        self.assertEqual(arbiter.first_takeover_deltas,
                         {key: 0.0 for key in arbiter.first_takeover_deltas})
        self.assertEqual(len(statuses), 1)
        status = SafetyStatusPacket.from_bytes(statuses[0][1])
        self.assertEqual(status.invalidated_epoch, 0)
        self.assertEqual(status.reference_epoch, 1)
        self.assertTrue(math.isclose(safe.u_ff_after_lifecycle_nm,
                                     normal.u_ff_after_lifecycle_nm))

    def test_fixed_active_next_capacity_and_stale_block_drop(self):
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        for sequence, start in ((0, 0), (1, 25), (2, 50)):
            receiver.receive(block(sequence, start).to_bytes(), 0.0)
        self.assertEqual(receiver.active.sequence_id, 0)
        self.assertEqual(receiver.next.sequence_id, 1)
        self.assertEqual(receiver.events[-1]["event"], "reference_capacity_rejected")
        receiver.receive(block(3, 0).to_bytes(), 0.01)
        self.assertEqual(receiver.events[-1]["event"], "stale_reference_packet")
        receiver.invalidate_current_epoch()
        self.assertIsNone(receiver.active)
        self.assertIsNone(receiver.next)
        receiver.receive(block(4, 75).to_bytes(), 0.1)
        self.assertEqual(receiver.events[-1]["event"], "invalid_epoch_rejected")

    def test_resume_requires_explicit_arm_fresh_future_block_and_safe_hold(self):
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        receiver.receive(block(0, 0).to_bytes(), 0.0)
        receiver.receive(block(1, 25).to_bytes(), 0.0)
        statuses = []
        arbiter = MCUSafetyArbiter(
            receiver, SafetyBrakeConfig(0.5, 0.8, 0.002, 0.002),
            heartbeat_timeout_s=0.15, starvation_margin_s=0.012,
            nominal_pitch_rad=0.0,
            status_sink=lambda payload, time_s: statuses.append(
                SafetyStatusPacket.from_bytes(payload)),
        )
        arbiter.receive("heartbeat", HeartbeatPacket(
            PROTOCOL_VERSION, 0, 0.0).to_bytes(), 0.0)
        arbiter.command(0.0)
        arbiter.receive("safety", SafetyCommandPacket(
            PROTOCOL_VERSION, 0, 0.002, "BRAKE", "OBSTACLE"
        ).to_bytes(), 0.002)
        arbiter.command(0.002)
        early_resume = ResumeReferenceCommandPacket(
            PROTOCOL_VERSION, 0, 0.004, 1
        )
        arbiter.receive("resume", early_resume.to_bytes(), 0.004)
        self.assertEqual(arbiter.events[-1]["event"], "resume_rejected")
        arbiter.state = "SAFE_HOLD"  # Exercise protocol after local brake completion.
        arbiter.receive("reference", block(2, 25, epoch=0).to_bytes(), 0.004)
        self.assertIsNone(receiver.next)
        arbiter.receive("reference", block(0, 25, epoch=1).to_bytes(), 0.004)
        for tick in range(2, 26):
            arbiter.command(tick * 0.002)
        self.assertEqual(arbiter.state, "SAFE_HOLD")
        self.assertIsNone(receiver.next)
        resume = ResumeReferenceCommandPacket(PROTOCOL_VERSION, 1, 0.05, 1)
        arbiter.receive("resume", resume.to_bytes(), 0.05)
        arbiter.receive("reference", block(0, 50, epoch=1).to_bytes(), 0.05)
        arbiter.receive("safety", SafetyCommandPacket(
            PROTOCOL_VERSION, 1, 0.05, "BRAKE", "DUPLICATE_HAZARD"
        ).to_bytes(), 0.05)
        arbiter.command(0.05)
        self.assertEqual(arbiter.state, "SAFE_HOLD")
        self.assertEqual(arbiter.resume_start_tick, 50)
        for tick in range(26, 50):
            arbiter.command(tick * 0.002)
        normal = arbiter.command(0.1)
        self.assertEqual(arbiter.state, "NORMAL")
        self.assertEqual(normal.planning_path, "WIRE_NORMAL")
        self.assertEqual(receiver.reference_epoch, 1)
        self.assertEqual(statuses[-1].state, "NORMAL")
        self.assertIsNone(arbiter.pending_safety)

    def test_safe_hold_heartbeat_loss_is_health_only(self):
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        receiver.receive(block(0, 0).to_bytes(), 0.0)
        statuses = []
        arbiter = MCUSafetyArbiter(
            receiver, SafetyBrakeConfig(0.5, 0.8, 0.002, 0.002),
            heartbeat_timeout_s=0.15, starvation_margin_s=0.012,
            nominal_pitch_rad=0.0,
            status_sink=lambda payload, time_s: statuses.append(
                SafetyStatusPacket.from_bytes(payload)),
        )
        arbiter.receive("heartbeat", HeartbeatPacket(
            PROTOCOL_VERSION, 0, 0.0).to_bytes(), 0.0)
        arbiter.command(0.0)
        arbiter.receive("safety", SafetyCommandPacket(
            PROTOCOL_VERSION, 0, 0.002, "BRAKE", "OBSTACLE"
        ).to_bytes(), 0.002)
        arbiter.command(0.002)
        arbiter.state = "SAFE_HOLD"
        arbiter.command(0.2)
        self.assertEqual(arbiter.state, "SAFE_HOLD")
        self.assertEqual(arbiter.health, "PI_LOST")
        self.assertEqual(statuses[-1].health, "PI_LOST")
        self.assertEqual(sum(event["event"] == "safety_takeover"
                             for event in arbiter.events), 1)

    def test_pi_resume_requires_clearance_fresh_state_and_new_epoch(self):
        class FreshProducer:
            dt_s = 0.002
            block_samples = 25
            reference_epoch = 1
            next_sequence_id = 1

            def __init__(self):
                self.produced_starts = []

            def produce(self, start_tick, generated_time_s,
                        *, observe_command=True):
                self.produced_starts.append(start_tick)
                packet = ReferenceBlockPacket.from_rows(
                    reference_epoch=1, sequence_id=self.next_sequence_id,
                    start_control_tick=start_tick,
                    generated_time_s=generated_time_s, dt_s=0.002,
                    yaw_rate_target_rad_s=0.0,
                    rows=[(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)] * 25,
                )
                self.next_sequence_id += 1
                return packet, 0.0

        transport = InMemoryPacketTransport()
        receiver = MCUReferenceBlockReceiver(dt_s=0.002, block_samples=25)
        arbiter = MCUSafetyArbiter(
            receiver, SafetyBrakeConfig(0.5, 0.8, 0.002, 0.002),
            heartbeat_timeout_s=0.15, starvation_margin_s=0.012,
            nominal_pitch_rad=0.0,
            status_sink=lambda payload, time_s: None,
        )
        link = Stage6PacketLink(
            transport=transport, producer=FreshProducer(), governor=None,
            arbiter=arbiter, crash_time_s=None, hazard_time_s=None,
        )
        link.latest_safety_status = SafetyStatusPacket(
            PROTOCOL_VERSION, 1, 0.2, "SAFE_HOLD", "OBSTACLE", 1, 0
        )
        link.latest_robot_state = RobotStatePacket(
            PROTOCOL_VERSION, 5, 0.25, 0.0, 0.0, 0.0, "SAFE_HOLD"
        )
        fresh = ReferenceBlockPacket.from_rows(
            reference_epoch=1, sequence_id=0, start_control_tick=150,
            generated_time_s=0.26, dt_s=0.002, yaw_rate_target_rad_s=0.0,
            rows=[(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)] * 25,
        )
        with self.assertRaisesRegex(ValueError, "clearance"):
            link.request_resume(time_s=0.26, hazard_cleared=False,
                                fresh_producer=FreshProducer(), first_block=fresh)
        with self.assertRaisesRegex(ValueError, "fresh resume"):
            link.request_resume(time_s=0.26, hazard_cleared=True,
                                fresh_producer=FreshProducer(),
                                first_block=block(0, 150, epoch=0))
        fresh_producer = FreshProducer()
        link.request_resume(time_s=0.26, hazard_cleared=True,
                            fresh_producer=fresh_producer, first_block=fresh)
        delivered = transport.deliver("mcu", 0.26)
        self.assertEqual([channel for channel, _ in delivered],
                         ["resume", "reference"])
        self.assertEqual(ResumeReferenceCommandPacket.from_bytes(
            delivered[0][1]).reference_epoch, 1)
        receiver.reference_epoch = 1
        receiver.paused = True
        transport.send("pi", "safety_status", SafetyStatusPacket(
            PROTOCOL_VERSION, 2, 0.3, "NORMAL", "NONE", 1, 0
        ).to_bytes(), 0.3)
        link.before_control_tick(0.302)
        self.assertEqual(fresh_producer.produced_starts, [175])
        self.assertIsNone(link.resume_followup_start_tick)

    def test_transport_delay_and_injected_loss_only_move_bytes(self):
        transport = InMemoryPacketTransport(latency_s=0.004)
        transport.send("mcu", "heartbeat", b"alive", 0.0)
        self.assertEqual(transport.deliver("mcu", 0.002), [])
        self.assertEqual(transport.deliver("mcu", 0.004), [("heartbeat", b"alive")])
        transport.drop_after[("mcu", "heartbeat")] = 0.1
        transport.send("mcu", "heartbeat", b"lost", 0.1)
        self.assertEqual(transport.deliver("mcu", 1.0), [])


if __name__ == "__main__":
    unittest.main()
