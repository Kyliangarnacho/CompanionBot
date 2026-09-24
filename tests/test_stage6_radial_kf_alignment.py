"""Variable capture dt, packet interpolation, and Governor estimator boundary."""

import math
import unittest

from control.follow_governor import FollowGovernor, FollowGovernorConfig
from control.pi_mcu_packets import (PROTOCOL_VERSION, RobotStatePacket,
    TargetObservationPacket)
from control.radial_velocity_kf import (PiRadialObservationPipeline,
    PiRobotStateHistory, RadialEstimate, RadialVelocityKFConfig,
    RadialVelocityKalmanFilter)
from control.rolling_reference import TargetObservation


def config():
    return RadialVelocityKFConfig(0.05, 0.6, 0.03, 0.8)


def state(sequence, time_s, velocity):
    return RobotStatePacket(PROTOCOL_VERSION, sequence, time_s,
                            velocity, 0.0, 0.0, "NORMAL")


class RadialKFAlignmentTest(unittest.TestCase):
    def test_variable_capture_dt_and_known_ego_motion(self):
        kf = RadialVelocityKalmanFilter(config())
        self.assertEqual(kf.update(capture_time_s=0.0,
            distance_meas_m=1.0, robot_radial_velocity_m_s=0.5),
            (1.0, 0.0, 0.0))
        d, v, innovation = kf.update(capture_time_s=0.2,
            distance_meas_m=0.9, robot_radial_velocity_m_s=0.5)
        self.assertAlmostEqual(innovation, 0.0, places=12)
        self.assertAlmostEqual(d, 0.9, places=12)
        self.assertAlmostEqual(v, 0.0, places=12)
        d, v, innovation = kf.update(capture_time_s=0.27,
            distance_meas_m=0.865, robot_radial_velocity_m_s=0.5)
        self.assertAlmostEqual(innovation, 0.0, places=12)
        self.assertAlmostEqual(d, 0.865, places=12)
        self.assertAlmostEqual(v, 0.0, places=12)
        with self.assertRaisesRegex(ValueError, "strictly increase"):
            kf.update(capture_time_s=0.27, distance_meas_m=0.865,
                      robot_radial_velocity_m_s=0.5)

    def test_capture_time_interpolation_and_history_miss_drop(self):
        history = PiRobotStateHistory()
        history.accept(RobotStatePacket.from_bytes(state(0, 0.0, 0.0).to_bytes()))
        history.accept(RobotStatePacket.from_bytes(state(1, 0.1, 1.0).to_bytes()))
        pipeline = PiRadialObservationPipeline(
            RadialVelocityKalmanFilter(config()), history)
        packet = TargetObservationPacket(PROTOCOL_VERSION, 0, 0.05,
                                         1.0, 0.0, True, 1.0)
        obs = pipeline.accept_observation(
            TargetObservationPacket.from_bytes(packet.to_bytes()), 0.2)
        self.assertIsNotNone(obs)
        self.assertEqual(pipeline.rows[-1]["alignment"], "INTERPOLATED")
        self.assertAlmostEqual(pipeline.rows[-1]["aligned_v_robot_m_s"], 0.5)
        self.assertAlmostEqual(pipeline.rows[-1]["aligned_robot_state_time_s"], 0.05)
        self.assertAlmostEqual(pipeline.rows[-1]["packet_arrival_time_s"], 0.2)
        future = TargetObservationPacket(PROTOCOL_VERSION, 1, 0.15,
                                         1.0, 0.0, True, 1.0)
        self.assertIsNone(pipeline.accept_observation(future, 0.3))
        self.assertEqual(pipeline.rows[-1]["alignment"], "STATE_HISTORY_MISS")
        self.assertEqual(pipeline.kf.last_capture_time_s, 0.05)

    def test_governor_decides_from_kf_distance_and_velocity(self):
        governor = FollowGovernor(FollowGovernorConfig(
            desired_distance_m=1.1, d1_m=1.4, d2_m=1.1,
            catch_time_s=3.0, max_abs_velocity_m_s=0.6),
            radial_estimate_provider=lambda obs: RadialEstimate(
                obs.sequence_id, obs.capture_time_s, 1.1, 0.0,
                0.0, 0.0, 0.0))
        # The raw packet looks far away; the controller decision uses d_hat=1.1.
        intent = governor(TargetObservation(0.0, 5.0, 0.0,
                                            sequence_id=0))
        self.assertEqual(intent.linear_velocity_target_m_s, 0.0)
        self.assertEqual(governor.distance_m, 1.1)
        self.assertEqual(governor.events, [])

    def test_cruise_latches_median_of_last_five_valid_kf_outputs(self):
        samples = {
            0: (1.8, 0.40), 1: (1.6, 0.42), 3: (1.6, 0.39),
            4: (1.6, 0.41), 5: (1.1, 0.10), 6: (1.2, 0.55),
        }
        calls = []

        def estimate(obs):
            self.assertTrue(obs.valid)
            calls.append(obs.sequence_id)
            distance, velocity = samples[obs.sequence_id]
            return RadialEstimate(obs.sequence_id, obs.capture_time_s,
                                  distance, velocity, 0.0, 0.0, 0.0)

        governor = FollowGovernor(FollowGovernorConfig(
            desired_distance_m=1.1, d1_m=1.4, d2_m=1.1,
            catch_time_s=3.0, max_abs_velocity_m_s=0.6),
            radial_estimate_provider=estimate)
        for sequence in (0, 1):
            governor(TargetObservation(sequence * 0.05, 1.8, 0.0,
                                       sequence_id=sequence))
        governor(TargetObservation(0.10, 1.8, 0.0, sequence_id=2,
                                   valid=False))
        for sequence in (3, 4):
            governor(TargetObservation(sequence * 0.05, 1.8, 0.0,
                                       sequence_id=sequence))
        governor(TargetObservation(0.20, 1.8, 0.0, sequence_id=4))
        intent = governor(TargetObservation(0.25, 1.8, 0.0,
                                            sequence_id=5))
        event = governor.events[-1]
        self.assertEqual(event["event"], "cruise_resumed")
        self.assertEqual(event["cruise_velocity_window_samples"], 5)
        self.assertAlmostEqual(event["single_sample_kf_master_radial_velocity_m_s"],
                               0.10)
        self.assertAlmostEqual(event["single_sample_shadow_latched_velocity_m_s"],
                               0.10)
        self.assertAlmostEqual(event["cruise_velocity_median_m_s"], 0.40)
        self.assertAlmostEqual(event["latched_velocity_m_s"], 0.40)
        self.assertAlmostEqual(intent.linear_velocity_target_m_s, 0.40)
        self.assertEqual(calls, [0, 1, 3, 4, 5])
        governor(TargetObservation(0.30, 1.8, 0.0, sequence_id=6))
        self.assertAlmostEqual(governor.latched_velocity_m_s, 0.40)

    def test_cruise_median_uses_available_samples_without_waiting(self):
        values = {0: (1.8, 0.20), 1: (1.1, 0.45)}
        governor = FollowGovernor(FollowGovernorConfig(
            desired_distance_m=1.1, d1_m=1.4, d2_m=1.1,
            catch_time_s=3.0, max_abs_velocity_m_s=0.6),
            radial_estimate_provider=lambda obs: RadialEstimate(
                obs.sequence_id, obs.capture_time_s, *values[obs.sequence_id],
                0.0, 0.0, 0.0))
        governor(TargetObservation(0.0, 1.8, 0.0, sequence_id=0))
        intent = governor(TargetObservation(0.05, 1.1, 0.0, sequence_id=1))
        self.assertEqual(governor.events[-1]["cruise_velocity_window_samples"], 2)
        self.assertAlmostEqual(intent.linear_velocity_target_m_s, 0.325)

    def test_long_history_rejects_false_slowdown_but_short_stop_stays_fast(self):
        velocities = {sequence: (0.4 if sequence < 15 else 0.37)
                      for sequence in range(20)}
        velocities.update({20: 0.30, 21: 0.30, 22: 0.30,
                           23: 0.0, 24: 0.0, 25: 0.0})

        def estimate(obs):
            distance = (1.8 if obs.sequence_id == 0 else
                        1.1 if obs.sequence_id >= 19 else 1.6)
            return RadialEstimate(obs.sequence_id, obs.capture_time_s,
                                  distance, velocities[obs.sequence_id],
                                  0.4, 0.0, 0.0)

        governor = FollowGovernor(FollowGovernorConfig(
            desired_distance_m=1.1, d1_m=1.4, d2_m=1.1,
            catch_time_s=3.0, max_abs_velocity_m_s=0.6),
            radial_estimate_provider=estimate)
        for sequence in range(20):
            governor(TargetObservation(sequence * 0.05, 1.8, 0.0,
                                       sequence_id=sequence))
        cruise = governor.events[-1]
        self.assertEqual(cruise["event"], "cruise_resumed")
        self.assertAlmostEqual(cruise["cruise_velocity_median_m_s"], 0.37)
        self.assertAlmostEqual(cruise["cruise_velocity_long_median_m_s"], 0.4)
        self.assertAlmostEqual(governor.latched_velocity_m_s, 0.4)

        for sequence in range(20, 23):
            governor(TargetObservation(sequence * 0.05, 1.1, 0.0,
                                       sequence_id=sequence))
        self.assertEqual(len(governor.events), 2)
        self.assertEqual(governor.observation_history[-1][
            "slowdown_rejection_reason"],
            "short_kf_dip_without_long_window_slowdown")
        self.assertAlmostEqual(governor.latched_velocity_m_s, 0.4)

        for sequence in range(23, 26):
            governor(TargetObservation(sequence * 0.05, 1.1, 0.0,
                                       sequence_id=sequence))
        self.assertEqual(governor.events[-1]["event"], "slowdown_wait")
        self.assertEqual(governor.latched_velocity_m_s, 0.0)


if __name__ == "__main__":
    unittest.main()
