"""Sparse CRUISE slowdown/wait behavior without a 20 Hz command stream."""

import unittest

from control.follow_governor import FollowGovernor, FollowGovernorConfig
from control.rolling_reference import (
    SparseVelocityCommandScheduler, TargetObservation,
)


class FollowGovernorWaitTest(unittest.TestCase):
    def test_locked_full_keeps_only_latest_slowdown_target(self):
        scheduler = SparseVelocityCommandScheduler(
            accept_delta_v_m_s=0.03, full_delta_v_m_s=0.08,
            stable_window_s=0.2, stable_range_m_s=0.03,
            initial_accepted_velocity_m_s=0.3,
            require_stable_candidate=False, minimum_delta_enabled=False,
            force_full_dynamic=True,
        )
        for index, target in enumerate((0.2, 0.1, 0.0)):
            command = scheduler.update(0.05 * index, target,
                                       allow_accept=False)
            self.assertFalse(command.accepted)
            self.assertEqual(scheduler.latest_pending_velocity_m_s, target)
        accepted = scheduler.update(0.2, 0.0, allow_accept=True)
        self.assertTrue(accepted.accepted)
        self.assertEqual(scheduler.accepted_velocity_m_s, 0.0)
        self.assertEqual([event["candidate_v_cmd_m_s"]
                          for event in scheduler.events
                          if event["event"] == "accepted"], [0.0])

    def test_two_real_slowdowns_then_stable_stop_emit_only_two_events(self):
        governor = FollowGovernor(FollowGovernorConfig(
            desired_distance_m=1.1, d1_m=1.4, d2_m=1.1,
            catch_time_s=3.0, max_abs_velocity_m_s=0.6,
            initial_velocity_m_s=0.25, a_brake_effective_m_s2=0.25,
            slowdown_velocity_deadband_m_s=0.05,
        ))
        distance = 1.1
        for sequence in range(7):
            if 1 <= sequence <= 3:
                distance -= 0.15 * 0.05
            elif 4 <= sequence <= 6:
                distance -= 0.25 * 0.05
            governor.set_robot_velocity_hat(0.25)
            governor(TargetObservation(sequence * 0.05, distance, 0.0,
                                       sequence_id=sequence))
        for sequence in range(7, 107):
            governor.set_robot_velocity_hat(0.0)
            governor(TargetObservation(sequence * 0.05, distance, 0.0,
                                       sequence_id=sequence))
        self.assertEqual([event["event"] for event in governor.events],
                         ["slowdown_wait", "slowdown_wait"])
        self.assertAlmostEqual(governor.events[0]["latched_velocity_m_s"],
                               0.10, places=9)
        self.assertEqual(governor.events[1]["latched_velocity_m_s"], 0.0)
        self.assertEqual(governor.latched_velocity_m_s, 0.0)


if __name__ == "__main__":
    unittest.main()
