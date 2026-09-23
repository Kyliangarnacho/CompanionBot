import unittest

from control.rolling_reference import TargetObservation
from scripts.run_stage5_v1_5_pi_mcu_stream import SimpleFollower


class Stage5FollowerTest(unittest.TestCase):
    def test_distance_dead_zone_holds_last_accepted_velocity(self):
        follower = SimpleFollower({
            "desired_forward_distance_m": 1.0,
            "distance_dead_zone_m": [0.9, 1.1],
            "linear_distance_gain_s_inv": 0.8,
            "yaw_bearing_gain_s_inv": 1.5,
            "maximum_abs_linear_velocity_m_s": 0.6,
            "maximum_abs_yaw_rate_rad_s": 0.5,
        })
        initial = follower(TargetObservation(0.0, 1.0, 0.0))
        outside = follower(TargetObservation(0.05, 1.2, 0.0))
        held = follower(TargetObservation(0.10, 1.05, 0.0))
        close = follower(TargetObservation(0.15, 0.8, 0.0))

        self.assertEqual(initial.linear_velocity_target_m_s, 0.0)
        self.assertAlmostEqual(outside.linear_velocity_target_m_s, 0.16)
        self.assertEqual(
            held.linear_velocity_target_m_s,
            outside.linear_velocity_target_m_s,
        )
        self.assertAlmostEqual(close.linear_velocity_target_m_s, -0.16)


if __name__ == "__main__":
    unittest.main()
