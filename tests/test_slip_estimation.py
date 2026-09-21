import math
import unittest

from sim.slip_estimation import LongitudinalSlipObserver, SlipObserverConfig
from sim.slope_estimation import SlopePlantParameters


def plant() -> SlopePlantParameters:
    return SlopePlantParameters(
        body_mass_kg=1.0,
        gravitational_mass_kg=1.2,
        equivalent_translation_mass_kg=1.4,
        body_com_length_m=0.08,
        wheel_radius_m=0.04,
        theta_flat_rad=0.0,
        wheel_joint_damping_nm_s_rad=0.0,
        wheel_joint_frictionloss_nm=0.0,
    )


def update(observer, wheel, steps, dt=0.002):
    rows = []
    for _ in range(steps):
        rows.append(observer.update(
            wheel_velocity_m_s=wheel,
            theta_world_rad=0.0,
            pitch_rate_rad_s=0.0,
            actual_sum_torque_nm=0.0,
            alpha_hat_rad=0.0,
            accelerometer_m_s2=[0.0, 0.0, 9.81],
            saturated=False,
            dt_s=dt,
        ))
    return rows


class SlipEstimationTest(unittest.TestCase):
    def test_no_slip_remains_clear_after_warmup(self):
        observer = LongitudinalSlipObserver(plant())
        observer.reset(0.2)
        rows = update(observer, 0.2, 1000)
        self.assertFalse(any(row.slip_active for row in rows))
        self.assertTrue(math.isfinite(rows[-1].slip_score))

    def test_persistent_wheel_disturbance_latches_and_clears(self):
        config = SlipObserverConfig(
            enter_persistence_s=0.04, exit_persistence_s=0.08
        )
        observer = LongitudinalSlipObserver(plant(), config)
        update(observer, 0.0, 300)
        slipped = update(observer, 0.35, 250)
        self.assertTrue(any(row.slip_active for row in slipped))
        recovered = update(observer, slipped[-1].body_velocity_hat_m_s, 500)
        self.assertFalse(recovered[-1].slip_active)

    def test_hysteresis_configuration_is_validated(self):
        with self.assertRaises(ValueError):
            SlipObserverConfig(enter_chi=2.0, exit_chi=3.0)


if __name__ == "__main__":
    unittest.main()
