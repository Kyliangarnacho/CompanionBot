import math
import unittest

import numpy as np
from scipy.linalg import solve_discrete_are

from control.stage4_runtime import (
    EnvironmentMode, NaturalTransientPayloadId, PayloadIdConfig,
    PayloadLifecycle, PayloadPlantBuilder, QChangeTriggerConfig,
    SagittalPayloadParameters, SlopeSupervisor, SlopeSupervisorConfig,
)


class Stage4RuntimeTest(unittest.TestCase):
    def test_supervisor_has_only_flat_and_slope(self):
        self.assertEqual({item.value for item in EnvironmentMode}, {"FLAT", "SLOPE"})
        supervisor = SlopeSupervisor(SlopeSupervisorConfig())
        for _ in range(100):
            out = supervisor.update(
                dt_s=0.002, theta_hat_rad=math.radians(4.0), theta_eq_rad=0.0,
                theta_dyn_ref_rad=math.radians(0.1), velocity_hat_m_s=0.2,
                alpha_hat_rad=0.0, alpha_std_deg=5.0,
            )
        self.assertIs(out.mode, EnvironmentMode.FLAT)
        for _ in range(100):
            out = supervisor.update(
                dt_s=0.002, theta_hat_rad=math.radians(4.0), theta_eq_rad=0.0,
                theta_dyn_ref_rad=0.0, velocity_hat_m_s=0.2,
                alpha_hat_rad=0.0, alpha_std_deg=5.0,
            )
        self.assertIs(out.mode, EnvironmentMode.SLOPE)

    def test_optional_slope_entry_gate_resets_only_flat_enter_timer(self):
        supervisor = SlopeSupervisor(SlopeSupervisorConfig(
            enter_persistence_s=0.50
        ))
        for _ in range(250):
            out = supervisor.update(
                dt_s=0.002, theta_hat_rad=math.radians(5.0), theta_eq_rad=0.0,
                theta_dyn_ref_rad=0.0, velocity_hat_m_s=0.2,
                alpha_hat_rad=0.0, alpha_std_deg=5.0,
                slope_entry_allowed=False,
            )
        self.assertIs(out.mode, EnvironmentMode.FLAT)
        self.assertEqual(out.enter_timer_s, 0.0)
        for _ in range(250):
            out = supervisor.update(
                dt_s=0.002, theta_hat_rad=math.radians(5.0), theta_eq_rad=0.0,
                theta_dyn_ref_rad=0.0, velocity_hat_m_s=0.2,
                alpha_hat_rad=0.0, alpha_std_deg=5.0,
                slope_entry_allowed=True,
            )
        self.assertIs(out.mode, EnvironmentMode.SLOPE)

    def test_q_change_triggers_one_pending_then_next_transient(self):
        lifecycle = PayloadLifecycle(QChangeTriggerConfig(
            dt_s=0.01, enter_persistence_s=0.03,
            baseline_persistence_s=0.03,
        ))
        for index in range(3):
            lifecycle.observe(time_s=index * 0.01, q_filtered_nm=0.0, flat=True, transient=False)
        self.assertTrue(lifecycle.baseline_ready)
        for index in range(3, 8):
            lifecycle.observe(time_s=index * 0.01, q_filtered_nm=0.02, flat=True, transient=False)
        self.assertTrue(lifecycle.payload_id_pending)
        self.assertFalse(lifecycle.payload_id_active)
        lifecycle.observe(time_s=0.08, q_filtered_nm=0.02, flat=True, transient=False)
        lifecycle.observe(time_s=0.09, q_filtered_nm=0.02, flat=True, transient=True)
        self.assertFalse(lifecycle.payload_id_pending)
        self.assertTrue(lifecycle.payload_id_active)

    def test_two_parameter_batch_id_recovers_mass_and_forward_position(self):
        reduced = {
            "parameters": {
                "body_mass_kg": 0.8, "body_com_forward_m": 0.002,
                "body_com_height_m": 0.034,
                "equivalent_translation_mass_kg": 1.1,
                "single_wheel_rotating_mass_kg": 0.1,
                "wheel_radius_m": 0.042,
                "body_pitch_inertia_about_axle_kg_m2": 0.0019,
            }
        }
        builder = PayloadPlantBuilder(reduced, known_height_m=0.16)
        estimator = NaturalTransientPayloadId(PayloadIdConfig(
            update_period_s=0.02, minimum_updates=30, maximum_updates=50,
        ), builder, np.asarray([0.01, 0.1, 0.02, 0.5]))
        estimator.start()
        truth = SagittalPayloadParameters(0.25, 0.25 * 0.015, 0.16)
        ad, bd, theta_flat = builder.discrete_state_space(truth, 0.02)
        riccati = solve_discrete_are(
            ad, bd, np.diag([0.1, 0.1, 20.0, 1.0]), np.asarray([[1.0]])
        )
        feedback = np.linalg.solve(
            np.asarray([[1.0]]) + bd.T @ riccati @ bd,
            bd.T @ riccati @ ad,
        )
        state_error = np.asarray([0.0, 0.0, 0.01, 0.0])
        torque = 0.0
        absolute = state_error.copy()
        absolute[2] += theta_flat
        result = estimator.observe(
            dt_s=0.02, reference_acceleration_m_s2=0.3, saturated=False,
            state_absolute=absolute, actual_sum_torque_nm=torque,
        )
        for index in range(50):
            torque = float(
                -(feedback @ state_error)[0]
                + 0.02 * math.sin(index * 0.31)
            )
            state_error = ad @ state_error + bd[:, 0] * torque
            absolute = state_error.copy()
            absolute[2] += theta_flat
            result = estimator.observe(
                dt_s=0.02, reference_acceleration_m_s2=0.3, saturated=False,
                state_absolute=absolute, actual_sum_torque_nm=torque,
            )
        self.assertIsNotNone(result)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.parameters.mass_kg, 0.25, delta=0.005)
        self.assertAlmostEqual(result.parameters.forward_m, 0.015, delta=0.002)


if __name__ == "__main__":
    unittest.main()
