import json
import math
from pathlib import Path
import unittest

from sim.slope_estimation import (
    SlopePlantParameters,
    analytic_theta_eq,
    equilibrium_sum_torque_nm,
    inverse_analytic_alpha,
)


ROOT = Path(__file__).resolve().parents[1]


class SlopeEstimationSignTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        reduced = json.loads(
            (ROOT / "models/minisegway/reduced_twip.json").read_text(encoding="utf-8")
        )
        cls.plant = SlopePlantParameters.from_reduced(reduced)

    def test_zero_and_plus_minus_eight_degree_roundtrip(self):
        for alpha_deg in (0.0, 8.0, -8.0):
            alpha = math.radians(alpha_deg)
            theta = analytic_theta_eq(alpha, self.plant)
            recovered = inverse_analytic_alpha(theta, self.plant)
            self.assertAlmostEqual(recovered, alpha, places=12)

    def test_companionbot_slope_and_torque_signs(self):
        flat_theta = analytic_theta_eq(0.0, self.plant)
        uphill_theta = analytic_theta_eq(math.radians(8.0), self.plant)
        downhill_theta = analytic_theta_eq(math.radians(-8.0), self.plant)
        self.assertGreater(uphill_theta, flat_theta)
        self.assertGreater(flat_theta, downhill_theta)
        flat_u = equilibrium_sum_torque_nm(0.0, 0.4, 0.0, self.plant)
        uphill_u = equilibrium_sum_torque_nm(math.radians(8.0), 0.4, 0.0, self.plant)
        downhill_u = equilibrium_sum_torque_nm(math.radians(-8.0), 0.4, 0.0, self.plant)
        self.assertGreater(uphill_u, flat_u)
        self.assertGreater(flat_u, downhill_u)


if __name__ == "__main__":
    unittest.main()
