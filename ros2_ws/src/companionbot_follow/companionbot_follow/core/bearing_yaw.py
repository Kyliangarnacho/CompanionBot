"""Pi bearing-to-yaw intent; no MCU authority."""
from __future__ import annotations
import math

class BearingYawServo:
    def __init__(self, *, gain_s_inv: float, max_rate_rad_s: float,
                 deadband_rad: float = 0.0) -> None:
        if gain_s_inv <= 0 or max_rate_rad_s <= 0 or deadband_rad < 0:
            raise ValueError("invalid bearing servo parameters")
        self.gain_s_inv = float(gain_s_inv)
        self.max_rate_rad_s = float(max_rate_rad_s)
        self.deadband_rad = float(deadband_rad)

    def command(self, x_forward_m: float, y_left_m: float) -> tuple[float, float]:
        beta = math.atan2(y_left_m, x_forward_m)
        target = 0.0 if abs(beta) <= self.deadband_rad else self.gain_s_inv * beta
        return beta, max(-self.max_rate_rad_s,
                         min(target, self.max_rate_rad_s))

