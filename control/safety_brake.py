"""Fixed-step MCU safety reference with no trajectory solver dependency."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SafetyBrakeConfig:
    safe_max_deceleration_m_s2: float
    safe_max_jerk_m_s3: float
    hidden_reference_handoff_time_s: float
    dt_s: float

    def __post_init__(self) -> None:
        values = (self.safe_max_deceleration_m_s2, self.safe_max_jerk_m_s3,
                  self.hidden_reference_handoff_time_s, self.dt_s)
        if not all(math.isfinite(v) and v > 0.0 for v in values):
            raise ValueError("safety brake parameters must be finite and positive")


@dataclass(frozen=True)
class SafetyReference:
    p_m: float
    v_m_s: float
    a_m_s2: float
    theta_rad: float
    theta_dot_rad_s: float
    u_ff_nm: float


class SafetyBrakePrimitive:
    """Jerk-limited acceleration servo plus a cubic hidden-state handoff."""

    def __init__(self, config: SafetyBrakeConfig, initial: SafetyReference) -> None:
        self.config = config
        self.initial = initial
        self.current = initial
        self.step_index = 0
        self.velocity_settled = False

    def step(self) -> SafetyReference:
        if self.step_index == 0:
            self.step_index = 1
            return self.current

        dt = self.config.dt_s
        p, v, a = self.current.p_m, self.current.v_m_s, self.current.a_m_s2
        # The 2/s proportional target damps velocity; only the acceleration
        # change is limited at each MCU tick. No Pi planner is called here.
        target_a = max(-self.config.safe_max_deceleration_m_s2,
                       min(self.config.safe_max_deceleration_m_s2, -2.0 * v))
        max_da = self.config.safe_max_jerk_m_s3 * dt
        new_a = a + max(-max_da, min(max_da, target_a - a))
        new_v = v + new_a * dt
        new_p = p + 0.5 * (v + new_v) * dt

        t = min(self.step_index * dt,
                self.config.hidden_reference_handoff_time_s)
        duration = self.config.hidden_reference_handoff_time_s
        s = t / duration
        h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
        h10 = s**3 - 2.0 * s**2 + s
        theta = (h00 * self.initial.theta_rad
                 + h10 * duration * self.initial.theta_dot_rad_s)
        theta_dot = ((6.0 * s**2 - 6.0 * s) * self.initial.theta_rad / duration
                     + (3.0 * s**2 - 4.0 * s + 1.0)
                     * self.initial.theta_dot_rad_s)
        u_ff = (1.0 - 3.0 * s**2 + 2.0 * s**3) * self.initial.u_ff_nm

        self.current = SafetyReference(new_p, new_v, new_a,
                                       theta, theta_dot, u_ff)
        self.step_index += 1
        self.velocity_settled = abs(new_v) <= 0.005 and abs(new_a) <= 0.015
        return self.current

    @property
    def hidden_handoff_done(self) -> bool:
        return (self.step_index - 1) * self.config.dt_s >= (
            self.config.hidden_reference_handoff_time_s - 1e-12
        )
