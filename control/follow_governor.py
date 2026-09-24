"""Sparse longitudinal follow commands from relative target observations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from statistics import median
from typing import Callable

from .rolling_reference import MotionIntent, TargetObservation
from .radial_velocity_kf import RadialEstimate


@dataclass(frozen=True)
class FollowGovernorConfig:
    desired_distance_m: float
    d1_m: float
    d2_m: float
    catch_time_s: float
    max_abs_velocity_m_s: float
    initial_velocity_m_s: float = 0.0
    a_brake_effective_m_s2: float = 0.25
    catch_retrigger_margin_m_s: float = 0.05
    catch_retrigger_persistence_samples: int = 3
    slowdown_velocity_deadband_m_s: float = 0.05

    def __post_init__(self) -> None:
        values = (
            self.desired_distance_m,
            self.d1_m,
            self.d2_m,
            self.catch_time_s,
            self.max_abs_velocity_m_s,
            self.a_brake_effective_m_s2,
            self.catch_retrigger_margin_m_s,
            self.slowdown_velocity_deadband_m_s,
            self.initial_velocity_m_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("follow governor configuration must be finite")
        if self.d2_m >= self.d1_m:
            raise ValueError("d2_m must be less than d1_m")
        if self.desired_distance_m < 0.0 or self.d2_m < self.desired_distance_m:
            raise ValueError("desired distance must be non-negative and <= d2")
        if self.catch_time_s <= 0.0 or self.max_abs_velocity_m_s <= 0.0:
            raise ValueError("catch time and velocity limit must be positive")
        if self.a_brake_effective_m_s2 <= 0.0:
            raise ValueError("effective braking acceleration must be positive")
        if self.catch_retrigger_margin_m_s <= 0.0:
            raise ValueError("catch retrigger margin must be positive")
        if self.slowdown_velocity_deadband_m_s <= 0.0:
            raise ValueError("slowdown velocity deadband must be positive")
        if (not isinstance(self.catch_retrigger_persistence_samples, int)
                or self.catch_retrigger_persistence_samples < 1):
            raise ValueError("catch retrigger persistence must be a positive integer")
        if abs(self.initial_velocity_m_s) > self.max_abs_velocity_m_s:
            raise ValueError("initial velocity exceeds configured limit")


class FollowGovernor:
    """Latch one catch-up or cruise command until the next distance event."""

    CRUISE = "CRUISE"
    CATCH_UP = "CATCH_UP"

    def __init__(self, config: FollowGovernorConfig,
                 radial_estimate_provider: Callable[[TargetObservation], RadialEstimate]
                 | None = None) -> None:
        self.config = config
        self.radial_estimate_provider = radial_estimate_provider
        self.state = self.CRUISE
        self.latched_velocity_m_s = float(config.initial_velocity_m_s)
        self.robot_velocity_hat_m_s = 0.0
        self.master_velocity_hat_m_s = 0.0
        self.last_observation: TargetObservation | None = None
        self.last_sequence_id: int | None = None
        self.events: list[dict] = []
        self.observation_history: list[dict] = []
        self.duplicate_observation_count = 0
        self.closing_speed_m_s = 0.0
        self.transition_distance_m = 0.0
        self.switch_distance_m = float(config.d2_m)
        self.distance_m = float(config.desired_distance_m)
        self.beta_rad = 0.0
        self.distance_rate_hat_m_s = 0.0
        self.robot_radial_velocity_hat_m_s = 0.0
        self.increasing_distance_samples = 0
        self.unable_to_close_sample_count = 0
        self.slowdown_samples = 0
        self._recent_kf_master_radial_velocity_m_s: deque[float] = deque(maxlen=20)

    def set_robot_velocity_hat(self, velocity_m_s: float) -> None:
        value = float(velocity_m_s)
        if not math.isfinite(value):
            raise ValueError("robot velocity estimate must be finite")
        self.robot_velocity_hat_m_s = value

    def __call__(self, observation: TargetObservation) -> MotionIntent:
        if self.last_sequence_id is not None and observation.sequence_id < self.last_sequence_id:
            raise ValueError(
                "target observation sequence_id must increase "
                f"({self.last_sequence_id} -> {observation.sequence_id} at "
                f"{observation.capture_time_s:.6f}s)"
            )
        if self.last_sequence_id is not None and observation.sequence_id == self.last_sequence_id:
            self.duplicate_observation_count += 1
            if self.last_observation is not None:
                return self._intent(self.last_observation)
            return self._intent(observation)
        self.last_sequence_id = int(observation.sequence_id)
        if not observation.valid:
            self.increasing_distance_samples = 0
            self.slowdown_samples = 0
            self.observation_history.append(self._observation_row(
                observation, derivative_m_s=None, accepted=False
            ))
            return self._intent(observation)

        distance_m = math.hypot(observation.x_forward_m, observation.y_left_m)
        beta_rad = math.atan2(observation.y_left_m, observation.x_forward_m)
        derivative: float | None = None
        previous = self.last_observation
        if previous is not None:
            delta_t = float(observation.capture_time_s - previous.capture_time_s)
            if delta_t <= 0.0:
                raise ValueError("target observation timestamps must increase")
            derivative = (
                distance_m - math.hypot(previous.x_forward_m, previous.y_left_m)
            ) / delta_t
        if self.radial_estimate_provider is not None:
            estimate = self.radial_estimate_provider(observation)
            if not math.isfinite(estimate.master_radial_velocity_hat_m_s):
                raise ValueError("KF Master radial velocity must be finite")
            distance_m = estimate.distance_hat_m
            self.robot_velocity_hat_m_s = estimate.aligned_robot_velocity_m_s
            self.master_velocity_hat_m_s = estimate.master_radial_velocity_hat_m_s
            self._recent_kf_master_radial_velocity_m_s.append(
                self.master_velocity_hat_m_s
            )
            radial_robot = self.robot_velocity_hat_m_s * math.cos(beta_rad)
            decision_derivative = self.master_velocity_hat_m_s-radial_robot
        else:
            radial_robot = self.robot_velocity_hat_m_s * math.cos(beta_rad)
            decision_derivative = derivative
            self.master_velocity_hat_m_s = (
                radial_robot if derivative is None else radial_robot+derivative
            )
        self.distance_m = distance_m
        self.beta_rad = beta_rad
        self.distance_rate_hat_m_s = (0.0 if decision_derivative is None
                                      else decision_derivative)
        self.robot_radial_velocity_hat_m_s = radial_robot
        self.last_observation = observation

        self.closing_speed_m_s = max(
            -self.distance_rate_hat_m_s, 0.0
        )
        self.transition_distance_m = (
            self.closing_speed_m_s**2 / (2.0 * self.config.a_brake_effective_m_s2)
        )
        self.switch_distance_m = self.config.d2_m + self.transition_distance_m
        insufficient_catch = (
            self.master_velocity_hat_m_s
            > self.latched_velocity_m_s + self.config.catch_retrigger_margin_m_s
        )
        self.increasing_distance_samples = (
            self.increasing_distance_samples + 1
            if self.state == self.CATCH_UP
            and distance_m > self.config.d1_m
            and decision_derivative is not None
            and decision_derivative > 0.0
            and insufficient_catch
            else 0
        )
        unable_to_close = (
            self.state == self.CATCH_UP
            and insufficient_catch
            and self.latched_velocity_m_s >= self.config.max_abs_velocity_m_s - 1e-12
        )
        self.unable_to_close_sample_count += int(unable_to_close)
        slowing = (
            self.state == self.CRUISE
            and self.latched_velocity_m_s - self.master_velocity_hat_m_s
            > self.config.slowdown_velocity_deadband_m_s
            and self.closing_speed_m_s > 0.0
            and distance_m <= self.switch_distance_m
        )
        self.slowdown_samples = self.slowdown_samples + 1 if slowing else 0
        accepted = False
        if self.state == self.CRUISE and distance_m > self.config.d1_m:
            velocity = self.master_velocity_hat_m_s + (
                distance_m - self.config.d2_m
            ) / self.config.catch_time_s
            self._transition(
                observation,
                self.CATCH_UP,
                velocity,
                "distance_above_d1",
            )
            accepted = True
        elif self.state == self.CATCH_UP and distance_m <= self.switch_distance_m:
            cruise_velocity = self.master_velocity_hat_m_s
            if self.radial_estimate_provider is not None:
                recent = list(self._recent_kf_master_radial_velocity_m_s)
                short_median = float(median(recent[-5:]))
                long_median = float(median(recent))
                cruise_velocity = (
                    short_median
                    if long_median - short_median
                    > self.config.slowdown_velocity_deadband_m_s
                    else long_median
                )
            self._transition(
                observation,
                self.CRUISE,
                cruise_velocity,
                "distance_at_or_below_dynamic_switch",
            )
            accepted = True
        elif (
            self.state == self.CRUISE
            and self.slowdown_samples
            >= self.config.catch_retrigger_persistence_samples
        ):
            slowdown_velocity = self.master_velocity_hat_m_s
            slowdown_rejected = False
            if self.radial_estimate_provider is not None:
                recent = list(self._recent_kf_master_radial_velocity_m_s)
                short_median = float(median(recent[-5:]))
                long_median = float(median(recent))
                if short_median <= self.config.slowdown_velocity_deadband_m_s:
                    slowdown_velocity = short_median
                elif (self.latched_velocity_m_s - long_median
                      <= self.config.slowdown_velocity_deadband_m_s):
                    slowdown_rejected = True
                else:
                    slowdown_velocity = long_median
            if not slowdown_rejected:
                self._transition(
                    observation,
                    self.CRUISE,
                    slowdown_velocity,
                    "master_slowed_at_dynamic_switch",
                )
                accepted = True
            else:
                self.observation_history.append(self._observation_row(
                    observation, derivative_m_s=derivative, accepted=False
                ))
                self.observation_history[-1]["slowdown_rejection_reason"] = (
                    "short_kf_dip_without_long_window_slowdown"
                )
                self.slowdown_samples = 0
                return self._intent(observation)
            self.slowdown_samples = 0
        elif (
            self.state == self.CATCH_UP
            and self.increasing_distance_samples
            >= self.config.catch_retrigger_persistence_samples
        ):
            velocity = self.master_velocity_hat_m_s + (
                distance_m - self.config.d2_m
            ) / self.config.catch_time_s
            bounded_velocity = min(velocity, self.config.max_abs_velocity_m_s)
            if bounded_velocity > self.latched_velocity_m_s + 1e-12:
                self._transition(
                    observation, self.CATCH_UP, velocity,
                    "distance_increasing_and_catch_command_insufficient",
                )
                accepted = True
            self.increasing_distance_samples = 0

        self.observation_history.append(self._observation_row(
            observation, derivative_m_s=derivative, accepted=accepted
        ))
        return self._intent(observation)

    def _transition(
        self,
        observation: TargetObservation,
        state: str,
        generated_velocity_m_s: float,
        reason: str,
    ) -> None:
        previous_state = self.state
        bounded_velocity = max(
            0.0, min(self.config.max_abs_velocity_m_s,
                     float(generated_velocity_m_s)),
        )
        if (state == self.CRUISE
                and bounded_velocity <= self.config.slowdown_velocity_deadband_m_s):
            bounded_velocity = 0.0
        self.state = state
        if state != self.CATCH_UP:
            self.increasing_distance_samples = 0
        self.latched_velocity_m_s = bounded_velocity
        event = {
            "time_s": float(observation.capture_time_s),
            "sequence_id": int(observation.sequence_id),
            "event": (
                "catch_up_retriggered" if state == self.CATCH_UP
                and previous_state == self.CATCH_UP else
                "catch_up_started" if state == self.CATCH_UP else
                "slowdown_wait" if previous_state == self.CRUISE else
                "cruise_resumed"
            ),
            "reason": reason,
            "previous_state": previous_state,
            "state": state,
            "distance_m": float(self.distance_m),
            "beta_rad": float(self.beta_rad),
            "d_dot_hat_m_s": float(self.distance_rate_hat_m_s),
            "robot_radial_velocity_hat_m_s": float(
                self.robot_radial_velocity_hat_m_s),
            "master_velocity_hat_m_s": float(self.master_velocity_hat_m_s),
            "robot_velocity_hat_m_s": float(self.robot_velocity_hat_m_s),
            "closing_speed_m_s": float(self.closing_speed_m_s),
            "transition_distance_m": float(self.transition_distance_m),
            "switch_distance_m": float(self.switch_distance_m),
            "generated_velocity_m_s": float(generated_velocity_m_s),
            "latched_velocity_m_s": bounded_velocity,
            "command_saturated": bounded_velocity != float(generated_velocity_m_s),
        }
        if (previous_state == self.CATCH_UP and state == self.CRUISE
                and self.radial_estimate_provider is not None):
            recent = list(self._recent_kf_master_radial_velocity_m_s)
            short_median = float(median(recent[-5:]))
            long_median = float(median(recent))
            single_sample_shadow = max(
                0.0, min(self.config.max_abs_velocity_m_s,
                         float(self.master_velocity_hat_m_s)),
            )
            if single_sample_shadow <= self.config.slowdown_velocity_deadband_m_s:
                single_sample_shadow = 0.0
            event.update({
                "single_sample_kf_master_radial_velocity_m_s": float(
                    self.master_velocity_hat_m_s
                ),
                "single_sample_shadow_latched_velocity_m_s": (
                    single_sample_shadow
                ),
                "cruise_velocity_median_m_s": short_median,
                "cruise_velocity_window_samples": min(len(recent), 5),
                "cruise_velocity_long_median_m_s": long_median,
                "cruise_velocity_long_window_samples": len(recent),
            })
        self.events.append(event)

    def _intent(self, observation: TargetObservation) -> MotionIntent:
        return MotionIntent(
            source_time_s=float(observation.capture_time_s),
            linear_velocity_target_m_s=float(self.latched_velocity_m_s),
            yaw_rate_target_rad_s=0.0,
        )

    def _observation_row(
        self,
        observation: TargetObservation,
        *,
        derivative_m_s: float | None,
        accepted: bool,
    ) -> dict:
        return {
            "capture_time_s": float(observation.capture_time_s),
            "sequence_id": int(observation.sequence_id),
            "version": int(observation.version),
            "valid": bool(observation.valid),
            "confidence": float(observation.confidence),
            "x_forward_m": float(observation.x_forward_m),
            "y_left_m": float(observation.y_left_m),
            "distance_m": float(self.distance_m),
            "beta_rad": float(self.beta_rad),
            "d_dot_hat_m_s": float(self.distance_rate_hat_m_s),
            "robot_radial_velocity_hat_m_s": float(
                self.robot_radial_velocity_hat_m_s),
            "master_radial_velocity_hat_m_s": float(
                self.master_velocity_hat_m_s),
            "x_rel_dot_hat_m_s": derivative_m_s,
            "robot_velocity_hat_m_s": float(self.robot_velocity_hat_m_s),
            "master_velocity_hat_m_s": float(self.master_velocity_hat_m_s),
            "d2_m": float(self.config.d2_m),
            "closing_speed_m_s": float(self.closing_speed_m_s),
            "transition_distance_m": float(self.transition_distance_m),
            "switch_distance_m": float(self.switch_distance_m),
            "distance_increasing_samples": self.increasing_distance_samples,
            "slowdown_samples": self.slowdown_samples,
            "catch_command_saturated": (
                self.state == self.CATCH_UP
                and self.latched_velocity_m_s
                >= self.config.max_abs_velocity_m_s - 1e-12
            ),
            "unable_to_close": (
                self.state == self.CATCH_UP
                and self.master_velocity_hat_m_s
                > self.latched_velocity_m_s
                + self.config.catch_retrigger_margin_m_s
                and self.latched_velocity_m_s
                >= self.config.max_abs_velocity_m_s - 1e-12
            ),
            "governor_state": self.state,
            "latched_v_cmd_m_s": float(self.latched_velocity_m_s),
            "command_changed": bool(accepted),
        }
