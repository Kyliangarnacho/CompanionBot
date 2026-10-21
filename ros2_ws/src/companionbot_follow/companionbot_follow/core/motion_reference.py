"""Minimal acceleration-limited longitudinal reference generation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class LongitudinalReference:
    position_m: float
    velocity_m_s: float
    acceleration_m_s2: float


class LongitudinalReferenceGenerator:
    """Generate consistent ``p_ref``/``v_ref`` pairs at a fixed control rate.

    Position mode follows the classic acceleration-limited trapezoidal-profile
    baseline.  Velocity-derived mode rate-limits velocity and integrates the
    same shaped velocity into position.  The generator is deliberately not a
    feedback controller and never consumes plant or ground-truth state.
    """

    def __init__(
        self,
        controller_dt_s: float,
        max_velocity_m_s: float,
        max_acceleration_m_s2: float,
    ) -> None:
        self.dt_s = float(controller_dt_s)
        self.max_velocity_m_s = float(max_velocity_m_s)
        self.max_acceleration_m_s2 = float(max_acceleration_m_s2)
        if not math.isfinite(self.dt_s) or self.dt_s <= 0.0:
            raise ValueError("controller period must be finite and positive")
        if not math.isfinite(self.max_velocity_m_s) or self.max_velocity_m_s <= 0.0:
            raise ValueError("maximum velocity must be finite and positive")
        if (
            not math.isfinite(self.max_acceleration_m_s2)
            or self.max_acceleration_m_s2 <= 0.0
        ):
            raise ValueError("maximum acceleration must be finite and positive")
        self.reset()

    @property
    def reference(self) -> LongitudinalReference:
        return LongitudinalReference(self._position_m, self._velocity_m_s, 0.0)

    def reset(
        self, position_m: float = 0.0, velocity_m_s: float = 0.0
    ) -> LongitudinalReference:
        if not math.isfinite(position_m) or not math.isfinite(velocity_m_s):
            raise ValueError("initial reference state must be finite")
        if abs(velocity_m_s) > self.max_velocity_m_s + 1e-12:
            raise ValueError("initial reference velocity exceeds configured limit")
        self._position_m = float(position_m)
        self._velocity_m_s = float(velocity_m_s)
        return self.reference

    def step_position(self, target_position_m: float) -> LongitudinalReference:
        """Advance one period toward a zero-velocity position target."""

        target = float(target_position_m)
        if not math.isfinite(target):
            raise ValueError("position target must be finite")
        old_position = self._position_m
        old_velocity = self._velocity_m_s
        direction = -1.0 if old_position > target else 1.0
        current_position = direction * old_position
        current_velocity = direction * float(
            np.clip(
                old_velocity,
                -self.max_velocity_m_s,
                self.max_velocity_m_s,
            )
        )
        goal_position = direction * target
        acceleration = self.max_acceleration_m_s2
        acceleration_time = self.max_velocity_m_s / acceleration
        cutoff_begin_time = current_velocity / acceleration
        cutoff_begin_distance = (
            0.5 * acceleration * cutoff_begin_time * cutoff_begin_time
        )
        full_distance = (
            cutoff_begin_distance + goal_position - current_position
        )
        full_speed_distance = (
            full_distance - acceleration_time * acceleration_time * acceleration
        )
        if full_speed_distance < 0.0:
            acceleration_time = math.sqrt(max(0.0, full_distance / acceleration))
            full_speed_distance = 0.0

        end_acceleration = acceleration_time - cutoff_begin_time
        end_full_speed = (
            end_acceleration + full_speed_distance / self.max_velocity_m_s
        )
        end_deceleration = end_full_speed + acceleration_time
        time_s = self.dt_s
        if time_s < end_acceleration:
            directed_velocity = current_velocity + time_s * acceleration
            directed_position = current_position + (
                current_velocity + 0.5 * time_s * acceleration
            ) * time_s
        elif time_s < end_full_speed:
            directed_velocity = self.max_velocity_m_s
            directed_position = current_position + (
                current_velocity + 0.5 * end_acceleration * acceleration
            ) * end_acceleration
            directed_position += self.max_velocity_m_s * (
                time_s - end_acceleration
            )
        elif time_s <= end_deceleration:
            time_left = end_deceleration - time_s
            directed_velocity = time_left * acceleration
            directed_position = goal_position - 0.5 * time_left * time_left * acceleration
        else:
            directed_position = goal_position
            directed_velocity = 0.0

        new_position = direction * directed_position
        new_velocity = direction * directed_velocity

        acceleration = (new_velocity - old_velocity) / self.dt_s
        self._position_m = float(new_position)
        self._velocity_m_s = float(new_velocity)
        return LongitudinalReference(
            self._position_m,
            self._velocity_m_s,
            float(acceleration),
        )

    def step_velocity(self, velocity_command_m_s: float) -> LongitudinalReference:
        """Rate-limit velocity command and integrate a consistent position."""

        command = float(velocity_command_m_s)
        if not math.isfinite(command):
            raise ValueError("velocity command must be finite")
        limited_command = float(
            np.clip(command, -self.max_velocity_m_s, self.max_velocity_m_s)
        )
        old_velocity = self._velocity_m_s
        velocity_step = self.max_acceleration_m_s2 * self.dt_s
        new_velocity = old_velocity + float(
            np.clip(limited_command - old_velocity, -velocity_step, velocity_step)
        )
        acceleration = (new_velocity - old_velocity) / self.dt_s
        self._position_m += 0.5 * (old_velocity + new_velocity) * self.dt_s
        self._velocity_m_s = float(new_velocity)
        return LongitudinalReference(
            self._position_m,
            self._velocity_m_s,
            float(acceleration),
        )


class JerkLimitedLongitudinalReferenceGenerator:
    """Minimal single-axis symmetric S-curve generator for Stage 3A ablation.

    The position interface plans rest-to-rest moves.  The velocity interface
    plans between arbitrary velocities with zero acceleration at both ends.
    Command schedules must allow the active plan to finish before changing the
    target; this keeps the implementation deliberately narrower than a general
    online trajectory generator such as Ruckig.
    """

    def __init__(
        self,
        controller_dt_s: float,
        max_velocity_m_s: float,
        max_acceleration_m_s2: float,
        max_jerk_m_s3: float,
    ) -> None:
        self.dt_s = float(controller_dt_s)
        self.max_velocity_m_s = float(max_velocity_m_s)
        self.max_acceleration_m_s2 = float(max_acceleration_m_s2)
        self.max_jerk_m_s3 = float(max_jerk_m_s3)
        for name, value in (
            ("controller period", self.dt_s),
            ("maximum velocity", self.max_velocity_m_s),
            ("maximum acceleration", self.max_acceleration_m_s2),
            ("maximum jerk", self.max_jerk_m_s3),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        self.reset()

    @property
    def reference(self) -> LongitudinalReference:
        return LongitudinalReference(
            self._position_m,
            self._velocity_m_s,
            self._acceleration_m_s2,
        )

    @property
    def plan_active(self) -> bool:
        return bool(self._segments)

    def reset(
        self,
        position_m: float = 0.0,
        velocity_m_s: float = 0.0,
        acceleration_m_s2: float = 0.0,
    ) -> LongitudinalReference:
        values = (position_m, velocity_m_s, acceleration_m_s2)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("initial reference state must be finite")
        if abs(velocity_m_s) > self.max_velocity_m_s + 1e-12:
            raise ValueError("initial reference velocity exceeds configured limit")
        if abs(acceleration_m_s2) > self.max_acceleration_m_s2 + 1e-12:
            raise ValueError("initial reference acceleration exceeds configured limit")
        self._position_m = float(position_m)
        self._velocity_m_s = float(velocity_m_s)
        self._acceleration_m_s2 = float(acceleration_m_s2)
        self._segments: list[list[float]] = []
        self._target_position_m: float | None = None
        self._target_velocity_m_s: float | None = None
        self._mode: str | None = None
        return self.reference

    def _require_zero_acceleration(self) -> None:
        if abs(self._acceleration_m_s2) > 1e-10:
            raise ValueError("target changed before the active jerk-limited plan finished")

    def _set_segments(self, segments: list[tuple[float, float]]) -> None:
        self._segments = [
            [float(duration), float(jerk)]
            for duration, jerk in segments
            if duration > 1e-14
        ]

    def _advance(self) -> None:
        remaining_dt = self.dt_s
        while remaining_dt > 1e-14 and self._segments:
            segment_remaining, jerk = self._segments[0]
            step = min(remaining_dt, segment_remaining)
            self._position_m += (
                self._velocity_m_s * step
                + 0.5 * self._acceleration_m_s2 * step * step
                + jerk * step * step * step / 6.0
            )
            self._velocity_m_s += (
                self._acceleration_m_s2 * step + 0.5 * jerk * step * step
            )
            self._acceleration_m_s2 += jerk * step
            remaining_dt -= step
            segment_remaining -= step
            if segment_remaining <= 1e-14:
                self._segments.pop(0)
            else:
                self._segments[0][0] = segment_remaining

        if not self._segments:
            if self._mode == "position" and self._target_position_m is not None:
                self._position_m = self._target_position_m
                self._velocity_m_s = 0.0
                self._acceleration_m_s2 = 0.0
            elif self._mode == "velocity" and self._target_velocity_m_s is not None:
                self._velocity_m_s = self._target_velocity_m_s
                self._acceleration_m_s2 = 0.0
                self._position_m += self._velocity_m_s * remaining_dt

    def _begin_position_plan(self, target_position_m: float) -> None:
        self._require_zero_acceleration()
        if abs(self._velocity_m_s) > 1e-10:
            raise ValueError("position target changed before the prior rest-to-rest plan finished")
        distance = target_position_m - self._position_m
        if abs(distance) <= 1e-14:
            self._target_position_m = target_position_m
            self._segments = []
            return
        direction = math.copysign(1.0, distance)
        distance_abs = abs(distance)
        jerk = self.max_jerk_m_s3
        acceleration = self.max_acceleration_m_s2
        velocity = self.max_velocity_m_s
        jerk_time_at_acceleration_limit = acceleration / jerk
        acceleration_plateau_at_velocity_limit = (
            velocity / acceleration - jerk_time_at_acceleration_limit
        )
        if acceleration_plateau_at_velocity_limit < 0.0:
            jerk_time_at_velocity_limit = math.sqrt(velocity / jerk)
            distance_at_velocity_limit = 2.0 * jerk * jerk_time_at_velocity_limit**3
            if distance_abs >= distance_at_velocity_limit:
                jerk_time = jerk_time_at_velocity_limit
                acceleration_plateau = 0.0
                cruise_time = (
                    distance_abs - distance_at_velocity_limit
                ) / velocity
            else:
                jerk_time = (distance_abs / (2.0 * jerk)) ** (1.0 / 3.0)
                acceleration_plateau = 0.0
                cruise_time = 0.0
        else:
            distance_at_velocity_limit = velocity * (
                2.0 * jerk_time_at_acceleration_limit
                + acceleration_plateau_at_velocity_limit
            )
            if distance_abs >= distance_at_velocity_limit:
                jerk_time = jerk_time_at_acceleration_limit
                acceleration_plateau = acceleration_plateau_at_velocity_limit
                cruise_time = (
                    distance_abs - distance_at_velocity_limit
                ) / velocity
            else:
                triangular_distance = 2.0 * acceleration**3 / jerk**2
                if distance_abs >= triangular_distance:
                    jerk_time = jerk_time_at_acceleration_limit
                    acceleration_plateau = 0.5 * (
                        -3.0 * jerk_time
                        + math.sqrt(
                            jerk_time * jerk_time
                            + 4.0 * distance_abs / acceleration
                        )
                    )
                    cruise_time = 0.0
                else:
                    jerk_time = (distance_abs / (2.0 * jerk)) ** (1.0 / 3.0)
                    acceleration_plateau = 0.0
                    cruise_time = 0.0

        signed_jerk = direction * jerk
        self._set_segments(
            [
                (jerk_time, signed_jerk),
                (acceleration_plateau, 0.0),
                (jerk_time, -signed_jerk),
                (cruise_time, 0.0),
                (jerk_time, -signed_jerk),
                (acceleration_plateau, 0.0),
                (jerk_time, signed_jerk),
            ]
        )
        self._target_position_m = float(target_position_m)

    def _begin_velocity_plan(self, target_velocity_m_s: float) -> None:
        self._require_zero_acceleration()
        velocity_delta = target_velocity_m_s - self._velocity_m_s
        if abs(velocity_delta) <= 1e-14:
            self._target_velocity_m_s = target_velocity_m_s
            self._segments = []
            return
        direction = math.copysign(1.0, velocity_delta)
        jerk = self.max_jerk_m_s3
        acceleration = self.max_acceleration_m_s2
        delta_abs = abs(velocity_delta)
        jerk_time_at_acceleration_limit = acceleration / jerk
        velocity_delta_without_plateau = acceleration**2 / jerk
        if delta_abs >= velocity_delta_without_plateau:
            jerk_time = jerk_time_at_acceleration_limit
            acceleration_plateau = delta_abs / acceleration - jerk_time
        else:
            jerk_time = math.sqrt(delta_abs / jerk)
            acceleration_plateau = 0.0
        signed_jerk = direction * jerk
        self._set_segments(
            [
                (jerk_time, signed_jerk),
                (acceleration_plateau, 0.0),
                (jerk_time, -signed_jerk),
            ]
        )
        self._target_velocity_m_s = float(target_velocity_m_s)

    def step_position(self, target_position_m: float) -> LongitudinalReference:
        target = float(target_position_m)
        if not math.isfinite(target):
            raise ValueError("position target must be finite")
        if self._mode not in (None, "position"):
            raise ValueError("cannot switch reference mode without reset")
        self._mode = "position"
        if self._target_position_m is None or not math.isclose(
            target, self._target_position_m, rel_tol=0.0, abs_tol=1e-12
        ):
            if self.plan_active:
                raise ValueError("position command changed before trajectory completion")
            self._begin_position_plan(target)
        self._advance()
        return self.reference

    def step_velocity(self, velocity_command_m_s: float) -> LongitudinalReference:
        command = float(velocity_command_m_s)
        if not math.isfinite(command):
            raise ValueError("velocity command must be finite")
        if self._mode not in (None, "velocity"):
            raise ValueError("cannot switch reference mode without reset")
        self._mode = "velocity"
        target = float(
            np.clip(command, -self.max_velocity_m_s, self.max_velocity_m_s)
        )
        if self._target_velocity_m_s is None or not math.isclose(
            target, self._target_velocity_m_s, rel_tol=0.0, abs_tol=1e-12
        ):
            if self.plan_active:
                raise ValueError("velocity command changed before trajectory completion")
            self._begin_velocity_plan(target)
        self._advance()
        return self.reference
