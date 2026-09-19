"""Hysteretic persistent gate for an already-filtered disturbance estimate."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DisturbanceGateConfig:
    controller_dt_s: float
    d_on_nm: float
    d_off_nm: float
    on_persistence_s: float = 0.20
    off_persistence_s: float = 0.50
    ramp_duration_s: float = 0.15


@dataclass(frozen=True)
class DisturbanceGateOutput:
    requested_active: bool
    state: str
    alpha: float
    on_timer_s: float
    off_timer_s: float
    transitioned_on: bool
    transitioned_off: bool


class PersistentDisturbanceGate:
    """Gate driven only by ``abs(d_hat)`` with hysteresis and persistence."""

    def __init__(self, config: DisturbanceGateConfig):
        self.config = config
        if config.controller_dt_s <= 0.0:
            raise ValueError("controller_dt_s must be positive")
        if not 0.0 <= config.d_off_nm < config.d_on_nm:
            raise ValueError("thresholds must satisfy 0 <= d_off < d_on")
        if config.on_persistence_s <= 0.0 or config.off_persistence_s <= 0.0:
            raise ValueError("persistence times must be positive")
        if config.ramp_duration_s <= 0.0:
            raise ValueError("ramp_duration_s must be positive")
        self.requested_active = False
        self.alpha = 0.0
        self.on_timer_s = 0.0
        self.off_timer_s = 0.0
        self._ramp_elapsed_s = config.ramp_duration_s
        self._ramp_start_alpha = 0.0
        self._ramp_target_alpha = 0.0

    @staticmethod
    def _smoothstep(value: float) -> float:
        s = min(1.0, max(0.0, value))
        return 3.0 * s * s - 2.0 * s * s * s

    def _start_ramp(self, target: float) -> None:
        self._ramp_start_alpha = self.alpha
        self._ramp_target_alpha = target
        self._ramp_elapsed_s = 0.0

    def update(self, d_hat_nm: float) -> DisturbanceGateOutput:
        magnitude = abs(float(d_hat_nm))
        if not math.isfinite(magnitude):
            raise ValueError("d_hat_nm must be finite")
        transitioned_on = False
        transitioned_off = False
        dt_s = self.config.controller_dt_s

        if self.requested_active:
            self.on_timer_s = 0.0
            if magnitude < self.config.d_off_nm:
                self.off_timer_s += dt_s
            else:
                self.off_timer_s = 0.0
            if self.off_timer_s + 1e-12 >= self.config.off_persistence_s:
                self.requested_active = False
                self.off_timer_s = 0.0
                transitioned_off = True
                self._start_ramp(0.0)
        else:
            self.off_timer_s = 0.0
            if magnitude > self.config.d_on_nm:
                self.on_timer_s += dt_s
            else:
                self.on_timer_s = 0.0
            if self.on_timer_s + 1e-12 >= self.config.on_persistence_s:
                self.requested_active = True
                self.on_timer_s = 0.0
                transitioned_on = True
                self._start_ramp(1.0)

        if self._ramp_elapsed_s < self.config.ramp_duration_s:
            self._ramp_elapsed_s = min(
                self.config.ramp_duration_s,
                self._ramp_elapsed_s + dt_s,
            )
            blend = self._smoothstep(
                self._ramp_elapsed_s / self.config.ramp_duration_s
            )
            self.alpha = self._ramp_start_alpha + blend * (
                self._ramp_target_alpha - self._ramp_start_alpha
            )
        else:
            self.alpha = self._ramp_target_alpha

        if 0.0 < self.alpha < 1.0:
            state = "RAMPING_ON" if self._ramp_target_alpha > self.alpha else "RAMPING_OFF"
        else:
            state = "ACTIVE" if self.requested_active else "INACTIVE"
        return DisturbanceGateOutput(
            requested_active=self.requested_active,
            state=state,
            alpha=float(self.alpha),
            on_timer_s=float(self.on_timer_s),
            off_timer_s=float(self.off_timer_s),
            transitioned_on=transitioned_on,
            transitioned_off=transitioned_off,
        )
