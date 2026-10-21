"""Residual-driven lifecycle manager for active identification probes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from .probing import CenteredPRBS, PRBSConfig


@dataclass(frozen=True)
class AutoProbeConfig:
    controller_dt_s: float
    residual_reference_scaled_rms: float
    residual_ewma_time_constant_s: float
    trigger_ratio: float
    trigger_absolute_floor: float
    trigger_persistence_s: float
    clear_ratio: float
    clear_absolute_floor: float
    clear_persistence_s: float
    cooldown_s: float
    minimum_session_duration_s: float
    maximum_session_duration_s: float
    minimum_effective_updates: int
    pe_qualified_persistence_s: float
    plateau_window_s: float
    maximum_parameter_relative_span: float
    maximum_residual_relative_improvement_for_plateau: float
    probe_amplitude_sum_torque_nm: float
    probe_bit_period_s: float
    probe_lfsr_bits: int
    probe_seed: int


@dataclass(frozen=True)
class ProbeTransition:
    started: bool = False
    stopped: bool = False
    reason: str | None = None


class AutoProbeManager:
    """EWMA/persistence change detector plus information-aware probe stop logic.

    The detector observes only the current model's normalized one-step innovation.
    A session cannot start from one isolated sample, and cannot finish until useful
    RLS updates and persistent excitation have both been observed. A maximum
    duration remains as a safety fallback.
    """

    def __init__(self, config: AutoProbeConfig):
        self.config = config
        self.active = False
        self.events: list[dict] = []
        self.session_count = 0
        self.session_start_s: float | None = None
        self.session_end_s: float | None = None
        self.stop_reason: str | None = None
        self.ewma_residual = float(config.residual_reference_scaled_rms)
        self._trigger_count = 0
        self._clear_count = 0
        self._pe_count = 0
        self._cooldown_until_s = 0.0
        self._armed = True
        self._updates_at_start = 0
        self._last_accepted_updates = 0
        self._residual_history: deque[tuple[float, float]] = deque()
        self._parameter_history: deque[tuple[float, NDArray[np.float64]]] = deque()
        self._probe: CenteredPRBS | None = None

    @property
    def trigger_threshold(self) -> float:
        return max(
            self.config.trigger_absolute_floor,
            self.config.trigger_ratio
            * self.config.residual_reference_scaled_rms,
        )

    @property
    def clear_threshold(self) -> float:
        return max(
            self.config.clear_absolute_floor,
            self.config.clear_ratio
            * self.config.residual_reference_scaled_rms,
        )

    def probe_value(self, time_s: float) -> float:
        if not self.active or self._probe is None:
            return 0.0
        return self._probe.value(time_s)

    def observe(
        self,
        time_s: float,
        normalized_prediction_residual: NDArray[np.float64],
        *,
        pe_qualified: bool,
        accepted_updates: int,
        parameter_vector: NDArray[np.float64] | None,
    ) -> ProbeTransition:
        residual = np.asarray(normalized_prediction_residual, dtype=float)
        scalar = float(np.sqrt(np.mean(residual**2)))
        if not math.isfinite(scalar):
            scalar = float("inf")
        alpha = 1.0 - math.exp(
            -self.config.controller_dt_s
            / self.config.residual_ewma_time_constant_s
        )
        self.ewma_residual += alpha * (scalar - self.ewma_residual)

        if self.active:
            return self._observe_active(
                time_s,
                pe_qualified,
                accepted_updates,
                parameter_vector,
            )
        return self._observe_inactive(time_s)

    def _observe_inactive(self, time_s: float) -> ProbeTransition:
        if not self._armed:
            if (
                time_s >= self._cooldown_until_s
                and self.ewma_residual <= self.clear_threshold
            ):
                self._clear_count += 1
            else:
                self._clear_count = 0
            clear_needed = self._samples(self.config.clear_persistence_s)
            if self._clear_count >= clear_needed:
                self._armed = True
                self._trigger_count = 0
                self.events.append(
                    {
                        "time_s": float(time_s),
                        "event": "rearmed",
                        "ewma_scaled_residual_rms": self.ewma_residual,
                    }
                )
            return ProbeTransition()

        if time_s < self._cooldown_until_s:
            self._trigger_count = 0
            return ProbeTransition()
        if self.ewma_residual >= self.trigger_threshold:
            self._trigger_count += 1
        else:
            # A single spike therefore decays instead of satisfying persistence.
            self._trigger_count = max(0, self._trigger_count - 1)
        if self._trigger_count < self._samples(self.config.trigger_persistence_s):
            return ProbeTransition()

        self.active = True
        self._armed = False
        self._trigger_count = 0
        self._pe_count = 0
        self.session_count += 1
        self.session_start_s = float(time_s + self.config.controller_dt_s)
        self.session_end_s = None
        self.stop_reason = None
        self._residual_history.clear()
        self._parameter_history.clear()
        self._probe = CenteredPRBS(
            PRBSConfig(
                self.config.probe_amplitude_sum_torque_nm,
                self.session_start_s,
                self.config.maximum_session_duration_s,
                self.config.probe_bit_period_s,
                self.config.probe_lfsr_bits,
                self.config.probe_seed + self.session_count - 1,
            )
        )
        self.events.append(
            {
                "time_s": self.session_start_s,
                "event": "probe_started",
                "reason": "persistent_normalized_prediction_mismatch",
                "ewma_scaled_residual_rms": self.ewma_residual,
                "trigger_threshold": self.trigger_threshold,
            }
        )
        self._updates_at_start = 0
        return ProbeTransition(True, False, "persistent_model_mismatch")

    def _observe_active(
        self,
        time_s: float,
        pe_qualified: bool,
        accepted_updates: int,
        parameter_vector: NDArray[np.float64] | None,
    ) -> ProbeTransition:
        assert self.session_start_s is not None
        elapsed = time_s - self.session_start_s
        if not self._residual_history:
            self._updates_at_start = accepted_updates
        self._residual_history.append((float(time_s), self.ewma_residual))
        cutoff = time_s - self.config.plateau_window_s
        while self._residual_history and self._residual_history[0][0] < cutoff:
            self._residual_history.popleft()
        if parameter_vector is not None and np.isfinite(parameter_vector).all():
            self._parameter_history.append(
                (float(time_s), np.asarray(parameter_vector, dtype=float).copy())
            )
        while self._parameter_history and self._parameter_history[0][0] < cutoff:
            self._parameter_history.popleft()

        self._pe_count = self._pe_count + 1 if pe_qualified else 0
        useful_updates = accepted_updates - self._updates_at_start
        self._last_accepted_updates = accepted_updates
        data_ready = bool(
            elapsed >= self.config.minimum_session_duration_s
            and useful_updates >= self.config.minimum_effective_updates
            and self._pe_count
            >= self._samples(self.config.pe_qualified_persistence_s)
        )
        parameter_stable = self._parameter_stable(time_s)
        residual_plateau = self._residual_plateau(time_s)
        if data_ready and (parameter_stable or residual_plateau):
            return self._stop(time_s, "information_and_learning_plateau")
        if elapsed >= self.config.maximum_session_duration_s:
            return self._stop(time_s, "maximum_duration_safety_stop")
        return ProbeTransition()

    def _parameter_stable(self, time_s: float) -> bool:
        if len(self._parameter_history) < 2:
            return False
        if self._parameter_history[0][0] > time_s - 0.9 * self.config.plateau_window_s:
            return False
        values = np.asarray([value for _, value in self._parameter_history])
        span = float(np.linalg.norm(np.ptp(values, axis=0)))
        scale = max(float(np.linalg.norm(values[-1])), 1.0)
        return span / scale <= self.config.maximum_parameter_relative_span

    def _parameter_relative_span(self) -> float | None:
        if len(self._parameter_history) < 2:
            return None
        values = np.asarray([value for _, value in self._parameter_history])
        span = float(np.linalg.norm(np.ptp(values, axis=0)))
        scale = max(float(np.linalg.norm(values[-1])), 1.0)
        return span / scale

    def _residual_plateau(self, time_s: float) -> bool:
        if len(self._residual_history) < 4:
            return False
        if self._residual_history[0][0] > time_s - 0.9 * self.config.plateau_window_s:
            return False
        values = np.asarray([value for _, value in self._residual_history])
        split = len(values) // 2
        early = float(np.mean(values[:split]))
        late = float(np.mean(values[split:]))
        improvement = (early - late) / max(early, 1e-12)
        # A low residual alone is not enough while it is still falling quickly;
        # stop only after it is back in the clear band and no longer improving.
        return bool(
            late <= self.clear_threshold
            and improvement
            <= self.config.maximum_residual_relative_improvement_for_plateau
        )

    def _residual_window_metrics(self) -> tuple[float | None, float | None]:
        if len(self._residual_history) < 4:
            return None, None
        values = np.asarray([value for _, value in self._residual_history])
        split = len(values) // 2
        early = float(np.mean(values[:split]))
        late = float(np.mean(values[split:]))
        improvement = (early - late) / max(early, 1e-12)
        return late, improvement

    def _stop(self, time_s: float, reason: str) -> ProbeTransition:
        assert self.session_start_s is not None
        self.active = False
        self.session_end_s = float(time_s + self.config.controller_dt_s)
        self.stop_reason = reason
        self._cooldown_until_s = self.session_end_s + self.config.cooldown_s
        self._clear_count = 0
        residual_late, residual_improvement = self._residual_window_metrics()
        self.events.append(
            {
                "time_s": self.session_end_s,
                "event": "probe_stopped",
                "reason": reason,
                "duration_s": self.session_end_s - self.session_start_s,
                "ewma_scaled_residual_rms": self.ewma_residual,
                "effective_update_count": self._last_accepted_updates
                - self._updates_at_start,
                "pe_qualified_consecutive_samples": self._pe_count,
                "parameter_relative_span": self._parameter_relative_span(),
                "residual_window_late_mean": residual_late,
                "residual_window_relative_improvement": residual_improvement,
            }
        )
        return ProbeTransition(False, True, reason)

    def _samples(self, duration_s: float) -> int:
        return max(1, int(math.ceil(duration_s / self.config.controller_dt_s)))


def auto_probe_config_from_dict(
    raw: dict,
    *,
    probe_seed: int,
) -> AutoProbeConfig:
    config = raw["auto_probe_manager"]
    probe = raw["probe"]
    controller_dt = 1.0 / float(raw["controller_frequency_hz"])
    return AutoProbeConfig(
        controller_dt_s=controller_dt,
        residual_reference_scaled_rms=float(
            config["residual_reference_scaled_rms"]
        ),
        residual_ewma_time_constant_s=float(
            config["residual_ewma_time_constant_s"]
        ),
        trigger_ratio=float(config["trigger_ratio"]),
        trigger_absolute_floor=float(config["trigger_absolute_floor"]),
        trigger_persistence_s=float(config["trigger_persistence_s"]),
        clear_ratio=float(config["clear_ratio"]),
        clear_absolute_floor=float(config["clear_absolute_floor"]),
        clear_persistence_s=float(config["clear_persistence_s"]),
        cooldown_s=float(config["cooldown_s"]),
        minimum_session_duration_s=float(config["minimum_session_duration_s"]),
        maximum_session_duration_s=float(config["maximum_session_duration_s"]),
        minimum_effective_updates=int(config["minimum_effective_updates"]),
        pe_qualified_persistence_s=float(config["pe_qualified_persistence_s"]),
        plateau_window_s=float(config["plateau_window_s"]),
        maximum_parameter_relative_span=float(
            config["maximum_parameter_relative_span"]
        ),
        maximum_residual_relative_improvement_for_plateau=float(
            config["maximum_residual_relative_improvement_for_plateau"]
        ),
        probe_amplitude_sum_torque_nm=float(probe["amplitude_sum_torque_nm"]),
        probe_bit_period_s=float(probe["bit_period_s"]),
        probe_lfsr_bits=int(probe["lfsr_bits"]),
        probe_seed=int(probe_seed),
    )
