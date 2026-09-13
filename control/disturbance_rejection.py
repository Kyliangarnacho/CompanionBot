"""Two-timescale matched-disturbance rejection around a fixed nominal LQR.

The controller deliberately does not identify a new operating point or redesign
the LQR.  It projects the normalized one-step innovation onto the known nominal
input direction, then splits that equivalent input disturbance into slow and
fast complementary bands.  Payload ground truth is never an input.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray

from .full_state_identification import DiscreteStateSpaceModel


@dataclass(frozen=True)
class DisturbanceRejectionConfig:
    controller_dt_s: float
    slow_cutoff_hz: float
    fast_cutoff_hz: float
    innovation_projection_bound_nm: float
    slow_compensation_bound_nm: float
    fast_compensation_bound_nm: float
    combined_compensation_bound_nm: float
    combined_slew_rate_nm_s: float
    projection_ridge: float


@dataclass(frozen=True)
class DisturbanceObservation:
    prediction: NDArray[np.float64]
    normalized_innovation: NDArray[np.float64]
    scaled_innovation_rms: float
    matched_disturbance_raw_nm: float
    matched_disturbance_projected_nm: float
    matched_residual_fraction: float
    projection_clipped: bool


@dataclass(frozen=True)
class CompensationCommand:
    slow_nm: float
    fast_nm: float
    total_nm: float
    slow_estimate_nm: float
    fast_band_estimate_nm: float


def config_from_dict(raw: dict) -> DisturbanceRejectionConfig:
    return DisturbanceRejectionConfig(
        controller_dt_s=float(raw["controller_dt_s"]),
        slow_cutoff_hz=float(raw["slow_cutoff_hz"]),
        fast_cutoff_hz=float(raw["fast_cutoff_hz"]),
        innovation_projection_bound_nm=float(
            raw["innovation_projection_bound_nm"]
        ),
        slow_compensation_bound_nm=float(raw["slow_compensation_bound_nm"]),
        fast_compensation_bound_nm=float(raw["fast_compensation_bound_nm"]),
        combined_compensation_bound_nm=float(
            raw["combined_compensation_bound_nm"]
        ),
        combined_slew_rate_nm_s=float(raw["combined_slew_rate_nm_s"]),
        projection_ridge=float(raw["projection_ridge"]),
    )


class TwoTimescaleDisturbanceCompensator:
    """Matched EID/fast-DOB augmentation for a fixed discrete model.

    At the end of interval k, ``observe`` compares measured x[k+1] with the
    nominal prediction from x[k] and the torque actually held over that complete
    interval.  The resulting scalar estimate is available for interval k+1.

    The fast low-pass estimate contains all supported compensation bandwidth.
    The slow estimate carries its low-frequency portion; their difference is the
    fast band.  Consequently ``u_slow + u_fast`` does not double-count DC bias.
    """

    def __init__(
        self,
        model: DiscreteStateSpaceModel,
        state_scales: NDArray[np.float64],
        input_scale_nm: float,
        config: DisturbanceRejectionConfig,
    ):
        self.model = model
        self.state_scales = np.asarray(state_scales, dtype=float)
        self.input_scale_nm = float(input_scale_nm)
        self.config = config
        if self.state_scales.shape != (4,) or np.any(self.state_scales <= 0.0):
            raise ValueError("state_scales must contain four positive values")
        if self.input_scale_nm <= 0.0:
            raise ValueError("input_scale_nm must be positive")
        if not 0.0 < config.slow_cutoff_hz < config.fast_cutoff_hz:
            raise ValueError("require 0 < slow cutoff < fast cutoff")
        if config.fast_cutoff_hz >= 0.5 / config.controller_dt_s:
            raise ValueError("fast cutoff must remain below controller Nyquist")

        # In normalized coordinates, this is the response to one normalized
        # input unit.  It provides a dimensionally consistent least-squares
        # projection despite very different state units.
        self._normalized_input_direction = (
            self.model.B[:, 0] * self.input_scale_nm / self.state_scales
        )
        self._input_information = float(
            self._normalized_input_direction @ self._normalized_input_direction
        )
        if self._input_information <= np.finfo(float).eps:
            raise ValueError("nominal B has no usable matched input direction")
        self._alpha_slow = 1.0 - math.exp(
            -2.0 * math.pi * config.slow_cutoff_hz * config.controller_dt_s
        )
        self._alpha_fast = 1.0 - math.exp(
            -2.0 * math.pi * config.fast_cutoff_hz * config.controller_dt_s
        )
        self.reset()

    def reset(self) -> None:
        self.slow_estimate_nm = 0.0
        self.fast_lowpass_estimate_nm = 0.0
        self._previous_slow_compensation_nm = 0.0
        self._previous_fast_compensation_nm = 0.0
        self.observation_count = 0
        self.clipped_observation_count = 0
        self.last_observation: DisturbanceObservation | None = None

    def observe(
        self,
        state_k: NDArray[np.float64],
        held_sum_torque_nm: float,
        state_k1: NDArray[np.float64],
    ) -> DisturbanceObservation:
        state = np.asarray(state_k, dtype=float)
        next_state = np.asarray(state_k1, dtype=float)
        prediction = (
            self.model.A @ state
            + self.model.B[:, 0] * float(held_sum_torque_nm)
        )
        normalized_innovation = (next_state - prediction) / self.state_scales
        denominator = self._input_information + self.config.projection_ridge
        normalized_disturbance = float(
            self._normalized_input_direction @ normalized_innovation
        ) / denominator
        disturbance_raw_nm = normalized_disturbance * self.input_scale_nm
        disturbance_projected_nm = float(
            np.clip(
                disturbance_raw_nm,
                -self.config.innovation_projection_bound_nm,
                self.config.innovation_projection_bound_nm,
            )
        )
        clipped = not np.isclose(
            disturbance_raw_nm, disturbance_projected_nm, rtol=0.0, atol=1e-12
        )

        projected_residual = (
            self._normalized_input_direction
            * (disturbance_projected_nm / self.input_scale_nm)
        )
        residual_norm = float(np.linalg.norm(normalized_innovation))
        matched_fraction = float(
            min(1.0, np.linalg.norm(projected_residual) / max(residual_norm, 1e-12))
        )

        self.slow_estimate_nm += self._alpha_slow * (
            disturbance_projected_nm - self.slow_estimate_nm
        )
        self.fast_lowpass_estimate_nm += self._alpha_fast * (
            disturbance_projected_nm - self.fast_lowpass_estimate_nm
        )
        self.observation_count += 1
        self.clipped_observation_count += int(clipped)
        observation = DisturbanceObservation(
            prediction=prediction,
            normalized_innovation=normalized_innovation,
            scaled_innovation_rms=float(
                np.sqrt(np.mean(normalized_innovation**2))
            ),
            matched_disturbance_raw_nm=disturbance_raw_nm,
            matched_disturbance_projected_nm=disturbance_projected_nm,
            matched_residual_fraction=matched_fraction,
            projection_clipped=clipped,
        )
        self.last_observation = observation
        return observation

    def command(
        self,
        *,
        enable_slow: bool = True,
        enable_fast: bool = True,
    ) -> CompensationCommand:
        slow_estimate = self.slow_estimate_nm
        fast_band_estimate = self.fast_lowpass_estimate_nm - slow_estimate
        slow = (
            -float(
                np.clip(
                    slow_estimate,
                    -self.config.slow_compensation_bound_nm,
                    self.config.slow_compensation_bound_nm,
                )
            )
            if enable_slow
            else 0.0
        )
        fast = (
            -float(
                np.clip(
                    fast_band_estimate,
                    -self.config.fast_compensation_bound_nm,
                    self.config.fast_compensation_bound_nm,
                )
            )
            if enable_fast
            else 0.0
        )
        target_total = slow + fast
        if abs(target_total) > self.config.combined_compensation_bound_nm:
            scale = self.config.combined_compensation_bound_nm / abs(target_total)
            slow *= scale
            fast *= scale
        maximum_step = (
            self.config.combined_slew_rate_nm_s * self.config.controller_dt_s
        )
        # Both channels share one aggregate slew budget.  This preserves the
        # individual projection bounds even when their desired values nearly
        # cancel, and guarantees that the applied sum changes by at most the
        # configured amount per controller interval.
        slow_delta = float(
            np.clip(
                slow - self._previous_slow_compensation_nm,
                -maximum_step,
                maximum_step,
            )
        )
        slow = self._previous_slow_compensation_nm + slow_delta
        remaining_step = max(0.0, maximum_step - abs(slow_delta))
        fast_delta = float(
            np.clip(
                fast - self._previous_fast_compensation_nm,
                -remaining_step,
                remaining_step,
            )
        )
        fast = self._previous_fast_compensation_nm + fast_delta
        total = slow + fast
        if abs(total) > self.config.combined_compensation_bound_nm:
            scale = self.config.combined_compensation_bound_nm / abs(total)
            slow *= scale
            fast *= scale
            total = slow + fast
        self._previous_slow_compensation_nm = slow
        self._previous_fast_compensation_nm = fast
        return CompensationCommand(
            slow_nm=slow,
            fast_nm=fast,
            total_nm=total,
            slow_estimate_nm=slow_estimate,
            fast_band_estimate_nm=fast_band_estimate,
        )
