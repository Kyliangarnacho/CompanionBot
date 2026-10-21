"""Single-channel matched-disturbance rejection around a fixed nominal LQR.

The observer projects the normalized one-step state innovation onto the known
nominal input direction. A single bounded Q-filter estimate then produces one
actuator augmentation command.
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
    q_filter_cutoff_hz: float
    innovation_projection_bound_nm: float
    augmentation_authority_bound_nm: float
    augmentation_slew_rate_nm_s: float | None
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
    requested_u_dr_nm: float
    u_dr_nm: float
    q_filter_estimate_nm: float
    authority_limited: bool
    slew_limited: bool


def config_from_dict(raw: dict) -> DisturbanceRejectionConfig:
    q_filter = raw["q_filter"]
    slew_rate = raw.get("augmentation_slew_rate_nm_s")
    return DisturbanceRejectionConfig(
        controller_dt_s=float(raw["controller_dt_s"]),
        q_filter_cutoff_hz=float(q_filter["selected_cutoff_hz"]),
        innovation_projection_bound_nm=float(
            raw["innovation_projection_bound_nm"]
        ),
        augmentation_authority_bound_nm=float(
            raw["augmentation_authority_bound_nm"]
        ),
        augmentation_slew_rate_nm_s=(
            None if slew_rate is None else float(slew_rate)
        ),
        projection_ridge=float(raw["projection_ridge"]),
    )


class FilteredDisturbanceCompensator:
    """One-dimensional matched DOB/L1-style filtered augmentation.

    At the end of interval k, :meth:`observe` compares measured x[k+1] with the
    nominal prediction from x[k] and the torque actually held during the whole
    interval. The projected equivalent input disturbance is bounded before a
    single Q-filter. :meth:`command` applies the augmentation authority bound,
    followed by an optional final slew safety envelope.
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
        if not 0.0 < config.q_filter_cutoff_hz < 0.5 / config.controller_dt_s:
            raise ValueError("Q-filter cutoff must lie between zero and Nyquist")
        if config.innovation_projection_bound_nm <= 0.0:
            raise ValueError("innovation projection bound must be positive")
        if config.augmentation_authority_bound_nm <= 0.0:
            raise ValueError("augmentation authority bound must be positive")
        if (
            config.augmentation_slew_rate_nm_s is not None
            and config.augmentation_slew_rate_nm_s <= 0.0
        ):
            raise ValueError("augmentation slew rate must be positive or disabled")

        self._configure_model(model)
        self._alpha_q = self._lowpass_alpha(config.q_filter_cutoff_hz)
        self.reset()

    def _configure_model(self, model: DiscreteStateSpaceModel) -> None:
        self.model = model
        self._normalized_input_direction = (
            self.model.B[:, 0] * self.input_scale_nm / self.state_scales
        )
        self._input_information = float(
            self._normalized_input_direction @ self._normalized_input_direction
        )
        if self._input_information <= np.finfo(float).eps:
            raise ValueError("model B has no usable matched input direction")

    def update_model(self, model: DiscreteStateSpaceModel) -> None:
        """Switch prediction to the accepted operating model without resetting Q."""
        self._configure_model(model)

    def _lowpass_alpha(self, cutoff_hz: float) -> float:
        return 1.0 - math.exp(
            -2.0 * math.pi * cutoff_hz * self.config.controller_dt_s
        )

    def reset(self) -> None:
        self.q_filter_estimate_nm = 0.0
        self._previous_u_dr_nm = 0.0
        self.observation_count = 0
        self.clipped_observation_count = 0
        self.command_count = 0
        self.authority_limit_count = 0
        self.slew_limit_count = 0
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

        self.q_filter_estimate_nm += self._alpha_q * (
            disturbance_projected_nm - self.q_filter_estimate_nm
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

    def command(self, *, enabled: bool = True) -> CompensationCommand:
        requested = -self.q_filter_estimate_nm if enabled else 0.0
        authority_limited_command = float(
            np.clip(
                requested,
                -self.config.augmentation_authority_bound_nm,
                self.config.augmentation_authority_bound_nm,
            )
        )
        authority_limited = not np.isclose(
            requested, authority_limited_command, rtol=0.0, atol=1e-12
        )

        command = authority_limited_command
        slew_limited = False
        if self.config.augmentation_slew_rate_nm_s is not None:
            maximum_step = (
                self.config.augmentation_slew_rate_nm_s
                * self.config.controller_dt_s
            )
            command = self._previous_u_dr_nm + float(
                np.clip(
                    authority_limited_command - self._previous_u_dr_nm,
                    -maximum_step,
                    maximum_step,
                )
            )
            slew_limited = not np.isclose(
                command, authority_limited_command, rtol=0.0, atol=1e-12
            )

        self._previous_u_dr_nm = command
        self.command_count += 1
        self.authority_limit_count += int(authority_limited)
        self.slew_limit_count += int(slew_limited)
        return CompensationCommand(
            requested_u_dr_nm=requested,
            u_dr_nm=command,
            q_filter_estimate_nm=self.q_filter_estimate_nm,
            authority_limited=authority_limited,
            slew_limited=slew_limited,
        )
