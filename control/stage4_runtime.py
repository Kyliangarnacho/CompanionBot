"""Frozen Stage 4 runtime: two-mode slope arbitration and short payload ID.

Q remains diagnostic-only.  Its filtered matched projection can request one
short, natural-transient payload identification session, but is never applied
to the actuators and is never interpreted as payload ground truth.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import cont2discrete

from sim.slope_estimation import SlopePlantParameters


class EnvironmentMode(str, Enum):
    FLAT = "FLAT"
    SLOPE = "SLOPE"


@dataclass(frozen=True)
class SlopeSupervisorConfig:
    enter_error_deg: float = 3.0
    enter_persistence_s: float = 0.20
    enter_minimum_abs_velocity_m_s: float = 0.08
    theta_dyn_epsilon_rad: float = 1e-12
    exit_abs_alpha_deg: float = 0.7
    exit_abs_error_deg: float = 3.0
    exit_persistence_s: float = 0.75
    control_maximum_std_deg: float = 1.0


@dataclass(frozen=True)
class SlopeSupervisorOutput:
    mode: EnvironmentMode
    previous_mode: EnvironmentMode
    transition_reason: str | None
    theta_error_rad: float
    alpha_control_rad: float
    enter_timer_s: float
    exit_timer_s: float


class SlopeSupervisor:
    """Two-mode supervisor.  There is deliberately no payload-ID state."""

    def __init__(self, config: SlopeSupervisorConfig) -> None:
        self.config = config
        self.mode = EnvironmentMode.FLAT
        self.enter_timer_s = 0.0
        self.exit_timer_s = 0.0
        self._trusted_alpha_rad = 0.0

    def update(
        self, *, dt_s: float, theta_hat_rad: float, theta_eq_rad: float,
        theta_dyn_ref_rad: float, velocity_hat_m_s: float,
        alpha_hat_rad: float, alpha_std_deg: float,
    ) -> SlopeSupervisorOutput:
        dt = float(dt_s)
        previous = self.mode
        reason = None
        theta_error = _wrap(
            float(theta_hat_rad) - (float(theta_eq_rad) + float(theta_dyn_ref_rad))
        )
        if self.mode is EnvironmentMode.FLAT:
            if abs(theta_dyn_ref_rad) > self.config.theta_dyn_epsilon_rad:
                self.enter_timer_s = 0.0
            else:
                evidence = (
                    abs(math.degrees(theta_error)) > self.config.enter_error_deg
                    and abs(velocity_hat_m_s)
                    > self.config.enter_minimum_abs_velocity_m_s
                )
                self.enter_timer_s = self.enter_timer_s + dt if evidence else 0.0
            if self.enter_timer_s + 1e-12 >= self.config.enter_persistence_s:
                self.mode = EnvironmentMode.SLOPE
                self.enter_timer_s = 0.0
                self.exit_timer_s = 0.0
                reason = "persistent_pitch_error"
        else:
            exit_evidence = (
                abs(math.degrees(alpha_hat_rad)) < self.config.exit_abs_alpha_deg
                and abs(math.degrees(theta_error)) < self.config.exit_abs_error_deg
            )
            self.exit_timer_s = self.exit_timer_s + dt if exit_evidence else 0.0
            if self.exit_timer_s + 1e-12 >= self.config.exit_persistence_s:
                self.mode = EnvironmentMode.FLAT
                self.enter_timer_s = 0.0
                self.exit_timer_s = 0.0
                self._trusted_alpha_rad = 0.0
                reason = "persistent_flat_return"

        if (
            self.mode is EnvironmentMode.SLOPE
            and alpha_std_deg < self.config.control_maximum_std_deg
        ):
            self._trusted_alpha_rad = float(alpha_hat_rad)
        alpha_control = (
            self._trusted_alpha_rad
            if self.mode is EnvironmentMode.SLOPE else 0.0
        )
        return SlopeSupervisorOutput(
            mode=self.mode,
            previous_mode=previous,
            transition_reason=reason,
            theta_error_rad=theta_error,
            alpha_control_rad=alpha_control,
            enter_timer_s=self.enter_timer_s,
            exit_timer_s=self.exit_timer_s,
        )


@dataclass(frozen=True)
class QChangeTriggerConfig:
    dt_s: float = 0.002
    ewma_time_constant_s: float = 0.05
    enter_abs_delta_q_nm: float = 0.006
    exit_abs_delta_q_nm: float = 0.003
    enter_persistence_s: float = 0.08
    baseline_persistence_s: float = 0.25


class PayloadLifecycle:
    """One-shot-per-mismatch latch with only pending/active public flags."""

    def __init__(self, config: QChangeTriggerConfig) -> None:
        self.config = config
        self.payload_id_pending = False
        self.payload_id_active = False
        self.q_baseline_nm = 0.0
        self.delta_q_nm = 0.0
        self.ewma_abs_delta_q_nm = 0.0
        self.events: list[dict] = []
        self._baseline_ready = False
        self._baseline_count = 0
        self._baseline_ewma_nm = 0.0
        self._trigger_count = 0
        self._armed = False
        self._pending_seen_quiet = False
        self._needs_baseline_refresh = True

    @property
    def baseline_ready(self) -> bool:
        return self._baseline_ready

    def observe(
        self, *, time_s: float, q_filtered_nm: float,
        flat: bool, transient: bool,
    ) -> str | None:
        q_value = float(q_filtered_nm)
        if not flat:
            self._trigger_count = 0
            if self.payload_id_active:
                self.payload_id_active = False
                self.events.append({
                    "time_s": float(time_s), "event": "id_aborted_on_slope",
                })
                return "id_aborted_on_slope"
            return None

        if self._needs_baseline_refresh:
            if transient:
                self._baseline_count = 0
                return None
            alpha = 1.0 - math.exp(
                -self.config.dt_s / self.config.ewma_time_constant_s
            )
            self._baseline_ewma_nm += alpha * (q_value - self._baseline_ewma_nm)
            self._baseline_count += 1
            needed = _samples(
                self.config.baseline_persistence_s, self.config.dt_s
            )
            if self._baseline_count >= needed:
                self.q_baseline_nm = self._baseline_ewma_nm
                self.delta_q_nm = 0.0
                self.ewma_abs_delta_q_nm = 0.0
                self._baseline_ready = True
                self._needs_baseline_refresh = False
                self._armed = True
                self.events.append({
                    "time_s": float(time_s), "event": "q_baseline_updated",
                    "q_baseline_nm": self.q_baseline_nm,
                })
                return "q_baseline_updated"
            return None

        self.delta_q_nm = q_value - self.q_baseline_nm
        alpha = 1.0 - math.exp(
            -self.config.dt_s / self.config.ewma_time_constant_s
        )
        self.ewma_abs_delta_q_nm += alpha * (
            abs(self.delta_q_nm) - self.ewma_abs_delta_q_nm
        )

        if self.payload_id_pending:
            if not transient:
                self._pending_seen_quiet = True
            elif self._pending_seen_quiet:
                self.payload_id_pending = False
                self.payload_id_active = True
                self.events.append({
                    "time_s": float(time_s), "event": "payload_id_started",
                })
                return "payload_id_started"
            return None
        if self.payload_id_active or not self._armed:
            return None

        # Compare Q only in the same non-transient operating condition used
        # for q_baseline.  Command transients are reserved for the later ID
        # session and cannot themselves create another mismatch episode.
        if transient:
            self._trigger_count = 0
            return None

        if self.ewma_abs_delta_q_nm >= self.config.enter_abs_delta_q_nm:
            self._trigger_count += 1
        elif self.ewma_abs_delta_q_nm <= self.config.exit_abs_delta_q_nm:
            self._trigger_count = 0
        if self._trigger_count < _samples(
            self.config.enter_persistence_s, self.config.dt_s
        ):
            return None
        self.payload_id_pending = True
        self._pending_seen_quiet = False
        self._armed = False
        self._trigger_count = 0
        self.events.append({
            "time_s": float(time_s), "event": "payload_id_pending",
            "delta_q_nm": self.delta_q_nm,
            "ewma_abs_delta_q_nm": self.ewma_abs_delta_q_nm,
        })
        return "payload_id_pending"

    def finish_identification(self, time_s: float, accepted: bool) -> None:
        self.payload_id_active = False
        self.payload_id_pending = False
        self._pending_seen_quiet = False
        self._baseline_ready = False
        self._baseline_count = 0
        self._baseline_ewma_nm = self.q_baseline_nm
        self._needs_baseline_refresh = True
        self.events.append({
            "time_s": float(time_s),
            "event": "payload_id_finished",
            "accepted": bool(accepted),
        })


@dataclass(frozen=True)
class SagittalPayloadParameters:
    mass_kg: float
    forward_first_moment_kg_m: float
    known_height_m: float

    @property
    def forward_m(self) -> float:
        return self.forward_first_moment_kg_m / self.mass_kg


@dataclass(frozen=True)
class PayloadAwarePlant:
    payload: SagittalPayloadParameters
    body_mass_kg: float
    body_com_forward_m: float
    body_com_height_m: float
    slope_plant: SlopePlantParameters


class PayloadPlantBuilder:
    """Map the two sagittal payload base parameters into the frozen plant."""

    def __init__(
        self, reduced: Mapping, known_height_m: float,
        payload_full_size_m: np.ndarray | None = None,
    ) -> None:
        p = reduced["parameters"]
        self.known_height_m = float(known_height_m)
        self.nominal_body_mass_kg = float(p["body_mass_kg"])
        self.nominal_forward_m = float(p["body_com_forward_m"])
        self.nominal_height_m = float(p["body_com_height_m"])
        self.nominal_equivalent_mass_kg = float(p["equivalent_translation_mass_kg"])
        self.wheel_mass_kg = float(p["single_wheel_rotating_mass_kg"])
        self.wheel_radius_m = float(p["wheel_radius_m"])
        self.nominal_pitch_inertia_axle_kg_m2 = float(
            p["body_pitch_inertia_about_axle_kg_m2"]
        )
        size = (
            np.asarray([0.08, 0.04, 0.04], dtype=float)
            if payload_full_size_m is None
            else np.asarray(payload_full_size_m, dtype=float)
        )
        self.payload_pitch_inertia_com_per_kg_m2 = float(
            (size[1] ** 2 + size[2] ** 2) / 12.0
        )

    def empty(self) -> PayloadAwarePlant:
        return self.build(SagittalPayloadParameters(0.0, 0.0, self.known_height_m))

    def build(self, payload: SagittalPayloadParameters) -> PayloadAwarePlant:
        mass = float(payload.mass_kg)
        if mass < 0.0 or not math.isfinite(mass):
            raise ValueError("payload mass must be finite and nonnegative")
        forward_first = (
            self.nominal_body_mass_kg * self.nominal_forward_m
            + payload.forward_first_moment_kg_m
        )
        vertical_first = (
            self.nominal_body_mass_kg * self.nominal_height_m
            + mass * payload.known_height_m
        )
        body_mass = self.nominal_body_mass_kg + mass
        forward = forward_first / body_mass
        height = vertical_first / body_mass
        length = math.hypot(forward, height)
        theta_flat = -math.atan2(forward, height)
        plant = SlopePlantParameters(
            body_mass_kg=body_mass,
            gravitational_mass_kg=body_mass + 2.0 * self.wheel_mass_kg,
            equivalent_translation_mass_kg=self.nominal_equivalent_mass_kg + mass,
            body_com_length_m=length,
            wheel_radius_m=self.wheel_radius_m,
            theta_flat_rad=theta_flat,
        )
        return PayloadAwarePlant(payload, body_mass, forward, height, plant)

    def continuous_state_space(
        self, payload: SagittalPayloadParameters,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """Physical small-angle TWIP model derived from payload mass properties."""
        model = self.build(payload)
        mass = payload.mass_kg
        forward = 0.0 if mass <= 0.0 else payload.forward_m
        first_moment = model.body_mass_kg * model.slope_plant.body_com_length_m
        pitch_inertia = self.nominal_pitch_inertia_axle_kg_m2 + mass * (
            forward * forward + payload.known_height_m * payload.known_height_m
            + self.payload_pitch_inertia_com_per_kg_m2
        )
        inertia = np.asarray([
            [model.slope_plant.equivalent_translation_mass_kg, first_moment],
            [first_moment, pitch_inertia],
        ], dtype=float)
        gravity_column = np.linalg.solve(
            inertia, np.asarray([0.0, first_moment * 9.81], dtype=float)
        )
        input_column = np.linalg.solve(
            inertia, np.asarray([1.0 / self.wheel_radius_m, -1.0], dtype=float)
        )
        a = np.zeros((4, 4), dtype=float)
        a[0, 1] = 1.0
        a[1, 2] = gravity_column[0]
        a[2, 3] = 1.0
        a[3, 2] = gravity_column[1]
        b = np.zeros((4, 1), dtype=float)
        b[1, 0] = input_column[0]
        b[3, 0] = input_column[1]
        return a, b, model.slope_plant.theta_flat_rad

    def discrete_state_space(
        self, payload: SagittalPayloadParameters, dt_s: float,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        a, b, theta_flat = self.continuous_state_space(payload)
        ad, bd, _, _, _ = cont2discrete(
            (a, b, np.eye(4), np.zeros((4, 1))), float(dt_s), method="zoh"
        )
        return np.asarray(ad), np.asarray(bd), theta_flat


@dataclass(frozen=True)
class PayloadIdConfig:
    update_period_s: float = 0.02
    minimum_updates: int = 30
    maximum_updates: int = 100
    minimum_abs_reference_acceleration_m_s2: float = 0.12
    maximum_condition_number: float = 1000.0
    minimum_mass_kg: float = 0.02
    maximum_mass_kg: float = 0.50
    minimum_forward_m: float = -0.03
    maximum_forward_m: float = 0.03


@dataclass(frozen=True)
class PayloadIdResult:
    accepted: bool
    reason: str
    update_count: int
    rank: int
    condition_number: float
    parameters: SagittalPayloadParameters | None
    normalized_residual_rms: float | None
    estimated_mass_kg: float | None
    estimated_forward_position_m: float | None


class NaturalTransientPayloadId:
    """One short 50 Hz batch least-squares session; no candidate layer."""

    def __init__(
        self, config: PayloadIdConfig, builder: PayloadPlantBuilder,
        state_scales: np.ndarray,
    ) -> None:
        self.config = config
        self.builder = builder
        self.state_scales = np.asarray(state_scales, dtype=float)
        if self.state_scales.shape != (4,) or np.any(self.state_scales <= 0.0):
            raise ValueError("state_scales must contain four positive values")
        self.active = False
        self.update_count = 0
        self.last_result: PayloadIdResult | None = None
        self._elapsed_s = 0.0
        self._saw_excitation = False
        self._previous_state: np.ndarray | None = None
        self._states_k: list[np.ndarray] = []
        self._states_k1: list[np.ndarray] = []
        self._inputs_nm: list[float] = []
        self._input_integral_nm_s = 0.0

    def start(self) -> None:
        self.active = True
        self.update_count = 0
        self._elapsed_s = 0.0
        self._saw_excitation = False
        self._previous_state = None
        self._states_k.clear()
        self._states_k1.clear()
        self._inputs_nm.clear()
        self._input_integral_nm_s = 0.0
        self.last_result = None

    def abort(self) -> None:
        self.active = False
        self._previous_state = None
        self._states_k.clear()
        self._states_k1.clear()
        self._inputs_nm.clear()

    def observe(
        self, *, dt_s: float, reference_acceleration_m_s2: float,
        saturated: bool, state_absolute: np.ndarray,
        actual_sum_torque_nm: float,
    ) -> PayloadIdResult | None:
        if not self.active:
            return None
        excited = (
            abs(reference_acceleration_m_s2)
            > self.config.minimum_abs_reference_acceleration_m_s2
            and not saturated
        )
        if not excited:
            if self._saw_excitation:
                return self._finish()
            return None
        self._saw_excitation = True
        self._elapsed_s += float(dt_s)
        self._input_integral_nm_s += float(actual_sum_torque_nm) * float(dt_s)
        if self._elapsed_s + 1e-12 < self.config.update_period_s:
            return None
        interval = self._elapsed_s
        state = np.asarray(state_absolute, dtype=float).copy()
        average_input = self._input_integral_nm_s / interval
        self._elapsed_s = 0.0
        self._input_integral_nm_s = 0.0
        if self._previous_state is not None and np.isfinite(state).all():
            self._states_k.append(self._previous_state)
            self._states_k1.append(state)
            self._inputs_nm.append(float(average_input))
            self.update_count += 1
        self._previous_state = state
        if self.update_count >= self.config.maximum_updates:
            return self._finish()
        return None

    def _finish(self) -> PayloadIdResult:
        self.active = False
        count = len(self._states_k)
        if count < self.config.minimum_updates:
            return self._record(False, "insufficient_updates", count, 0, math.inf, None, None, None, None)
        states_k = np.asarray(self._states_k, dtype=float)
        states_k1 = np.asarray(self._states_k1, dtype=float)
        inputs = np.asarray(self._inputs_nm, dtype=float)

        def residual(vector: np.ndarray) -> np.ndarray:
            mass, forward = map(float, vector)
            payload = SagittalPayloadParameters(
                mass, mass * forward, self.builder.known_height_m
            )
            ad, bd, theta_flat = self.builder.discrete_state_space(
                payload, self.config.update_period_s
            )
            current = states_k.copy()
            following = states_k1.copy()
            current[:, 2] -= theta_flat
            following[:, 2] -= theta_flat
            predicted = current @ ad.T + inputs[:, None] * bd[:, 0]
            return ((following - predicted) / self.state_scales).reshape(-1)

        result = least_squares(
            residual,
            x0=np.asarray([
                0.5 * (self.config.minimum_mass_kg + self.config.maximum_mass_kg),
                0.0,
            ]),
            bounds=(
                np.asarray([self.config.minimum_mass_kg, self.config.minimum_forward_m]),
                np.asarray([self.config.maximum_mass_kg, self.config.maximum_forward_m]),
            ),
            loss="linear",
            max_nfev=100,
        )
        singular = np.linalg.svd(result.jac, compute_uv=False)
        rank = int(np.linalg.matrix_rank(result.jac))
        condition = (
            math.inf if len(singular) < 2 or singular[-1] <= 0.0
            else float(singular[0] / singular[-1])
        )
        normalized_residual_rms = float(np.sqrt(np.mean(result.fun * result.fun)))
        mass, forward = map(float, result.x)
        if not result.success:
            return self._record(False, "solver_failed", count, rank, condition, None, normalized_residual_rms, mass, forward)
        if rank != 2 or condition > self.config.maximum_condition_number:
            return self._record(False, "unidentifiable", count, rank, condition, None, normalized_residual_rms, mass, forward)
        physical = (
            self.config.minimum_mass_kg <= mass <= self.config.maximum_mass_kg
            and self.config.minimum_forward_m <= forward <= self.config.maximum_forward_m
        )
        if not physical:
            return self._record(False, "nonphysical_parameters", count, rank, condition, None, normalized_residual_rms, mass, forward)
        parameters = SagittalPayloadParameters(
            mass_kg=mass,
            forward_first_moment_kg_m=mass * forward,
            known_height_m=self.builder.known_height_m,
        )
        return self._record(True, "accepted", count, rank, condition, parameters, normalized_residual_rms, mass, forward)

    def _record(
        self, accepted: bool, reason: str, count: int, rank: int,
        condition: float, parameters: SagittalPayloadParameters | None,
        residual_rms: float | None, estimated_mass: float | None,
        estimated_forward: float | None,
    ) -> PayloadIdResult:
        self.last_result = PayloadIdResult(
            accepted, reason, count, rank, condition, parameters, residual_rms,
            estimated_mass, estimated_forward,
        )
        return self.last_result


def _samples(duration_s: float, dt_s: float) -> int:
    return max(1, int(math.ceil(float(duration_s) / float(dt_s))))


def _wrap(angle_rad: float) -> float:
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi
