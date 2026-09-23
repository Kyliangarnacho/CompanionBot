"""Stage 5 rolling velocity previews and in-memory reference blocks.

Ruckig owns only the one-dimensional jerk-limited ``p/v/a`` retarget.  The
existing CompanionBot projection still supplies pitch, pitch rate, and nominal
feedforward for the frozen identified TWIP model.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from collections import deque
import math
import time
from typing import Callable

import numpy as np
from numpy.typing import NDArray
from ruckig import (
    ControlInterface,
    DurationDiscretization,
    InputParameter,
    Result,
    Ruckig,
    Trajectory,
)

from .reference_lifecycle import (
    VelocityLifecyclePhase,
    VelocityReferenceLifecycle,
    VelocityTransitionPlan,
)
from .trajectory_feedforward import (
    SparseProjectionFactorizationCache,
    project_hidden_reference_and_input,
)


@dataclass(frozen=True)
class TargetObservation:
    capture_time_s: float
    x_forward_m: float
    y_left_m: float


@dataclass(frozen=True)
class MotionIntent:
    source_time_s: float
    linear_velocity_target_m_s: float
    yaw_rate_target_rad_s: float


@dataclass(frozen=True)
class ReferenceBlock:
    start_time_s: float
    dt_s: float
    p_ref: NDArray[np.float64]
    v_ref: NDArray[np.float64]
    a_ref: NDArray[np.float64]
    theta_ref: NDArray[np.float64]
    theta_dot_ref: NDArray[np.float64]
    u_ff_raw: NDArray[np.float64]
    yaw_rate_target_rad_s: float
    sequence_id: int = 0
    start_control_tick: int = 0
    generated_time_s: float = 0.0
    available_time_s: float = 0.0
    mode: str = "NORMAL"

    def __post_init__(self) -> None:
        arrays = tuple(
            np.asarray(value, dtype=float)
            for value in (
                self.p_ref,
                self.v_ref,
                self.a_ref,
                self.theta_ref,
                self.theta_dot_ref,
                self.u_ff_raw,
            )
        )
        if not arrays or arrays[0].ndim != 1 or arrays[0].size == 0:
            raise ValueError("reference block arrays must be non-empty vectors")
        if any(value.shape != arrays[0].shape for value in arrays[1:]):
            raise ValueError("reference block arrays must have equal lengths")
        if not math.isfinite(self.start_time_s) or not math.isfinite(self.dt_s):
            raise ValueError("reference block timing must be finite")
        if self.dt_s <= 0.0 or not math.isfinite(self.yaw_rate_target_rad_s):
            raise ValueError("reference block dt must be positive and yaw finite")
        if self.sequence_id < 0 or self.start_control_tick < 0:
            raise ValueError("reference block sequence/tick must be non-negative")
        if not np.isfinite([
            self.generated_time_s, self.available_time_s
        ]).all():
            raise ValueError("reference block delivery times must be finite")
        if not np.isfinite(np.concatenate(arrays)).all():
            raise ValueError("reference block samples must be finite")

    @property
    def sample_count(self) -> int:
        return int(np.asarray(self.p_ref).size)

    @property
    def sample_matrix_float32(self) -> NDArray[np.float32]:
        """Wire-order samples: p, v, a, theta, theta_dot, applied u_ff."""
        return np.column_stack((
            self.p_ref, self.v_ref, self.a_ref,
            self.theta_ref, self.theta_dot_ref, self.u_ff_raw,
        )).astype(np.float32, copy=False)


@dataclass(frozen=True)
class RollingVelocityPlan:
    planning_mode: str
    plan: VelocityTransitionPlan
    ruckig_duration_s: float
    dynamic_nominal_horizon_s: float
    normalized_residual_rms: float
    projection_diagnostics: tuple[dict, ...]
    planning_diagnostics: dict = field(default_factory=dict)


class _RuckigVelocityPlannerBase:
    """Shared Ruckig setup and sampling primitives for Stage 5 V1.5 paths."""

    def __init__(
        self,
        *,
        controller_dt_s: float,
        max_velocity_m_s: float,
        max_acceleration_m_s2: float,
        max_jerk_m_s3: float,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        state_scales: NDArray[np.float64],
        input_scale_nm: float,
    ) -> None:
        self.dt_s = float(controller_dt_s)
        self.max_velocity_m_s = float(max_velocity_m_s)
        self.max_acceleration_m_s2 = float(max_acceleration_m_s2)
        self.max_jerk_m_s3 = float(max_jerk_m_s3)
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self.state_scales = np.asarray(state_scales, dtype=float)
        self.input_scale_nm = float(input_scale_nm)
        self._ruckig = Ruckig(1, self.dt_s)

    def _calculate_ruckig_trajectory(
        self,
        *,
        position_m: float,
        velocity_m_s: float,
        acceleration_m_s2: float,
        target_velocity_m_s: float,
    ) -> tuple[Trajectory, float]:
        target = float(target_velocity_m_s)
        if abs(target) > self.max_velocity_m_s + 1e-12:
            raise ValueError("target velocity exceeds the frozen Stage 5 envelope")
        inp = InputParameter(1)
        inp.control_interface = ControlInterface.Velocity
        inp.duration_discretization = DurationDiscretization.Discrete
        inp.current_position = [float(position_m)]
        inp.current_velocity = [float(velocity_m_s)]
        inp.current_acceleration = [float(acceleration_m_s2)]
        inp.target_velocity = [target]
        inp.target_acceleration = [0.0]
        inp.max_velocity = [self.max_velocity_m_s]
        inp.max_acceleration = [self.max_acceleration_m_s2]
        inp.max_jerk = [self.max_jerk_m_s3]
        trajectory = Trajectory(1)
        result = self._ruckig.calculate(inp, trajectory)
        if result not in (Result.Working, Result.Finished):
            raise RuntimeError(f"Ruckig velocity plan failed: {result}")
        return trajectory, float(trajectory.duration)

    @staticmethod
    def _sample_ruckig_trajectory(
        trajectory: Trajectory,
        *,
        sample_dt_s: float,
        horizon_s: float,
        target_velocity_m_s: float,
    ) -> NDArray[np.float64]:
        intervals = round(float(horizon_s) / float(sample_dt_s))
        if intervals < 1 or not math.isclose(
            intervals * sample_dt_s, horizon_s, rel_tol=0.0, abs_tol=1e-10
        ):
            raise ValueError("sample horizon must be a positive multiple of sample dt")
        duration_s = float(trajectory.duration)
        end_position = float(trajectory.at_time(duration_s)[0][0])
        samples = np.empty((intervals + 1, 3), dtype=float)
        for index in range(intervals + 1):
            sample_time_s = index * sample_dt_s
            if sample_time_s >= duration_s - 1e-12:
                samples[index] = [
                    end_position + (sample_time_s - duration_s) * target_velocity_m_s,
                    target_velocity_m_s,
                    0.0,
                ]
            else:
                position, velocity, acceleration = trajectory.at_time(sample_time_s)
                samples[index] = [position[0], velocity[0], acceleration[0]]
        return samples

    def _kinematic_preview(
        self,
        *,
        position_m: float,
        velocity_m_s: float,
        acceleration_m_s2: float,
        target_velocity_m_s: float,
    ) -> tuple[NDArray[np.float64], float]:
        trajectory, duration_s = self._calculate_ruckig_trajectory(
            position_m=position_m,
            velocity_m_s=velocity_m_s,
            acceleration_m_s2=acceleration_m_s2,
            target_velocity_m_s=target_velocity_m_s,
        )
        interval_count = int(round(duration_s / self.dt_s))
        preview = self._sample_ruckig_trajectory(
            trajectory,
            sample_dt_s=self.dt_s,
            horizon_s=interval_count * self.dt_s,
            target_velocity_m_s=float(target_velocity_m_s),
        )
        preview[0] = [position_m, velocity_m_s, acceleration_m_s2]
        preview[-1, 1:] = [float(target_velocity_m_s), 0.0]
        return preview, duration_s


class RuckigLightweightVelocityPlanner(_RuckigVelocityPlannerBase):
    """Cheap Stage 5 path: jerk-limited p/v/a with zero lean and feedforward."""

    def plan(
        self,
        start_reference: NDArray[np.float64],
        start_acceleration_m_s2: float,
        target_velocity_m_s: float,
    ) -> RollingVelocityPlan:
        start = np.asarray(start_reference, dtype=float)
        if start.shape != (4,) or not np.isfinite(start).all():
            raise ValueError("lightweight start reference must be finite length-4")
        if not np.allclose(start[2:], 0.0, atol=1e-12, rtol=0.0):
            raise ValueError("lightweight path requires an exact quiet start")
        profile, duration_s = self._kinematic_preview(
            position_m=float(start[0]),
            velocity_m_s=float(start[1]),
            acceleration_m_s2=float(start_acceleration_m_s2),
            target_velocity_m_s=float(target_velocity_m_s),
        )
        states = np.zeros((len(profile), 4), dtype=float)
        states[:, :2] = profile[:, :2]
        transition = VelocityTransitionPlan(
            target_velocity_m_s=float(target_velocity_m_s),
            planned_transient_duration_s=duration_s,
            feedforward_rearmed=False,
            reference_states=states,
            reference_accelerations_m_s2=profile[:, 2],
            shaped_reference_finished=(
                np.arange(len(profile), dtype=float) * self.dt_s
                >= duration_s - 1e-12
            ),
            feedforward_inputs_nm=np.zeros(len(profile) - 1, dtype=float),
        )
        return RollingVelocityPlan(
            planning_mode="LIGHTWEIGHT",
            plan=transition,
            ruckig_duration_s=duration_s,
            dynamic_nominal_horizon_s=duration_s,
            normalized_residual_rms=0.0,
            projection_diagnostics=(),
        )


class RuckigFullHorizonVelocityPlanner(_RuckigVelocityPlannerBase):
    """Single-shot Stage 5 full path with optional cached multirate projection."""

    def __init__(
        self,
        *,
        minimum_horizon_s: float,
        dynamic_settle_margin_s: float,
        planning_dt_s: float | None = None,
        horizon_quantum_s: float = 0.0,
        projection_backend: str = "lsqr",
        factorization_cache: SparseProjectionFactorizationCache | None = None,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        self.minimum_horizon_s = float(minimum_horizon_s)
        self.dynamic_settle_margin_s = float(dynamic_settle_margin_s)
        self.planning_dt_s = (
            self.dt_s if planning_dt_s is None else float(planning_dt_s)
        )
        self.horizon_quantum_s = float(horizon_quantum_s)
        self.projection_backend = str(projection_backend)
        if self.projection_backend not in {"lsqr", "cached_kkt"}:
            raise ValueError("projection_backend must be 'lsqr' or 'cached_kkt'")
        if self.planning_dt_s < self.dt_s or not math.isclose(
            self.planning_dt_s / self.dt_s,
            round(self.planning_dt_s / self.dt_s),
            rel_tol=0.0,
            abs_tol=1e-10,
        ):
            raise ValueError("planning_dt_s must be an integer multiple of control dt")
        if self.horizon_quantum_s < 0.0 or (
            self.horizon_quantum_s > 0.0
            and not math.isclose(
                self.horizon_quantum_s / self.dt_s,
                round(self.horizon_quantum_s / self.dt_s),
                rel_tol=0.0,
                abs_tol=1e-10,
            )
        ):
            raise ValueError("horizon quantum must be zero or a multiple of control dt")
        self.factorization_cache = factorization_cache or SparseProjectionFactorizationCache()
        plan_step_count = round(self.planning_dt_s / self.dt_s)
        self.A_plan = np.linalg.matrix_power(self.A, plan_step_count)
        self.B_plan = np.zeros_like(self.B)
        for power in range(plan_step_count):
            self.B_plan += np.linalg.matrix_power(self.A, power) @ self.B
        intervals = round(self.minimum_horizon_s / self.dt_s)
        if intervals < 1 or not math.isclose(
            intervals * self.dt_s,
            self.minimum_horizon_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("minimum full horizon must be a positive multiple of dt")
        margin_intervals = round(self.dynamic_settle_margin_s / self.dt_s)
        if margin_intervals < 0 or not math.isclose(
            margin_intervals * self.dt_s,
            self.dynamic_settle_margin_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("dynamic settle margin must be a non-negative multiple of dt")
        self.minimum_horizon_intervals = intervals
        self.dynamic_settle_margin_intervals = margin_intervals

    def plan(
        self,
        start_reference: NDArray[np.float64],
        start_acceleration_m_s2: float,
        target_velocity_m_s: float,
    ) -> RollingVelocityPlan:
        start = np.asarray(start_reference, dtype=float)
        if start.shape != (4,) or not np.isfinite(start).all():
            raise ValueError("full-path start reference must be finite length-4")
        planning_started = time.perf_counter()
        trajectory, duration_s = self._calculate_ruckig_trajectory(
            position_m=float(start[0]),
            velocity_m_s=float(start[1]),
            acceleration_m_s2=float(start_acceleration_m_s2),
            target_velocity_m_s=float(target_velocity_m_s),
        )
        raw_horizon_s = max(
            self.minimum_horizon_s,
            duration_s + self.dynamic_settle_margin_s,
        )
        if self.horizon_quantum_s > 0.0:
            horizon_bucket_s = math.ceil(
                raw_horizon_s / self.horizon_quantum_s - 1e-12
            ) * self.horizon_quantum_s
        else:
            horizon_bucket_s = raw_horizon_s
        projected_horizon_s = math.ceil(
            horizon_bucket_s / self.planning_dt_s - 1e-12
        ) * self.planning_dt_s
        projected_intervals = round(projected_horizon_s / self.planning_dt_s)
        control_intervals = round(projected_horizon_s / self.dt_s)
        profile = self._sample_ruckig_trajectory(
            trajectory,
            sample_dt_s=self.dt_s,
            horizon_s=control_intervals * self.dt_s,
            target_velocity_m_s=float(target_velocity_m_s),
        )
        profile[0] = [start[0], start[1], start_acceleration_m_s2]
        plan_profile = self._sample_ruckig_trajectory(
            trajectory,
            sample_dt_s=self.planning_dt_s,
            horizon_s=projected_intervals * self.planning_dt_s,
            target_velocity_m_s=float(target_velocity_m_s),
        )
        plan_profile[0] = [start[0], start[1], start_acceleration_m_s2]

        projection_timing: dict = {}
        cache_timing = None
        if self.projection_backend == "cached_kkt":
            cache_timing = self.factorization_cache.solve_fixed_terminal(
                self.A_plan,
                self.B_plan,
                self.state_scales,
                self.input_scale_nm,
                self.planning_dt_s,
                plan_profile[:, :2],
                start[2:].copy(),
            )
            projection = cache_timing.projection
        else:
            projection = project_hidden_reference_and_input(
                self.A_plan,
                self.B_plan,
                self.state_scales,
                self.input_scale_nm,
                plan_profile[:, :2],
                [(0, projected_intervals)],
                segment_hidden_boundaries=[(start[2:].copy(), np.zeros(2))],
                timing_diagnostics=projection_timing,
            )
        plan_states = projection.reference_states
        if not math.isclose(self.planning_dt_s, self.dt_s, rel_tol=0.0, abs_tol=1e-12):
            control_times = np.arange(control_intervals + 1, dtype=float) * self.dt_s
            plan_times = np.arange(projected_intervals + 1, dtype=float) * self.planning_dt_s
            control_states = np.column_stack([
                profile[:, :2],
                np.interp(control_times, plan_times, plan_states[:, 2]),
                np.interp(control_times, plan_times, plan_states[:, 3]),
            ])
            plan_inputs_at_nodes = np.r_[
                projection.inputs_nm, projection.inputs_nm[-1]
            ]
            control_inputs = np.interp(
                control_times[:-1], plan_times, plan_inputs_at_nodes
            )
        else:
            control_states = np.column_stack([
                profile[:, :2], plan_states[:, 2:]
            ])
            control_inputs = projection.inputs_nm.copy()
        predicted = (
            control_states[:-1] @ self.A.T
            + control_inputs[:, None] * self.B[:, 0]
        )
        residuals = control_states[1:] - predicted
        normalized_rms = float(np.sqrt(np.mean(
            (residuals / self.state_scales) ** 2
        )))
        if not np.isfinite(np.r_[
            control_states.ravel(), control_inputs, residuals.ravel(),
        ]).all():
            raise FloatingPointError("full-path projection is not finite")
        shaped_finished = (
            np.arange(control_intervals + 1, dtype=float) * self.dt_s
            >= duration_s - 1e-12
        )
        planning_diagnostics = {
            "projection_backend": self.projection_backend,
            "planning_dt_s": self.planning_dt_s,
            "raw_horizon_s": raw_horizon_s,
            "horizon_bucket_s": horizon_bucket_s,
            "projected_horizon_s": projected_horizon_s,
            "control_interval_count": control_intervals,
            "projection_interval_count": projected_intervals,
            "residual_500hz_post_interpolation_rms": normalized_rms,
            "theta_ref_peak_rad": float(np.max(np.abs(control_states[:, 2]))),
            "theta_dot_ref_peak_rad_s": float(np.max(np.abs(control_states[:, 3]))),
            "u_ff_peak_nm": float(np.max(np.abs(control_inputs))),
            "planning_wall_time_s": time.perf_counter() - planning_started,
            **projection_timing,
        }
        if cache_timing is not None:
            planning_diagnostics.update({
                "cache_hit": cache_timing.cache_hit,
                "matrix_build_time_s": cache_timing.matrix_build_time_s,
                "factorization_time_s": cache_timing.factorization_time_s,
                "rhs_build_time_s": cache_timing.rhs_build_time_s,
                "solve_time_s": cache_timing.solve_time_s,
                "kkt_residual_relative": cache_timing.kkt_residual_relative,
                "diagonal_pivot_ratio": cache_timing.diagonal_pivot_ratio,
            })
        transition = VelocityTransitionPlan(
            target_velocity_m_s=float(target_velocity_m_s),
            planned_transient_duration_s=duration_s,
            feedforward_rearmed=True,
            reference_states=control_states,
            reference_accelerations_m_s2=profile[:, 2],
            shaped_reference_finished=shaped_finished,
            feedforward_inputs_nm=control_inputs,
        )
        return RollingVelocityPlan(
            planning_mode="FULL_DYNAMIC",
            plan=transition,
            ruckig_duration_s=duration_s,
            dynamic_nominal_horizon_s=projected_horizon_s,
            normalized_residual_rms=normalized_rms,
            projection_diagnostics=projection.segment_diagnostics,
            planning_diagnostics=planning_diagnostics,
        )


@dataclass(frozen=True)
class ScheduledVelocityCommand:
    accepted: bool
    mode: str
    raw_velocity_m_s: float
    accepted_velocity_m_s: float
    candidate_delta_v_m_s: float
    candidate_stable: bool
    candidate_target_m_s: float | None


class SparseVelocityCommandScheduler:
    """One-window persistent command acceptance against the accepted command."""

    def __init__(
        self,
        *,
        accept_delta_v_m_s: float,
        full_delta_v_m_s: float,
        stable_window_s: float,
        stable_range_m_s: float,
        initial_accepted_velocity_m_s: float = 0.0,
    ) -> None:
        self.accept_delta_v_m_s = float(accept_delta_v_m_s)
        self.full_delta_v_m_s = float(full_delta_v_m_s)
        self.stable_window_s = float(stable_window_s)
        self.stable_range_m_s = float(stable_range_m_s)
        self.accepted_velocity_m_s = float(initial_accepted_velocity_m_s)
        if not all(math.isfinite(value) and value > 0.0 for value in (
            self.accept_delta_v_m_s, self.full_delta_v_m_s,
            self.stable_window_s, self.stable_range_m_s,
        )):
            raise ValueError("scheduler thresholds must be finite and positive")
        self._candidate_start_time_s: float | None = None
        self._candidate_baseline_m_s: float | None = None
        self._candidate_window: deque[tuple[float, float]] = deque()
        self._candidate_stable = False
        self.latest_pending_velocity_m_s = self.accepted_velocity_m_s
        self.events: list[dict] = []

    def _clear_candidate(self) -> None:
        self._candidate_start_time_s = None
        self._candidate_baseline_m_s = None
        self._candidate_window.clear()
        self._candidate_stable = False

    @property
    def candidate_active(self) -> bool:
        return self._candidate_start_time_s is not None

    @property
    def candidate_target_m_s(self) -> float | None:
        return self._candidate_window[-1][1] if self._candidate_window else None

    @property
    def candidate_delta_v_m_s(self) -> float:
        if self._candidate_baseline_m_s is None or self.candidate_target_m_s is None:
            return 0.0
        return self.candidate_target_m_s - self._candidate_baseline_m_s

    def update(
        self,
        time_s: float,
        raw_velocity_m_s: float,
        *,
        allow_accept: bool = True,
    ) -> ScheduledVelocityCommand:
        now = float(time_s)
        raw = float(raw_velocity_m_s)
        if not np.isfinite([now, raw]).all():
            raise ValueError("scheduler input must be finite")
        self.latest_pending_velocity_m_s = raw
        delta = raw - self.accepted_velocity_m_s
        if abs(delta) < self.accept_delta_v_m_s:
            if self.candidate_active:
                self.events.append({
                    "time_s": now,
                    "event": "candidate_cancelled",
                    "raw_velocity_m_s": raw,
                    "frozen_accepted_baseline_m_s": self._candidate_baseline_m_s,
                    "reason": "returned_within_accept_threshold",
                })
            self._clear_candidate()
            return ScheduledVelocityCommand(
                False, "HOLD", raw, self.accepted_velocity_m_s, 0.0,
                False, None,
            )
        if self._candidate_start_time_s is None:
            self._candidate_start_time_s = now
            self._candidate_baseline_m_s = self.accepted_velocity_m_s
            self._candidate_window.clear()
            self._candidate_stable = False
            self.events.append({
                "time_s": now,
                "event": "candidate_started",
                "raw_velocity_m_s": raw,
                "frozen_accepted_baseline_m_s": self._candidate_baseline_m_s,
            })

        self._candidate_window.append((now, raw))
        window_start = now - self.stable_window_s
        while (
            len(self._candidate_window) > 1
            and self._candidate_window[0][0] < window_start - 1e-12
        ):
            self._candidate_window.popleft()
        elapsed = now - float(self._candidate_start_time_s)
        values = [value for _, value in self._candidate_window]
        spread = max(values) - min(values)
        is_stable = (
            elapsed + 1e-12 >= self.stable_window_s
            and self._candidate_window[-1][0] - self._candidate_window[0][0]
            + 1e-12 >= self.stable_window_s
            and spread <= self.stable_range_m_s + 1e-12
        )
        if is_stable and not self._candidate_stable:
            self.events.append({
                "time_s": now,
                "event": "candidate_stable",
                "candidate_v_cmd_m_s": raw,
                "candidate_delta_v_m_s": delta,
                "window_range_m_s": spread,
                "window_start_time_s": self._candidate_window[0][0],
                "window_end_time_s": self._candidate_window[-1][0],
            })
        if not is_stable and self._candidate_stable:
            self.events.append({
                "time_s": now,
                "event": "candidate_unstable",
                "raw_velocity_m_s": raw,
                "window_range_m_s": spread,
            })
        self._candidate_stable = is_stable
        candidate_target = raw if is_stable else None
        if not is_stable or not allow_accept:
            return ScheduledVelocityCommand(
                False,
                "CANDIDATE",
                raw,
                self.accepted_velocity_m_s,
                delta,
                is_stable,
                candidate_target,
            )

        assert self._candidate_baseline_m_s is not None
        candidate_delta = candidate_target - self.accepted_velocity_m_s
        if abs(candidate_delta) < self.accept_delta_v_m_s:
            self.events.append({
                "time_s": now,
                "event": "candidate_cancelled",
                "candidate_v_cmd_m_s": candidate_target,
                "candidate_delta_v_m_s": candidate_delta,
                "reason": "stable_target_below_accept_threshold",
            })
            self._clear_candidate()
            return ScheduledVelocityCommand(
                False, "HOLD", raw, self.accepted_velocity_m_s,
                candidate_delta, False, None,
            )
        mode = (
            "FULL_DYNAMIC"
            if abs(candidate_delta) >= self.full_delta_v_m_s
            else "LIGHTWEIGHT"
        )
        previous = self.accepted_velocity_m_s
        self.accepted_velocity_m_s = float(candidate_target)
        self.events.append({
            "time_s": now,
            "event": "accepted",
            "mode": mode,
            "candidate_v_cmd_m_s": float(candidate_target),
            "previous_accepted_velocity_m_s": previous,
            "accepted_velocity_m_s": self.accepted_velocity_m_s,
            "candidate_delta_v_m_s": candidate_delta,
            "candidate_stable_window_s": (
                self._candidate_window[-1][0] - self._candidate_window[0][0]
            ),
            "candidate_window_range_m_s": spread,
        })
        self._clear_candidate()
        return ScheduledVelocityCommand(
            True, mode, raw, self.accepted_velocity_m_s,
            candidate_delta, True, float(candidate_target),
        )


@dataclass(frozen=True)
class RollingReferenceCommand:
    reference_state: NDArray[np.float64]
    reference_acceleration_m_s2: float
    u_ff_raw_nm: float
    u_ff_after_lifecycle_nm: float
    linear_velocity_target_m_s: float
    yaw_rate_target_rad_s: float
    feedforward_phase: str
    velocity_phase: str
    fade_alpha: float
    block_id: int
    block_sample_index: int
    block_start_time_s: float
    intent_source_time_s: float
    replanned: bool
    raw_linear_velocity_target_m_s: float | None = None
    scheduler_mode: str = "HOLD"
    accepted_velocity_changed: bool = False
    planning_path: str = "LEGACY"
    candidate_delta_v_m_s: float = 0.0
    candidate_target_m_s: float | None = None
    candidate_stable: bool = False
    latest_pending_raw_v_cmd_m_s: float | None = None
    pending_command_active: bool = False
    stream_sequence_id: int | None = None
    start_control_tick: int | None = None
    generated_time_s: float | None = None
    available_time_s: float | None = None
    buffer_remaining_s: float | None = None
    next_block_ready: bool | None = None
    next_block_sequence_id: int | None = None


@dataclass(frozen=True)
class _TwoPathReferenceBlock:
    states: NDArray[np.float64]
    accelerations_m_s2: NDArray[np.float64]
    raw_feedforward_nm: NDArray[np.float64]
    lifecycle_feedforward_nm: NDArray[np.float64]
    feedforward_phase: tuple[str, ...]
    velocity_phase: tuple[str, ...]
    fade_alpha: NDArray[np.float64]
    scheduler_mode: tuple[str, ...]
    raw_velocity_m_s: float
    accepted_velocity_m_s: float
    yaw_rate_rad_s: float
    source_time_s: float
    replanned: NDArray[np.bool_]
    accepted_changed: NDArray[np.bool_]
    candidate_delta_v_m_s: float
    candidate_target_m_s: float
    candidate_stable: bool
    pending_velocity_m_s: float
    pending_active: bool
    block_id: int
    start_time_s: float


class SparseTwoPathReferenceSource:
    """Current Stage 5 scheduler and two planning paths at a 20 Hz boundary."""

    def __init__(
        self,
        *,
        lightweight_planner: RuckigLightweightVelocityPlanner,
        full_planner: RuckigFullHorizonVelocityPlanner,
        scheduler: SparseVelocityCommandScheduler,
        block_samples: int,
        feedforward_fade_s: float,
        observation_reader: Callable[[], TargetObservation],
        follower: Callable[[TargetObservation], MotionIntent],
        quiet_theta_rad: float,
        quiet_theta_dot_rad_s: float,
    ) -> None:
        if block_samples < 1:
            raise ValueError("reference block must contain at least one sample")
        if not math.isclose(
            lightweight_planner.dt_s,
            full_planner.dt_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("two-path planners must use the same dt")
        self.lightweight_planner = lightweight_planner
        self.full_planner = full_planner
        self.scheduler = scheduler
        self.block_samples = int(block_samples)
        self.dt_s = lightweight_planner.dt_s
        self.lifecycle = VelocityReferenceLifecycle(self.dt_s, feedforward_fade_s)
        self.lifecycle.reset(0.0, scheduler.accepted_velocity_m_s)
        self.observation_reader = observation_reader
        self.follower = follower
        self.quiet_theta_rad = float(quiet_theta_rad)
        self.quiet_theta_dot_rad_s = float(quiet_theta_dot_rad_s)
        self._block: _TwoPathReferenceBlock | None = None
        self._next_index = 0
        self._block_count = 0
        self._active_mode = "HOLD"
        self._full_locked = False
        self._full_quiet_latched = False
        self._active_full_event: dict | None = None
        self._pending_from_full_exit = False
        self._last_observation: TargetObservation | None = None
        self._last_intent: MotionIntent | None = None
        self.full_exit_events: list[dict] = []
        self.intent_history: list[dict] = []
        self.replan_events: list[dict] = []
        self.fade_events: list[dict] = []
        self.quiet_events: list[dict] = []

    def _begin_plan(
        self,
        time_s: float,
        scheduled: ScheduledVelocityCommand,
        *,
        request_time_s: float | None = None,
    ) -> None:
        start = self.lifecycle.reference_state
        start_acceleration = self.lifecycle.reference_acceleration_m_s2
        start_raw_feedforward = self.lifecycle.raw_feedforward_nm
        mid_trajectory_replan = (
            scheduled.mode == "LIGHTWEIGHT"
            and self.lifecycle.phase == VelocityLifecyclePhase.VELOCITY_TRANSIENT
            and self._active_mode == "LIGHTWEIGHT"
        )
        planner = (
            self.lightweight_planner
            if scheduled.mode == "LIGHTWEIGHT"
            else self.full_planner
        )
        planning_start = time.perf_counter()
        rolling = planner.plan(
            start,
            start_acceleration,
            scheduled.accepted_velocity_m_s,
        )
        wall_time = time.perf_counter() - planning_start
        self.lifecycle.begin_transition(rolling.plan)
        self._active_mode = scheduled.mode
        if scheduled.mode == "FULL_DYNAMIC":
            self._full_locked = True
            self._full_quiet_latched = False
        first = rolling.plan.reference_states[0]
        terminal = rolling.plan.reference_states[-1]
        event = {
            "time_s": float(time_s),
            "planning_request_time_s": float(
                time_s if request_time_s is None else request_time_s
            ),
            "execution_start_time_s": float(time_s),
            "planning_complete_time_s": float(
                (time_s if request_time_s is None else request_time_s) + wall_time
            ),
            "planning_path": scheduled.mode,
            "target_velocity_m_s": scheduled.accepted_velocity_m_s,
            "candidate_delta_v_m_s": scheduled.candidate_delta_v_m_s,
            "candidate_target_m_s": scheduled.candidate_target_m_s,
            "mid_trajectory_replan": mid_trajectory_replan,
            "planning_wall_time_s": wall_time,
            "ruckig_duration_s": rolling.ruckig_duration_s,
            "horizon_s": rolling.dynamic_nominal_horizon_s,
            "normalized_residual_rms": rolling.normalized_residual_rms,
            "p_start_delta_m": float(first[0] - start[0]),
            "v_start_delta_m_s": float(first[1] - start[1]),
            "a_start_delta_m_s2": float(
                rolling.plan.reference_accelerations_m_s2[0]
                - start_acceleration
            ),
            "theta_start_delta_rad": float(first[2] - start[2]),
            "theta_dot_start_delta_rad_s": float(first[3] - start[3]),
            "u_ff_raw_start_delta_nm": float(
                rolling.plan.feedforward_inputs_nm[0]
                - start_raw_feedforward
            ),
            "terminal_theta_rad": float(terminal[2]),
            "terminal_theta_dot_rad_s": float(terminal[3]),
            **rolling.planning_diagnostics,
            "projection_solver": (
                rolling.projection_diagnostics[0].get("solver")
                if rolling.projection_diagnostics else None
            ),
            "projection_solver_converged": (
                rolling.projection_diagnostics[0].get("lsqr_stop_code") in (1, 2)
                if rolling.projection_diagnostics else None
            ),
            "projection_solver_status": (
                rolling.projection_diagnostics[0].get("lsqr_stop_code")
                if rolling.projection_diagnostics else None
            ),
        }
        self.replan_events.append(event)
        if scheduled.mode == "FULL_DYNAMIC":
            event["full_dynamic_horizon_s"] = rolling.dynamic_nominal_horizon_s
            event["quiet_time_s"] = None
            event["fade_completion_time_s"] = None
            event["full_exit_time_s"] = None
            self._active_full_event = event


class ReferenceBlockUnderrun(RuntimeError):
    """A scheduled block was not available at its control-tick boundary."""


class DeterministicReferenceBlockStream:
    """Timestamped Pi-producer / MCU-consumer boundary with ACTIVE/NEXT slots."""

    def __init__(
        self,
        producer: SparseTwoPathReferenceSource,
        *,
        link_latency_s: float,
    ) -> None:
        self.producer = producer
        self.dt_s = producer.dt_s
        self.block_samples = producer.block_samples
        self.block_duration_s = self.dt_s * self.block_samples
        self.link_latency_s = float(link_latency_s)
        if not math.isfinite(self.link_latency_s) or self.link_latency_s < 0.0:
            raise ValueError("link latency must be finite and non-negative")
        self._in_flight: list[ReferenceBlock] = []
        self._ready: dict[int, ReferenceBlock] = {}
        self._commands_by_sequence: dict[int, tuple[RollingReferenceCommand, ...]] = {}
        self._active: ReferenceBlock | None = None
        self._next_sequence_id = 0
        self._last_delivered_sequence_id: int | None = None
        self._last_available_time_s = 0.0
        self._initialized = False
        self._last_tick: int | None = None
        self.block_events: list[dict] = []
        self.planning_events: list[dict] = []
        self.underrun_events: list[dict] = []
        self.stale_events: list[dict] = []
        self.sequence_gap_events: list[dict] = []
        self.consumer_events: list[dict] = []
        self.next_ready_at_swap_count = 0

    def _produce(
        self,
        *,
        start_control_tick: int,
        generated_time_s: float,
        observe_command: bool,
        startup_prefill: bool = False,
    ) -> ReferenceBlock:
        sequence_id = self._next_sequence_id
        self._next_sequence_id += 1
        replan_count_before = len(self.producer.replan_events)
        wall_started = time.perf_counter()
        start_time_s = start_control_tick * self.dt_s
        commands = tuple(
            self.producer.command(
                start_time_s + index * self.dt_s,
                observe_command=(observe_command if index == 0 else True),
                planning_request_time_s=(
                    generated_time_s if index == 0 else None
                ),
            )
            for index in range(self.block_samples)
        )
        producer_compute_time_s = time.perf_counter() - wall_started
        samples = np.column_stack([
            np.asarray([item.reference_state[0] for item in commands]),
            np.asarray([item.reference_state[1] for item in commands]),
            np.asarray([item.reference_acceleration_m_s2 for item in commands]),
            np.asarray([item.reference_state[2] for item in commands]),
            np.asarray([item.reference_state[3] for item in commands]),
            np.asarray([item.u_ff_after_lifecycle_nm for item in commands]),
        ]).astype(np.float32)
        generated_time_s = float(generated_time_s)
        if startup_prefill:
            available_time_s = 0.0
        else:
            available_time_s = max(
                generated_time_s + producer_compute_time_s + self.link_latency_s,
                self._last_available_time_s,
            )
        self._last_available_time_s = available_time_s
        block = ReferenceBlock(
            start_time_s=start_time_s,
            dt_s=self.dt_s,
            p_ref=samples[:, 0],
            v_ref=samples[:, 1],
            a_ref=samples[:, 2],
            theta_ref=samples[:, 3],
            theta_dot_ref=samples[:, 4],
            u_ff_raw=samples[:, 5],
            yaw_rate_target_rad_s=float(commands[0].yaw_rate_target_rad_s),
            sequence_id=sequence_id,
            start_control_tick=int(start_control_tick),
            generated_time_s=generated_time_s,
            available_time_s=available_time_s,
            mode=commands[0].scheduler_mode,
        )
        self._commands_by_sequence[sequence_id] = commands
        self._in_flight.append(block)
        self.block_events.append({
            "event": "block_generated",
            "sequence_id": sequence_id,
            "start_control_tick": int(start_control_tick),
            "sample_count": self.block_samples,
            "generated_time_s": generated_time_s,
            "producer_compute_time_s": producer_compute_time_s,
            "available_time_s": available_time_s,
            "link_latency_s": self.link_latency_s,
            "mode": block.mode,
            "startup_prefill": startup_prefill,
        })
        new_plans = self.producer.replan_events[replan_count_before:]
        for plan in new_plans:
            self.planning_events.append({
                **plan,
                "producer_block_sequence_id": sequence_id,
                "producer_block_generated_time_s": generated_time_s,
                "producer_block_available_time_s": available_time_s,
            })
        return block

    def _deliver(self, time_s: float) -> None:
        time_s = float(time_s)
        waiting = []
        for block in self._in_flight:
            if block.available_time_s <= time_s + 1e-12:
                if block.start_time_s < time_s - 1e-12:
                    event = {
                        "event": "stale_block",
                        "sequence_id": block.sequence_id,
                        "start_control_tick": block.start_control_tick,
                        "available_time_s": block.available_time_s,
                        "delivery_observed_time_s": time_s,
                    }
                    self.stale_events.append(event)
                if self._last_delivered_sequence_id is not None:
                    expected = self._last_delivered_sequence_id + 1
                    if block.sequence_id != expected:
                        self.sequence_gap_events.append({
                            "event": "sequence_gap",
                            "expected_sequence_id": expected,
                            "received_sequence_id": block.sequence_id,
                            "time_s": time_s,
                        })
                self._last_delivered_sequence_id = block.sequence_id
                self._ready[block.start_control_tick] = block
                self.block_events.append({
                    "event": "block_received",
                    "sequence_id": block.sequence_id,
                    "start_control_tick": block.start_control_tick,
                    "available_time_s": block.available_time_s,
                    "delivered_time_s": time_s,
                })
            else:
                waiting.append(block)
        self._in_flight = waiting

    def _underrun(self, *, tick: int, time_s: float, reason: str) -> None:
        event = {
            "event": "buffer_underrun",
            "control_tick": int(tick),
            "time_s": float(time_s),
            "reason": reason,
            "active_sequence_id": (
                None if self._active is None else self._active.sequence_id
            ),
            "expected_sequence_id": (
                None if self._active is None else self._active.sequence_id + 1
            ),
            "ready_start_ticks": sorted(self._ready),
            "in_flight_sequence_ids": [block.sequence_id for block in self._in_flight],
        }
        self.underrun_events.append(event)
        raise ReferenceBlockUnderrun(str(event))

    def _initialize(self, time_s: float) -> None:
        first = self._produce(
            start_control_tick=0,
            generated_time_s=float(time_s),
            observe_command=True,
            startup_prefill=True,
        )
        second = self._produce(
            start_control_tick=self.block_samples,
            generated_time_s=float(time_s),
            observe_command=False,
            startup_prefill=True,
        )
        self._deliver(float(time_s))
        self._active = self._ready.pop(first.start_control_tick, None)
        if self._active is None or second.start_control_tick not in self._ready:
            self._underrun(
                tick=0, time_s=time_s, reason="startup ACTIVE/NEXT prefill failed"
            )
        self._initialized = True
        self.block_events.append({
            "event": "active_next_prefill_ready",
            "active_sequence_id": first.sequence_id,
            "next_sequence_id": second.sequence_id,
            "time_s": float(time_s),
        })

    def command(self, time_s: float) -> RollingReferenceCommand:
        time_s = float(time_s)
        tick_float = time_s / self.dt_s
        tick = int(round(tick_float))
        if not math.isclose(tick_float, tick, rel_tol=0.0, abs_tol=1e-7):
            raise ValueError("consumer time is not aligned to the control tick")
        if self._last_tick is not None and tick != self._last_tick + 1:
            self.sequence_gap_events.append({
                "event": "consumer_control_tick_gap",
                "previous_tick": self._last_tick,
                "current_tick": tick,
            })
        self._last_tick = tick
        if not self._initialized:
            self._initialize(time_s)
        self._deliver(time_s)

        assert self._active is not None
        active_end_tick = self._active.start_control_tick + self.block_samples
        if tick == active_end_tick:
            next_block = self._ready.pop(active_end_tick, None)
            if next_block is None:
                self._underrun(
                    tick=tick, time_s=time_s,
                    reason="NEXT block not ready when ACTIVE block ended",
                )
            if next_block.sequence_id != self._active.sequence_id + 1:
                self.sequence_gap_events.append({
                    "event": "sequence_gap_at_swap",
                    "expected_sequence_id": self._active.sequence_id + 1,
                    "received_sequence_id": next_block.sequence_id,
                    "time_s": time_s,
                })
            self._active = next_block
            self.next_ready_at_swap_count += 1
            self.block_events.append({
                "event": "active_swap",
                "sequence_id": next_block.sequence_id,
                "control_tick": tick,
                "time_s": time_s,
            })
            self._produce(
                start_control_tick=active_end_tick + self.block_samples,
                generated_time_s=time_s,
                observe_command=True,
            )
            self._deliver(time_s)

        if tick < self._active.start_control_tick or tick >= active_end_tick + self.block_samples:
            self._underrun(
                tick=tick, time_s=time_s, reason="consumer tick outside ACTIVE/NEXT window"
            )
        index = tick - self._active.start_control_tick
        if not 0 <= index < self.block_samples:
            self._underrun(tick=tick, time_s=time_s, reason="ACTIVE sample index invalid")
        source_commands = self._commands_by_sequence[self._active.sequence_id]
        source_command = source_commands[index]
        row = self._active.sample_matrix_float32[index]
        next_tick = self._active.start_control_tick + self.block_samples
        next_block = self._ready.get(next_tick)
        buffer_remaining_s = max(
            0.0,
            (self._active.start_control_tick + self.block_samples - tick)
            * self.dt_s,
        )
        command = replace(
            source_command,
            reference_state=np.asarray([row[0], row[1], row[3], row[4]], dtype=float),
            reference_acceleration_m_s2=float(row[2]),
            u_ff_after_lifecycle_nm=float(row[5]),
            block_id=self._active.sequence_id,
            block_sample_index=index,
            block_start_time_s=self._active.start_time_s,
            stream_sequence_id=self._active.sequence_id,
            start_control_tick=self._active.start_control_tick,
            generated_time_s=self._active.generated_time_s,
            available_time_s=self._active.available_time_s,
            buffer_remaining_s=buffer_remaining_s,
            next_block_ready=next_block is not None,
            next_block_sequence_id=(
                None if next_block is None else next_block.sequence_id
            ),
        )
        self.consumer_events.append({
            "control_tick": tick,
            "time_s": time_s,
            "active_sequence_id": self._active.sequence_id,
            "active_sample_index": index,
            "active_start_control_tick": self._active.start_control_tick,
            "active_mode": self._active.mode,
            "generated_time_s": self._active.generated_time_s,
            "available_time_s": self._active.available_time_s,
            "buffer_remaining_s": buffer_remaining_s,
            "next_block_ready": next_block is not None,
            "next_block_sequence_id": (
                None if next_block is None else next_block.sequence_id
            ),
        })
        return command

    def _next_sample(self) -> tuple[NDArray[np.float64], float]:
        if self._active is None or self._last_tick is None:
            return np.zeros(4, dtype=float), 0.0
        next_tick = self._last_tick + 1
        active_end = self._active.start_control_tick + self.block_samples
        if next_tick < active_end:
            index = next_tick - self._active.start_control_tick
            row = self._active.sample_matrix_float32[index]
        elif next_tick == active_end:
            next_block = self._ready.get(active_end)
            if next_block is None:
                self._underrun(
                    tick=next_tick, time_s=next_tick * self.dt_s,
                    reason="NEXT block not ready at ACTIVE tail",
                )
            row = next_block.sample_matrix_float32[0]
        else:
            self._underrun(
                tick=next_tick, time_s=next_tick * self.dt_s,
                reason="reference lookahead exceeded two-slot buffer",
            )
        return (
            np.asarray([row[0], row[1], row[3], row[4]], dtype=float),
            float(row[2]),
        )

    @property
    def next_reference_state(self) -> NDArray[np.float64]:
        return self._next_sample()[0]

    @property
    def next_reference_acceleration_m_s2(self) -> float:
        return self._next_sample()[1]

    def summary(self) -> dict:
        return {
            "block_duration_s": self.block_duration_s,
            "block_samples": self.block_samples,
            "control_dt_s": self.dt_s,
            "link_latency_s": self.link_latency_s,
            "block_events": list(self.block_events),
            "planning_events": list(self.planning_events),
            "underrun_events": list(self.underrun_events),
            "stale_events": list(self.stale_events),
            "sequence_gap_events": list(self.sequence_gap_events),
            "next_ready_at_swap_count": self.next_ready_at_swap_count,
            "consumer_events": list(self.consumer_events),
            "generated_block_count": self._next_sequence_id,
        }

class SparseTwoPathReferenceSource(SparseTwoPathReferenceSource):
    """Complete the two-path source methods after the stream adapter definition."""

    def _build_block(
        self, start_time_s: float, *, observe_command: bool = True,
        planning_request_time_s: float | None = None,
    ) -> _TwoPathReferenceBlock:
        if observe_command:
            observation = self.observation_reader()
            intent = self.follower(observation)
            self._last_observation = observation
            self._last_intent = intent
        else:
            if self._last_observation is None or self._last_intent is None:
                raise RuntimeError("cannot prefill a block before the first observation")
            observation = self._last_observation
            intent = self._last_intent
        raw = float(intent.linear_velocity_target_m_s)
        if observe_command:
            scheduled = self.scheduler.update(
                float(intent.source_time_s), raw, allow_accept=not self._full_locked
            )
        else:
            scheduled = ScheduledVelocityCommand(
                False,
                "CANDIDATE" if self.scheduler.candidate_active else "HOLD",
                raw,
                self.scheduler.accepted_velocity_m_s,
                self.scheduler.candidate_delta_v_m_s,
                self.scheduler._candidate_stable,
                self.scheduler.candidate_target_m_s,
            )
        if scheduled.accepted:
            if self._pending_from_full_exit:
                self.scheduler.events[-1]["processed_after_full_exit"] = True
                self._pending_from_full_exit = False
            self._begin_plan(
                start_time_s, scheduled, request_time_s=float(
                    intent.source_time_s
                    if planning_request_time_s is None
                    else planning_request_time_s
                )
            )
        self.intent_history.append({
            "source_time_s": float(intent.source_time_s),
            "raw_linear_velocity_target_m_s": raw,
            "accepted_linear_velocity_target_m_s": (
                self.scheduler.accepted_velocity_m_s
            ),
            "candidate_target_m_s": self.scheduler.candidate_target_m_s,
            "candidate_delta_v_m_s": self.scheduler.candidate_delta_v_m_s,
            "candidate_stable": self.scheduler._candidate_stable,
            "latest_pending_raw_v_cmd_m_s": (
                self.scheduler.latest_pending_velocity_m_s
            ),
            "full_locked": self._full_locked,
            "observation_reused_for_prefill": not observe_command,
            "yaw_rate_target_rad_s": float(intent.yaw_rate_target_rad_s),
            "x_forward_m": float(observation.x_forward_m),
            "y_left_m": float(observation.y_left_m),
        })

        states = np.empty((self.block_samples, 4), dtype=float)
        acceleration = np.empty(self.block_samples, dtype=float)
        raw_ff = np.empty(self.block_samples, dtype=float)
        lifecycle_ff = np.empty(self.block_samples, dtype=float)
        fade_alpha = np.empty(self.block_samples, dtype=float)
        ff_phase: list[str] = []
        velocity_phase: list[str] = []
        modes: list[str] = []
        pending_commands = np.empty(self.block_samples, dtype=float)
        pending_active = np.zeros(self.block_samples, dtype=bool)
        replanned = np.zeros(self.block_samples, dtype=bool)
        accepted_changed = np.zeros(self.block_samples, dtype=bool)
        replanned[0] = scheduled.accepted
        accepted_changed[0] = scheduled.accepted

        for index in range(self.block_samples):
            sample_time = float(start_time_s) + index * self.dt_s
            command = self.lifecycle.command(sample_time)
            before_quiet = command.reference_state.copy()
            if (
                self._full_locked
                and not self._full_quiet_latched
                and self.lifecycle.phase == VelocityLifecyclePhase.VELOCITY_TRANSIENT
                and self.lifecycle.shaped_reference_reached
                and abs(command.reference_acceleration_m_s2) <= 1e-12
                and abs(float(command.reference_state[2]))
                <= self.quiet_theta_rad
                and abs(float(command.reference_state[3]))
                <= self.quiet_theta_dot_rad_s
            ):
                command = self.lifecycle.enter_quiet_hold(sample_time)
                self._full_quiet_latched = True
                self.quiet_events.append({
                    "time_s": sample_time,
                    "event": "quiet_snap",
                    "theta_before_rad": float(before_quiet[2]),
                    "theta_dot_before_rad_s": float(before_quiet[3]),
                })
                if self._active_full_event is not None:
                    self._active_full_event["quiet_time_s"] = sample_time
            states[index] = command.reference_state
            acceleration[index] = command.reference_acceleration_m_s2
            raw_ff[index] = self.lifecycle.raw_feedforward_nm
            lifecycle_ff[index] = command.feedforward_nm
            fade_alpha[index] = command.fade_alpha
            ff_phase.append(command.feedforward_phase)
            velocity_phase.append(command.phase.value)
            current_mode = (
                "FULL_DYNAMIC"
                if self._full_locked
                else (
                    "CANDIDATE"
                    if self.scheduler.candidate_active
                    else (
                        "LIGHTWEIGHT"
                        if self._active_mode == "LIGHTWEIGHT"
                        and self.lifecycle.phase
                        == VelocityLifecyclePhase.VELOCITY_TRANSIENT
                        else "NORMAL"
                    )
                )
            )
            modes.append(current_mode)
            pending_commands[index] = self.scheduler.latest_pending_velocity_m_s
            pending_active[index] = self._full_locked and (
                self.scheduler.candidate_active
                or abs(
                    self.scheduler.latest_pending_velocity_m_s
                    - self.scheduler.accepted_velocity_m_s
                ) >= self.scheduler.accept_delta_v_m_s
            )
            if command.fade_started:
                self.fade_events.append({
                    "time_s": sample_time, "event": "fade_started"
                })
            if command.fade_finished:
                self.fade_events.append({
                    "time_s": sample_time, "event": "fade_finished"
                })
                if self._full_locked and self._full_quiet_latched:
                    self._full_locked = False
                    self._active_mode = "HOLD"
                    self._pending_from_full_exit = self.scheduler.candidate_active
                    exit_event = {
                        "time_s": sample_time,
                        "event": "full_exit",
                        "accepted_velocity_m_s": self.scheduler.accepted_velocity_m_s,
                        "latest_pending_raw_v_cmd_m_s": self.scheduler.latest_pending_velocity_m_s,
                        "pending_candidate_active": self.scheduler.candidate_active,
                        "pending_candidate_stable": self.scheduler._candidate_stable,
                    }
                    self.full_exit_events.append(exit_event)
                    if self._active_full_event is not None:
                        self._active_full_event["fade_completion_time_s"] = sample_time
                        self._active_full_event["full_exit_time_s"] = sample_time
                    self._active_full_event = None
            reached_end = self.lifecycle.advance()
            if (
                reached_end
                and self._active_mode == "LIGHTWEIGHT"
                and self.lifecycle.phase == VelocityLifecyclePhase.VELOCITY_HOLD
            ):
                self._active_mode = "HOLD"

        block = _TwoPathReferenceBlock(
            states=states,
            accelerations_m_s2=acceleration,
            raw_feedforward_nm=raw_ff,
            lifecycle_feedforward_nm=lifecycle_ff,
            feedforward_phase=tuple(ff_phase),
            velocity_phase=tuple(velocity_phase),
            fade_alpha=fade_alpha,
            scheduler_mode=tuple(modes),
            raw_velocity_m_s=raw,
            accepted_velocity_m_s=self.scheduler.accepted_velocity_m_s,
            yaw_rate_rad_s=float(intent.yaw_rate_target_rad_s),
            source_time_s=float(intent.source_time_s),
            replanned=replanned,
            accepted_changed=accepted_changed,
            candidate_delta_v_m_s=scheduled.candidate_delta_v_m_s,
            candidate_target_m_s=(
                np.nan
                if (
                    scheduled.candidate_target_m_s is None
                    and self.scheduler.candidate_target_m_s is None
                )
                else (
                    scheduled.candidate_target_m_s
                    if scheduled.candidate_target_m_s is not None
                    else self.scheduler.candidate_target_m_s
                )
            ),
            candidate_stable=scheduled.candidate_stable,
            pending_velocity_m_s=pending_commands[-1],
            pending_active=bool(pending_active[-1]),
            block_id=self._block_count,
            start_time_s=float(start_time_s),
        )
        self._block_count += 1
        return block

    def command(
        self, time_s: float, *, observe_command: bool = True,
        planning_request_time_s: float | None = None,
    ) -> RollingReferenceCommand:
        if self._block is None or self._next_index >= self.block_samples:
            self._block = self._build_block(
                float(time_s), observe_command=observe_command,
                planning_request_time_s=planning_request_time_s,
            )
            self._next_index = 0
        block = self._block
        index = self._next_index
        self._next_index += 1
        return RollingReferenceCommand(
            reference_state=block.states[index].copy(),
            reference_acceleration_m_s2=float(block.accelerations_m_s2[index]),
            u_ff_raw_nm=float(block.raw_feedforward_nm[index]),
            u_ff_after_lifecycle_nm=float(
                block.lifecycle_feedforward_nm[index]
            ),
            linear_velocity_target_m_s=block.accepted_velocity_m_s,
            yaw_rate_target_rad_s=block.yaw_rate_rad_s,
            feedforward_phase=block.feedforward_phase[index],
            velocity_phase=block.velocity_phase[index],
            fade_alpha=float(block.fade_alpha[index]),
            block_id=block.block_id,
            block_sample_index=index,
            block_start_time_s=block.start_time_s,
            intent_source_time_s=block.source_time_s,
            replanned=bool(block.replanned[index]),
            raw_linear_velocity_target_m_s=block.raw_velocity_m_s,
            scheduler_mode=block.scheduler_mode[index],
            accepted_velocity_changed=bool(block.accepted_changed[index]),
            planning_path=block.scheduler_mode[index],
            candidate_delta_v_m_s=block.candidate_delta_v_m_s,
            candidate_target_m_s=(
                None
                if math.isnan(block.candidate_target_m_s)
                else block.candidate_target_m_s
            ),
            candidate_stable=block.candidate_stable,
            latest_pending_raw_v_cmd_m_s=block.pending_velocity_m_s,
            pending_command_active=block.pending_active,
        )

    @property
    def next_reference_state(self) -> NDArray[np.float64]:
        if self._block is None or self._next_index >= self.block_samples:
            return self.lifecycle.reference_state
        return self._block.states[self._next_index].copy()

    @property
    def next_reference_acceleration_m_s2(self) -> float:
        if self._block is None or self._next_index >= self.block_samples:
            return self.lifecycle.reference_acceleration_m_s2
        return float(self._block.accelerations_m_s2[self._next_index])

    def summary(self) -> dict:
        return {
            "block_count": self._block_count,
            "block_samples": self.block_samples,
            "block_duration_s": self.block_samples * self.dt_s,
            "intent_count": len(self.intent_history),
            "intents": list(self.intent_history),
            "scheduler_events": list(self.scheduler.events),
            "replan_count": len(self.replan_events),
            "replan_events": list(self.replan_events),
            "full_exit_events": list(self.full_exit_events),
            "fade_events": list(self.fade_events),
            "quiet_events": list(self.quiet_events),
            "latest_pending_velocity_m_s": (
                self.scheduler.latest_pending_velocity_m_s
            ),
        }
