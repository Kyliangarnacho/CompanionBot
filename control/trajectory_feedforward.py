"""Nominal scalar feedforward for the frozen discrete longitudinal model."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np
from numpy.typing import NDArray
from scipy.sparse import bmat, coo_matrix, eye, csr_matrix
from scipy.sparse.linalg import lsqr, splu


@dataclass(frozen=True)
class NominalFeedforwardCommand:
    torque_nm: float
    residual: NDArray[np.float64]
    normalized_residual_rms: float


class NominalDiscreteFeedforward:
    """Weighted least-squares nominal input for a four-state, scalar-input model."""

    def __init__(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        state_scales: NDArray[np.float64],
    ) -> None:
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self.state_scales = np.asarray(state_scales, dtype=float)
        if self.A.shape != (4, 4):
            raise ValueError("A must have shape (4, 4)")
        if self.B.shape != (4, 1):
            raise ValueError("B must have shape (4, 1)")
        if self.state_scales.shape != (4,) or np.any(self.state_scales <= 0.0):
            raise ValueError("state_scales must be a positive length-4 vector")
        if not np.isfinite(
            np.r_[self.A.ravel(), self.B.ravel(), self.state_scales]
        ).all():
            raise ValueError("feedforward model and scales must be finite")
        self.weights = 1.0 / self.state_scales**2
        input_direction = self.B[:, 0]
        self.denominator = float(
            np.dot(input_direction * self.weights, input_direction)
        )
        if self.denominator <= 0.0:
            raise ValueError("weighted input direction must be nonzero")

    def command(
        self,
        reference_state: NDArray[np.float64],
        next_reference_state: NDArray[np.float64],
    ) -> NominalFeedforwardCommand:
        reference_state = np.asarray(reference_state, dtype=float)
        next_reference_state = np.asarray(next_reference_state, dtype=float)
        if reference_state.shape != (4,) or next_reference_state.shape != (4,):
            raise ValueError("reference states must be length-4 vectors")
        desired_increment = next_reference_state - self.A @ reference_state
        input_direction = self.B[:, 0]
        torque = float(
            np.dot(input_direction * self.weights, desired_increment)
            / self.denominator
        )
        residual = desired_increment - input_direction * torque
        normalized_residual_rms = float(
            np.sqrt(np.mean((residual / self.state_scales) ** 2))
        )
        return NominalFeedforwardCommand(
            torque_nm=torque,
            residual=residual,
            normalized_residual_rms=normalized_residual_rms,
        )


@dataclass(frozen=True)
class NominalTrajectoryProjection:
    reference_states: NDArray[np.float64]
    inputs_nm: NDArray[np.float64]
    residuals: NDArray[np.float64]
    segment_diagnostics: tuple[dict, ...]


@dataclass(frozen=True)
class CachedProjectionSolve:
    projection: NominalTrajectoryProjection
    matrix_build_time_s: float
    factorization_time_s: float
    rhs_build_time_s: float
    solve_time_s: float
    kkt_residual_relative: float
    diagonal_pivot_ratio: float
    cache_hit: bool


@dataclass(frozen=True)
class _ProjectionFactorEntry:
    matrix: csr_matrix
    factor: object
    matrix_build_time_s: float
    factorization_time_s: float
    row_count: int
    variable_count: int
    input_offset: int
    diagonal_pivot_ratio: float


class SparseProjectionFactorizationCache:
    """Cached fixed-terminal LS projection matrix and augmented sparse LU."""

    def __init__(self) -> None:
        self._entries: dict[tuple, _ProjectionFactorEntry] = {}

    @staticmethod
    def _key(
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        state_scales: NDArray[np.float64],
        input_scale_nm: float,
        planning_dt_s: float,
        interval_count: int,
    ) -> tuple:
        return (
            float(planning_dt_s),
            int(interval_count),
            np.asarray(A, dtype=np.float64).tobytes(),
            np.asarray(B, dtype=np.float64).tobytes(),
            np.asarray(state_scales, dtype=np.float64).tobytes(),
            float(input_scale_nm),
        )

    @staticmethod
    def _build_entry(
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        state_scales: NDArray[np.float64],
        input_scale_nm: float,
        interval_count: int,
    ) -> _ProjectionFactorEntry:
        started = time.perf_counter()
        hidden_variable_count = 2 * max(0, interval_count - 1)
        input_offset = hidden_variable_count
        variable_count = hidden_variable_count + interval_count
        rows: list[int] = []
        columns: list[int] = []
        values: list[float] = []
        for sample in range(interval_count):
            for state_row in range(4):
                equation_row = 4 * sample + state_row
                row_scale = 1.0 / state_scales[state_row]
                if sample > 0:
                    for hidden_index in range(2):
                        rows.append(equation_row)
                        columns.append(2 * (sample - 1) + hidden_index)
                        values.append(float(
                            -A[state_row, 2 + hidden_index]
                            * state_scales[2 + hidden_index]
                            * row_scale
                        ))
                if sample + 1 < interval_count and state_row >= 2:
                    rows.append(equation_row)
                    columns.append(2 * sample + state_row - 2)
                    values.append(float(state_scales[state_row] * row_scale))
                rows.append(equation_row)
                columns.append(input_offset + sample)
                values.append(float(-B[state_row, 0] * input_scale_nm * row_scale))

        row_count = 4 * interval_count
        matrix = coo_matrix(
            (values, (rows, columns)),
            shape=(row_count, variable_count),
        ).tocsr()
        kkt = bmat(
            [
                [eye(row_count, format="csc"), matrix.tocsc()],
                [matrix.T.tocsc(), None],
            ],
            format="csc",
        )
        matrix_build_time_s = time.perf_counter() - started
        factor_started = time.perf_counter()
        factor = splu(kkt)
        factorization_time_s = time.perf_counter() - factor_started
        diagonal = np.abs(factor.U.diagonal())
        positive = diagonal[diagonal > 0.0]
        pivot_ratio = (
            float(np.min(positive) / np.max(positive))
            if positive.size else 0.0
        )
        return _ProjectionFactorEntry(
            matrix=matrix,
            factor=factor,
            matrix_build_time_s=matrix_build_time_s,
            factorization_time_s=factorization_time_s,
            row_count=row_count,
            variable_count=variable_count,
            input_offset=input_offset,
            diagonal_pivot_ratio=pivot_ratio,
        )

    def solve_fixed_terminal(
        self,
        A: NDArray[np.float64],
        B: NDArray[np.float64],
        state_scales: NDArray[np.float64],
        input_scale_nm: float,
        planning_dt_s: float,
        position_velocity_reference: NDArray[np.float64],
        start_hidden: NDArray[np.float64],
    ) -> CachedProjectionSolve:
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        scales = np.asarray(state_scales, dtype=float)
        pv = np.asarray(position_velocity_reference, dtype=float)
        start_hidden = np.asarray(start_hidden, dtype=float)
        input_scale_nm = float(input_scale_nm)
        if A.shape != (4, 4) or B.shape != (4, 1):
            raise ValueError("A/B must have shapes (4, 4)/(4, 1)")
        if scales.shape != (4,) or np.any(scales <= 0.0):
            raise ValueError("state_scales must be a positive length-4 vector")
        if pv.ndim != 2 or pv.shape[1] != 2 or len(pv) < 2:
            raise ValueError("position_velocity_reference must have shape (N+1, 2)")
        if start_hidden.shape != (2,):
            raise ValueError("start_hidden must have shape (2,)")
        if not np.isfinite(np.r_[A.ravel(), B.ravel(), scales, pv.ravel(), start_hidden]).all():
            raise ValueError("cached projection inputs must be finite")

        interval_count = len(pv) - 1
        key = self._key(
            A, B, scales, input_scale_nm, planning_dt_s, interval_count
        )
        cache_hit = key in self._entries
        if cache_hit:
            entry = self._entries[key]
        else:
            entry = self._build_entry(
                A, B, scales, input_scale_nm, interval_count
            )
            self._entries[key] = entry

        rhs_started = time.perf_counter()
        rhs = np.empty(entry.row_count, dtype=float)
        for sample in range(interval_count):
            value = A[:, :2] @ pv[sample] - np.r_[pv[sample + 1], 0.0, 0.0]
            if sample == 0:
                value += A[:, 2:] @ start_hidden
            rhs[4 * sample : 4 * sample + 4] = value / scales
        rhs_build_time_s = time.perf_counter() - rhs_started

        augmented_rhs = np.r_[rhs, np.zeros(entry.variable_count, dtype=float)]
        solve_started = time.perf_counter()
        augmented_solution = entry.factor.solve(augmented_rhs)
        solve_time_s = time.perf_counter() - solve_started
        residual_vector = (
            augmented_solution[: entry.row_count]
            + entry.matrix @ augmented_solution[entry.row_count :]
            - rhs
        )
        stationarity = entry.matrix.T @ augmented_solution[: entry.row_count]
        denominator = max(float(np.linalg.norm(augmented_rhs)), 1e-30)
        kkt_residual_relative = float(np.linalg.norm(np.r_[residual_vector, stationarity]) / denominator)

        normalized_solution = augmented_solution[entry.row_count :]
        reference_states = np.column_stack([pv, np.zeros((interval_count + 1, 2))])
        reference_states[0, 2:] = start_hidden
        if entry.input_offset:
            reference_states[1:interval_count, 2:] = (
                normalized_solution[: entry.input_offset].reshape(-1, 2)
                * scales[2:]
            )
        inputs_nm = normalized_solution[entry.input_offset:] * input_scale_nm
        predicted = reference_states[:-1] @ A.T + inputs_nm[:, None] * B[:, 0]
        residuals = reference_states[1:] - predicted
        projection = NominalTrajectoryProjection(
            reference_states=reference_states,
            inputs_nm=inputs_nm,
            residuals=residuals,
            segment_diagnostics=({
                "solver": "cached_sparse_kkt_lu",
                "interval_count": interval_count,
                "kkt_residual_relative": kkt_residual_relative,
                "diagonal_pivot_ratio": entry.diagonal_pivot_ratio,
                "cache_hit": cache_hit,
            },),
        )
        return CachedProjectionSolve(
            projection=projection,
            matrix_build_time_s=(0.0 if cache_hit else entry.matrix_build_time_s),
            factorization_time_s=(0.0 if cache_hit else entry.factorization_time_s),
            rhs_build_time_s=rhs_build_time_s,
            solve_time_s=solve_time_s,
            kkt_residual_relative=kkt_residual_relative,
            diagonal_pivot_ratio=entry.diagonal_pivot_ratio,
            cache_hit=cache_hit,
        )


def project_hidden_reference_and_input(
    A: NDArray[np.float64],
    B: NDArray[np.float64],
    state_scales: NDArray[np.float64],
    input_scale_nm: float,
    position_velocity_reference: NDArray[np.float64],
    segments: list[tuple[int, int]],
    segment_hidden_boundaries: list[
        tuple[NDArray[np.float64], NDArray[np.float64]]
    ] | None = None,
    timing_diagnostics: dict | None = None,
) -> NominalTrajectoryProjection:
    """Project prescribed ``p/v`` onto hidden pitch/rate and nominal input.

    Each segment minimizes the normalized full-state dynamics defects while
    fixing pitch error and pitch rate at both segment boundaries. Boundaries
    default to zero. The unknown columns are scaled using the identification
    state/input scales for numerical conditioning.
    """

    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)
    state_scales = np.asarray(state_scales, dtype=float)
    position_velocity_reference = np.asarray(
        position_velocity_reference, dtype=float
    )
    input_scale_nm = float(input_scale_nm)
    if A.shape != (4, 4) or B.shape != (4, 1):
        raise ValueError("A/B must have shapes (4, 4)/(4, 1)")
    if state_scales.shape != (4,) or np.any(state_scales <= 0.0):
        raise ValueError("state_scales must be a positive length-4 vector")
    if not np.isfinite(input_scale_nm) or input_scale_nm <= 0.0:
        raise ValueError("input_scale_nm must be finite and positive")
    if (
        position_velocity_reference.ndim != 2
        or position_velocity_reference.shape[1] != 2
        or len(position_velocity_reference) < 2
    ):
        raise ValueError("position_velocity_reference must have shape (N+1, 2)")
    if not np.isfinite(
        np.r_[
            A.ravel(),
            B.ravel(),
            state_scales,
            position_velocity_reference.ravel(),
        ]
    ).all():
        raise ValueError("projection inputs must be finite")

    interval_count = len(position_velocity_reference) - 1
    if segment_hidden_boundaries is None:
        boundary_pairs = [
            (np.zeros(2, dtype=float), np.zeros(2, dtype=float))
            for _ in segments
        ]
    else:
        if len(segment_hidden_boundaries) != len(segments):
            raise ValueError(
                "segment_hidden_boundaries must match the projection segments"
            )
        boundary_pairs = []
        for start_hidden, end_hidden in segment_hidden_boundaries:
            start_array = np.asarray(start_hidden, dtype=float)
            end_array = np.asarray(end_hidden, dtype=float)
            if start_array.shape != (2,) or end_array.shape != (2,):
                raise ValueError("hidden boundary states must be length-2 vectors")
            if not np.isfinite(np.r_[start_array, end_array]).all():
                raise ValueError("hidden boundary states must be finite")
            boundary_pairs.append((start_array.copy(), end_array.copy()))

    reference_states = np.column_stack(
        [position_velocity_reference, np.zeros((interval_count + 1, 2))]
    )
    inputs_nm = np.zeros(interval_count)
    occupied = np.zeros(interval_count, dtype=bool)
    assigned_boundaries = np.zeros(interval_count + 1, dtype=bool)
    diagnostics = []
    for (start, end), (start_hidden, end_hidden) in zip(
        segments, boundary_pairs
    ):
        start = int(start)
        end = int(end)
        if start < 0 or end > interval_count or end <= start:
            raise ValueError(f"invalid projection segment ({start}, {end})")
        if np.any(occupied[start:end]):
            raise ValueError("projection segments must not overlap")
        occupied[start:end] = True
        fixed_boundaries = [(start, start_hidden), (end, end_hidden)]
        for boundary_index, hidden_state in fixed_boundaries:
            if assigned_boundaries[boundary_index] and not np.allclose(
                reference_states[boundary_index, 2:],
                hidden_state,
                atol=1e-12,
                rtol=0.0,
            ):
                raise ValueError("adjacent projection boundary states disagree")
            reference_states[boundary_index, 2:] = hidden_state
            assigned_boundaries[boundary_index] = True
        segment_pv = position_velocity_reference[start : end + 1]
        segment_length = end - start
        hidden_variable_count = 2 * max(0, segment_length - 1)
        input_offset = hidden_variable_count
        variable_count = hidden_variable_count + segment_length
        rows: list[int] = []
        columns: list[int] = []
        values: list[float] = []
        right_hand_side: list[float] = []
        system_build_started = time.perf_counter()
        rhs_build_time_s = 0.0

        def hidden_column(sample: int, state_index: int) -> int:
            return 2 * (sample - 1) + state_index

        for sample in range(segment_length):
            rhs_started = (
                time.perf_counter() if timing_diagnostics is not None else 0.0
            )
            rhs = A[:, :2] @ segment_pv[sample] - np.r_[
                segment_pv[sample + 1], 0.0, 0.0
            ]
            if sample == 0:
                rhs += A[:, 2:] @ start_hidden
            if sample + 1 == segment_length:
                rhs[2:] -= end_hidden
            for state_row in range(4):
                equation_row = 4 * sample + state_row
                row_scale = 1.0 / state_scales[state_row]
                right_hand_side.append(float(rhs[state_row] * row_scale))
            if timing_diagnostics is not None:
                rhs_build_time_s += time.perf_counter() - rhs_started

            for state_row in range(4):
                equation_row = 4 * sample + state_row
                row_scale = 1.0 / state_scales[state_row]
                if sample > 0:
                    for hidden_index in range(2):
                        rows.append(equation_row)
                        columns.append(hidden_column(sample, hidden_index))
                        values.append(
                            float(
                                -A[state_row, 2 + hidden_index]
                                * state_scales[2 + hidden_index]
                                * row_scale
                            )
                        )
                if sample + 1 < segment_length and state_row >= 2:
                    rows.append(equation_row)
                    columns.append(hidden_column(sample + 1, state_row - 2))
                    values.append(float(state_scales[state_row] * row_scale))
                rows.append(equation_row)
                columns.append(input_offset + sample)
                values.append(
                    float(-B[state_row, 0] * input_scale_nm * row_scale)
                )

        matrix = coo_matrix(
            (values, (rows, columns)),
            shape=(4 * segment_length, variable_count),
        ).tocsr()
        system_build_time_s = time.perf_counter() - system_build_started
        solve_started = time.perf_counter()
        solution = lsqr(
            matrix,
            np.asarray(right_hand_side),
            atol=1e-11,
            btol=1e-11,
            iter_lim=max(1000, 3 * variable_count),
            show=False,
        )
        solve_time_s = time.perf_counter() - solve_started
        if timing_diagnostics is not None:
            timing_diagnostics.update({
                "matrix_and_rhs_build_time_s": system_build_time_s,
                "rhs_build_time_s": rhs_build_time_s,
                "matrix_build_time_s": max(
                    0.0, system_build_time_s - rhs_build_time_s
                ),
                "solve_time_s": solve_time_s,
            })
        if solution[1] not in (1, 2):
            raise RuntimeError(
                "nominal trajectory projection did not converge: "
                f"istop={solution[1]}"
            )
        normalized_solution = solution[0]
        solver_diagnostics = {
            "solver": "lsqr",
            "lsqr_stop_code": int(solution[1]),
            "lsqr_iteration_count": int(solution[2]),
            "scaled_matrix_condition_estimate": float(solution[6]),
            "normal_equation_residual_norm": float(solution[7]),
        }
        if hidden_variable_count:
            reference_states[start + 1 : end, 2:] = normalized_solution[
                :input_offset
            ].reshape(-1, 2) * state_scales[2:]
        inputs_nm[start:end] = (
            normalized_solution[input_offset:] * input_scale_nm
        )
        diagnostics.append(
            {
                "start_index": start,
                "end_index": end,
                "interval_count": segment_length,
                **solver_diagnostics,
            }
        )

    predicted = reference_states[:-1] @ A.T + inputs_nm[:, None] * B[:, 0]
    residuals = reference_states[1:] - predicted
    if not np.isfinite(
        np.r_[reference_states.ravel(), inputs_nm, residuals.ravel()]
    ).all():
        raise FloatingPointError("nominal trajectory projection is not finite")
    return NominalTrajectoryProjection(
        reference_states=reference_states,
        inputs_nm=inputs_nm,
        residuals=residuals,
        segment_diagnostics=tuple(diagnostics),
    )
