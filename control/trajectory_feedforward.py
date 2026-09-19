"""Nominal scalar feedforward for the frozen discrete longitudinal model."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import lsqr


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


def project_hidden_reference_and_input(
    A: NDArray[np.float64],
    B: NDArray[np.float64],
    state_scales: NDArray[np.float64],
    input_scale_nm: float,
    position_velocity_reference: NDArray[np.float64],
    segments: list[tuple[int, int]],
) -> NominalTrajectoryProjection:
    """Project prescribed ``p/v`` onto hidden pitch/rate and nominal input.

    Each segment minimizes the normalized full-state dynamics defects while
    fixing pitch error and pitch rate to zero at both segment boundaries.  The
    unknown columns are scaled using the identification state/input scales for
    numerical conditioning; this does not add a regularization objective.
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
    reference_states = np.column_stack(
        [position_velocity_reference, np.zeros((interval_count + 1, 2))]
    )
    inputs_nm = np.zeros(interval_count)
    occupied = np.zeros(interval_count, dtype=bool)
    diagnostics = []
    for start, end in segments:
        start = int(start)
        end = int(end)
        if start < 0 or end > interval_count or end <= start:
            raise ValueError(f"invalid projection segment ({start}, {end})")
        if np.any(occupied[start:end]):
            raise ValueError("projection segments must not overlap")
        occupied[start:end] = True
        segment_pv = position_velocity_reference[start : end + 1]
        segment_length = end - start
        hidden_variable_count = 2 * max(0, segment_length - 1)
        input_offset = hidden_variable_count
        variable_count = hidden_variable_count + segment_length
        rows: list[int] = []
        columns: list[int] = []
        values: list[float] = []
        right_hand_side: list[float] = []

        def hidden_column(sample: int, state_index: int) -> int:
            return 2 * (sample - 1) + state_index

        for sample in range(segment_length):
            rhs = A[:, :2] @ segment_pv[sample] - np.r_[
                segment_pv[sample + 1], 0.0, 0.0
            ]
            for state_row in range(4):
                equation_row = 4 * sample + state_row
                row_scale = 1.0 / state_scales[state_row]
                right_hand_side.append(float(rhs[state_row] * row_scale))
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
        solution = lsqr(
            matrix,
            np.asarray(right_hand_side),
            atol=1e-11,
            btol=1e-11,
            iter_lim=max(1000, 3 * variable_count),
            show=False,
        )
        if solution[1] not in (1, 2):
            raise RuntimeError(
                f"nominal trajectory projection did not converge: istop={solution[1]}"
            )
        normalized_solution = solution[0]
        if segment_length > 1:
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
                "lsqr_stop_code": int(solution[1]),
                "lsqr_iteration_count": int(solution[2]),
                "scaled_matrix_condition_estimate": float(solution[6]),
                "normal_equation_residual_norm": float(solution[7]),
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
