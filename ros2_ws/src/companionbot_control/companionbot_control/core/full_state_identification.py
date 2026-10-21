"""Offline full-state discrete identification and LQR design utilities."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray
from scipy.linalg import lstsq, solve_discrete_are


@dataclass(frozen=True)
class DiscreteStateSpaceModel:
    A: NDArray[np.float64]
    B: NDArray[np.float64]

    def predict(
        self, states: NDArray[np.float64], inputs: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        states = np.asarray(states, dtype=float)
        inputs = np.asarray(inputs, dtype=float).reshape(-1, 1)
        return states @ self.A.T + inputs @ self.B.T


@dataclass(frozen=True)
class FullStateFit:
    model: DiscreteStateSpaceModel
    state_scales: NDArray[np.float64]
    input_scale: float
    normalized_regressor_condition_number: float
    normalized_regressor_singular_values: NDArray[np.float64]
    regressor_rank: int
    ridge_lambda: float


def fit_full_state_ridge(
    states: NDArray[np.float64],
    inputs: NDArray[np.float64],
    next_states: NDArray[np.float64],
    ridge_lambda: float,
    minimum_state_scales: NDArray[np.float64],
    minimum_input_scale: float,
) -> FullStateFit:
    """Fit x[k+1] = A x[k] + B u[k] in normalized coordinates."""

    states = np.asarray(states, dtype=float)
    inputs = np.asarray(inputs, dtype=float).reshape(-1, 1)
    next_states = np.asarray(next_states, dtype=float)
    state_scales = np.maximum(
        np.sqrt(np.mean(states**2, axis=0)),
        np.asarray(minimum_state_scales, dtype=float),
    )
    input_scale = max(float(np.sqrt(np.mean(inputs**2))), float(minimum_input_scale))
    normalized_regressor = np.column_stack(
        [states / state_scales, inputs[:, 0] / input_scale]
    )
    normalized_targets = next_states / state_scales
    singular_values = np.linalg.svd(normalized_regressor, compute_uv=False)
    condition = float(singular_values[0] / singular_values[-1])
    rank = int(np.linalg.matrix_rank(normalized_regressor))
    if ridge_lambda > 0.0:
        dimension = normalized_regressor.shape[1]
        augmented_regressor = np.vstack(
            [normalized_regressor, np.sqrt(ridge_lambda) * np.eye(dimension)]
        )
        augmented_targets = np.vstack(
            [normalized_targets, np.zeros((dimension, normalized_targets.shape[1]))]
        )
    else:
        augmented_regressor = normalized_regressor
        augmented_targets = normalized_targets
    normalized_theta, _, _, _ = lstsq(
        augmented_regressor, augmented_targets, lapack_driver="gelsd"
    )
    normalized_A = normalized_theta[:4].T
    normalized_B = normalized_theta[4:].T
    state_scale_matrix = np.diag(state_scales)
    A = state_scale_matrix @ normalized_A @ np.diag(1.0 / state_scales)
    B = state_scale_matrix @ normalized_B / input_scale
    if not np.isfinite(np.r_[A.ravel(), B.ravel()]).all():
        raise FloatingPointError("identified state-space matrices are not finite")
    return FullStateFit(
        DiscreteStateSpaceModel(A, B),
        state_scales,
        input_scale,
        condition,
        singular_values,
        rank,
        float(ridge_lambda),
    )


def design_discrete_lqr(
    model: DiscreteStateSpaceModel,
    Q: NDArray[np.float64],
    R: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.complex128], int]:
    controllability = np.hstack(
        [
            np.linalg.matrix_power(model.A, power) @ model.B
            for power in range(model.A.shape[0])
        ]
    )
    rank = int(np.linalg.matrix_rank(controllability))
    riccati = solve_discrete_are(model.A, model.B, Q, R)
    gain = np.linalg.solve(
        R + model.B.T @ riccati @ model.B,
        model.B.T @ riccati @ model.A,
    )
    poles = np.linalg.eigvals(model.A - model.B @ gain)
    if not np.isfinite(np.r_[gain.ravel(), poles.real, poles.imag]).all():
        raise FloatingPointError("identified-model LQR result is not finite")
    return gain, poles, rank
