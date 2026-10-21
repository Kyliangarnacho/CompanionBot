"""Discrete fixed-gain LQR based on the checked-in reduced TWIP model."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
from scipy.linalg import solve_discrete_are
from scipy.signal import cont2discrete


@dataclass(frozen=True)
class LQRDesign:
    controller_dt_s: float
    A_discrete: NDArray[np.float64]
    B_discrete: NDArray[np.float64]
    Q: NDArray[np.float64]
    R: NDArray[np.float64]
    K: NDArray[np.float64]
    closed_loop_poles: NDArray[np.complex128]


@dataclass(frozen=True)
class ControlCommand:
    raw_sum_torque_nm: float
    limited_sum_torque_nm: float
    left_torque_nm: float
    right_torque_nm: float
    saturated: bool


def design_from_files(reduced_path: str | Path, config_path: str | Path) -> LQRDesign:
    reduced = json.loads(Path(reduced_path).read_text(encoding="utf-8"))
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    A = np.asarray(reduced["continuous_time"]["A"], dtype=float)
    B = np.asarray(reduced["continuous_time"]["B"], dtype=float)
    controller_dt = 1.0 / float(config["controller_frequency_hz"])
    Q = np.diag(np.asarray(config["Q_diag"], dtype=float))
    R = np.array([[float(config["R"])]])
    A_d, B_d, _, _, _ = cont2discrete(
        (A, B, np.eye(A.shape[0]), np.zeros((A.shape[0], B.shape[1]))),
        controller_dt,
        method="zoh",
    )
    riccati = solve_discrete_are(A_d, B_d, Q, R)
    gain = np.linalg.solve(R + B_d.T @ riccati @ B_d, B_d.T @ riccati @ A_d)
    poles = np.linalg.eigvals(A_d - B_d @ gain)
    return LQRDesign(controller_dt, A_d, B_d, Q, R, gain, poles)


class FixedLQR:
    def __init__(self, design: LQRDesign, per_wheel_peak_nm: float):
        self.design = design
        self.per_wheel_peak_nm = float(per_wheel_peak_nm)
        self.sum_peak_nm = 2.0 * self.per_wheel_peak_nm

    def command(self, state: NDArray[np.float64]) -> ControlCommand:
        raw_sum = -float(self.design.K @ np.asarray(state, dtype=float))
        limited_sum = float(np.clip(raw_sum, -self.sum_peak_nm, self.sum_peak_nm))
        wheel_torque = limited_sum / 2.0
        return ControlCommand(
            raw_sum_torque_nm=raw_sum,
            limited_sum_torque_nm=limited_sum,
            left_torque_nm=wheel_torque,
            right_torque_nm=wheel_torque,
            saturated=not np.isclose(raw_sum, limited_sum, rtol=0.0, atol=1e-12),
        )
