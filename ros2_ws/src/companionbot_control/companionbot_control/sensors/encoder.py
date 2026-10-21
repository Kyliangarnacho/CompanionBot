"""Encoder profile/quantization contract, reusable without MuJoCo."""
from __future__ import annotations
from dataclasses import dataclass
import json
import math
from pathlib import Path
import numpy as np
from numpy.typing import NDArray
from companionbot_contracts.paths import repository_root

DEFAULT_ENCODER_PROFILE_PATH = repository_root()/"models/minisegway/encoder_profile.json"

@dataclass(frozen=True)
class QuadratureEncoderProfile:
    name: str
    motor_shaft_decoded_counts_per_rev: int
    exact_gearbox_ratio: float
    output_decoded_counts_per_rev: float
    quantization: str
    left_count_sign: int
    right_count_sign: int

    def counts_from_angle_delta(self, angle_delta_rad: NDArray[np.float64]) -> NDArray[np.int64]:
        """Quantize unwrapped output-shaft angle relative to the captured zero."""

        angles = np.asarray(angle_delta_rad, dtype=float)
        if angles.shape != (2,) or not np.isfinite(angles).all():
            raise ValueError("left/right encoder angle deltas must be a finite length-2 vector")
        signs = np.asarray([self.left_count_sign, self.right_count_sign], dtype=float)
        scaled_counts = angles * signs * self.output_decoded_counts_per_rev / (2.0 * math.pi)
        return np.copysign(np.floor(np.abs(scaled_counts) + 0.5), scaled_counts).astype(np.int64)

def load_encoder_profile(
    path: str | Path = DEFAULT_ENCODER_PROFILE_PATH,
) -> QuadratureEncoderProfile:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    profile = QuadratureEncoderProfile(**raw)
    expected_output_resolution = (
        profile.motor_shaft_decoded_counts_per_rev * profile.exact_gearbox_ratio
    )
    if profile.motor_shaft_decoded_counts_per_rev <= 0 or profile.exact_gearbox_ratio <= 0.0:
        raise ValueError("encoder resolution and gearbox ratio must be positive")
    if not math.isclose(
        profile.output_decoded_counts_per_rev,
        expected_output_resolution,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("output encoder resolution must equal motor resolution times gearbox ratio")
    if profile.quantization != "round_to_nearest_away_from_zero":
        raise ValueError(f"unsupported encoder quantization: {profile.quantization}")
    if profile.left_count_sign not in (-1, 1) or profile.right_count_sign not in (-1, 1):
        raise ValueError("left/right encoder count signs must be +1 or -1")
    return profile

