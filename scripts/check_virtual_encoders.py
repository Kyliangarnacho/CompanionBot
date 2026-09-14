"""Bring up integer wheel counts without velocity or state estimation."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim import MiniSegwaySim


MODEL_DIR = ROOT / "models" / "minisegway"
MODEL_PATHS = (
    MODEL_DIR / "mini_segway.xml",
    MODEL_DIR / "mini_segway_moving_payload.xml",
)


def set_wheel_angles(sim: MiniSegwaySim, angles_rad: np.ndarray) -> np.ndarray:
    sim.reset()
    for joint_name, angle_rad in zip(
        ("left_wheel_hinge", "right_wheel_hinge"), angles_rad
    ):
        joint = sim.model.joint(joint_name).id
        sim.data.qpos[sim.model.jnt_qposadr[joint]] = angle_rad
    mujoco.mj_forward(sim.model, sim.data)
    return sim.snapshot().wheel_encoder_counts


def main() -> None:
    reference_profile = None
    for model_path in MODEL_PATHS:
        sim = MiniSegwaySim(model_path)
        profile = sim.encoder_profile
        if reference_profile is None:
            reference_profile = profile
        else:
            assert profile == reference_profile

        count_angle = 2.0 * math.pi / profile.output_decoded_counts_per_rev
        zero = sim.snapshot().wheel_encoder_counts
        positive = set_wheel_angles(sim, np.array([0.25, 0.25]))
        negative = set_wheel_angles(sim, np.array([-0.25, -0.25]))
        positive_turn = set_wheel_angles(sim, np.full(2, 2.0 * math.pi))
        negative_turn = set_wheel_angles(sim, np.full(2, -2.0 * math.pi))
        expected_turn_count = math.floor(profile.output_decoded_counts_per_rev + 0.5)

        small_steps = np.array([-1.51, -1.49, -0.51, -0.49, 0.0, 0.49, 0.51, 1.49, 1.51])
        quantized = np.array(
            [set_wheel_angles(sim, np.full(2, scale * count_angle))[0] for scale in small_steps]
        )

        sim.reset()
        for _ in range(10):
            sim.step(0.0, 0.0)
        start_y = float(sim.data.qpos[1])
        start_counts = sim.wheel_encoder_counts()
        for _ in range(20):
            sim.step(0.05, 0.05)
        delta_y = float(sim.data.qpos[1]) - start_y
        delta_counts = sim.wheel_encoder_counts() - start_counts

        assert np.array_equal(zero, [0, 0])
        assert np.all(positive > 0) and np.array_equal(negative, -positive)
        assert np.array_equal(positive_turn, [expected_turn_count, expected_turn_count])
        assert np.array_equal(negative_turn, [-expected_turn_count, -expected_turn_count])
        assert np.array_equal(quantized, [-2, -1, -1, 0, 0, 0, 1, 1, 2])
        assert delta_y < 0.0 and np.all(delta_counts > 0)

        print(f"\n{model_path.name}: {profile.name}")
        print(f"  output resolution: {profile.output_decoded_counts_per_rev:.4f} counts/rev")
        print(f"  zero={zero}, +0.25rad={positive}, -0.25rad={negative}")
        print(f"  +1rev={positive_turn}, -1rev={negative_turn}")
        print(f"  small-angle scales={small_steps}")
        print(f"  quantized counts={quantized}")
        print(f"  forward check: delta_y={delta_y:.9f} m, delta_counts={delta_counts}")

    print("\nPASS: integer quantization, signs, one-turn counts, and profile parity verified")


if __name__ == "__main__":
    main()
