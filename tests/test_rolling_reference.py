import json
from pathlib import Path
import unittest

import numpy as np

from control.rolling_reference import (
    DeterministicReferenceBlockStream,
    RuckigFullHorizonVelocityPlanner,
    RuckigLightweightVelocityPlanner,
    RollingReferenceCommand,
    SparseVelocityCommandScheduler,
)
from control.trajectory_feedforward import (
    SparseProjectionFactorizationCache,
    project_hidden_reference_and_input,
)


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "models" / "minisegway"
CONFIG_DIR = MODEL_DIR / "stage3" / "results" / "config"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def planner_inputs() -> dict:
    manifest = load_json(CONFIG_DIR / "baseline.json")
    motion = load_json(CONFIG_DIR / "stage3a_commanded_motion_config.json")
    dynamic = load_json(
        CONFIG_DIR / "stage3a_dynamic_nominal_reference_config.json"
    )
    offline = load_json(MODEL_DIR / manifest["feedback"]["source"])
    nominal = load_json(MODEL_DIR / manifest["nominal_model"]["source"])
    offline["fit"]["A_identified"] = nominal[
        manifest["nominal_model"]["A_field"]
    ]
    offline["fit"]["B_identified"] = nominal[
        manifest["nominal_model"]["B_field"]
    ]
    fit = offline["fit"]
    limits = motion["reference_limits"]
    return {
        "controller_dt_s": float(motion["controller_dt_s"]),
        "max_velocity_m_s": float(limits["max_velocity_m_s"]),
        "max_acceleration_m_s2": float(limits["max_acceleration_m_s2"]),
        "max_jerk_m_s3": float(dynamic["max_jerk_m_s3"]),
        "A": np.asarray(fit["A_identified"], dtype=float),
        "B": np.asarray(fit["B_identified"], dtype=float),
        "state_scales": np.asarray(fit["state_scales"], dtype=float),
        "input_scale_nm": float(fit["input_scale_nm"]),
    }


def make_planner() -> RuckigFullHorizonVelocityPlanner:
    return RuckigFullHorizonVelocityPlanner(
        minimum_horizon_s=1.5,
        dynamic_settle_margin_s=0.3,
        **planner_inputs(),
    )


def make_lightweight_planner() -> RuckigLightweightVelocityPlanner:
    return RuckigLightweightVelocityPlanner(**planner_inputs())


class RollingReferenceTest(unittest.TestCase):
    def test_scheduler_uses_accepted_baseline_and_classifies_before_update(self):
        scheduler = SparseVelocityCommandScheduler(
            accept_delta_v_m_s=0.12,
            full_delta_v_m_s=0.30,
            stable_window_s=0.20,
            stable_range_m_s=0.03,
        )
        scheduler.update(0.00, 0.0)
        scheduler.update(0.05, 0.13)
        scheduler.update(0.10, 0.18)
        scheduler.update(0.15, 0.19)
        scheduler.update(0.20, 0.19)
        scheduler.update(0.25, 0.19)
        accepted = scheduler.update(0.30, 0.19)
        self.assertTrue(accepted.accepted)
        self.assertEqual(accepted.mode, "LIGHTWEIGHT")
        self.assertAlmostEqual(accepted.candidate_delta_v_m_s, 0.19)
        self.assertAlmostEqual(scheduler.accepted_velocity_m_s, 0.19)
        for time_s in (0.35, 0.40, 0.45, 0.50):
            self.assertFalse(scheduler.update(time_s, 0.30).accepted)
        scheduler.update(0.55, 0.32)
        scheduler.update(0.60, 0.36)
        scheduler.update(0.65, 0.39)
        scheduler.update(0.70, 0.39)
        scheduler.update(0.75, 0.39)
        fast = scheduler.update(0.80, 0.39)
        self.assertTrue(fast.accepted)
        self.assertEqual(fast.mode, "LIGHTWEIGHT")

    def test_scheduler_cancels_candidate_inside_accepted_deadband(self):
        scheduler = SparseVelocityCommandScheduler(
            accept_delta_v_m_s=0.12,
            full_delta_v_m_s=0.30,
            stable_window_s=0.20,
            stable_range_m_s=0.03,
        )
        scheduler.update(0.0, 0.13)
        scheduler.update(0.1, 0.05)
        self.assertFalse(scheduler.update(0.2, 0.14).accepted)
        scheduler.update(0.25, 0.14)
        scheduler.update(0.30, 0.14)
        scheduler.update(0.35, 0.14)
        self.assertTrue(scheduler.update(0.40, 0.14).accepted)

    def test_scheduler_remembers_fast_jump_when_window_ends_on_plateau(self):
        scheduler = SparseVelocityCommandScheduler(
            accept_delta_v_m_s=0.12,
            full_delta_v_m_s=0.30,
            stable_window_s=0.20,
            stable_range_m_s=0.03,
            initial_accepted_velocity_m_s=0.22,
        )
        scheduler.update(0.00, 0.22)
        scheduler.update(0.05, -0.60)
        scheduler.update(0.15, -0.60)
        accepted = scheduler.update(0.25, -0.60)
        self.assertTrue(accepted.accepted)
        self.assertEqual(accepted.mode, "FULL_DYNAMIC")
        self.assertAlmostEqual(accepted.candidate_delta_v_m_s, -0.82)
        self.assertAlmostEqual(accepted.candidate_target_m_s, -0.60)

    def test_scheduler_uses_latest_value_after_candidate_direction_reversal(self):
        scheduler = SparseVelocityCommandScheduler(
            accept_delta_v_m_s=0.12,
            full_delta_v_m_s=0.30,
            stable_window_s=0.20,
            stable_range_m_s=0.03,
        )
        scheduler.update(0.00, 0.0)
        scheduler.update(0.05, 0.20)
        scheduler.update(0.10, -0.20)
        scheduler.update(0.15, -0.25)
        scheduler.update(0.20, -0.25)
        scheduler.update(0.25, -0.25)
        scheduler.update(0.30, -0.25)
        accepted = scheduler.update(0.35, -0.25)
        self.assertTrue(accepted.accepted)
        self.assertEqual(accepted.mode, "LIGHTWEIGHT")
        self.assertAlmostEqual(accepted.candidate_target_m_s, -0.25)

    def test_lightweight_path_has_exact_zero_dynamic_reference(self):
        planner = make_lightweight_planner()
        planned = planner.plan(np.zeros(4), 0.0, 0.3)
        np.testing.assert_array_equal(
            planned.plan.reference_states[:, 2:], 0.0
        )
        np.testing.assert_array_equal(planned.plan.feedforward_inputs_nm, 0.0)
        self.assertFalse(planned.plan.feedforward_rearmed)

    def test_full_path_uses_sparse_lsqr_and_zero_terminal_hidden_state(self):
        planner = make_planner()
        planned = planner.plan(np.zeros(4), 0.0, 0.3)
        states = planned.plan.reference_states
        expected_intervals = max(
            planner.minimum_horizon_intervals,
            round(planned.ruckig_duration_s / planner.dt_s)
            + planner.dynamic_settle_margin_intervals,
        )
        self.assertEqual(len(states), expected_intervals + 1)
        self.assertTrue(np.isfinite(states).all())
        self.assertTrue(np.isfinite(planned.plan.feedforward_inputs_nm).all())
        np.testing.assert_array_equal(states[-1, 2:], 0.0)
        self.assertEqual(
            planned.projection_diagnostics[0]["solver"],
            "lsqr",
        )

    def test_cached_kkt_projection_matches_sparse_lsqr(self):
        planner = make_planner()
        cache = SparseProjectionFactorizationCache()
        intervals = 18
        times = np.arange(intervals + 1) * planner.dt_s
        pv = np.column_stack([
            0.4 * times + 0.02 * np.sin(3.0 * times),
            0.4 + 0.03 * np.cos(3.0 * times),
        ])
        start_hidden = np.asarray([0.004, -0.012])
        reference = project_hidden_reference_and_input(
            planner.A,
            planner.B,
            planner.state_scales,
            planner.input_scale_nm,
            pv,
            [(0, intervals)],
            segment_hidden_boundaries=[(start_hidden, np.zeros(2))],
        )
        first = cache.solve_fixed_terminal(
            planner.A,
            planner.B,
            planner.state_scales,
            planner.input_scale_nm,
            planner.dt_s,
            pv,
            start_hidden,
        )
        second = cache.solve_fixed_terminal(
            planner.A,
            planner.B,
            planner.state_scales,
            planner.input_scale_nm,
            planner.dt_s,
            pv,
            start_hidden,
        )
        np.testing.assert_allclose(
            first.projection.reference_states,
            reference.reference_states,
            rtol=1e-7,
            atol=1e-9,
        )
        np.testing.assert_allclose(
            first.projection.inputs_nm,
            reference.inputs_nm,
            rtol=1e-7,
            atol=1e-9,
        )
        self.assertFalse(first.cache_hit)
        self.assertTrue(second.cache_hit)
        self.assertLess(first.kkt_residual_relative, 1e-10)

    def test_timestamped_reference_stream_keeps_active_next_ready_with_small_delay(self):
        class Producer:
            dt_s = 0.002
            block_samples = 25
            replan_events: list[dict] = []

            def command(
                self, time_s: float, *, observe_command: bool = True,
                planning_request_time_s: float | None = None,
            ) -> RollingReferenceCommand:
                del observe_command, planning_request_time_s
                return RollingReferenceCommand(
                    reference_state=np.asarray([time_s, 0.2, 0.0, 0.0]),
                    reference_acceleration_m_s2=0.0,
                    u_ff_raw_nm=0.0,
                    u_ff_after_lifecycle_nm=0.0,
                    linear_velocity_target_m_s=0.2,
                    yaw_rate_target_rad_s=0.0,
                    feedforward_phase="OFF",
                    velocity_phase="HOLD",
                    fade_alpha=0.0,
                    block_id=0,
                    block_sample_index=0,
                    block_start_time_s=0.0,
                    intent_source_time_s=time_s,
                    replanned=False,
                )

        stream = DeterministicReferenceBlockStream(
            Producer(), link_latency_s=0.004
        )
        commands = [stream.command(tick * 0.002) for tick in range(100)]
        summary = stream.summary()
        self.assertEqual(len(commands), 100)
        self.assertEqual(len(summary["underrun_events"]), 0)
        self.assertEqual(len(summary["stale_events"]), 0)
        self.assertEqual(len(summary["sequence_gap_events"]), 0)
        self.assertGreaterEqual(summary["next_ready_at_swap_count"], 3)
        self.assertTrue(all(command.next_block_ready for command in commands[:25]))

    def test_default_projection_boundary_is_unchanged(self):
        planner = make_planner()
        seed_plan = planner.plan(np.zeros(4), 0.0, 0.2)
        pv = seed_plan.plan.reference_states[:, :2]
        end = len(pv) - 1
        default = project_hidden_reference_and_input(
            planner.A,
            planner.B,
            planner.state_scales,
            planner.input_scale_nm,
            pv,
            [(0, end)],
        )
        explicit = project_hidden_reference_and_input(
            planner.A,
            planner.B,
            planner.state_scales,
            planner.input_scale_nm,
            pv,
            [(0, end)],
            segment_hidden_boundaries=[(np.zeros(2), np.zeros(2))],
        )
        np.testing.assert_array_equal(
            default.reference_states, explicit.reference_states
        )
        np.testing.assert_array_equal(default.inputs_nm, explicit.inputs_nm)

    def test_nonzero_acceleration_and_hidden_state_replan_is_continuous(self):
        planner = make_planner()
        first = planner.plan(np.zeros(4), 0.0, 0.4)
        index = 25
        start = first.plan.reference_states[index].copy()
        start_acceleration = float(
            first.plan.reference_accelerations_m_s2[index]
        )
        self.assertGreater(abs(start_acceleration), 1e-6)
        self.assertGreater(abs(float(start[2])), 1e-6)
        replanned = planner.plan(start, start_acceleration, 0.42)
        np.testing.assert_allclose(
            replanned.plan.reference_states[0], start, atol=1e-12, rtol=0.0
        )
        self.assertAlmostEqual(
            float(replanned.plan.reference_accelerations_m_s2[0]),
            start_acceleration,
            delta=1e-12,
        )
        np.testing.assert_allclose(
            replanned.plan.reference_states[-1, 1:],
            [0.42, 0.0, 0.0],
            atol=1e-9,
            rtol=0.0,
        )
        acceleration = replanned.plan.reference_accelerations_m_s2
        self.assertLessEqual(float(np.max(np.abs(acceleration))), 0.6 + 1e-10)
        jerk = np.diff(acceleration) / planner.dt_s
        self.assertLessEqual(float(np.max(np.abs(jerk))), 1.0 + 1e-8)

if __name__ == "__main__":
    unittest.main()
