# Stage 3 frozen baseline

The authoritative entry point is `config/baseline.json`. It records the selected fixed
A/B, retained K4, estimator mode, yaw gains, feedforward lifecycle and the Stage 3C-R
decision that the Q observer remains diagnostic-only.

Retained result set:

- `results/stage3a_refit_ab_0p60_validation_results.json`
- `results/stage3a_longitudinal_baseline_results.json`
- `results/stage3a_velocity_feedforward_lean_tail_results.json`
- `results/stage3b_yaw_control_results.json`
- `results/stage3c_mechanical_payload_q_revalidation_results.json`
- `results/stage3c_failed_payload_summary.json`

Retained runners create `results/history/` on demand for reproducible intermediate
outputs; large raw traces and plots used only during tuning are intentionally not kept
in the baseline tree.
