# Stage 4E-R — Slope qualification resume

The failed payload-removal acceptance was explicitly waived only to continue the remaining slope qualification. Its failure remains recorded in the original Stage 4E-R artifacts.

Resume result: **PASS**.
Stop reason: `none`.

| Scene | Result | Entered slope | Entry segment | Returned flat | Slope MAE deg | Failures |
|---|---:|---:|---|---:|---:|---|
| identified_payload_flat_acceleration | PASS | False | - | False | - | none |
| payload_slope_m8 | PASS | True | transition_up | True | 1.096 | none |
| payload_slope_p8 | PASS | True | transition_up | True | 0.990 | none |

Skipped after first resumed failure: none.

Legacy Stage 4E slope-MAE gate: 1.0 deg; exceeding scenes: payload_slope_m8. This diagnostic gate is reported separately from the requested entry/return qualification.

Slope entry uses 3.0 deg for 0.20 s; slope exit uses abs(e_theta) < 3.0 deg, abs(alpha_hat) < 0.7 deg for 0.75 s. No controller, estimator, physics, state, Q, or slip setting was changed.
