# Stage 4E-R — Minimal fixes only

Overall qualification: **FAIL**.
Stop reason: `payload_remove_failed`.

## Changes

- In `FLAT_NORMAL`, the existing scalar payload-mass RLS now runs in shadow mode whenever its existing commanded-acceleration and regressor excitation gates pass. Residual threshold crossing is not a prerequisite.
- Any `abs(theta_dyn_ref) > 1e-12 rad` clears the slope-entry timer and suppresses slope acquisition. The existing 1.5 deg / 0.20 s entry gate is unchanged.
- LQR, Stage 4C EKF, Q actuation, slip handling, physics, and the three supervisor states are unchanged.

## Qualification

| Scene | Result | Final payload kg | Slope entry segment | Returned flat | Failures |
|---|---:|---:|---|---:|---|
| payload_add | PASS | 0.2924 | - | False | none |
| payload_remove | FAIL | 0.2924 | - | False | payload_mass, no_valid_payload_model_update, no_shadow_flat_acceptance |

Skipped after first failure: identified_payload_flat_acceleration, payload_slope_m8, payload_slope_p8.

## Failure diagnosis

The removal run did execute 2456 `SHADOW_FLAT` RLS updates without entering
`PAYLOAD_ID`. Its terminal scalar candidate was 0.0366 kg (down from the
accepted 0.2924 kg), with covariance 0.00311, information 321.71, and stability
range 0.0228 kg. The unchanged residual-improvement ratio was 1.236, above its
frozen 0.90 validity limit, so the candidate was correctly left in shadow and
the accepted payload model remained 0.2924 kg. No removal-specific threshold
or acceptance path was added.

The Stage 4E calibrated residual thresholds were reused unchanged; no calibration or unrelated scene was rerun.
