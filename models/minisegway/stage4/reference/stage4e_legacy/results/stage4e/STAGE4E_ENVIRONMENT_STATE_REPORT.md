# Stage 4E — V1 Fixed-Payload / Slope State Arbitration

Overall qualification: **FAIL**.
Stop rule: **payload_slope_not_distinguishable**.

## Implementation summary

A three-state guarded supervisor was added in `control/environment_state_manager.py`. It reuses the frozen Stage 4C EKF and analytic equilibrium, keeps Q diagnostic-only, does not accept slip state, and identifies only fixed payload mass at the known mount.

## Transition table

| From | To | Guard | Action |
|---|---|---|---|
| FLAT_NORMAL | PAYLOAD_ID | quasi-static + calibrated residual > threshold for 0.50 s | alpha_control=0, invalidate payload model, enable scalar RLS only on natural acceleration |
| FLAT_NORMAL | SLOPE_TRACK | reference-consistent pitch error >1.5° for 0.20 s, abs(v)>0.08 m/s; timer paused for dynamic lean >0.5° | freeze RLS; hold trusted equilibrium until EKF std<1° |
| PAYLOAD_ID | FLAT_NORMAL | excitation, covariance, stability and residual-decline gates all pass | atomically accept mass and recompute mass/CoM/inertia/Stage4C plant |
| SLOPE_TRACK | FLAT_NORMAL | abs(alpha_hat)<0.7° and abs(e_theta)<0.8° for 0.75 s | alpha_control=0 |

## Scalar regression

On flat ground, with `H0=m_b*l_b`, known mount radius `l_p`, and `F=(u-loss)/r`: `y = F - [M0*a - H0*w² sin(beta) + H0*theta_ddot cos(beta)] = phi*m_payload`, where `phi = a - l_p*w² sin(beta) + l_p*theta_ddot cos(beta)`. The mount is on the nominal pitch ray, so both equivalent translation mass and first moment are affine in the single unknown mass.

## Threshold source

The payload residual threshold is **0.012034**, derived as flat-empty normal-start/stop P99.9 **0.010940** plus a 10% frozen margin. The payload-aware force-residual removal threshold is likewise P99.9+margin: **0.284764** from P99.9 **0.258876**. Neither was selected from payload truth.

## Qualification metrics

| Scene | Result | Transitions | Mass error kg | Slope MAE deg | Notes |
|---|---:|---:|---:|---:|---|
| flat_empty | PASS | 0 | - | - | none |
| payload_add | PASS | 2 | 0.0169 | - | none |
| payload_remove | FAIL | 0 | 0.2669 | - | payload_arbitration, payload_mass |
| payload_slope_m08 | FAIL | 1 | 0.0169 | - | slope_transition |

## Limits and failures

The fixed load is deliberately placed on the nominal pitch ray. At rest it does not change the flat equilibrium and is unobservable from IMU/encoders alone; the first natural start/stop supplies model-residual evidence, and a later normal acceleration supplies RLS excitation. No probe is injected. Payload changes on a slope are logged unsupported and never trigger online identification.
Observed stop condition: `payload_slope_not_distinguishable`. Fixed-payload acceleration produced a persistent >1.5° reference-consistent pitch error on flat ground before the grade. A development full-route audit also failed to sustain the strict flat-return guard for 0.75 s. Payload removal remained below the independently calibrated force-residual threshold. These failures are retained; no threshold or controller was changed to hide them.
Skipped after the stop rule: payload_slope_p08, small_slope_m01, small_slope_p01.

Artifacts: `stage4e_environment_state_metrics.json`, `stage4e_history.json`, and `STAGE4E_UPSTREAM_NOTES.md` in this directory.
