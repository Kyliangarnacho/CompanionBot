# Stage 4D — External Disturbance Closure Report

## Decision

Stage 4D status: **CLOSED WITH DECLARED V1 LIMITS**. Slip decision: **FAIL — PRE-FALL RECALL GATE MISSED; FUTURE INDEPENDENT CAMERA/VISUAL-ODOMETRY VELOCITY REQUIRED**. Payload decision: **PERMANENT PRODUCTION OFF**.

## Required answers

1. **Slip upstream Borrow / Adapt / Reject.** Borrowed the explicit slip-disturbance state, IMU propagation, wheel observation, covariance-normalized residual/NIS, and short persistence principle. Adapted them to a scalar longitudinal `[v_body, d_slip]` observer driven by IMU specific-force projection, with frozen TWIP force balance retained as a diagnostic. Rejected full SE(3) InEKF/ROS, terrain ML/CNN, a unified classifier, and a new robust/traction controller.
2. **Body velocity independence.** Existing `v_hat` is not independent: it is exactly wheel-radius times mean relative wheel PLL rate plus pitch rate. IMU acceleration does not propagate longitudinal velocity, and no velocity covariance exists. Stage 4D therefore adds only the permitted scalar augmentation; GT velocity/friction/contact never enters it.
3. **Pure-scene detector.** Development precision/recall=0.609/0.475. Frozen held-out precision=0.947, recall=0.419, F1=0.581, FPR=0.001, median latency=0.100 s. Gate: **FAIL**.
4. **Safe slowdown.** Not connected: the pre-fall standalone detector recall Gate failed.
5. **Slope EKF HOLD/recovery.** Not run because standalone slip did not pass its Gate.
6. **Q payload benefit.** PERMANENT PRODUCTION OFF; fixed/free A/B values are retained in the metrics artifact.
7. **Only Q correction.** Used=False. Reason: current Q did not show consistent, clear improvement in both payload modes.
8. **Bump/rough baseline.** bump=OUTSIDE ENVELOPE, rough=PASS.
9. **External push baseline.** 12/12 representative force/direction/speed cases recovered to their event-local pre-push baseline without fall or sustained saturation.
10. **CLOSED disturbances.** See closure table below; no extra terrain classes were invented.
11. **OUT OF V1 SCOPE.** Large curbs, holes, and physically non-traversable obstacles belong to perception/planning. Sensor dropout/watchdog/hand faults belong to a hardware fault-handling stage.
12. **Frozen low-level architecture.** Stage 4C TWIP-EKF → analytic `x_eq/u_eq` scheduling → frozen LQR; Stage 4D no production slip path; scalar observer rejected by the pre-fall recall Gate, with independent camera/visual-odometry velocity recorded as the dependency; Q actuator OFF (observer diagnostics only); frozen LQR directly rejects in-envelope bump/rough/push disturbances.

The requested mild held-out scenes did not cross the GT slip threshold, and the requested sustained-recoverable scenes crossed directly into fall. Those outcomes are retained and not relabeled. After strictly truncating evaluation at the 45° fall boundary, recall fails the Gate; post-fall samples are not allowed to inflate it.

## Safe slowdown A/B/C

| Scene | Slip-fraction reduction | Pitch-RMS change | Fall not increased | Saturation not increased |
|---|---:|---:|---|---|
| — | not executed: detector Gate failed | — | — | — |

## Payload Q final A/B

| Payload | Q | Pitch RMS (deg) | Pitch-rate RMS | Velocity RMSE | Position peak | Q mean / peak (N·m) | Saturation | Fall |
|---|---|---:|---:|---:|---:|---:|---:|---|
| fixed | OFF | 3.437 | 0.128 | 0.0214 | 0.0424 | -0.0006 / 0.0166 | 0.0000 | False |
| fixed | ON | 3.427 | 0.127 | 0.0207 | 0.0410 | -0.0006 / 0.0165 | 0.0000 | False |
| free | OFF | 7.109 | 0.896 | 0.1896 | 0.2116 | 0.0041 / 0.0800 | 0.0018 | False |
| free | ON | 7.134 | 0.981 | 0.1935 | 0.1567 | 0.0034 / 0.0960 | 0.0000 | False |

Fixed Q ON changed pitch RMS by -0.29% and velocity RMSE by -3.29%; free Q ON changed them by 0.36% and 2.10%. The alpha-hat ON-minus-OFF changes were 0.003° fixed and 0.046° free, so Q does not repair payload-induced slope-model mismatch.

The conditioned free payload remained contained in both arms. Q ON changed wall-collision episodes from 34 to 37 and maximum wall force from 5.11 N to 5.60 N; this is not a collision benefit.

## Frozen LQR disturbance baselines

| Terrain | Peak pitch (deg) | Peak pitch-rate | Peak velocity deviation | Settle (s) | Saturation | Fall | Decision |
|---|---:|---:|---:|---:|---:|---|---|
| bump | 33.40 | 1.742 | 0.578 | None | 0.0000 | False | OUTSIDE ENVELOPE |
| rough | 11.40 | 0.634 | 0.151 | None | 0.0000 | False | PASS |

All 12 push cases passed. Worst peak pitch was 11.76°, worst actual wheel torque was 0.151 N·m, and worst event-local settling time was 0.58 s.

## Closure table

| Physical issue | Production handling | Status |
|---|---|---|
| Slope | Stage 4C TWIP-EKF + x_eq/u_eq | CLOSED |
| Low friction / slip | Stage 4D scalar observer diagnostic; slowdown rejected | FAIL — PRE-FALL RECALL GATE MISSED; FUTURE INDEPENDENT CAMERA/VISUAL-ODOMETRY VELOCITY REQUIRED |
| Fixed/free payload | frozen LQR; Q diagnostics | CLOSED PERMANENT PRODUCTION OFF |
| Small bump | frozen LQR | OUTSIDE CURRENT V1 ENVELOPE |
| Rough transient | frozen LQR | CLOSED |
| External push / minor impulse | frozen LQR | CLOSED |
| Large curb / hole / non-traversable obstacle | future perception/planning | OUT OF BALANCE-CONTROL SCOPE |
| Sensor dropout / watchdog / hand fault | future hardware fault handling | NOT AN EXTERNAL PHYSICAL DISTURBANCE |

## Frozen data boundary

GT body velocity, slip ratio, friction, terrain angle/contact, payload state, and post-hoc equilibrium are evaluator-only. Logs preserve requested sum torque, clamped sum torque, actual per-wheel torque, saturation, observer rejection/HOLD, and termination reasons.

## Artifacts

- Metrics: `models/minisegway/stage4/results/stage4d/stage4d_external_disturbance_metrics.json`
- Upstream audit: `models/minisegway/stage4/results/stage4d/STAGE4D_UPSTREAM_NOTES.md`
