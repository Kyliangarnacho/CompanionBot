# Stage 4C — Physics-Based Slope Estimation

## Decision

**KEEP TWIP-EKF**

This stage remained slope-only: smooth terrain, nominal `mu=1.0`, zero yaw,
empty payload for qualification, no push/rough/bump/induced slip, and Q actuator
OFF. Frozen LQR gains, longitudinal estimator, feedforward, allocator, yaw
controller, and Stage 4B-R artifacts were not retuned.

## Required questions

1. **Morphology match.** CompanionBot and the Parravicini et al. YAPE platform
   are both TWIPs with a chassis IMU, two wheel encoders, and two driven wheels.
   CompanionBot differs in mass/inertia geometry, sign conventions, virtual IMU
   timing, MuJoCo contact, and directly known actuator torque.
2. **Borrow versus adapt.** The decoupled slope-observer architecture,
   nonlinear longitudinal balance, friction awareness, EKF covariance and
   innovation are borrowed. Coordinates, absolute-pitch semantics, parameters,
   actual applied-torque path, and a gated steady pitch measurement are adapted.
3. **Friction.** Both wheel hinges already contain `0.002` damping and `0.002`
   frictionloss per wheel; these known terms are included. MuJoCo's existing
   contact rolling coefficient is left unchanged, with no invented plant loss
   and no test-slope friction fitting. Corrected `wheel_and_terrain_v2` sets
   both wheel and terrain friction to 1.0. Oracle equilibrium residuals were
   already within the control Gate, so no extra flat-data rolling-loss fit was
   justified.
4. **Static inverse.** Held-out MAE is 0.832°, p95
   2.066°, steady bias
   0.057°.
5. **EKF increment.** Held-out MAE is 0.206°, p95
   0.400°. Its force-balance update uses
   actual applied torque during transients. Static inverse failed the hard
   convergence check (44/
   48 valid episodes converged;
   maximum 7.48 s), whereas EKF converged
   in all 48 valid episodes with
   a 0.08 s maximum.
6. **Clean-slope Gate.** Static passed=False; EKF
   passed=True for the 1° MAE / 2.5° p95 / 0.5° steady-bias /
   5 s hard-convergence gates.
7. **Speed sensitivity.** Per-speed results are stored under
   `estimation.test.*.by_speed_m_s`; all 0.2/0.4/0.6 m/s groups are retained.
   EKF MAE is 0.198° / 0.260° / 0.171° respectively.
8. **Initialization.** The online observer ran parallel 0°, +5°, and -5° EKFs;
   sensitivity metrics are in `initialization_sensitivity`.
9. **Fixed payload.** It significantly pollutes the empty-model observer: MAE 1.888°, p95 2.564°; this fails and means payload-aware model parameters are required. No payload adaptation is added here.
10. **Oracle control.** Oracle `x_eq/u_eq` sanity passed=True.
11. **Estimated closed loop.** All A/B/C runs had no fall or saturation. Estimated-alpha velocity RMSE is 0.0031–0.0054 m/s and differs from oracle by at most 0.0012 m/s.
12. **Final selection.** **KEEP TWIP-EKF**.

## Error-source separation

- Model/sign error is isolated by the exact 0/+8/-8 round-trip tests and the
  GT-state replay diagnostic (`gt_state_diagnostic`). Historical Stage 4A
  Q-OFF logs were also replayed before generating the new qualification matrix.
- Production state-estimator error is the difference between GT-state replay
  and the production-state online result.
- Transition/insufficient excitation is reported separately through convergence
  time and update/hold status.
- Steady model mismatch is the post-entry 3 s bias, where hub/contact losses can
  no longer be hidden as transient error.

## Tuning and physical validity

The nominal development tune used a 0.35° steady-pitch measurement standard
deviation. The single permitted correction increased it to 5.0° after
development runs exposed seed-dependent production pitch bias; held-out slopes
were rerun only after that freeze. No grid, random, or Bayesian search was used.

The development/held-out sets contain 1/0 invalid slope-only conditions. Invalid
conditions are retained in `per_episode` with slip, contact, saturation, fall,
and boundary fields, but excluded from the slope Gate exactly as declared.

## Artifacts

- Metrics: `models/minisegway/stage4/results/stage4c_slope_metrics.json`
- Upstream notes: `models/minisegway/stage4/results/STAGE4C_UPSTREAM_NOTES.md`
- Plots: models/minisegway/stage4/results/plots/stage4c/alpha_trace.svg, models/minisegway/stage4/results/plots/stage4c/alpha_error_trace.svg, models/minisegway/stage4/results/plots/stage4c/summary_by_slope.svg, models/minisegway/stage4/results/plots/stage4c/summary_by_speed.svg, models/minisegway/stage4/results/plots/stage4c/closed_loop_comparison.svg
- Representative 50 Hz innovation/confidence history:
  `models/minisegway/stage4/results/stage4c_representative_history.json`
