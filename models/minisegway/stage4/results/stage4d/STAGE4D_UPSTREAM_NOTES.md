# Stage 4D Slip Upstream Notes

## Technical radar / Borrow–Adapt–Reject

- **Standard term:** wheel-slip-aware proprioceptive state estimation using a disturbance/slip-velocity state and covariance-normalized innovation (NIS).
- **Borrow:** UMich CURLY's explicit slip velocity, wheel observation, covariance-aware residual/chi statistic, covariance adaptation idea, and startup bias discipline; the InEKF wheeled stack's IMU propagation plus wheel update separation; the TWIP literature's warning that wheel slip corrupts wheel-derived body state.
- **Adapt:** CompanionBot uses one longitudinal `[v_body, d_slip]` augmentation. IMU specific force independently propagates `v_body`, the frozen TWIP force balance remains a diagnostic, and encoder odometry supplies `v_wheel = v_body + d_slip + noise`; `chi=e²/S` plus one enter/exit persistence latch creates `slip_active`.
- **Reject:** full SE(3)/Lie-group InEKF, ROS messages/build system, 3-D localization, learned terrain/slip classifiers, and a new robust/traction controller.
- **Complexity gate:** this scalar observer must pass pure-slip recall/precision/FPR/latency before slowdown is connected. Failure means stop and record the need for an independent velocity source (camera/visual odometry), not add a CNN.

## Source audit

- `UMich-CURLY/slip_detection_DOB` README and `src/system/husky_system.cpp` (master): wheel velocity is `(right+left)/2 * radius`; the estimator carries disturbance/slip velocity, initializes IMU bias from 250 stationary samples, normalizes slip residuals by covariance, and adapts wheel covariance with disturbance magnitude. The source's active `slipEstimator` also contains a fixed-covariance chi test (`4.642`), while `slipEstimator_SlipModel` evaluates measured wheel velocity against body plus disturbance prediction.
- Yu et al., *Fully Proprioceptive Slip-Velocity-Aware State Estimation for Mobile Robots via Invariant Kalman Filtering and Disturbance Observer*, arXiv:2209.15140 / IROS 2023: conceptual basis only.
- `XihangYU630/inekf_wheeled`: secondary confirmation of IMU propagation, wheel velocity update, and covariance handling; no code vendored.
- Parravicini, Corno, Savaresi, *Robust State Observers for Two Wheeled Inverted Pendulum under wheel-slip*, ITSC 2019: TWIP-specific sanity reference; no reproduction attempted.
