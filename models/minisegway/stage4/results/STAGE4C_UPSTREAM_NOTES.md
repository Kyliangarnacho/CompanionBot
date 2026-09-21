# Stage 4C upstream notes

Primary upstream: Filippo Parravicini, Matteo Corno, and Sergio Savaresi,
“EKF based slope angle estimation on TWIP” ([paper](https://i-rim.it/wp-content/uploads/2020/12/I-RIM_2020_paper_94.pdf),
[record](https://zenodo.org/records/4781480)).

## What is borrowed

- The upstream robot is a two-wheeled inverted pendulum (TWIP), specifically the
  YAPE parcel-delivery platform.
- Its production-available sensors are a chassis-fixed 6-axis IMU mounted at the
  axle centre and one relative rotary encoder per wheel.
- The slope observer consumes the wheel torques `u = [tau_r, tau_l]`, an already
  estimated TWIP dynamic state, and sensor measurements.
- The architecture is deliberately decoupled: an existing observer first
  estimates the vehicle dynamic state, then a separate nonlinear observer
  estimates road slope. Stage 4C therefore keeps CompanionBot's production
  longitudinal estimator and does not put all vehicle states and slope into a
  replacement EKF.
- The upstream nonlinear model explicitly contains road slope, wheel-hub
  friction, and rolling resistance. The paper warns that dissipative forces and
  slope have matched effects in the longitudinal dynamics, so an unmodelled
  loss becomes slope bias.

## CompanionBot adaptation

CompanionBot uses world forward `-Y`; positive wheel rotation and positive
summed wheel torque about `+X` drive it forward; positive pitch is right-hand
rotation about `+X` (nose down); positive road slope rises in the forward
direction. `theta_hat` is absolute/world chassis pitch. Only
`LongitudinalEstimate.plant_state(theta_eq)` subtracts an equilibrium pitch.
The flat equilibrium is `theta_flat = -2.3704449 deg`.

The paper's scalar symbols are not copied blindly. Stage 4C maps its
longitudinal force balance into CompanionBot's signs and parameters. The
already-validated quasi-static pitch relation remains

`theta_eq(alpha) = theta_flat + asin(r M sin(alpha)/(m_b l))`.

For constant speed, the full equilibrium input is obtained from the nonlinear
longitudinal balance, not from a pitch-reference substitution alone:

`u_eq = r M g sin(alpha) + tau_hinge(v) + tau_roll(v)`.

Here `tau_hinge` uses the two MuJoCo wheel-joint parameters already in the
plant (`damping=0.002 N m s/rad`, `frictionloss=0.002 N m` per wheel). Contact
rolling loss is not invented or added to the simulator; any unresolved
effective residual may be identified once from flat development runs only.
The slope EKF consumes the actual limited/applied left and right actuator
torques. Ground-truth slope remains evaluation-only.

## Borrow / Adapt / Reject and complexity gate

- **Borrow:** separate slope observer, nonlinear force balance, covariance,
  innovation, normalized innovation, validity, and hold/convergence status.
- **Adapt:** coordinates, absolute-pitch semantics, CompanionBot masses and
  inertias, known MuJoCo hub loss, actual applied torque, and existing state
  estimator outputs.
- **Reject:** replacing the production state estimator, adding YAPE-specific
  friction to the plant, online friction identification, ML slope regression,
  Q-actuator assistance, or controller retuning.
- **Complexity gate:** verify signs at `0/+8/-8 deg`, then verify flat versus
  oracle `x_eq/u_eq` control at `+/-8` and `+/-15 deg`. If oracle control fails,
  estimator/control integration stops. Static inverse is evaluated before the
  EKF and is preferred if it clears every production gate.
