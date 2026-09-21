# Stage 4E upstream / Borrow–Adapt–Reject notes

## Technical Radar

The standard field term is a **guarded hybrid supervisor with hysteresis and
dwell/persistence time**, combined with **one-parameter recursive least
squares** for a known physical regressor. Hybrid-system references justify
explicit guards and state-dependent continuous dynamics; standard online RLS
uses `y = phi*theta + e` and a covariance update.

## Borrow / Adapt / Reject

- **Borrow:** the conventional scalar RLS update and covariance/information
  semantics documented by MathWorks; the cart-pole/TWIP Lagrangian mass terms
  in MIT Underactuated Robotics; explicit guarded switching from hybrid-system
  practice.
- **Adapt:** Stage 4C's frozen scalar EKF and force balance, existing Stage 3
  sensorized state, frozen Q diagnostics, and the repository's rigid-body mass
  property helpers. The only estimated parameter is fixed payload mass.
- **Reject:** HMM/classifiers, a learned state selector, full online A/B
  identification, Q actuation, slip-state coupling, and free-payload dynamics.

## Complexity Gate

Passed. The requested behavior needs only three states and one scalar physical
parameter. Any additional state, estimator, or learned arbitration layer is
out of scope until this baseline has a quantified failure.

References:

- https://www.mathworks.com/help/ident/ug/algorithms-for-online-estimation.html
- https://underactuated.mit.edu/acrobot.html
- https://underactuated.mit.edu/sysid.html
- https://ieeexplore.ieee.org/document/4806347
