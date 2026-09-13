# Longitudinal TWIP definition

CAD axes are `+X` right/wheel-axis, `+Y` rear and `+Z` up. Forward motion is
therefore CAD `-Y`. Positive pitch is right-hand rotation about `+X`, which is
nose-down.

The reduced state and input are

`x = [p, p_dot, theta - theta_eq, theta_dot]`

`u = tau_left + tau_right`.

Here `p` is axle-midpoint forward displacement and positive wheel torque is
about `+X`. Under the pure-rolling constraint, actuator virtual work gives
generalized force `[u/r, -u]`: positive `u` accelerates the axle forward and
reacts on the chassis nose-up.

The small-angle continuous model is generated in `reduced_twip.json` from
`plant_report.json`. It assumes straight symmetric motion, rigid bodies and
no slip. Contact compliance, motor electrical dynamics, backlash, caster,
drag and differential/yaw motion are deliberately excluded.
