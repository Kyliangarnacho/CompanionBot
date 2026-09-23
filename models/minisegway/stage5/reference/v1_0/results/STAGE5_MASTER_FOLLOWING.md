# Stage 5 V1 rolling Master-following

- OTG: Ruckig 0.19.4 velocity interface; each changed 20 Hz intent replans from the currently executing p/v/a.
- TWIP preview: existing identified-A/B joint projection, with inherited start theta/theta-dot and zero terminal lean manifold.
- Stage 5-only recovery-tail search cap: 1.00 s (maximum used 0.95 s); the frozen 0.020 residual threshold is unchanged.
- Planning wall time mean/p95/max: 169.0/633.8/777.7 ms on this host.
- v_ref-v_cmd RMS/peak: 0.0087/0.0268 m/s; v_GT-v_ref RMS/peak: 0.0404/0.0906 m/s.
- Replan seams: boundary theta delta 0.000000 deg; adjacent a step 0.002000 m/s^2; applied FF/total torque step 0.00969/0.14266 N m.
- Fade start/finish/interrupted: 58/0/57.
- Yaw-rate GT-command RMS: 0.0032 rad/s; allocator differential clipping 0.0000%.
- Fall: False; wheel saturation: 0.0000%; final allocator guard clipping: 0.0000%.
- Frozen Stage 4 controller/estimator/yaw/allocator/slope/payload/Q settings were not changed; GT remains inside the test-only observation provider and post-hoc evaluator.

## Plots

- `models/minisegway/stage5/results/plots/stage5_velocity_response.png`
- `models/minisegway/stage5/results/plots/stage5_acceleration_pitch_response.png`
- `models/minisegway/stage5/results/plots/stage5_feedforward_torque_response.png`
- `models/minisegway/stage5/results/plots/stage5_yaw_response.png`
- `models/minisegway/stage5/results/plots/stage5_replan_seam_zoom.png`
