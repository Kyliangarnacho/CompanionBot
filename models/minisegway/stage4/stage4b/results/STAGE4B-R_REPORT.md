# Stage 4B-R — Final Proprioceptive Estimator Qualification

## Scope and implementation correction

This is a patch to Stage4B, not a new dataset/trainer stack. It reuses the same parameterized runner, NPZ episode format, episode-exclusive split, lazy sliding-window dataset, train-only normalization, statistics extractor, Ridge baseline, Tiny Conv1D, evaluator and counterfactual definitions. The previously audited [BorealTC](https://github.com/norlab-ulaval/BorealTC), [T_DEEP](https://github.com/Ph0bi0/T_DEEP), [UMich slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB) and [inekf_wheeled](https://github.com/XihangYU630/inekf_wheeled) decisions remain unchanged; no new literature search or vendoring was performed.

The audit found one material Stage4B physics bug: terrain friction changed while both wheel collision geoms remained at 1.0. Stage4B-R uses `wheel_and_terrain_v2`, setting the same friction on terrain and wheel collisions. Revision episodes use new IDs, so historical Stage4B raw data and reported baselines remain untouched. This correction makes low-friction failures real and means the old and revised absolute scores are informative engineering comparisons, not a pure window-length ablation on identical physics.

## Dataset and label integrity

- Generated 157 independent revision episodes: 108 train, 9 val, 40 test. Nominal train coverage is exactly 12 episodes per known slope; usable train episodes after retaining physical failures are `{'0': 11, '4': 12, '-4': 10, '8': 11, '-8': 11, '12': 9, '-12': 12, '15': 7, '-15': 11}`.
- Status counts: `{'insufficient_constant_grade': 19, 'usable': 138}`; falls retained: 85. No failure was silently discarded from the manifest.
- Long/short windows: {'train': 1131, 'val': 72, 'test': 456} / {'train': 1896, 'val': 134, 'test': 777}.
- Split-disjoint audit: **PASS**. Numeric OOD slopes remain absent from train: **PASS**.
- GT is per-timestamp local terrain grade: flat samples are 0°, transition/grade samples use their actual local geom angle. The 2.4 s target is the median local grade over the final 0.2 s; earlier window context may contain flat/entry samples. GT/world pose/time/episode identity are not features. Normalization is train-only.

## Final model comparison

| Model | Physics/data | Online alpha MAE | p95 | max | >5° windows | Decision |
|---|---|---:|---:|---:|---:|---|
| OLD 0.8 s Statistics + Ridge | historical Stage4B | 1.190° | 3.348° | not recorded | not recorded | historical reject |
| OLD raw Tiny Conv1D | historical Stage4B | 2.007° | 3.815° | not recorded | not recorded | reject |
| NEW 2.4 s Statistics + Ridge | expanded, corrected friction | 2.615° | 8.187° | 21.954° | 67 | fail |
| NEW Ridge + bounded residual | expanded, corrected friction | 2.579° | 7.284° | 24.744° | 53 | fail |

Episode aggregation is diagnostic only, not the Gate: Ridge MAE/p95 2.136°/6.238°; hybrid 1.933°/5.625°. It also fails.

The 2.4 s Ridge did **not** improve qualification. Every fixed Gate check failed: overall, numeric OOD, combination OOD, coupled-disturbance OOD, friction drift and payload drift. Ridge friction/payload drift is 4.006°/3.433°; hybrid is 3.502°/2.926°. Fixed-nuisance alpha remains monotonic, but monotonicity alone is insufficient.

CF4's two original same-low-friction arms are retained as `[{'episode_id': 'rbase_ep_054_test_p06p0', 'status': 'insufficient_constant_grade', 'fell': True}, {'episode_id': 'rbase_ep_055_test_p06p0', 'status': 'insufficient_constant_grade', 'fell': True}]`: corrected friction made both fall before a qualifying constant-grade window, so no fabricated steady slip/no-slip comparison is reported. On CF5 unseen rough/bump geometry, episode-level absolute error reaches 6.171° for Ridge and 5.403° for hybrid.

## Residual behavior and nuisance coupling

The event encoder was frozen and the 6,241-parameter MLP produced a fixed ±3° tanh-bounded correction; total learned parameters are 11,284, with no joint fine-tune. It reduced overall p95 and the number of >5° windows, but only marginally changed MAE and increased maximum error from 21.95° to 24.74°. It helps some slip/rough episodes and harms some payload combinations; it does not disentangle nuisances.

| CF6 intended class | Slip actually realized | Rough realized | Ridge MAE / p95 | Hybrid MAE / p95 |
|---|:---:|:---:|---:|---:|
| clean | False | False | 1.727° / 5.047° | 1.397° / 5.300° |
| payload_only | False | False | 2.021° / 4.288° | 2.468° / 3.626° |
| slip_only | False | False | 6.535° / 17.442° | 6.991° / 19.963° |
| rough_only | True | True | 2.227° / 4.528° | 1.795° / 4.529° |
| payload_slip | False | False | 6.907° / 17.808° | 7.322° / 20.437° |
| payload_rough | True | True | 1.593° / 3.831° | 2.799° / 3.702° |
| slip_rough | True | True | 5.393° / 8.887° | 4.270° / 7.892° |
| payload_slip_rough | True | True | 9.249° / 18.809° | 9.171° / 20.986° |

The intended smooth `slip_only` arm did not cross the physical 0.20 slip threshold at a stable constant grade; lowering friction further caused fall/no constant-grade window. Rough contact itself produced slip in the rough arms. This is reported as an observability/reachability limitation, not relabeled. Coupling failure becomes severe once low-friction reversal is combined with payload/rough; the all-nuisance hybrid MAE remains 9.171°.

Catastrophic tails remain: hybrid has 210 / 137 / 53 windows above 2° / 3° / 5°, with 24.74° maximum error. Therefore current proprioception is not sufficient for production slope sensing.

## Slip and rough heads

| Method | Slip precision / recall / F1 | Rough precision / recall / F1 |
|---|---:|---:|
| OLD cheap baseline | 0.824/0.737/0.778 | 0.972/0.814/0.886 |
| NEW short event CNN | 0.867/0.689/0.768 | 0.303/0.816/0.441 |

Slip recall 0.689 misses the unchanged 0.90 Gate: **DEFER/REJECT production slip detector**. The old cheap rough result remains better than the new CNN; keep its result/code only, with no controller connection absent a product need.

## Engineering answers and terminal decision

1. The old CNN failure was **not mainly data scarcity**. Independent train episodes increased from 27 to 108, yet corrected-friction OOD and nuisance coupling remain poor.
2. Long context did **not** make Ridge production-qualified; its online tail is substantially outside Gate.
3. The bounded residual mostly shifts predictions under slip/rough context but cannot remove payload/friction ambiguity and sometimes worsens the tail.
4. Clean and single payload/rough cases are less bad, but already fail the 1°/2.5° Gate; stable pure-slip could not be physically isolated.
5. Two/three-nuisance coupling, especially low friction with payload/rough, is the dominant collapse mode.
6. Yes, 5°-class catastrophic errors remain frequent.
7. No, current production-available proprioception has not demonstrated sufficient slope observability.
8. Final slope decision: **PERMANENT REJECT**. No more proprioceptive ML expansion; move slope sensing to RGB-D or dual-ToF geometry.
9. Q observer: **KEEP** for diagnostics/logging/possible future feature studies. Q actuator: **PRODUCTION OFF**; no cutoff/authority retuning and no new theta-aware benefit claim.
10. Freeze the existing Stage4 controller/Q production baseline and proceed to the visual/geometric sensing phase.

## Downstream gate

The analytic alpha→theta_eq mapping remains valid at 0.237° Stage4A steady-pitch MAE, but estimator qualification failed. Per the declared rule, A/B/C/D closed-loop execution is **skipped**, C/D are not run, and no ML theta_eq model is introduced.
