# Stage 4B — Proprioceptive Environment Pilot

## Upstream Borrow / Adapt / Reject

- **[BorealTC (MIT)](https://github.com/norlab-ulaval/BorealTC):** read `borealtc.py`, `utils/preprocessing.py`, `utils/models.py`, and README. Borrowed run/episode identity, aligned sequences, lazy `(episode,start,end)` sliding windows, train-only preprocessing, AdamW / ReduceLROnPlateau / early stopping. Adapted its IMU+wheel concept to raw 100 Hz CompanionBot production signals and a 0.8 s Conv1D input. Rejected random-window split, spectrogram Conv2D, LSTM and Mamba.
- **[T_DEEP / Vulpi](https://github.com/Ph0bi0/T_DEEP):** used only as the older proprioceptive terrain-CNN reference identified by BorealTC. Rejected porting its MATLAB/data/network stack.
- **[UMich slip_detection_DOB](https://github.com/UMich-CURLY/slip_detection_DOB):** read README and `slipEstimator_SlipModel`; borrowed wheel/body motion inconsistency as slip semantics. Adapted it to simulator-privileged wheel-surface versus chassis tangential speed GT with a predeclared 0.20 ratio. Rejected ROS, Husky and full DOB/InEKF.
- **[inekf_wheeled](https://github.com/XihangYU630/inekf_wheeled):** borrowed the IMU+encoder slip-observability vocabulary only; rejected the complete InEKF/ROS estimator because Stage3 estimation is frozen.

## Dataset integrity

- Generated 68 episodes: train/val/test = 27/9/32; usable windows = {'train': 261, 'val': 90, 'test': 329}.
- Episode-exclusive split audit: **PASS**. No episode or overlapping window crosses splits.
- Network inputs contain only noisy IMU, wheel/control, estimated state, command and frozen-Q observer features. Alpha/friction/payload/contact/world pose/time/episode id are absent from features and exist only in manifest/labels/audit.
- Normalization sample count 12159 was fit on the train split only. Raw episodes and checkpoints are under git-ignored `generated/`.
- Each train slope has all three friction, payload, terrain and speed levels via independent seeded permutations (27 unique full tuples); audit: **PASS**. Numeric OOD uses unseen continuous alpha values [3, -3, 6, -6, 10, -10, 14, -14]; explicit combination OOD and counterfactual groups are test-only.

## Model results and decision

| Model | Test alpha MAE | p95 | Slip P/R/F1 | Rough P/R/F1 |
|---|---:|---:|---:|---:|
| Statistics + Ridge/Logistic | 1.190° | 3.348° | 0.824/0.737/0.778 | 0.972/0.814/0.886 |
| Tiny raw Conv1D (5043 params) | 2.007° | 3.815° | 0.403/0.658/0.500 | 0.667/0.628/0.647 |

Decision: **REJECT**. The selection rule deliberately prefers the simple model unless Conv1D improves alpha MAE by more than 0.20°. Conv1D checkpoint is 27522 bytes, estimated FP32 parameter memory 20172 bytes, measured batch-one CPU latency 0.179 ms.

Best cheap-baseline test slope: MAE 1.190°, RMSE 2.821°, p95 3.348°, bias +0.654°.

Best cheap-baseline counterfactual alpha drift: friction 0.330°, payload 1.070°. With friction/payload/terrain held fixed, unseen-alpha prediction is strictly monotonic: True. Low-mu no-slip versus bump-induced-slip and unseen terrain parameter outputs are preserved in `model_metrics.json`, including failures; no threshold/controller retuning was done.

## Alpha to equilibrium and closed loop

The analytic quasi-static mapping is `theta_flat + asin(r (m_body + 2 m_wheel) sin(alpha) / (m_body l_com))`. Against the four Stage4A Q-OFF steady mean pitches its MAE is 0.237°.

The declared alpha Gate failed, so the A/B/C/D closed-loop experiment was **not executed**; running C/D would violate the request's Section 14 Stop Rule. No theta-aware Q benefit is claimed.

## Final decisions

- Proprioceptive environment estimator: **REJECT; fallback is dual ToF or RGB-D ground geometry**.
- Q observer: **KEEP** as a diagnostic/physics-feature producer; it remained active in every generated episode.
- Q features: **KEEP for logging/future sensing baselines**, but there is no production learned estimator and no claim that Q features alone caused any result.
- Q actuator: **PRODUCTION_OFF**. Parameters were not retuned; when the alpha Gate fails this retains the prior production-OFF decision rather than inventing a new ablation claim.
- Stage4 lower layer: **freeze the existing controller/Q production baseline and move environment sensing to ToF or RGB-D/camera geometry; do not continue scaling the proprioceptive model**.

Detailed per-episode OOD/counterfactual predictions and the exact ablation execution/skip record are in the JSON artifacts.
