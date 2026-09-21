# Stage4 final baseline closeout

## 1. 删除/保留了什么

删除 PAYLOAD_ID environment state、continuous/shadow RLS、candidate/accepted 双层模型及多重 acceptance gates。保留两态 FLAT/SLOPE、Q one-step innovation 的 matched projection/filtered estimate、短时自然 transient batch ID。

## 2. 最终 runtime 流程

Q delta 经 EWMA+persistence+hysteresis 只置 pending；下一次自然 transient 置 active；50 Hz 收集一段数据并一次 least-squares；只做 rank/condition 与物理合法性检查，合格即更新 plant 并 freeze；首次 FLAT 非 transient 刷新 q_baseline 后 re-arm。

## 3. payload ID update count 和结果

Sessions: 1; updates: 31; accepted: True; reason: accepted. Estimated mass 0.3476 kg versus post-hoc truth 0.2500 kg; estimated forward position 0.01518 m versus post-hoc truth 0.0150 m. The mass bias is retained as a baseline limitation; no extra acceptance layer or tuning was added.

## 4. 左/右偏载结果

| Case | Offset m | Fall | Saturation | Max pitch deg | Peak yaw deg | Yaw drift deg/s | Velocity RMS m/s | Left/Right torque peak N m |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| centered | +0.000 | False | 0.0000 | 7.156 | 0.147 | 0.0105 | 0.0496 | 0.068 / 0.070 |
| left | -0.010 | False | 0.0000 | 7.324 | 0.718 | 0.0627 | 0.0485 | 0.168 / 0.168 |
| right | +0.010 | False | 0.0000 | 7.234 | 0.194 | 0.0187 | 0.0476 | 0.044 / 0.041 |

## 5. 左/右单轮冲激结果

Recovery is measured relative to the 0.5 s pre-impulse steady baseline; absolute peak pitch is retained separately.

| Wheel | Fall | Saturation | Peak pitch abs/dev deg | Peak yaw dev deg | Peak velocity dev m/s | Settling s |
|---|---:|---:|---:|---:|---:|---:|
| left | False | 0.0000 | 0.950 / 1.344 | 0.692 | 0.0533 | 0.062 |
| right | False | 0.0000 | 0.955 / 1.274 | 0.710 | 0.0487 | 0.002 |

## 6. 最终 frozen baseline

500 Hz main control; 50 Hz short-session payload ID; 100 Hz Stage4C EKF in SLOPE only; Q actuator OFF; slip OFF; LQR/FF/yaw/allocator/torque limits/friction frozen.
