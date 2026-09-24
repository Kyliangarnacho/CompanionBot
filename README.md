# CompanionBot

CompanionBot 是一个双轮自平衡机器人 MuJoCo 研究项目。当前冻结 baseline 已经不只是“原地站住”：
它能够完成速度/转向控制、上下坡、固定偏载运行，并从小型外力与单轮冲激中自行恢复。

> 当前结论仅适用于仿真研究，不代表实机安全认证、额定载荷或最终硬件性能。

## 当前抗扰动能力

| 场景 | 冻结 baseline 结果 | 少量关键数据 |
|---|---|---|
| 平滑恒坡 | **±8° 通过**，无摔倒、无轮端饱和 | +8° / −8° 稳态速度 RMSE：0.0148 / 0.0469 m/s |
| 外力推扰 | **12/12 全部恢复** | 0.5 / 1.0 / 1.5 N × 0.10 s，静止/运动、双方向；最坏恢复 0.58 s，最大 pitch 11.76° |
| 固定载荷偏置 | **0.25 kg、左右 ±10 mm 均稳定** | 无摔倒/饱和；最坏速度 RMS 0.0496 m/s，peak yaw 0.718° |
| 单轮纵向冲激 | **左右轮均恢复** | 1 N × 0.10 s；最大 pitch 增量 1.344°，peak yaw 0.710°，最坏恢复 0.062 s |
| 篮内自由载荷 | **机械整形后保持 contained** | 0.10 kg；无摔倒、无饱和、无大幅碰撞相关速度跌落 |

这些结果使用同一套冻结 LQR、feedforward、yaw controller、allocator、torque limit 和 friction
baseline；没有为了某个 smoke case 单独调控制器。

详细结果：

- [Stage 4 最终 baseline closeout](models/minisegway/stage4/results/final_baseline/STAGE4_FINAL_BASELINE.md)
- [坡度 robustness](models/minisegway/stage4/results/STAGE4A_REPORT.md)
- [外部扰动与 push 结果](models/minisegway/stage4/results/stage4d/STAGE4D_EXTERNAL_DISTURBANCE_REPORT.md)

## 已知边界

- **±15° 不属于已验收坡度范围**：压力测试没有摔倒或饱和，但 pitch/velocity 指标未过 Gate。
- 现有 modest bump 会使车辆停在障碍前；rough surface 可通过，但不能据此声称具备普遍越障能力。
- Slip observer 未达到 recall Gate，因此 slip detector/control 没有进入最终 runtime。
- Fixed-payload ID 的前后位置估计准确，但本次 0.25 kg 真值被估为 0.3476 kg；该质量偏差被明确保留。
- Q observer 仅用于 model-change diagnostic；Q actuator 永久 OFF。

## 当前冻结 runtime

- MuJoCo physics：1 kHz；main controller：500 Hz。
- Longitudinal：fixed nominal ID-LQR + transient-only dynamic lean/feedforward。
- Yaw：relative-heading / yaw-rate PD，allocator 优先保护 longitudinal common-mode torque。
- Payload：Q change trigger → 下一次自然 transient → 50 Hz 短时 sagittal ID → 参数冻结。
- Slope：两态 `FLAT/SLOPE` supervisor；Stage4C physics EKF 仅在 `SLOPE` 以 100 Hz 运行。
- Production：Q actuator OFF，slip detector/control OFF。
- Controller/estimator 不读取 terrain、payload 或姿态 GT；GT 只用于 post-hoc 验收。

冻结配置：

- [Stage 3 baseline manifest](models/minisegway/stage3/results/config/baseline.json)
- [Stage 4 final baseline config](models/minisegway/stage4/config/stage4_final_baseline_config.json)

Stage 5 V1.5 已在该 runtime 上接入一条可复用的上层速度命令链：20 Hz observation 经 candidate scheduler、LIGHTWEIGHT/FULL planner 后形成 50 ms reference blocks，再由现有 controller 消费。它不改变 Stage 3/4 冻结控制器或估计器。

V1.5 的 synthetic replay 与 stream check 给出了当前阶段的量化边界：端到端 warm planning 为 3.94 ms（V1.4 为 108.92 ms，约 27.6× 加快），benchmark warm 为 3.75 ms；reference block underrun/stale/gap 均为 0。250 Hz cached-KKT replay 的 500 Hz residual 为 0.01776，velocity error RMS/peak 为 0.05540/0.16734 m/s，无饱和、无摔倒。规划使用 25×2 ms block，FULL horizon 先按 50 ms bucket，再向 4 ms 网格上取整。耗时是开发机仿真结果，不是 Raspberry Pi 5 实测或实机认证。详见 [Stage 5 V1.5 summary](models/minisegway/stage5/results/STAGE5_V1_5_SUMMARY.md)。

## Stage 6 — 二维目标跟随与 Pi/MCU 数据链

Stage 6 在冻结 Stage 3/4 controller 与 Stage 5 planner/stream 上加入相对目标跟随。当前主链为 20 Hz `TargetObservationPacket(x_forward,y_left,capture_time)` → Pi 状态历史按 capture time 插值 → radial-velocity KF → 双窗口 Follow Governor → FULL longitudinal reference blocks 与独立 yaw command → MCU safety merge → MuJoCo。GT 只用于仿真显示和事后评估。

Governor 在 CATCH_UP→CRUISE 时通常锁存最近 20 帧有效 KF 径向速度 median；若最近 5 帧 median 比 20 帧低超过既有速度 deadband，则采用 5 帧结果快速响应减速。Vx/Vy 各 0.40 m/s 的 60 s 稳态对照中，最终方案的 20–60 s Governor 事件 / FULL 计划均为 **0/0**（旧单帧为 9/9、10/10；仅 5 帧方案为 11/11、12/12）。距离标准差由 Vx 的 0.110 m、Vy 的 0.109 m 降至 0.029 m、0.014 m；两组都无 fall、轮饱和或 reference underrun。带噪转弯 KF 径向速度 RMSE 为 **0.056 m/s**，raw finite-difference shadow 为 **0.830 m/s**；相应 Governor 事件为 7 对 46。无噪声回归中 KF 速度响应较慢（RMSE 0.029 vs raw 0.015 m/s），这一代价和减速瞬态风险均保留。

最近的直线变速检查显示，0.20→0.40 m/s 时 CATCH_UP 延续约 6.1–6.2 s 后才锁存约 0.40；0.40→0.20 m/s 时，20 帧历史在即时 transition 仍约 0.40，之后靠 Governor slowdown 更新在 0.85–3.65 s 收敛。没有观察到周期性 hunting，但一组减速曾让距离降到 0.759 m，因此双窗口不能视作瞬态距离安全保证。详见 [Stage 6 当前状态](CURRENT_STATE.md)、[Stage 6 学习日志](LEARNING_LOG.md)、[双窗口历史对照](models/minisegway/stage6/reference/viewer_cruise_median_diagnosis/REPORT.md) 和 [动态变速验证](models/minisegway/stage6/results/governor_dynamic_rate_validation/DYNAMIC_RATE_VALIDATION_REPORT.md)。

## 快速运行

在项目根目录使用现有 Python 3.11 虚拟环境：

```powershell
# 最终 payload lifecycle + 偏载/单轮冲激 smoke tests
.\.venv\Scripts\python.exe scripts\run_stage4_final_closeout.py

# 当前测试集
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

# 无 payload 的交互式速度/转向 demo
.\.venv\Scripts\python.exe scripts\view_stage3_baseline_demo.py --duration 30
```

当前 Stage 4 frozen slope-compensation viewer 依次展示 +8°、−8°、+15°、−15° 坡面（Q actuator OFF）：

```powershell
.\.venv\Scripts\python.exe scripts\view_stage4_slope_compensation_demo.py
```

Stage 5 V1.5 提供完整 baseline 下的手动命令试玩。运行后拖动控制窗口中的 raw `v_cmd` slider（−0.6 至 +0.6 m/s），或点 `v_cmd = 0`；命令仍经过正常 candidate/scheduler 与 reference stream。

```powershell
.\.venv\Scripts\python.exe scripts\view_stage5_v1_5_manual_demo.py
```

Stage 6 当前 2D follow viewer 使用完整 synthetic observation、capture-time RobotState 对齐、KF、Governor、FULL/yaw、MCU safety 与 MuJoCo 链；独立控制窗口的 forward/lateral 速度范围为 ±0.50 m/s，步进 0.02 m/s。MuJoCo HUD 和右侧内置面板默认隐藏。

```powershell
.\.venv\Scripts\python.exe scripts\view_stage6_5_manual_2d_follow.py --duration 300 --realtime-factor 1.0
```

各 viewer 都由用户亲自启动/关闭。旧 Stage 5 V1.0–V1.4 runner/config/results 已移入 `models/minisegway/stage5/reference/`，不再列为当前快速运行入口。

## 仓库结构

```text
control/                              冻结控制器、Q diagnostic 与 Stage 4 runtime
sim/                                  MuJoCo、sensor/estimator 与 payload support
models/minisegway/
  stage3/results/config/              Stage 3 baseline manifest 与运行配置
  stage4/config/                      当前 Stage 4 配置
  stage4/results/final_baseline/      最终 metrics、history 与短报告
  stage4/stage4b/                     已拒绝的 Stage4B/B-R 学习实验及复现材料
  stage4/reference/                   旧 Stage4E/E-R 历史痕迹，不属于当前 runtime
  stage5/config/                      当前 Stage 5 V1.5 command-stream 配置
  stage5/results/                     当前 Stage 5 V1.5 指标、history 与验收图
  stage5/reference/                   V1.0–V1.4 调试历史，不属于当前 runtime surface
  stage6/config/                      当前 2D follow 与 radial-KF 配置
  stage6/results/                     Stage 6.2–6.5 验收及当前变速验证
  stage6/reference/                   旧 follower、门限候选与 hunting 诊断历史
scripts/                              当前复现与 viewer 入口
CURRENT_STATE.md                      冻结状态、指标和能力边界
LEARNING_LOG.md                       调试路线、负结果与最终决策
```

更完整的当前状态见 [CURRENT_STATE.md](CURRENT_STATE.md)，模型与传感器语义见
[models/minisegway/README.md](models/minisegway/README.md)。
