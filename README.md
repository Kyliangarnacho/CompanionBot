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

Stage4A 坡面 viewer 可用于直观看车体进入 +8°、−8°、+15°、−15° 坡面：

```powershell
.\.venv\Scripts\python.exe scripts\view_stage4a_slope_demo.py
```

该 viewer 保留的是 Stage4A Q-ON 可视化实验臂；最终 production baseline 仍为 Q actuator OFF。
Viewer 由用户亲自启动并关闭。

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
scripts/                              当前复现与 viewer 入口
CURRENT_STATE.md                      冻结状态、指标和能力边界
LEARNING_LOG.md                       调试路线、负结果与最终决策
```

更完整的当前状态见 [CURRENT_STATE.md](CURRENT_STATE.md)，模型与传感器语义见
[models/minisegway/README.md](models/minisegway/README.md)。
