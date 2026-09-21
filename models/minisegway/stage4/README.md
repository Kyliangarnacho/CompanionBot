# Stage 4 artifacts

Stage 4B/B-R 的历史学习实验已统一归入 `stage4/stage4b/`；最终结论仍为拒绝
proprioceptive slope learning，不属于 production runtime。当前冻结 runtime 与 smoke-test
结果位于 `config/stage4_final_baseline_config.json` 和 `results/final_baseline/`。

## Stage 4A slope viewer

最简可视化入口使用冻结 Stage 3 controller 和统一 Q ON：

```powershell
# 顺序观看 +8°、-8°、+15°、-15°
.\.venv\Scripts\python.exe scripts\view_stage4a_slope_demo.py

# 单独观看一个场景
.\.venv\Scripts\python.exe scripts\view_stage4a_slope_demo.py --scene up8
.\.venv\Scripts\python.exe scripts\view_stage4a_slope_demo.py --scene down8
.\.venv\Scripts\python.exe scripts\view_stage4a_slope_demo.py --scene up15
.\.venv\Scripts\python.exe scripts\view_stage4a_slope_demo.py --scene down15
```

相机自动跟随 chassis。每个场景使用相同的 `0 -> +0.4 m/s` command schedule，先在平地起步，
再进入坡面。关闭 MuJoCo 窗口即可停止整个 demo。

viewer 只提供可视化；控制器、estimator、Q-filter 参数、torque limit 和 terrain GT 数据边界
与 `stage4/config/stage4a_slope_robustness_config.json` 一致。

该 viewer 的 Q ON 是 Stage4A 历史 ablation 展示，不是当前 production baseline；最终 baseline
的 Q actuator 永久 OFF。
