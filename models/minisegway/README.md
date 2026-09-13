# MiniSegway MuJoCo physics plant

`mini_segway.xml` 是 passive nominal plant；`mini_segway_moving_payload.xml`
增加了简化载物篮和自由刚体 payload。模型使用 CAD-derived visual mesh、
解析 collision proxy、显式 aggregate inertia 和两个自由 wheel hinge。
Actuator 是 `gear=1` 的直接 torque input，并保留 peak/stall hard limit；
模型本身不包含 controller，balance mode 也不使用 ball caster。

第三方 CAD 派生 mesh 已被 Git 忽略。若本地仍保留 upstream STEP，可使用独立
CAD 环境重新生成 mesh、MJCF 和质量属性报告：

```powershell
D:\project\MiniSegway_assembly_work\.venv\Scripts\python.exe tools\build_minisegway_plant.py
```

加载/contact smoke test：

```powershell
.venv\Scripts\python.exe scripts\smoke_test_minisegway.py
```

厂家值、CAD 计算值和 provisional 值的 provenance 记录在 `plant_report.json`；
未来必须实测的假设集中在 `plant_parameters.json`。项目总览与控制入口见根目录
`README.md`，冻结指标与边界见 `CURRENT_STATE.md`。
