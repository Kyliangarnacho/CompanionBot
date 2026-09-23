# Stage 5 historical references

本目录保存已退出当前 baseline 的 Stage 5 V1.0–V1.4 配置、启动脚本、结果和图表，用于还原调试过程；
这些脚本不是当前快速运行入口。旧 V1.0/V1.1 planner/projection 与 non-terminal-manifold HOLD compatibility path 已从当前 runtime 删除，
因此归档脚本快照仅供追溯，不保证可直接启动；历史 episode 的原始输出仍保留。

当前唯一生效的 Stage 5 配置和 runner 是：

- `../config/stage5_v1_5_pi_mcu_stream_config.json`
- `../../../../scripts/run_stage5_v1_5_pi_mcu_stream.py`
- 当前人工试玩入口：`../../../../scripts/view_stage5_v1_5_manual_demo.py`

Stage 3/4 frozen controller 与 estimator 没有因 Stage 5 调试而改参数；V1.5 只在上层命令调度、规划计算与 reference-block stream 侧接入。

## 归档索引

- `v1_0/`：20 Hz Master observation → follower → rolling reference 的初始链路，以及 λ_ff=0 对照图。
- `v1_1/`：distance dead-zone、recovery-tail/fixed-horizon 探索、terminal weight sweep，以及对应旧 follower 单测。
- `v1_2/`：LIGHTWEIGHT/FULL 双路径的第一版 scheduler/planner episode。
- `v1_3/`：slope transient gate、candidate 分类和 sparse LSQR 修正后的 episode。
- `v1_4/`：candidate-stability 生命周期与完整 FULL 执行/退出验证。

## 关键历史结论

- V1.1 失败 splice（`v_ref=0.3055 m/s`、`v_cmd=0.3260 m/s`、`a_ref=0.118 m/s²`）显示 recovery-tail 在既有限制内找不到可行 terminal-zero plan。25 组 horizon/weight 搜索没有同时通过当时的全部门限；最小 minimax 残差为 0.02324（H=1.0 s、weight=1e-5），因此没有把该版本作为当前执行路径。
- V1.3 的综合 episode 有 14 次 LIGHTWEIGHT、1 次 FULL；加速瞬态中没有误进 SLOPE、solver 没失败、没有 saturation/fall，但 quiet/fade 仅完成 0/1，说明 FULL 生命周期仍未收口。
- V1.4 达到 4 次 LIGHTWEIGHT、1 次 FULL、一次 mid-trajectory lightweight replan；quiet snap/fade 为 1/1、pending command 在 FULL 退出后得到处理，且无 false SLOPE、无 saturation、无 fall。它成为 V1.5 planner 性能对照。

完整调试顺序、失败原因和 V1.5 当前结果见仓库根目录 `LEARNING_LOG.md`、`CURRENT_STATE.md` 与 `README.md`。
