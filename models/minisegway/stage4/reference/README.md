# Stage 4 historical reference

这里仅保存已退出当前 baseline 的阶段性实验痕迹，不属于 production runtime surface。

- `stage4e_legacy/`：旧 Stage4E/E-R 的失败/过渡结果与对应生成 world。旧 runner、配置、
  `PAYLOAD_ID` 状态机、continuous/shadow RLS 和多层 acceptance 逻辑已经删除；归档内容保持
  当时输出，不应作为当前门限或实现来源。
- Stage4B/B-R 的被拒绝学习路线保留在 `../stage4b/`，因为其 dataset manifest、结果报告与
  可复现 runner 仍有对照价值；最终结论仍是不得接入 production controller。

当前权威实现为 `control/stage4_runtime.py`，权威结果为
`results/final_baseline/STAGE4_FINAL_BASELINE.md`。
