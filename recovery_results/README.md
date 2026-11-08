# 局部恢复研究运行记录

本目录保留真实运行产物，包括负结果。不同阶段不得混合统计或当作正式方法比较。

## 可引用的本轮证据

- `legacy_audit.json`：历史十次任务日志终态复核，6 成功、4 失败；不是新算法基线。
- `probe_matrix_v1.json`：冻结预实验配置，15 配置 × 3 类强制恢复。
- `probe_v1/`：45 次冻结预实验的逐次参数、源码哈希/快照、日志和结果。
- `probe_v1_report.json`：预实验审计，9 条实际恢复转移全部失败，研究闸门未通过。
- `final_smoke/`：独立开发冒烟；完整取送及首次失败后换抓姿恢复均成功，不并入冻结批次。

`development_*`、`pilot_*`、`probe_frozen_01` 和 interim 报告是联调、诊断或中断记录，
保留以便追溯，但不能替代完整冻结预实验，更不能选择其中成功案例填入正式对照表。

每次运行以其 `manifest.json` 中的源码哈希和 `source_snapshot.tar.gz` 为版本依据。
`tracked.patch` 不包括当时未跟踪的新增文件，不能单独用来还原运行。
源码快照不包含全部上游模型资产，也不应被当作完整实机复现包。

当前没有正式训练模型或正式五方法实验结果。完整状态见仓库根目录的
`RECOVERY_IMPLEMENTATION_REPORT_CN.md`，后续运行方法见 `RECOVERY_RUNBOOK_CN.md`。
