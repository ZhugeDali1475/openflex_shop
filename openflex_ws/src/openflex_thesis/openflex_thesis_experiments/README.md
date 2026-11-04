# openflex_thesis_experiments

正式实验不允许临时改组。先将 `config/experiment_protocol.yaml` 中的
`frozen` 改为 `true`，记录代码提交、数据集和 checkpoint，再生成 trial manifest：

```bash
ros2 run openflex_thesis_experiments generate_trial_matrix \
  install/openflex_thesis_experiments/share/openflex_thesis_experiments/config/experiment_protocol.yaml \
  /tmp/thesis_manifest.jsonl
```

每次真机实验从 manifest 读取唯一 `trial_id` 启动 `trial_logger`。日志目录包含：

- `metadata.json`：实验条件与版本；
- `events.jsonl`：任务、热力图、记忆、控制权和抓放证据；
- `result.json`：终态、成功率字段、耗时和错误码。

最终流水线的事件日志还必须保存粗/近场 AnyGrasp request id、候选 grasp id、选定
`(b*, h*, g*)`、`ready_region_id`、planning group、map version、ACT 对位结果、
MoveIt 是否返回 `reposition_required` 以及局部重算次数。不同绑定的 ready 区域不得
在统计时合并为同一次成功。

汇总使用 Wilson 95% 区间，TC-WAM 物理标定还应报告热力图分数与经验成功率的
Spearman 相关系数。不得把 mock evaluator 日志计入正式数据。

## 正式证据预检

每个实验阶段先运行静态门控，例如：

```bash
# 只检查真实AnyGrasp阶段
ros2 run openflex_thesis_experiments thesis_preflight \
  --scope perception --through-gate G2_moveit_evaluator

# 完整取送正式实验
ros2 run openflex_thesis_experiments thesis_preflight \
  --scope end_to_end --through-gate G7_end_to_end
```

该命令检查 AnyGrasp SDK/checkpoint、物理输出标志、货架场景标定、MoveIt对象几何、
24-D/3-D ACT冻结状态、安全门以及指定顺序的硬件验收门。失败是预期的保护行为，
不得为了启动系统而直接把布尔量改成`true`；只能在对应标定记录、日志和视频齐全后更新。
