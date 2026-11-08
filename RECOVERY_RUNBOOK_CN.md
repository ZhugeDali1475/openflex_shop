# 恢复研究复现手册

在仓库根目录执行；全部命令使用真实 MuJoCo＋MoveIt 链，训练软件测试例外且不得作为算法性能证据。
运行产物写入新的目录，不覆盖已完成 trial。先阅读 `RECOVERY_RESEARCH_PLAN_CN.md` 和 `RECOVERY_THESIS_METHOD_CN.md`。
本轮实际结果与限制见 `RECOVERY_IMPLEMENTATION_REPORT_CN.md`：冻结 45 次预实验未通过研究闸门，暂不扩大训练或正式评测。

## 1. 构建

接口新增了恢复结果和指定底盘评价字段，旧安装空间不能直接混用：

```bash
source /opt/ros/humble/setup.bash
source openflex_ws/install_v2/setup.bash
colcon --log-base openflex_ws/log_v2 build \
  --base-paths openflex_ws/src/openflex_thesis \
  --build-base openflex_ws/build_v2 --install-base openflex_ws/install_v2 \
  --packages-select openflex_thesis_msgs openflex_care_agent openflex_tc_wam \
    openflex_thesis_control openflex_thesis_experiments openflex_thesis_mujoco \
  --symlink-install
```

## 2. 学习依赖和测试

```bash
bash scripts/run_recovery_research.sh setup
bash scripts/run_recovery_research.sh test
```

依赖安装到隔离 `.venvs/recovery`，不替换系统 ROS Python。训练和学习模型运行使用同一依赖版本。
Joblib 使用 pickle，只加载本项目自己训练且来源可信的模型，不能加载不可信模型文件。
模型 roundtrip 单元测试使用软件夹具，不是仿真采集结果。

## 3. 历史任务终态审计

```bash
bash scripts/run_recovery_research.sh report \
  --audit-legacy openflex_ws/migration_records/2026-11-05/g8_repeatability \
  --output recovery_results/legacy_audit.json
```

结构化任务终态优先于外层 launch 进程退出。没有终态的外层超时是未完成/基础设施问题，不能伪装物理抓取失败。
历史复核结果为六次任务成功、四次任务失败，不再将“10 次外层超时”当作任务成功率；该组不是新算法实验。

## 4. 预实验

```bash
bash scripts/run_recovery_research.sh matrix probe --output recovery_results/probe_matrix.json
bash scripts/run_recovery_research.sh run \
  --matrix recovery_results/probe_matrix.json --output recovery_results/probe_runs --timeout 480
bash scripts/run_recovery_research.sh report recovery_results/probe_runs/*/result.json \
  --output recovery_results/probe_report.json
```

默认 15 个开发配置、每配置三类强制恢复，最多 45 次运行。动作不可用时安全退出，不伪造转移。
`--limit 3` 只跑前三条，可用于检查成本；不能当成预实验完成。每次 ROS_DOMAIN_ID 隔离，终态出现后清理整个进程组。
Ctrl-C 会清理仿真并记录 interrupted；需要重跑时换新目录。预实验期间不要修改执行代码。
每个目录保存参数 YAML、manifest、源码哈希与快照、完整 runtime.log、恢复 JSONL、结果 JSON。
manifest 的 tracked.patch 不含新增文件，因此复现以 source_snapshot.tar.gz 和 source_sha256 为准。

`research_gate` 不通过时先看原因：动作缺失、无法复现同类故障、只有一种动作有效、重复失败不足。
不能通过换随机种子挑好结果或降低碰撞门限来过关。近似状态重放不等于精确快照对照。

## 5. 开发数据采集与训练

仅在预实验支持继续后采集。矩阵提供 60 个开发配置，允许多轮扩充；有效恢复转移数可能小于任务数。

```bash
bash scripts/run_recovery_research.sh matrix collect --output recovery_results/collect_matrix.json
bash scripts/run_recovery_research.sh run \
  --matrix recovery_results/collect_matrix.json --output recovery_results/collect_runs --timeout 480
bash scripts/run_recovery_research.sh train recovery_results/collect_runs/*/recovery.jsonl \
  --gate-report recovery_results/probe_report.json --output recovery_results/models/memory
bash scripts/run_recovery_research.sh train recovery_results/collect_runs/*/recovery.jsonl \
  --gate-report recovery_results/probe_report.json --no-memory --output recovery_results/models/no_memory
```

训练程序拒绝闸门未通过、配置组不足、划分中缺少正/负标签、未分类证据或正式 held-out 记录。
不要用伪造/重复行补足训练样本。开发集用于候选规则、lambda/mu、代价统计；测试集不调参。
输出 model.joblib 与 report.json。当前训练选择依据为验证 Brier，不代表在线策略必然优于启发式。

## 6. 正式对照

```bash
bash scripts/run_recovery_research.sh matrix evaluate \
  --model recovery_results/models/memory/model.joblib \
  --no-memory-model recovery_results/models/no_memory/model.joblib \
  --output recovery_results/evaluate_matrix.json
bash scripts/run_recovery_research.sh run \
  --matrix recovery_results/evaluate_matrix.json --output recovery_results/evaluate_runs --timeout 480
bash scripts/run_recovery_research.sh report recovery_results/evaluate_runs/*/result.json \
  --output recovery_results/evaluation_report.json
```

正式配置 90 个、五方法共 450 次，不能因为耗时较长就把尚未执行的矩阵写成已完成。
不要在正式测试中更换底层执行器或提高某方法的 Top-K/次数/时限。实机、文献新颖性和导师验收仍是独立工作。

## 7. 手动接入与回退

默认 `enable_recovery_research=false`，原脚本保留旧流程。
通过 `experiment_config:=绝对路径.yaml` 可覆写节点参数，不需要修改全局配置。
`recovery_local_only=true` 在抓取验证并达到运输姿态后结束，完整取送设为 false。
学习方法缺少模型会明确报错，不静默替换成启发式。安全中止和基础设施问题必须保留日志。
