# P10 交付审查与恢复记录

日期：2026-08-12（Asia/Shanghai）  
目标 workspace：`/home/openflex/old/openflex_ws`  
迁移源：`/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_ws/src/openflex_grasp`

## 交付范围

比赛抓取链只使用本地 Qwen3.5-4B + SAM2 感知和仓库内 GraspNetBaseline；完全不使用
AnyGrasp SDK、`openflex_anygrasp` 或其授权文件。Detect 负责 2D 框、实例 mask、商品/场景
点云、`object_pose` 和 detection cache；Pick 负责 GraspNet 候选、预抓取、Cartesian、
夹持判据和安全回撤；Place 明确后置。

## 文档入口

- 运行与迁移：`src/openflex_grasp/MIGRATION_GUIDE_CN.md`
- 核心说明：`src/openflex_grasp/openflex_grasp/README_CN.md`
- 模型/环境：`src/openflex_grasp/environment/COMPATIBILITY_MATRIX_CN.md`
- 接口：`src/openflex_grasp_interfaces/README_CN.md`
- 本执行清单：`GRASP_CONSOLE_MIGRATION_EXECUTION_PLAN_CN.md`

## 备份与恢复点

旧构建/安装缓存已移动到：

```text
/tmp/openflex_migration_backups/2026-08-12/
```

其中包含旧 `openflex_grasp_interfaces` build/install 和 `openflex_anygrasp` build/install
缓存。迁移没有执行不可恢复删除；恢复时只能恢复文件用于论文历史环境，不能恢复 AnyGrasp
作为比赛 backend。

模型资产位于 `/home/openflex/old/openflex_models`，其 `SHA256SUMS` 是部署校验入口。
两个 Python 环境分别为 `/home/openflex/old/.venvs/openflex_perception` 和
`/home/openflex/old/.venvs/openflex_graspnet`。

## P10-05 兼容项审查

- 活动代码和 launch 已无源 workspace 绝对路径、Qwen 9B 默认值和 AnyGrasp backend 引用。
- `/product_perception/*` 只存在于未由比赛 launch 启动的旧论文/可视化兼容入口；它们保留
  是为了不破坏历史实验，移除条件是负责人确认不再使用这些 launch。
- `competition_mock_pick` 只保留独立 `/competition/mock_pick_object`，不占用 `/pick_object`。
- 新增 backend owner lock，消除调试/比赛 launch 重复 owner 风险。

## 已知限制

P1 左腕相机缺失、G5/G6/G7/G8/G9 未完成；P8-05/P8-06 仍待正式 transport 字段接入；
所有真机、仿真、20 轮联测和 Place 均按负责人要求跳过。它们不能在本次交付中标为通过。

## 最终静态审查命令

```text
python -m compileall src/openflex_grasp/openflex_grasp/openflex_grasp 
colcon build --packages-select openflex_grasp_interfaces openflex_grasp openflex_navigation_gui openarmx_task_executor --symlink-install
rg -n 'openflex_anygrasp|anygrasp_sdk_v3' src --glob '!**/*.md' --glob '!**/__pycache__/**'
```

最后一条允许命中 `openflex_thesis` 的明确 legacy 文本，但不得命中比赛 launch 或新
`openflex_grasp` backend。
