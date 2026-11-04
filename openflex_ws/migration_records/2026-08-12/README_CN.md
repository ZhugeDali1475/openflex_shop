# 2026-08-12 抓取迁移执行记录

本目录是 `GRASP_CONSOLE_MIGRATION_EXECUTION_PLAN_CN.md` 的证据索引。所有状态以正式清单为准，本文只保存基线、测试和失败说明。

## 执行信息

- 开始时间：2026-08-12 09:32 CST
- 批准来源：项目负责人在当前会话明确要求按正式清单开始执行
- 迁移起点：`/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_ws/src/openflex_grasp`
- 迁移目标：`/home/openflex/old/openflex_ws`
- 本阶段边界：真实 Detect -> Pick -> 手臂安全回撤；Place 只保留 mock/仿真

## 证据文件

- `G0_FILE_BASELINE_CN.md`：源、目标、Git、包和磁盘基线
- `G0_ROS_BASELINE_CN.md`：当前 ROS graph 与代码 owner 基线
- `G0_HARDWARE_MODEL_BASELINE_CN.md`：GPU、相机和模型资产基线

后续每通过或失败一个关卡，都在相应阶段文件中记录命令、输出摘要、日志路径和恢复条件。
