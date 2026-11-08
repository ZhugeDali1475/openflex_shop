# OpenFlex 毕业设计项目入口

更新日期：2026-11-08

## 当前版本

当前项目主线为**失败记忆增强的代价敏感局部恢复决策**，研究范围是货架移动抓取出现失败后的恢复选择。
导航、TC-WAM、局部对位、MoveIt 和安全控制链作为已有系统能力保留，不再把旧版 ACT 学习算法论文作为当前毕业设计。

- 研究清单：[`RECOVERY_RESEARCH_PLAN_CN.md`](RECOVERY_RESEARCH_PLAN_CN.md)
- 方法、题目与论文结构：[`RECOVERY_THESIS_METHOD_CN.md`](RECOVERY_THESIS_METHOD_CN.md)
- 运行和复现：[`RECOVERY_RUNBOOK_CN.md`](RECOVERY_RUNBOOK_CN.md)
- 实现、实测和当前限制：[`RECOVERY_IMPLEMENTATION_REPORT_CN.md`](RECOVERY_IMPLEMENTATION_REPORT_CN.md)
- 恢复模块代码：`openflex_ws/src/openflex_thesis/openflex_care_agent/`、`openflex_ws/src/openflex_thesis/openflex_thesis_control/`、`openflex_ws/src/openflex_thesis/openflex_thesis_experiments/`

## 当前研究状态

工程接入、测试和初步仿真已经完成；但 45 次冻结预实验中只有 9 条实际恢复转移，且全部失败，未通过扩展研究闸门。
因此尚无正式训练模型、正式五方法比较或算法性能提升结论。不能把单次开发冒烟、软件测试或历史数据口径修正当作算法效果。
细节及后续验收顺序以实现报告和研究清单为准；是否满足毕业要求由导师确认。

## 运行边界

- 研究恢复流程默认关闭，须显式启用实验参数；原任务流程仍保留。
- 当前仿真使用 oracle 观测/抓姿、脚本化导航/对位和几何夹持代理，不等同于实机接触抓取。
- 不得为获得成功样本关闭碰撞检查、弱化夹持验证或越过控制权安全约束。
- 正式训练和正式对照只有在预实验闸门通过并采到足够有效数据后才能开展。

## 工程与历史资料

`openflex_ws/src/openflex_thesis/README.md` 说明论文 ROS 子系统；`MUJOCO_THESIS_EXPERIMENT_RUNBOOK_CN.md` 提供既有仿真实验操作参考；`scripts/README_CN.md` 介绍仓库脚本。
ACT 对位仍可能作为底层工程能力存在，但其旧版算法论文规划已从当前项目入口移除。历史验收记录保留用于追溯，不代表当前研究结论。
