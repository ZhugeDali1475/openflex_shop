# openflex_tc_wam

该包保留抓姿条件化的 TC-WAM 候选评估与 ready 区域绑定，供当前分层调度复用。
它不直接控制硬件，也不依赖比赛消息或执行器；不再把 TC-WAM/ACT 训练设为当前论文主线。
下文 AnyGrasp 命名为兼容接口，MuJoCo 中的 Oracle 后端不代表真实感知已接入。
项目方案以仓库根目录 `LLM_HIERARCHICAL_ROBOT_TASK_PLAN_CN.md` 为准。

## 最终输入与输出

第一次 AnyGrasp 在 Nav2 观察/交接区使用远距离点云生成 Top-K 抓姿
\(\mathcal G=\{g_k\}\)。TC-WAM 对底盘候选 \(b\)、升降柱—活动臂联合 IK 解层
\(h\) 和抓姿 \(g_k\) 评价：

\[
(b^*,h^*,g^*)=\arg\max_{b,h,g_k\in\mathcal F}S(b,h,g_k).
\]

`h` 由 `left_arm_with_lift` 或 `right_arm_with_lift` 求解，包含 `lift_joint` 与对应
七轴活动臂；配置中的 lift seed 只是 IK 冗余搜索层，不是 ACT 命令。输出包括：

- 全部 `(base, lift+arm solution, grasp)` 候选及硬门控/软评分；
- 二维可视化热力图和推荐底盘 `b*`；
- 包含联合关节解的 `ManipulationPlan`；
- 18维 `compact_state`；
- 与唯一 `(active_arm, planning_group, h*, g*, map_version)` 绑定的 ready 区域。

## ready区域不混合抓姿

ready 集只从与最优候选完全相同的抓姿、升降解层、活动臂、规划组和 map version 中
选取，并只保留包含 `b*` 的连通分量：

\[
\mathcal R_{h^*,g^*}=\operatorname{CC}_{b^*}
\{b\mid F(b,h^*,g^*)=1,\ S(b,h^*,g^*)\ge
\max(\tau_{abs},\rho S^*)\}.
\]

禁止把不同抓姿或不同升降解层的高分网格合并成一个 ready 区域。

## 两阶段AnyGrasp

- 粗阶段：为 TC-WAM 提供 Top-K 物理抓姿，决定初次 `(b*,h*,g*)`；
- 近场阶段：ACT 对位并锁定底盘后，用近距离点云重新生成抓姿；
- 新抓姿当前可达：MoveIt 从实际关节状态规划执行；
- 全部新抓姿不可达：保留新候选，局部重算 TC-WAM 并再次请求 ACT；
- `mock_anygrasp` 始终标记 `physical_perception=false`，只供离线检查。

## 物理评估边界

MoveIt evaluator 使用 `planning_groups.yaml` 中与集成 SRDF 一致的
`left_arm_with_lift` / `right_arm_with_lift`，只调用 IK、StateValidity 和临时
Planning Scene，不执行轨迹。正式地图要求：

1. `/joint_states`、真实 TCP、联合规划组和关节限位一致；
2. 货架碰撞体已标定且 `shelf_scene.yaml: calibrated=true`；
3. AnyGrasp 候选来自当前对象版本的物理点云；
4. 结果同时满足 `physical_perception=true` 和 `physical_evaluation=true`。

mock evaluator 或未标定场景的结果不得作为论文证据。
