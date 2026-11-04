# 最终论文工程接口契约

## 核心Action

| 接口 | 服务端 | 成功含义 |
|---|---|---|
| `/thesis/generate_grasps` | AnyGrasp gateway | 返回与对象版本和阶段一致的Top-K抓姿 |
| `/thesis/evaluate_candidates` | mock或MoveIt evaluator | 完成 `(b,h,g)` 候选评估；正式结果必须physical=true |
| `/thesis/compute_tc_wam` | TC-WAM server | 输出 `(b*,h*,g*)`、联合计划及唯一绑定ready区域 |
| `/thesis/execute_local_alignment` | ACT supervisor | ACT仅用底盘速度进入绑定ready区域并持续低速 |
| `/thesis/execute_moveit_grasp` | MoveIt executor | 当前底盘可达并完成升降柱+活动臂+夹爪抓取；或明确请求重对位 |
| `/thesis/execute_fixed_place` | place supervisor | 已到放置位、释放且被独立验证 |
| `/thesis/run_pick_deliver` | mission orchestrator | 粗导航、两阶段感知、对位、抓取、运输和放置全部成功 |

`ExecuteLocalPick.action` 仅作为 ROS 2 增量构建兼容类型保留，没有服务端，也不在最终
状态机中使用；正式流程只能调用 `ExecuteLocalAlignment -> ExecuteMoveItGrasp`。

## AnyGrasp契约

`GenerateGrasps` 的 stage 必须是 `COARSE` 或 `REFINED`。返回值必须与请求的
`object_id`、`map_version`、观测序号、active arm 和 stage 一致，抓姿 ID 在该响应内
唯一，位姿四元数有效。正式运行要求 `physical_perception=true`。

- COARSE：Nav2 停稳后，为 TC-WAM 提供 Top-K；
- REFINED：ACT 退出且底盘锁定后，用近场点云更新抓姿。

## `(b,h,g)`与ready契约

`h` 是 `left_arm_with_lift` 或 `right_arm_with_lift` 的升降柱—活动臂联合 IK 解层。
`ReadyRegion` 必须同时匹配：

```text
object_id, map_version, active_arm, grasp_id,
planning_group, lift_position, region_id
```

所有 `member_base_poses` 必须来自同一个 `(h,g)` 绑定和包含最优 `b*` 的连通分量。
禁止对不同抓姿、规划组或升降解层做二维 max 后再合并 ready 区域。

## 关键Topic

| Topic | 发布者 | 订阅者 | 约束 |
|---|---|---|---|
| `/thesis/object_observation` | 目标感知 | memory | pose在header.frame_id |
| `/thesis/object_memory` | memory | WAM/arbiter/FSM | map_version变化使旧抓姿、图和ready失效 |
| `/thesis/anygrasp/request` | gateway | 外部AnyGrasp | 携带request id和粗/近阶段 |
| `/thesis/anygrasp/candidates` | 外部AnyGrasp | gateway | 必须回显request id和版本 |
| `/thesis/tc_wam/state` | WAM | ACT/arbiter/logger | 18维条件及完整抓姿绑定 |
| `/thesis/act/policy_control` | alignment supervisor | ACT bridge | 只允许BASE_TWIST；disabled立即归零/清chunk |
| `/thesis/nav_cmd_vel` | Nav2 | arbiter | 不得直连控制器 |
| `/thesis/act_cmd_vel` | ACT | arbiter | 仅base_link三维速度 |
| `/thesis/cmd_vel_selected` | arbiter | collision monitor | 唯一底盘出口，不得绕过安全层 |
| `/thesis/grasp_status` | 独立验证器 | MoveIt executor | 执行器不能自证成功 |
| `/thesis/place/status` | 释放验证器 | place supervisor | 消息发送不等于释放成功 |

若目标感知只提供`PoseStamped`，使用`object_observation_adapter`转换。输入必须有非空
`frame_id`和非零真实时间戳；适配器把位姿及6×6协方差转换到`map`，拒绝过期观测、
非法四元数、意外源坐标系和TF失败。固定置信度只用于已标定且无置信度输出的检测器；
若检测器提供置信度，应通过独立`Float32` topic接入并设置超时。

## 控制权不变量

### ACT运行期间

```text
ControlMode.mode == ACT
PolicyControl.action_space == BASE_TWIST
ObjectMemory.map_version == AffordanceState.map_version == PolicyControl.map_version
ObjectMemory.visible && ObjectMemory.grasp_authorized
AffordanceState.valid && AffordanceState.physical_evaluation
selected_grasp_id / selected_lift_position / planning_group / ready_region_id 完全一致
ACT发布者集合 == {/thesis/act_cmd_vel}
```

### MoveIt运行期间

```text
ControlMode.mode == MOVEIT
实测底盘速度连续低于阈值
仲裁器底盘输出 == 0
ACT PolicyControl.enabled == false
MoveIt planning_group in {left_arm_with_lift, right_arm_with_lift}
规划起点 == 当前实际/joint_states
```

若第二次 AnyGrasp 至少一个抓姿在当前底盘可达，MoveIt 选择最优可达抓姿执行；若全部
不可达，返回 `reposition_required=true`，不得规划穿越约束。状态机在预注册次数内以
新抓姿局部重算 TC-WAM，再交给 ACT。
