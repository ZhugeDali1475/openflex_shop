# openflex_thesis_control

该包实现最终论文流水线的硬件所有权、ACT 底盘精对位、MoveIt 抓取执行和任务恢复，
不包含比赛语义。

## 底盘命令所有权

```text
Nav2 /thesis/nav_cmd_vel ─┐
ACT  /thesis/act_cmd_vel ─┼─ velocity_arbiter ─ /thesis/cmd_vel_selected
恢复 /thesis/recovery_cmd_vel ┘                         │
                                                        └─ collision monitor ─ base
MOVEIT / PLACE / HOLD ─────────────── BaseLock（恒为零）
```

Nav2、ACT 和恢复输入互斥。进入 `MOVEIT` 前必须由实测里程计连续确认底盘停稳；
MoveIt 运行期间仲裁器保持零速度锁定，任何输入都不能穿透该锁。

## ACT只做局部底盘对位

`local_alignment_supervisor` 提供 `/thesis/execute_local_alignment`，在 Nav2 已退出且
底盘停稳后重置回合坐标，并发布 `/thesis/act/policy_control`。ACT：

- 数值观测为底盘回合状态 6 维 + 抓姿条件化 TC-WAM 状态 18 维，共 24 维；
- 动作固定为 `base_link` 下 ([v_x,v_y,\omega_z]) 3 维速度；
- 底盘命令只能发布到 `/thesis/act_cmd_vel`；
- 从不发布升降柱、机械臂或夹爪命令；
- disabled、版本/绑定不一致或感知失效时立即清空 action chunk 并输出零速度。

ready 区域同时绑定 `object_id`、`map_version`、`active_arm`、`selected_grasp_id`、
`selected_lift_position`、`planning_group` 和 `ready_region_id`。进入其他抓姿对应的
高分格不算成功。

## MoveIt抓取执行

`moveit_grasp_executor` 提供 `/thesis/execute_moveit_grasp`。它只在 `MOVEIT` 模式且
底盘实测停稳时运行，使用当前 `/joint_states`：

1. 对第二次 AnyGrasp 的候选逐一执行 IK 和碰撞检查；
2. 使用 `left_arm_with_lift` 或 `right_arm_with_lift` 联合规划 `lift_joint` 与活动臂；
3. 执行预抓取、抓取、夹爪闭合、物体附着和运输姿态；
4. 等待独立 `/thesis/grasp_status`，不能自证抓取成功；
5. 若所有近场抓姿当前不可达，返回 `reposition_required=true`，不强行执行。

夹爪底层可以是独立 `GripperCommand` controller，但其时序与命令所有权归 MoveIt
抓取执行阶段；ACT 永远不控制夹爪。

## 最终状态机

```text
Nav2观察区 -> 停稳/重置ACT回合 -> AnyGrasp粗Top-K
-> TC-WAM选择(b*,h*,g*)及绑定ready -> ACT底盘对位
-> ACT取消/BaseLock -> AnyGrasp近场更新 -> MoveIt联合抓取
-> [不可达: 局部TC-WAM重算 -> ACT再次对位]
-> 抓取验证/附着/运输姿态 -> Nav2运输 -> 固定放置
```

历史 14-D/32-D checkpoint 和 29-D/11-D“手臂+夹爪+底盘”checkpoint 均与最终
24-D/3-D schema 不兼容。

