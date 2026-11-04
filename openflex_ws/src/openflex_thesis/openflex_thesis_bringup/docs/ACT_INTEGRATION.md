# ACT底盘精对位与TC-WAM接入说明

## 1. 最终职责

ACT 只负责 Nav2 交接区到抓姿条件化 ready 区域的短距离底盘精对位。它的唯一动作是

\[
a_t=[v_x,v_y,\omega_z]_{base\_link}.
\]

ACT 不控制 `lift_joint`、左右机械臂或夹爪。进入 ready 区域并持续低速后，监督器先
发送 `PolicyControl.enabled=false`、清空 temporal ensemble/action chunk，再由仲裁器
锁定底盘。随后 MoveIt 独占升降柱、活动臂和夹爪执行阶段。

## 2. 历史代码与checkpoint边界

当前工作区的 ACT/LeRobot bridge 若从历史目录恢复，必须另建最终论文 schema，不得
覆盖旧数据。以下两类 checkpoint 都不能直接复用：

- 14维状态/32维动作：包含八点底盘路径；
- 29维状态/11维动作：包含右臂、夹爪和三维底盘速度。

最终格式是 24维数值观测和3维底盘速度动作，必须重新完成 pilot 采集、训练、回放和
真机推理闭环。

## 3. 冻结观测

### 3.1 底盘回合状态6维

```text
x_e, y_e, yaw_e, vx_measured, vy_measured, wz_measured
```

- 收到 `PolicyControl(enabled=true, reset_episode=true)` 时，以当前底盘位姿重置
  ACT 局部回合原点；
- 回合相对位姿来自 FAST-LIO 的连续位姿；
- 实测速度单独来自 `/swerve_drive_controller/odom`；
- 上一时刻速度命令不得冒充实测速度。

bridge 必须维护独立 pose/velocity 缓存，不能因 topic 不同而覆盖订阅。

### 3.2 抓姿条件化TC-WAM状态18维

严格按 `act_schema.yaml` 顺序：

```text
target_dx, target_dy, target_z, sin_target_dyaw, cos_target_dyaw,
preferred_base_dx, preferred_base_dy,
sin_preferred_base_dyaw, cos_preferred_base_dyaw,
current_score, best_score, reachability, joint_margin, robustness,
grasp_quality, selected_lift_position, ready_distance, ready
```

总数值观测为 `6 + 18 = 24`。图像为头部 RGB 与活动腕部 RGB；机械臂、升降柱和
夹爪状态不进入该 base-only ACT 的冻结数值向量。

## 4. ready绑定与fail-closed检查

18维向量之外，推理帧必须携带并核对以下非归一化元数据：

```text
object_id
map_version
active_arm
selected_grasp_id
selected_lift_position
planning_group
ready_region_id
```

`PolicyControl`、`AffordanceState`、`ReadyRegion` 与对象记忆必须完全一致。ready
区域只允许包含同一 `(active_arm, planning_group, h*, g*, map_version)` 的候选；
不同抓姿或 lift stratum 的高分格不能合并。任何字段失配、对象不可见、地图失效、
状态超时或 `action_space != BASE_TWIST` 时，推理进程必须在一个控制周期内停机，不能
静默填零后继续旧 action chunk。

## 5. LeRobot bridge必须修改的内容

1. 独立订阅 FAST-LIO 回合位姿和 swerve 实测速度；
2. 订阅 `/thesis/tc_wam/state` 并追加18维条件；
3. 订阅 `/thesis/act/policy_control`，实现回合重置、绑定检查和 chunk 清空；
4. 只产生3维底盘速度，不创建任何 arm/lift/gripper action tensor；
5. 所有策略底盘输出只发 `/thesis/act_cmd_vel`，禁止直发 `/cmd_vel`；
6. disabled 时立即发布零速度，并等待新的 episode id 后才能恢复。

目标配置文件：

- `openflex_thesis_control/config/act_schema.yaml`：论文规范；
- `openflex_thesis_control/config/lerobot_thesis_m2_schema.yaml`：bridge目标配置。

## 6. Pilot验收

正式采集前依次验证：

1. schema 检查输出 `numeric_observation=24`、`action=3`；
2. 回合原点每次 Nav2—ACT 交接只重置一次；
3. 位姿与实测速度分别来自指定 topic；
4. ACT 进程中不存在升降柱、机械臂或夹爪发布器；
5. 所有底盘命令只出现在 `/thesis/act_cmd_vel`；
6. disabled、map version 或 ready binding 变化后一个周期内归零并清 chunk；
7. 比例教师经同一 arbiter 能进入绑定 ready 区域；
8. 录制20～30条 pilot，过拟合、离线回放并真机连续运行至少10次；
9. 检查 ACT 结束与 MoveIt 开始之间存在可追溯的低速确认和 BaseLock；
10. 通过后冻结 schema、归一化参数、数据版本和 checksum。

## 7. 必做对照

为了回答“为什么不用普通局部控制”，至少比较：Nav2 直接到点、PID/MPC 到 `b*`、
ACT 到单点、ACT 进入抓姿绑定 ready 区域。所有组使用相同初始位姿、速度上限和
AnyGrasp/TC-WAM 输入。

