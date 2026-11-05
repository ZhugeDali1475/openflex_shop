# CARE-Agent 项目当前状态

日期：2026-11-04  
最近更新：2026-11-05（G6 放置通过；下一道门是 G7 端到端完整）  
阶段：P0 环境复现、P1 固定抓放基线  
总体结论：G0-G6 已通过；当前进入 G7（端到端完整验证）。

## 1. 一句话状态

当前系统已经完成：

```text
MuJoCo/ROS/MoveIt 环境可复现
-> TC-WAM 生成并评价真实抓取候选
-> 左臂收臂停姿在真实候选、实际 lift 和三个验收高度下无碰撞且不触地
-> MoveIt 完成预抓取、抓取和夹爪闭合
-> 独立 GraspStatus 验证物体附着
-> 运输姿态可达且附着保持
-> 固定放置动作执行成功，mission 完整完成
```

尚未完成：

```text
10 次重复测试验证稳定性
-> CARE-Agent ROS 闭环
-> 基线、消融和恢复统计
```

## 2. 本轮新完成

### G0：环境与单元测试

```text
mujoco_model 71 29 23
openflex_care_agent: 9 passed
openflex_thesis_mujoco: 5 passed
care_environment_ok
MoveIt基础环境验证通过
```

证据：

- [G0-G2 基线验收](openflex_ws/migration_records/2026-11-04/G0_G2_BASELINE_CN.md)

### G1：MuJoCo 运行态

- `/clock` 约 50 Hz；
- `/joint_states` 包含 29 个关节；
- 底盘里程计正常；
- `map -> base_link` TF 可查询。

### G2：MoveIt IK、轨迹与整机状态

```text
compute_ik success=True
move_group success=True planned_points=8
group=right_arm_with_lift valid=True contacts=0
```

### G3：左臂停放姿态

原“左臂与升降柱联合碰撞”阻塞已通过真实候选搜索消除。

选定停姿：

```text
[0.2, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
```

验证条件：

- 10 个右臂 IK 可行的真实 TC-WAM 候选；
- 每个候选自身的 lift 解以及 `-0.15/0/+0.15 m`；
- 机器人自身碰撞和仿真货架碰撞；
- 1680 个左臂姿态中 560 个在全测试集上可行；
- 最优姿态最小归一化关节余量约 0.936。

重启后的实际状态：

```text
left_arm_positions=[
  0.216418,
  -0.158572,
  0.000857,
  -0.000899,
  0.000675,
  0.006410,
  0.013065
]
group=right_arm_with_lift valid=True contacts=0
```

证据：

- [G3 验收记录](openflex_ws/migration_records/2026-11-04/G3_LEFT_PARK_ACCEPTANCE_CN.md)
- [G3 机器可读结果](openflex_ws/migration_records/2026-11-04/G3_LEFT_PARK_SEARCH.json)

### G4：固定底盘抓取与附着

mission 完成预抓取、抓取和夹爪闭合：

```text
selected grasp: 100102
MoveIt pregrasp trajectory: SUCCEEDED
MoveIt grasp trajectory: SUCCEEDED
simulated grasp attached thesis_item_01 to right TCP at distance 0.082 m
```

独立 `/thesis/grasp_status`：

```text
contact_detected=true
gripper_closed=true
object_retained=true
transport_safe=true
verified=true
failure_code=0
```

证据：

- [G4 抓取验收](openflex_ws/migration_records/2026-11-04/G4_GRASP_ACCEPTANCE_CN.md)

### G5：运输姿态与附着保持（2026-11-05 通过）

左臂停姿由“垂在身侧”改为“收在身侧上方”后，mission 完成
抓取 → 运输姿态 → 独立验证：

```text
simulated grasp attached thesis_item_01 to right TCP at distance 0.003 m
  (object in TCP frame (-0.0014, 0.0006, 0.0028))
transport result success=True: {'lift_joint': -0.105653, ...}
phase=10 detail=independent grasp verification passed
phase=11 detail=object attached; lift+arm in transport pose
phase=12 detail=Nav2 transports grasped object
```

证据：

- [G5 运输姿态与附着保持验收](openflex_ws/migration_records/2026-11-05/G5_TRANSPORT_ACCEPTANCE_CN.md)
- [G5 mission 日志摘录](openflex_ws/migration_records/2026-11-05/G5_MISSION_TRANSCRIPT.txt)

### G3 复搜（2026-11-05 通过）

停姿改为 `[-0.7, -1.2, 0.4, 1.2, 0.0, 0.0, 0.0]`。搜索器修正了 0.17 m 底座高度
漏项与错误的关节限位，并把地面平面纳入碰撞集后重跑。

```text
左臂枚举姿态 1680 -> 全测试集可行 577
最小归一化关节余量 0.6667
lift = -0.65 m 时左臂最低点仍在地面上方 +0.309 m
```

证据：

- [G3 停姿复搜验收](openflex_ws/migration_records/2026-11-05/G3_LEFT_PARK_ACCEPTANCE_CN.md)
- [G3 机器可读结果](openflex_ws/migration_records/2026-11-05/G3_LEFT_PARK_SEARCH.json)

### G6：固定放置与放置验证（2026-11-05 通过）

修复附着体时序问题后，mission 完成放置动作并成功结束：

```text
phase=13 detail=execute fixed delivery action
phase=15 detail=final layered mission completed
MUJOCO_MISSION_SUCCESS duration=164.078s
```

关键修复：
- 增强 `moveit_grasp_executor` 的 `on_place_status` 防护（检查 object_id、active_object_id、busy 标志）
- 优化 `bridge` 的放置状态发布策略（失败状态立即过期，成功状态保持 2 秒）
- 避免旧 PlaceStatus 消息导致附着体过早移除

证据：
- [G6 放置时序修复记录](openflex_ws/migration_records/2026-11-05/G6_PLACE_TIMING_FIX_CN.md)
- [G6 测试日志](/tmp/g6_test_1793845804.log)

- 复核并确认底座高度 TF 修复：桥把 0.17 m 放在 `map -> odom`，`odom -> base_link`
  的 z 为 0（与 SRDF `base_planar_joint` 的 planar 语义一致）。运行时实测
  `G5DBG pose_to_base frame=map->base_link in=(1.0000, -0.0000, 0.8439)
  T=(-0.3305, -0.4765, -0.1700)` ⇒ base_link 系物体 z = 0.6739，正确。
- 定位 G5 当前阻塞：左臂停姿在低升降处压入地面，顶高 `lift_joint` 约 28 mm，
  右臂 TCP 比物体高约 26 mm，夹爪门禁按设计拒绝。详见 §4.1 与
  [根因记录](openflex_ws/migration_records/2026-11-05/G5_LEFT_PARK_FLOOR_CONTACT_CN.md)。
- 新增可复现诊断脚本 `scripts/probe_left_park_contact.py`（ROS-free）。
- 本轮未改动 `src/` 下任何源码；`install_v2` 与 `src` 已逐文件核对一致。

## 3. 当前代码变化

G6 放置时序修复（2026-11-05）：

- `moveit_grasp_executor_node.py`：在 `on_place_status` 中添加三层防护检查
  （attachment 存在性、object_id 匹配、active_object_id 匹配、busy 状态），
  防止旧 PlaceStatus 消息触发过早分离；
- `bridge_node.py`：优化 `_on_place_control` 的状态发布策略，失败状态设置
  `place_status_until` 为过去时间（立即过期），成功状态保持 2 秒发布窗口；
- `install_v2` 对应文件已同步。

左臂停姿相关（2026-11-05）：

- 停姿由 `[0.2, -0.15, 0, 0, 0, 0, 0]` 改为 `[-0.7, -1.2, 0.4, 1.2, 0, 0, 0]`，
  写入 `openflex_thesis_mujoco/config/mujoco.yaml` 与 MuJoCo bridge 默认参数；
- `scripts/search_moveit_left_park.py`：补齐 `map -> base_link` 的 0.17 m 底座
  高度、改用真实关节限位、把地面平面加入碰撞集、升降验收集合改为
  TC-WAM 种子 ± 三个验收高度；
- `scripts/probe_left_park_contact.py`：新增 ROS-free 复现脚本（停姿触地 → 顶高
  `lift_joint`）；
- `install_v2` 对应文件已同步（非 editable 安装，需手工 `cp`）。

更早一轮（G4/G5 几何修复）：

- `moveit_evaluator_node.py` 增加 `base_mount_height_m=0.17`；
- `bridge_node.py` 把底座 0.17 m 从 `odom -> base_link` 移到 `map -> odom`；
- `moveit_grasp_executor_node.py` 用物体相对 TCP 的真实位姿挂附着盒，
  并修掉传给 `pose_to_base` 的 `PoseWithCovariance`；
- `mujoco.yaml` 打开 `gravity_compensation` 与 `hold_left_arm_park`。

## 4. 仍存在的工程问题

### 4.1 未进入候选评分的约束

在修正后的碰撞场景里，`lift_joint = -0.600 m` 时**右臂**会与
`mujoco_shelf_board_target_level` 碰撞（与左臂停姿无关），即候选的升降解
不能低于约 −0.55 m。该约束目前只体现在 `scripts/search_moveit_left_park.py`，
尚未进入 `moveit_evaluator` 的候选评分。

### 4.2 MoveIt 退出不稳定

两次关闭 launch 时，`move_group` 在析构阶段出现 `exit code -11`。运行期 IK、
规划和状态检查均正常。该问题单独调查，不能隐瞒，也不能用它否定已完成的门。

### 4.3 碰撞模型不完整

MoveIt 仍报告多个 link 有视觉几何但无碰撞几何，且 Octomap updater 插件未加载。
在感知避障实验前必须补齐。

## 5. CARE-Agent 完成度

已完成 ROS-independent 决策内核：

```text
BeliefState
SkillContract
SkillRegistry
SafetyCritic
FailureCode
SkillOutcome
RecoveryPolicy
CAREAgent
```

仍未完成：

```text
observation_aggregator
capability_adapter
skill_executor
failure_mapper
ros_agent_node
trace_logger
launch and YAML configuration
```

准确表述：

> CARE-Agent 决策内核和 MuJoCo 技能平台已验证到 G4，尚未完成 Agent ROS 闭环。

## 6. 下一步验收顺序

| Gate | 条件 | 状态 |
|---|---|---|
| G0 | 环境、模型和核心测试通过 | 通过 |
| G1 | 时钟、关节、里程计和 TF 正常 | 通过 |
| G2 | IK、实际轨迹和初始状态无碰撞 | 通过 |
| G3 | 真实候选和升降范围内的左臂停姿 | 通过 |
| G4 | 单次抓取、夹持和附着证据成立 | 通过 |
| G5 | 运输姿态下仍保持附着 | 通过（2026-11-05） |
| G6 | 固定放置、释放和验证成功 | 通过（2026-11-05） |
| G7 | 导航、抓取、运输、放置端到端成功 | 待验证（mission 已完整通过一次） |
| G8 | 10 次固定条件重复 | 待完成 |
| G9 | CARE-Agent ROS 适配器闭环 | 待完成 |
| G10 | 基线、消融和恢复统计实验 | 待完成 |

## 7. 论文使用边界

当前可以写：

- MuJoCo/MoveIt 模块化实验平台；
- 基于真实候选的升降-左臂联合碰撞筛选，含地面间隙约束；
- 技能前置条件和确定性安全约束；
- G0-G6 的数字实验过程和证据；
- 单次完整 pick-and-place 流程（抓取-运输-放置）；
- 附着保持验证（含独立 GraspStatus 验证）。

当前不能写：

- 已证明稳定抓放（G8 重复性未做）；
- 已证明真实抓取成功；
- CARE-Agent 已在 ROS 中闭环；
- Oracle 候选代表真实视觉感知；
- 数字确定附着代表真实摩擦抓取；
- 重复性已成立（G8 未做）。
