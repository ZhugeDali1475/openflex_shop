# CARE-Agent 项目当前状态

日期：2026-11-04  
阶段：P0 环境复现、P1 固定抓放基线  
总体结论：G0-G4 已通过，下一道验收门是 G5 运输姿态与附着保持。

## 1. 一句话状态

当前系统已经完成：

```text
MuJoCo/ROS/MoveIt 环境可复现
-> TC-WAM 生成并评价真实抓取候选
-> 左臂停姿在真实候选、实际 lift 和三个验收高度下无碰撞
-> MoveIt 完成预抓取、抓取和夹爪闭合
-> 独立 GraspStatus 验证物体附着
```

尚未完成：

```text
安全运输姿态
-> 固定放置与放置验证
-> 10 次重复和正式实验
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

## 3. 当前代码变化

- `scripts/search_moveit_left_park.py` 已从任意参考 IK 搜索改为真实 TC-WAM
  候选搜索；
- 搜索时应用匹配候选底盘的货架碰撞体；
- 同时验证每个候选自身的 lift 解和三个验收升降高度；
- G4 首次运行暴露问题后，将最终停姿修正为 `[0.2, -0.15, 0, 0, 0, 0, 0]`；
- `mujoco.yaml` 和 MuJoCo bridge 默认参数已写入新左臂停姿；
- `openflex_thesis_mujoco` 已重新构建并通过重启复验。

## 4. 仍存在的工程问题

### 4.1 G5 运输姿态失败

抓取成功后，运输姿态规划失败：

```text
openarmx_right_link7 <-> thesis_grasped_thesis_item_01
Motion planning start tree could not be initialized
```

独立 G4 验证已经通过，但附着物体作为刚体与 `right_link7` 的碰撞仍需修复。

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
| G5 | 运输姿态下仍保持附着 | 下一项 |
| G6 | 固定放置、释放和验证成功 | 待完成 |
| G7 | 导航、抓取、运输、放置端到端成功 | 待完成 |
| G8 | 10 次固定条件重复 | 待完成 |
| G9 | CARE-Agent ROS 适配器闭环 | 待完成 |
| G10 | 基线、消融和恢复统计实验 | 待完成 |

## 7. 论文使用边界

当前可以写：

- MuJoCo/MoveIt 模块化实验平台；
- 基于真实候选的升降-左臂联合碰撞筛选；
- 技能前置条件和确定性安全约束；
- G0-G4 的数字实验过程和证据。

当前不能写：

- 已实现稳定抓放；
- 已证明真实抓取成功；
- CARE-Agent 已在 ROS 中闭环；
- Oracle 候选代表真实视觉感知；
- 数字确定附着代表真实摩擦抓取；
- 运输姿态已经通过。
