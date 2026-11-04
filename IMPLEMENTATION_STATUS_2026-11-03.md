# CARE-Agent 项目当前状态

日期：2026-11-03  
阶段：P0 平台复现与 P1 固定底盘抓放基线  
总体结论：仿真主栈、MoveIt 执行和 TC-WAM 候选评价已经打通，但单次端到端抓放尚未稳定通过，正式论文实验尚未开始。

## 1. 一句话状态

当前系统已经能完成：

```text
MuJoCo 启动
-> 导航到货架
-> Oracle 目标观测
-> AnyGrasp 候选
-> MoveIt IK/碰撞评价
-> TC-WAM 候选选择
-> 底盘局部对位
-> MoveIt 预抓取和抓取轨迹
```

但还不能稳定完成：

```text
无碰撞抓取
-> 夹爪附着验证
-> 安全运输姿态
-> 固定放置
-> 全流程重复实验
```

## 2. 已经完成并验证的工程

| 项目 | 状态 | 证据 |
|---|---|---|
| ROS 2 Humble | 完成 | `/opt/ros/humble` 可 source |
| MoveIt 2.5.10 | 完成 | MoveIt 包已安装并构建到 `install_v2` |
| Pick-IK 1.1.2 | 完成 | 插件和包可被 ROS 发现 |
| MuJoCo 3.4.0 | 完成 | EGL headless 可加载和渲染 |
| 整机 MJCF | 完成 | `nbody=71, njnt=29, nu=23` |
| MuJoCo/ROS 时间 | 完成 | `/clock` 和仿真时间正常 |
| 关节状态 | 完成 | 29 维 `/joint_states` 正常发布 |
| 三路 RGB-D | 完成 | 头、左腕、右腕均发布 RGB/Depth/CameraInfo |
| TF | 完成 | `map -> odom -> base_link` 和机器人 TF |
| 对象观测和记忆 | 完成 | `/thesis/object_observation`、`/thesis/object_memory` |
| Oracle Grasp | 完成 | 仅在数字实验中可用，明确标为非物理感知 |
| TC-WAM | 完成首版 | 可输出 grasp、lift、score、ready region 和 compact state |
| MoveIt IK | 完成 | 96 个右臂工作空间采样全部有解 |
| MoveIt 运动执行 | 完成 | `/move_action` 轨迹实际驱动 MuJoCo |
| 安全控制权 | 完成首版 | 底盘、MoveIt 和 gripper 有模式切换 |
| CARE-Agent 决策内核 | 完成首版 | 技能图、SafetyCritic、FailureCode、恢复路由和效用选择 |
| 实验协议 | 完成骨架 | manifest、trial logger、预检和 Wilson 统计 |

## 3. 关键验证结果

### 3.1 MoveIt

已验证：

```text
state_validity valid=True contacts=0
compute_ik success=True
move_group success=True
planned_points=8
```

MoveIt 能分别向以下控制器发送轨迹：

```text
/lift_joint_controller/follow_joint_trajectory
/right_joint_trajectory_controller/follow_joint_trajectory
/right_gripper_controller/gripper_cmd
```

仿真模式必须关闭 `moveit_command_bridge`，否则它会和 MuJoCo 桥同时提供同名
FollowJointTrajectory Action，造成双 Server 竞争。

### 3.2 TC-WAM

典型首轮输出为：

```text
grasp=100101
lift=-0.150
score=0.880
ready_region=v1:a2:g100101:h-150
physical=false
```

`physical=false` 是正确的。当前场景使用 Oracle 观测和确定性附着代理，不能把该结果
写成真实相机感知或真实接触力学结论。

### 3.3 单元测试

此前验证过的相关测试：

```text
CARE-Agent          9 passed
TC-WAM             15 passed
Thesis control      7 passed
Thesis experiments  7 passed
Thesis MuJoCo       5 passed
合计               43 passed
```

## 4. 当前关键阻塞

### 4.1 升降柱与未使用左臂的联合碰撞

问题不是右臂 IK 不可达，而是右臂工作时，升降柱高度改变后，未使用左臂的
`openarmx_left_link3` 可能和 `base_link` 发生碰撞。

已观察到的碰撞：

```text
openarmx_left_link3 <-> base_link
```

现有 `hands_up` 姿态：

```text
[0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0]
```

在升降柱为零时可以无碰撞，但不能保证在抓取所需的升降高度上仍无碰撞。

当前姿态搜索结果：

- 在 `lift=0` 时，粗网格中绝大多数左臂姿态被判为有效；
- 在 `lift=-0.15` 时，固定“当前右臂零位”会把右手指本身带入碰撞，不能作为
  合理的左臂搜索基线；
- 后续搜索必须固定一个**实际抓取候选中的右臂状态**，再只改变左臂和升降柱。

该问题解决前不能继续宣称固定底盘抓取基线完成。

### 4.2 抓取附着与验收时序

曾成功出现：

```text
simulated grasp attached thesis_item_01 to right TCP
verified=true
object_retained=true
transport_safe=true
```

但首次完整任务中，执行器在 3 秒验收窗口内没有及时接受该状态。

当前已做调整：

- 验收窗口从 3 秒扩展到 8 秒；
- 每秒记录一次 `GraspStatus` 字段不一致原因；
- 数字附着半径从 `0.13 m` 调整为 `0.16 m`，仅用于工程基线。

该调整尚未在无左臂碰撞的条件下完成最终复验。

### 4.3 资源压力

电脑只有 16 GiB 内存。完整栈同时运行 MuJoCo 渲染、MoveIt、TC-WAM 和相机时，
需要大量 swap。

当前策略：

- 基线实验使用 `publish_rgbd:=false`；
- 感知实验再开启 RGB-D；
- 批量实验关闭 RViz；
- Qwen、SAM2、GraspNet不要同时常驻；
- 每个 trial 结束后释放仿真进程。

## 5. CARE-Agent 的真实完成度

已实现的是 ROS-independent 决策内核：

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

尚未完成的是 ROS adapter：

```text
/thesis/care_agent/decide Action
BeliefState 从 ROS Topic 自动组装
技能 Action 的异步调用
失败结果到 FailureCode 的映射
恢复决策写回 mission orchestrator
决策轨迹 JSONL 审计
```

因此现在不能把 CARE-Agent 写成“已经在 MuJoCo 闭环运行”。准确表述应是：

> CARE-Agent 决策内核已完成，等待与 ROS 仿真技能层闭环集成。

## 6. 当前可复现启动方式

环境检查：

```bash
bash /home/wgb/openflex_shop/scripts/check_care_environment.sh
bash /home/wgb/openflex_shop/scripts/verify_moveit_basic.sh
```

无 MoveIt 离线栈：

```bash
bash /home/wgb/openflex_shop/scripts/run_thesis_offline.sh
```

低内存 MoveIt mission：

```bash
bash /home/wgb/openflex_shop/scripts/run_thesis_moveit_mission.sh
```

当前该 mission 仍可能在左臂联合碰撞处失败。这是已知工程阻塞，不是脚本参数错误。

## 7. 下一步验收门

必须按顺序通过：

| Gate | 验收条件 |
|---|---|
| G0 | 环境检查和 43 个相关测试通过 |
| G1 | MuJoCo 时钟、29 关节、TF 正常；需要感知时三路 RGB-D 正常 |
| G2 | 当前整机状态 `valid=True, contacts=0` |
| G3 | 找到在完整抓取升降范围内无碰撞的左臂停放姿态 |
| G4 | 固定底盘单次抓取成功，闭合和附着证据同时成立 |
| G5 | 运输姿态下物体仍保持附着 |
| G6 | 固定放置、释放和放置验证成功 |
| G7 | 导航、抓取、运输、放置单次端到端成功 |
| G8 | 10 次固定条件重复，记录每次结果，不筛选失败样本 |
| G9 | CARE-Agent ROS adapter 闭环 |
| G10 | 基线、消融和故障恢复正式实验 |

## 8. 论文使用边界

可以写：

- 分层技能建模；
- 物理能力评价；
- 结构化故障恢复；
- MuJoCo 数字实验结果；
- MoveIt IK 和碰撞评价；
- Oracle 条件下的系统集成前测。

暂时不能写：

- 真实 GraspNet 在仿真中已达到某成功率；
- 数字确定性附着代表真实摩擦抓取；
- CARE-Agent 已完成闭环验证；
- Nav2 已在所有主实验中替代比例导航基线；
- 单次成功代表统计显著。

