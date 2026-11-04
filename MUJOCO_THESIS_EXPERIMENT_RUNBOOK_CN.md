# MuJoCo 论文实验工程手册

版本：1.0  
日期：2026-11-03  
适用论文：

> 《面向货架取放任务的能力感知分层具身智能体决策与恢复方法研究》

本手册只解决一个问题：如何从当前工程状态出发，按可验收的步骤完成论文实验。
每一步必须通过验收门后才能进入下一步。

## 0. 实验原则

1. 先打通确定性执行，再接入感知，再接入 Agent。
2. 先固定底盘，再做局部对位，再做导航闭环。
3. 先单次成功，再做 10 次稳定性，再做正式统计。
4. Oracle、GraspNet、SAM2、Qwen 的实验结果必须分开标注。
5. MuJoCo 中的确定性夹持代理不能写成真实接触抓取。
6. 不允许通过关闭碰撞检查、忽略失败样本或改变验收阈值来制造成功率。
7. 每次正式 trial 必须记录代码版本、配置、随机种子、成功阶段和失败原因。
8. 任何论文图表都必须能由冻结的 manifest 和逐 trial 原始日志重新生成。

## 1. 环境与目录

工作区：

```text
/home/wgb/openflex_shop
```

ROS 隔离构建空间：

```text
openflex_ws/build_v2
openflex_ws/install_v2
openflex_ws/log_v2
```

所有命令都在工作区根目录执行：

```bash
cd /home/wgb/openflex_shop
```

MuJoCo headless 必须使用 EGL：

```bash
export MUJOCO_GL=egl
```

本项目脚本已经设置该变量。不要依赖桌面 GL，也不要在未设置 EGL 时启动 headless
RGB-D。

## 2. G0：环境冻结与可复现性

### 2.1 检查系统

```bash
bash scripts/check_care_environment.sh
```

验收：

```text
mujoco_model 71 29 23
care_environment_ok
```

### 2.2 检查 MoveIt

```bash
bash scripts/verify_moveit_basic.sh
```

验收：

```text
MoveIt基础环境验证通过
```

### 2.3 运行测试

```bash
source /opt/ros/humble/setup.bash
source openflex_ws/install_v2/setup.bash
colcon --log-base openflex_ws/log_v2 test \
  --base-paths openflex_ws/src/openflex_thesis \
  --build-base openflex_ws/build_v2 \
  --install-base openflex_ws/install_v2 \
  --packages-select \
  openflex_care_agent \
  openflex_tc_wam \
  openflex_thesis_control \
  openflex_thesis_experiments \
  openflex_thesis_mujoco
```

验收：

```text
care_agent 9 passed
tc_wam 15 passed
thesis_control 7 passed
thesis_experiments 7 passed
thesis_mujoco 5 passed
```

失败时先修复，不要进入仿真实验。

## 3. G1：MuJoCo 平台验收

### 3.1 启动无 MoveIt 离线栈

终端 A：

```bash
bash scripts/run_thesis_offline.sh
```

如需关闭 RGB-D 以减少内存：

```bash
bash scripts/run_thesis_offline.sh publish_rgbd:=false
```

### 3.2 验证基础话题

终端 B：

```bash
source /opt/ros/humble/setup.bash
source openflex_ws/install_v2/setup.bash
ros2 topic hz /clock
ros2 topic echo --once /joint_states
ros2 topic echo --once /swerve_drive_controller/odom
ros2 run tf2_ros tf2_echo map base_link
```

验收：

- `/clock` 持续推进；
- `/joint_states` 包含 29 个关节；
- 底盘 odom 连续；
- `map -> odom -> base_link` 可查询；
- 不出现重复时间戳或丢失 TF。

### 3.3 验证三路 RGB-D

开启 RGB-D 后检查：

```bash
ros2 topic hz /thesis_sim/head/color/image_raw
ros2 topic hz /thesis_sim/head/aligned_depth_to_color/image_raw
ros2 topic hz /thesis_sim/left_wrist/color/image_raw
ros2 topic hz /thesis_sim/right_wrist/color/image_raw
```

验收：

- RGB 为 `rgb8`；
- Depth 为 `16UC1`；
- 分辨率与配置一致；
- 三路 CameraInfo 正常；
- 时间戳严格递增。

## 4. G2：MoveIt 与整机状态验收

启动仿真和 MoveIt：

```bash
bash scripts/run_thesis_moveit_mission.sh run_mission:=false
```

这里使用 mission 脚本但关闭自动 mission，只保留 MuJoCo 和 MoveIt。

验证 IK：

```bash
bash scripts/test_moveit_ik.sh
```

验证实际运动：

```bash
bash scripts/test_moveit_move.sh
```

验证整机碰撞状态：

```bash
source /opt/ros/humble/setup.bash
source openflex_ws/install_v2/setup.bash
python3 scripts/check_moveit_current_state.py --group right_arm_with_lift
```

验收：

```text
compute_ik success=True
move_group success=True
group=right_arm_with_lift valid=True contacts=0
```

如果 `valid=False`，先使用诊断脚本：

```bash
bash scripts/diagnose_moveit_candidate.sh
```

不要用 `avoid_collisions=true` 隐藏问题。

## 5. G3：左臂停放与升降范围标定

这是当前项目的第一个未通过验收门。

### 5.1 目标

找到一组固定左臂姿态：

```text
left_park = [q1, q2, q3, q4, q5, q6, q7]
```

要求在论文实际使用的升降范围内：

- 左臂与底盘无碰撞；
- 左臂与轮组无碰撞；
- 左右臂无交叉；
- 不进入货架碰撞体；
- 距离关节限位有足够余量；
- 不允许在任务过程中移动左臂。

### 5.2 正确搜索方法

搜索时的机器人状态不能是“左臂姿态 + 右臂零位 + 任意 lift”，因为右臂零位本身
在部分升降高度会碰撞。

正确流程：

1. 从 `/thesis/tc_wam/candidates` 取一个真实抓取候选的
   `grasp_joint_positions`；
2. 该状态同时包含实际的 lift 和右臂解；
3. 固定该右臂和 lift；
4. 枚举左臂 `q1..q4`，其余关节保持 0；
5. 调用 `/check_state_validity`；
6. 保存所有 `valid=True, contacts=0` 的候选；
7. 对每个候选增加 lift 扰动和右臂候选扰动；
8. 选择在所有测试状态中都不碰撞、且关节余量最大的姿态；
9. 将该姿态写入 `mujoco.yaml` 的 `inactive_arm_park_positions`；
10. 重启 MuJoCo，确认实际 `/joint_states` 中的左臂位置接近配置值。

现有工具：

```bash
python3 scripts/search_moveit_left_park.py
```

当前版本的搜索仍需绑定真实右臂候选状态。不要在未完成此修正时把搜索结果写进论文。

### 5.3 验收

至少验证以下 lift 状态：

```text
-0.15 m
0.00 m
+0.15 m
```

如果抓取算法会使用更大升降范围，则全部加入标定集。

验收记录必须包含：

```text
left_park
right_reference_joint_positions
lift_positions
contact_pairs
joint_margins
```

## 6. G4：固定底盘抓取

### 6.1 固定条件

第一轮基线只允许一个物体、一个货架层、一个抓取方向：

```text
object_id: thesis_item_01
object_pose: [1.12, 0.0, 0.84, pi]
base_pose: 固定
active_arm: 2
perception: Oracle
evaluator: MoveIt
local_policy: scripted baseline
```

### 6.2 运行

```bash
bash scripts/run_thesis_moveit_mission.sh
```

观察：

```bash
ros2 topic echo /thesis/mission_state
ros2 topic echo /thesis/grasp_status
ros2 topic echo /thesis/tc_wam/candidates
```

### 6.3 成功证据

必须同时满足：

```text
TC-WAM candidate total_score > 0
candidate.collision_free = true
MoveIt pregrasp trajectory SUCCESS
MoveIt grasp trajectory SUCCESS
gripper_closed = true
object_retained = true
verified = true
```

只有“规划成功”不算抓取成功。

只有“日志里出现 attached”也不算统计成功，必须从 ROS 验证状态中读到对应字段。

### 6.4 10 次稳定性

单次通过后执行 10 次固定条件重复：

- 每次都重启仿真或调用 reset；
- 不跳过失败；
- 记录每个 trial 的 `success`、失败阶段和错误码；
- 10 次成功率只作为稳定性检查，不作为论文最终统计结论。

## 7. G5：运输姿态与附着保持

抓取后需要验证：

```text
lift = -0.10 m
right arm = transport positions
left arm = negotiated park
gripper closed
object_retained = true
transport_safe = true
```

检查：

```bash
ros2 topic echo --once /thesis/grasp_status
```

验收：

- 运输姿态 MoveIt 轨迹成功；
- 姿态改变后物体仍附着；
- 左臂仍无碰撞；
- 底盘保持 BaseLock；
- 不允许在 MoveIt 执行过程中同时发布底盘速度。

## 8. G6：固定放置与释放

先固定底盘，不导航，只验证：

```text
运输姿态
-> 放置目标
-> 夹爪打开
-> 物体脱离
-> PlaceStatus release_verified = true
```

验收：

- 放置目标在桌面有效区域；
- 夹爪打开后物体脱离；
- 无穿透；
- `place/status` 明确给出 release verified；
- 物体不再被 weld 或确定性附着逻辑拖走。

## 9. G7：导航和完整任务

只有 G4-G6 均通过后，才恢复完整导航：

```text
观察区导航
-> 目标观测
-> 粗抓取评价
-> 局部对位
-> ready region 绑定
-> 近场重观测
-> MoveIt 抓取
-> 运输
-> 交付点导航
-> 固定放置
```

阶段一可继续使用 `sim_navigation` 比例导航基线，不能把它称为 Nav2。

阶段二切换到 Nav2，并通过以下指标比较：

- 到达成功率；
- 路径长度；
- 到达时间；
- 最终位置/朝向误差；
- 重规划次数；
- 安全停车次数。

## 10. G8：从 Oracle 切换到真实感知

Oracle 只用于打通故障隔离链路。正式感知实验必须改为：

```text
RGB-D
-> 目标检测/分割
-> 实例点云
-> GraspNet
-> 抓取候选
```

推荐分阶段：

| 阶段 | 目标定位 | 抓姿 | 用途 |
|---|---|---|---|
| P0 | Oracle | Oracle | 执行链验证 |
| P1 | Oracle | GraspNet | 抓姿网络验证 |
| P2 | SAM2/Qwen | GraspNet | 感知闭环 |
| P3 | 实际 RGB-D 扰动 | GraspNet | 泛化实验 |

所有阶段都必须输出：

```text
physical_perception
physical_evaluation
source
confidence
inference_time
```

如果结果是 Oracle，论文中必须明确标为 `non-physical`。

## 11. G9：TC-WAM 能力评价实验

对每个候选保存：

```text
base pose
lift
grasp id
IK success
collision-free
joint margin
perturbation success rate
planning time
ready region id
```

主比较：

1. 固定底盘 + 仅抓姿；
2. 只评价物体抓取；
3. 联合评价 base + lift + arm + grasp；
4. 加入扰动鲁棒性；
5. 加入 cost-aware selection。

论文要回答：

- 物理评价是否减少不可执行动作；
- 是否减少 MoveIt 下游失败；
- 是否提高货架边缘和偏移场景成功率；
- 增加的时间成本是否可接受。

## 12. G10：CARE-Agent ROS 闭环

当前 CARE-Agent 只有离线决策内核，下一步必须增加 ROS adapter：

```text
BeliefState 组装器
-> /thesis/care_agent/decide Action
-> 技能 Action 调用
-> SkillOutcome
-> FailureCode
-> RecoveryPolicy
-> 下一次技能选择
```

先接三个技能：

```text
reobserve
recompute_capability
realign
```

再增加：

```text
navigate
grasp
transport
place
abort_hold
```

Agent 不许直接发：

```text
Twist
JointTrajectory
IK seed
电机命令
```

高层的所有输出必须映射到注册技能。

## 13. G11：故障注入

至少实现以下事件：

| 故障 | 注入方式 |
|---|---|
| 目标被遮挡 | MuJoCo 遮挡体或观测 visible=false |
| 目标移动 | 随机改变物体 pose |
| 地图版本失效 | 改变对象记忆版本 |
| ready region 失效 | 扰动底盘或抓取绑定 |
| 抓姿不可达 | 加入底盘偏移和货架边缘 |
| 夹爪闭合未附着 | 调小附着判定或注入失败状态 |
| 运输姿态不可行 | 改变运输高度/碰撞体 |
| 放置位置被占用 | 改变交付槽位 |
| 传感器陈旧 | 停止某个输入或增大延迟 |
| 控制权冲突 | 仿真中保持非法模式请求 |

每种故障必须可复现，并有唯一故障码。

## 14. G12：正式论文实验

实验顺序：

1. E0 平台可复现性；
2. E1 技能契约与安全壳单元实验；
3. E2 固定状态机基线；
4. E3 单故障恢复；
5. E4 复合故障恢复；
6. E5 消融实验；
7. E6 场景泛化；
8. E7 时间和资源；
9. E8 完整端到端。

比较组至少包括：

| 方法 | 说明 |
|---|---|
| FS | 固定状态机 |
| CARE-R | 规则效用恢复 Agent |
| CARE-D | 校准数据驱动恢复 Agent |
| CARE-no-ability | 去掉 TC-WAM |
| CARE-no-recovery | 去掉故障恢复 |
| flat-agent | 平面动作空间 Agent |

每次 trial 至少记录：

```text
trial_id
split
experiment
method
scene_id
seed
git_commit
config_hash
phase_failures
total_success
recovery_count
recovery_time
planning_time
inference_time
safety_violation
stale_plan_used
grasp_verified
transport_safe
place_verified
```

统计结果：

- 比率指标使用 Wilson 95% 置信区间；
- 配对 trial 使用 McNemar 或 bootstrap；
- 连续指标报告中位数、IQR 和均值；
- 时间、成功率、安全违规分别报告；
- 失败样本必须保留并分类。

## 15. 推荐执行顺序

按以下顺序推进，不跳步：

1. 修复左臂在不同 lift 下的安全停放。
2. 完成固定底盘的第一次抓取、附着和运输。
3. 完成固定底盘 10 次重复。
4. 完成固定放置。
5. 恢复局部对位和导航闭环。
6. 用 Oracle 完成一次完整 pick-and-place。
7. 接入 GraspNet。
8. 接入 SAM2/目标检测。
9. 完成 TC-WAM 对比实验。
10. 接入 CARE-Agent ROS adapter。
11. 完成故障注入。
12. 运行 E2-E8 正式实验。
13. 冻结版本并生成论文表格、图和附录。

## 16. 每次实验开始前的检查清单

```text
[ ] 没有旧仿真节点残留
[ ] source 了 install_v2
[ ] MUJOCO_GL=egl
[ ] 场景配置 hash 已记录
[ ] 当前任务只使用一个底盘 owner
[ ] 当前任务只使用一个 MoveIt owner
[ ] 左臂停放姿态属于当前配置
[ ] 实验方法是 Oracle 还是真实感知已经明确
[ ] 本次 trial_id 来自冻结 manifest
[ ] 失败样本不会被人工丢弃
```

## 17. 每次实验结束后的检查清单

```text
[ ] trial result 已落盘
[ ] 失败阶段和错误码已记录
[ ] 关节/底盘/物体状态已保存
[ ] 代表性失败有 RGB-D 或 rosbag
[ ] 仿真已正常停止
[ ] 未把数字附着写成真实抓取
[ ] 结果可从 manifest 重跑
```

## 18. 当前第一步

现在不要直接跑全量实验。首先完成：

```text
TC-WAM 实际候选
-> 固定其右臂和 lift
-> 搜索左臂安全姿态
-> 写入 MuJoCo 配置
-> 单次固定底盘抓取
```

这一步通过后，论文工程基线才真正开始。

