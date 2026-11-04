# OpenArmX Integrated MoveIt Configuration

OpenArmX 集成机器人的生产 MoveIt2 配置包。它负责底盘、升降和单臂的全身规划，并保留已经验证的底层控制与 VR 遥操作话题。

## 正式能力

1. RViz MotionPlanning 面板：人工设定单臂 TCP Pose，进行全身 IK、碰撞检查、规划与执行。
2. `/openflex/execute_joint_posture` Action：执行白名单规划组的完整关节姿态，用于货架观察和安全运输等确定性示教姿态。
3. `/openflex/plan_pregrasp` Action：上层任务传入 `odom` 中的 TCP 6D 目标，按 `max_dof=7/8/11` 限定手臂、升降或完整底盘方案；可只规划或执行。

全身组为 `left_whole_body` 与 `right_whole_body`，每组包含底盘 `(x, y, yaw)`、`lift_joint` 和对应手臂 7 轴。底盘参考链为动态 `odom -> base_link`。RViz 还提供 `lift_only` 组，通过 `lift_carriage_link` 的单轴交互标记和解析 IK 单独控制立柱升降。

完整中文操作、Action 字段、Stage A/B/C 原理、碰撞范围、诊断及 Nav2 替换方式见 [WHOLE_BODY_PLANNING_CN.md](WHOLE_BODY_PLANNING_CN.md)。

## 启动

MuJoCo 已启动后接入 MoveIt：

```bash
ros2 launch openarmx_integrated_moveit_config sim_moveit.launch.py use_rviz:=true
```

假硬件：

```bash
ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=true
```

真机直接启动同一入口：

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=false \
  use_rviz:=true
```

`integrated_robot_bringup_moveit.launch.py` 已经包含
`openarmx_integrated_bringup/integrated_robot_bringup.launch.py`，会创建真实硬件
控制器、`controller_manager`、状态发布和 MoveIt 执行链。不要先或并行单独启动
`integrated_robot_bringup.launch.py`，否则两个 launch 会争用同一 ROS 控制器和 CAN
硬件资源。

真机前置条件是 CAN 接口、急停、手臂活动空间和升降零位已经确认。底层 bringup
默认 `auto_homing:=false`，所以启动后仍需按现场安全流程使升降可用；在升降未回零
前不得执行包含 `lift_joint` 或 whole-body 的轨迹。真实硬件链路默认会启用双臂
重力前馈补偿。`sim_moveit.launch.py` 只等待 MuJoCo 提供 `/joint_states` 与
`odom -> base_link`，它不会启动真实硬件。

两个入口默认启动 `move_group`、MoveIt 执行桥、`/navigate_to_pose` 本地桥、`/openflex/execute_joint_posture` 和 `/openflex/plan_pregrasp` Action。仅使用编程接口时传入 `use_rviz:=false`。

## 运行结构

```text
launch
  sim_moveit.launch.py
  integrated_robot_bringup_moveit.launch.py
  surface_collision_profile.py
scripts
  moveit_command_bridge.py
  base_navigation_bridge.py
  wait_for_robot_state.py
src / include
  OpenflexKinematicsPlugin
  OpenflexIkPipeline
  OpenArmXControllerManager
  openflex_joint_posture_executor
  openflex_pick_task_executor
config
  MoveIt SRDF, IK, planning, controller, RViz and sensor configuration
```

## 唯一开发工具

`generate_pose_dataset` 保留为唯一的 IK 测试数据生成器。它使用权威 xacro、SRDF 和 MoveIt PlanningScene 生成无自碰撞的 FK TCP 目标；不属于运行时启动链路。

```bash
ros2 launch openarmx_integrated_moveit_config generate_pose_dataset.launch.py \
  output_path:=/tmp/openflex_left_arm_targets.jsonl \
  group_name:=left_arm \
  tip_link:=openarmx_left_hand
```

## 构建

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select openarmx_integrated_moveit_config --symlink-install
source install/setup.bash
```

许可证与第三方归属见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
