# OpenArmX 全身规划与预抓取接口

本文说明 `openarmx_integrated_moveit_config` 当前提供的两种全身规划入口：

1. RViz 的 MoveIt MotionPlanning 面板，用于人工交互式规划、检查和执行。
2. `/openflex/plan_pregrasp` Action，用于感知或上层任务程序传入 TCP 6D 目标后自动选择底盘、升降和单臂方案。

两种入口都依赖同一份机器人模型、`odom -> base_link` TF、MoveIt PlanningScene、关节限位和 `OpenflexKinematicsPlugin`。它们不会替换已有的 VR 遥操作、手动 RViz 面板、底盘控制器或手臂 position command 话题。

## 1. 范围与前提

当前全身规划只支持单臂任务：

| 规划组 | 自由度 | TCP |
| --- | --- | --- |
| `lift_only` | `lift_joint`，仅沿立柱轴上下移动 | `lift_carriage_link` |
| `left_whole_body` | 底盘 `(x, y, yaw)` + `lift_joint` + 左臂 7 轴 | `openarmx_left_hand` |
| `right_whole_body` | 底盘 `(x, y, yaw)` + `lift_joint` + 右臂 7 轴 | `openarmx_right_hand` |

所有全身 TCP 目标统一使用 `odom` 坐标系。底盘状态必须由动态 TF `odom -> base_link` 和当前 `/joint_states` 同步到 MoveIt。没有这两项时，MoveIt 无法正确计算全身 FK、碰撞和规划起点。

当前底盘执行接口是标准 `/navigate_to_pose`，但本地 `base_navigation_bridge.py` 是无地图的直接闭环桥：它根据 `/odom` 发布 `/cmd_vel`，不提供避障、地图或全局路径规划。接入 Nav2 后应关闭本桥，再由 Nav2 独占同一个 `/navigate_to_pose` Action 名称。

## 2. 启动

### 2.1 MuJoCo 仿真

先启动 MuJoCo，但关闭其 RViz；再启动 MoveIt 前端。这样只有一个 RViz 实例。

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash

ros2 launch openflex_mujoco_bringup all_body.launch.py use_rviz:=false
```

在另一个终端：

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash

ros2 launch openarmx_integrated_moveit_config sim_moveit.launch.py use_rviz:=true
```

`sim_moveit.launch.py` 会等待仿真已提供 `/joint_states` 与 `odom -> base_link`，随后启动：

- `move_group` 和 RViz MotionPlanning 面板。
- `moveit_command_bridge.py`，将 MoveIt 的手臂/升降轨迹接到既有控制话题。
- `base_navigation_bridge.py`，提供本地 `/navigate_to_pose`。
- `openflex_pick_task_executor`，提供 `/openflex/plan_pregrasp`。

只需要编程接口时可关闭 RViz：

```bash
ros2 launch openarmx_integrated_moveit_config sim_moveit.launch.py use_rviz:=false
```

### 2.2 真机或假硬件

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash

# 假硬件
ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=true

# 真机：省略 use_fake_hardware 或显式设为 false
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=false \
  use_rviz:=true
```

这个入口会自行 include `openarmx_integrated_bringup/integrated_robot_bringup.launch.py`，
因此真机模式不要再单独启动基础 bringup。重复启动会产生两个 `controller_manager`，
并争用同一 CAN 硬件和控制器资源。

启动前必须确认急停可用、CAN 接口配置正确、双臂周围无人员或障碍物，并确认升降
已经按现场流程回零。基础 bringup 默认 `auto_homing:=false`，不会因为启动 MoveIt
而自动回零；升降尚未可用时，禁止执行包含 `lift_joint` 或 `left/right_whole_body`
的轨迹。该真机入口默认启用双臂重力前馈补偿。

`sim_moveit.launch.py` 不会启动硬件或控制 CAN；它只能在 MuJoCo 已经提供
`/joint_states` 和 `odom -> base_link` 后使用。

两个启动入口都默认启动 Action 和本地底盘桥。需要由外部 Nav2 接管底盘时：

```bash
ros2 launch openarmx_integrated_moveit_config sim_moveit.launch.py \
  start_base_navigation_bridge:=false
```

之后再启动导航系统。不要同时运行本地桥和 Nav2，因为两者都会提供 `/navigate_to_pose`。

## 3. RViz 交互模式

### 3.1 操作步骤

1. 在 RViz 的 MotionPlanning 面板中将 `Planning Group` 选为 `left_whole_body` 或 `right_whole_body`。
2. 将 `Goal State` 设为 `Pose Goal`。
3. 拖动对应手部的 6D 交互 marker，目标在 `odom` 世界系下解释。
4. 点击 `Plan` 检查轨迹、底盘候选和碰撞显示。
5. 确认无误后点击 `Execute`，或直接点击 `Plan & Execute`。

`Plan` 只生成并显示轨迹，不会发送机器人命令。`Execute` 才会通过 MoveIt 控制器映射触发底盘目标与升降/手臂轨迹。

普通 `left_arm`、`right_arm`、`left_arm_with_lift`、`right_arm_with_lift` 组仍可按原有方式使用，适合固定底盘时的手臂或升降联动操作。这四个组也对应 Action 的运动范围：`left/right_arm` 为 7 DOF，`left/right_arm_with_lift` 为 8 DOF，`left/right_whole_body` 为完整 11 DOF。

`lift_only` 仅用于 RViz 单独控制立柱，不进入任务 Action。它使用 `lift_carriage_link` 交互标记和 1DOF 解析 IK；只有沿立柱轴的位移可解，横向位移和旋转会被拒绝。

### 3.2 RViz 全身 IK 的求解过程

RViz Pose Goal 调用 `OpenflexKinematicsPlugin`。它以本次请求的当前 `RobotState` 为 seed，不保存遥操作历史状态：

- 有限的越界实时状态 seed 会被裁剪到最近合法边界，解决编码器零点偏差导致的交互 IK 拒绝；
- 非有限状态（`NaN`/`inf`）仍会拒绝；
- 裁剪只作用于 IK 初始猜测，候选解仍必须严格满足 RobotModel 限位。

1. **Stage 0 路由**：以当前肩部为中心判断目标位置。它只决定优先路径，不会直接拒绝目标。
2. **Stage A**：固定当前底盘和 lift，使用官方 7 轴 KDL 求单臂完整位姿。
3. **Stage B**：固定当前底盘，使用官方 8 轴 KDL 求 `lift + arm`。
4. **Stage C**：仅在前两阶段不可解时执行。以当前底盘 yaw 为中心采样候选朝向，按每个朝向构造“目标落在机器人前方”的底盘位置；PickIK 求手臂姿态，再以 FK 解析补偿底盘 XY 和 lift Z。

Stage C 不会把 TCP 自身的 yaw 当成底盘朝向。TCP 姿态是抓取约束；没有任务级接近方向的通用 RViz Pose Goal，则以当前机器人朝向作为稳定的搜索中心。

每个候选都要通过关节限位、末端前方工作区、最终 TCP 误差和 MoveIt 碰撞 callback。当前生产参数中的最终 TCP 阈值为位置 `2 mm`、姿态 `2 deg`。

## 4. `/openflex/plan_pregrasp` Action

### 4.1 接口定位

`/openflex/plan_pregrasp` 面向上层任务调用，不依赖 RViz。虽然名称来自预抓取任务，它实际接收的是一个以 `odom` 表示的单臂 TCP 6D 目标；典型来源是货架物体识别、抓取生成器或脚本。

与通用 RViz Pose Goal 相比，Action 增加了任务级的底盘朝向选择和执行语义：

- 默认保持当前底盘朝向，或显式指定期望底盘 yaw。
- 对 yaw 候选前置构造“目标位于机器人前方”的底盘位置，避免手臂绕到身后抓取。
- `execute:=false` 只返回底盘候选、lift/arm 目标和 MoveIt 轨迹。
- `execute:=true` 先调用 `/navigate_to_pose` 到达底盘目标，再读取最新状态重新规划并执行升降和手臂。

Action 同一时刻只接受一个请求；执行或规划期间的新请求会被拒绝。可使用标准 ROS 2 Action 取消请求。

### 4.2 Goal 字段

| 字段 | 必填 | 默认或约束 | 含义 |
| --- | --- | --- | --- |
| `arm` | 是 | `LEFT=0`，`RIGHT=1` | 选择使用哪只手臂 |
| `max_dof` | 否 | `AUTO=0` 时采用 YAML 的 `11`；可为 `7`、`8`、`11` | 本次请求允许使用的最大运动范围，控制 A/B/C 的截断 |
| `tcp_pregrasp_pose` | 是 | `header.frame_id` 必须为 `odom`；四元数须非零且有限 | TCP 目标 6D 位姿 |
| `base_yaw_mode` | 否 | `CURRENT_STATE=0` | `0` 使用当前底盘 yaw；`EXPLICIT=1` 使用下列目标 yaw |
| `desired_base_yaw_odom` | 仅显式 yaw 时 | 有限弧度值 | 目标底盘在 `odom` 中的 yaw |
| `base_yaw_tolerance` | 否 | `0` 时采用 YAML `stage_c_yaw_range_rad`；最大值由 YAML `action_max_yaw_tolerance_rad` 限制 | 围绕中心 yaw 的采样半宽，单位 rad |
| `base_yaw_samples` | 否 | `0` 时采用 YAML 的 `stage_c_yaw_count=8`；最大值由 YAML `action_max_yaw_samples` 限制 | 包含中心 yaw 的候选数 |
| `front_distance` | 否 | `0` 时采用 YAML `0.45 m` | 构造候选时 TCP 到底盘中线前方距离 |
| `velocity_scale` | 否 | `0` 时采用 YAML `action_default_velocity_scale`；有效范围 `(0, 1]` | 轨迹速度缩放 |
| `acceleration_scale` | 否 | `0` 时采用 YAML `action_default_acceleration_scale`；有效范围 `(0, 1]` | 轨迹加速度缩放 |
| `planning_timeout_sec` | 否 | `0` 时采用 YAML `action_default_planning_timeout_sec` | MoveIt 规划时间上限 |
| `execute` | 否 | `false` | 是否执行底盘、升降与手臂 |

目前 `front_distance=0.45 m`、左/右 lateral offset 为 `+0.15/-0.15 m`。它们来自左右全身 IK 的 3000 个随机 FK 目标测评；横向偏置对应两肩中心距 305 mm 的一半。

`max_dof` 的状态机如下：

```text
7:  Stage 0 必须路由到 A；仅运行固定 base + lift 的 7 轴手臂规划。
8:  A 失败可进入 B；B 使用固定 base 的 lift + 手臂规划；不允许 C。
11: A -> B -> C、B -> C 或直接 C，允许完整底盘重定位。
```

当 Stage 0 已判断目标至少需要更多自由度，或当前阶段失败后下一阶段被 `max_dof` 禁止时，Action 返回 `MoveItErrorCodes::NO_IK_SOLUTION`，`message` 和 Feedback `dof_limit_reached` 会说明截断原因。

不填写 `max_dof` 等价于完整 `11` DOF。需要固定当前底盘和升降时传入 `max_dof: 7`；允许升降但禁止底盘重定位时传入 `max_dof: 8`。

### 4.3 最小 Plan-only 请求

下例使用左臂、保持当前机器人朝向，只规划不移动。`--feedback` 会显示 Stage 0 路由和候选统计。

```bash
ros2 action send_goal --feedback \
  /openflex/plan_pregrasp \
  openarmx_integrated_moveit_msgs/action/PlanPregrasp "{
    arm: 0,
    tcp_pregrasp_pose: {
      header: {frame_id: odom},
      pose: {
        position: {x: 1.10, y: 0.30, z: 0.65},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    execute: false
  }"
```

成功 Result 中的关键字段：

| 字段 | 含义 |
| --- | --- |
| `initial_route` | Stage 0 最初选择的 A、B 或 C 路由 |
| `selected_stage` | 最终候选使用的阶段 |
| `base_goal_odom` | 计算出的底盘 `(x, y, yaw)` 目标 |
| `lift_goal` | 最终升降高度 |
| `arm_goal` | 最终关节名称和位置 |
| `arm_trajectory` | MoveIt 已规划的升降/手臂轨迹 |
| `planning_time` | 轨迹规划耗时 |
| `message` / `error_code` | 成功、取消或失败原因 |

Feedback `phase` 的常见取值：`current_base_plan`、`stage_0_route_a`、`stage_0_route_b`、`stage_0_route_c`、`front_candidate_search`、`base_navigation`、`arm_replan`、`arm_execute`。

### 4.4 显式面向货架的请求

货架正面在 `odom` 中朝世界 X 正方向时，可令机器人 yaw 为 `0`。下例在 `+/-30 deg` 内采样 9 个底盘朝向，并执行。

```bash
ros2 action send_goal --feedback \
  /openflex/plan_pregrasp \
  openarmx_integrated_moveit_msgs/action/PlanPregrasp "{
    arm: 0,
    tcp_pregrasp_pose: {
      header: {frame_id: odom},
      pose: {
        position: {x: 1.10, y: 0.30, z: 0.65},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    base_yaw_mode: 1,
    desired_base_yaw_odom: 0.0,
    base_yaw_tolerance: 0.523599,
    base_yaw_samples: 9,
    front_distance: 0.45,
    velocity_scale: 0.30,
    acceleration_scale: 0.20,
    planning_timeout_sec: 5.0,
    execute: true
  }"
```

右臂仅需将 `arm: 1`，目标仍使用 `odom`。横向偏置由服务器按照左右臂自动选取，不需要调用方手动写入。

### 4.5 Action 内部流程

1. 读取当前 MoveIt `RobotState`，检查 `odom -> base_link` 与关节状态。
2. Stage 0 判断目标至少需要 7、8 或 11 DOF；超过 `max_dof` 时立即返回 `NO_IK_SOLUTION`。
3. Stage A 使用 `left/right_arm`，严格固定当前 base 和 lift；失败后仅当 `max_dof >= 8` 才进入 Stage B。
4. Stage B 使用 `left/right_arm_with_lift`，严格固定当前 base；失败后仅当 `max_dof = 11` 才进入 Stage C。
5. Stage C 在 `CURRENT_STATE` 或 `EXPLICIT` 的 yaw 中心附近采样前向底盘候选，以 `left/right_whole_body` 求解并通过 MoveIt PlanningScene 校验，再生成 lift+arm 关节轨迹。
6. 选择底盘平移距离最小、再按 yaw 改变量最小的有效候选。A/B 的执行直接发送已规划手臂轨迹；只有 C 会先调用 `/navigate_to_pose`，再从最新状态重规划 lift+arm。

因此，`execute:=true` 的底盘与手臂不是严格同步的全身时间轨迹：底盘先到候选位置，随后执行升降和手臂。这正适合当前无地图的“先摆位、后抓取”阶段。

## 5. 碰撞与环境物体

MoveIt PlanningScene 只会检查已加载的碰撞几何。当前启动入口使用机器人外壳碰撞模型；轮子、转向机构、传感器和内部支架不参与 MoveIt 自碰撞，避免视觉模型内部重叠造成错误碰撞。

Action 和 RViz 都会检查自碰撞。对货架、商品、桌面等环境障碍物，必须先将它们作为 `CollisionObject` 加入 PlanningScene，才会参与路径规划。当前预抓取 Action 本身不从 MuJoCo 自动导入超市场景。

在没有环境碰撞物体时，规划只保证机器人自身无碰撞和关节合法；它不保证底盘直移或手臂路径不会碰到 MuJoCo 中的货架。接入货架模型后，Action 的候选筛选和 MoveIt 轨迹规划会自动使用同一 PlanningScene。

## 6. 诊断与常见问题

### Action 不存在

```bash
ros2 action info /openflex/plan_pregrasp
```

若不存在，检查启动参数没有关闭 `start_pick_task_executor`，并确认 `move_group`、`/joint_states` 和 `odom -> base_link` 已就绪。

### 请求被立即拒绝

检查：

- `tcp_pregrasp_pose.header.frame_id` 是否严格为 `odom`。
- `arm` 是否为 `0` 或 `1`。
- `base_yaw_mode` 是否为 `0` 或 `1`；显式模式的 yaw 是否有限。
- `base_yaw_tolerance` 是否不大于 `pi/2`，`base_yaw_samples` 是否不大于 15。
- 当前是否已有另一个预抓取 Action 正在运行。

### Plan-only 成功但执行失败

执行前底盘会移动，随后 Action 会基于实际新状态重新规划手臂。因此执行阶段可能因 `/navigate_to_pose` 失败、状态变化、环境更新或重规划失败而终止。先用 `execute:=false` 检查 `base_goal_odom` 和轨迹；再检查 `/odom`、`/cmd_vel`、控制器和碰撞场景。

### RViz 目标在身后或规划失败

对于通用 RViz Pose Goal，Stage C 默认围绕当前底盘 yaw 搜索，不知道货架法向。需要指定“面向货架”的任务语义时，使用 Action 的 `EXPLICIT` yaw 模式。若目标超出前方工作区、关节限位或 PlanningScene 碰撞，插件会正确返回无解而不会输出无效关节解。

## 7. 参数来源

全身 IK、Stage C、Action 的共享数值参数都在：

```text
openarmx_integrated_moveit_config/config/kinematics.yaml
```

Action 固定读取 `left_whole_body.openflex_ik` 作为左右手共用的任务策略源，不在 C++ 中维护数值兜底；因此必须通过本包的正式 launch 启动。`right_whole_body.openflex_ik` 继续服务右臂 RViz 全身 IK，应与左侧共享参数保持一致。

重点参数位于 `left_whole_body.openflex_ik` 与 `right_whole_body.openflex_ik`：

- `stage_c_front_distance_m: 0.45`
- `stage_c_left_lateral_offset_m: 0.15`
- `stage_c_right_lateral_offset_m: -0.15`
- `stage_c_yaw_count: 8`
- `stage_c_seeds_per_yaw: 4`
- `stage_c_yaw_range_rad: 0.785398`
- `position_tolerance: 0.002`
- `rotation_tolerance_deg: 2.0`

修改 YAML 后重新构建并重启 MoveIt：

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select openarmx_integrated_moveit_config --symlink-install
source install/setup.bash
```

不要将这类 IK 参数写入 ros2_control；它们只影响 MoveIt 的求解和任务候选，不改变真实机器人底层控制器或 VR 遥操作数据流。
