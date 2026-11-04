# OpenFlex MoveIt

OpenFlex 的 MoveIt2 源码集合，包含全身运动规划实现和供感知、任务系统调用的 ROS 2 Action 接口。

```text
openflex_moveit/
  openarmx_integrated_moveit_config/  MoveIt 配置、全身 IK、RViz 与任务执行器
  openarmx_integrated_moveit_msgs/    /openflex/plan_pregrasp Action 接口定义
```

## 正式能力

1. RViz MotionPlanning 面板：人工设定左、右单臂 TCP Pose，进行全身 IK、碰撞检查、规划和执行。
2. `/openflex/plan_pregrasp` Action：感知或上层任务传入 `odom` 坐标系中的 TCP 6D 目标，按 `max_dof=7/8/11` 自动限制手臂、升降或完整底盘方案，可只规划或执行。

`openarmx_integrated_moveit_config` 负责 MoveIt、IK Pipeline、控制器执行桥和 Action Server。`openarmx_integrated_moveit_msgs` 只定义 Action 类型，使感知或任务包可独立依赖接口而无需依赖完整 MoveIt 实现。

## 启动

MuJoCo 仿真已启动后接入 MoveIt：

```bash
ros2 launch openarmx_integrated_moveit_config sim_moveit.launch.py use_rviz:=true
```

假硬件：

```bash
ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=true
```

真机由同一个入口完整启动，不需要、也不能在另一个终端额外启动
`integrated_robot_bringup.launch.py`。该 MoveIt 入口会自行包含真实硬件
bringup、`controller_manager`、状态发布、MoveIt 执行桥和 RViz：

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=false \
  use_rviz:=true
```

真机启动前确认 CAN 设备、急停、机械臂活动空间和升降零位均正常。该入口默认
不会自动执行升降回零；未完成回零时，不要执行包含 `lift_joint` 或底盘全身规划的
轨迹。`sim_moveit.launch.py` 只连接已经运行的 MuJoCo，不能用于启动或控制真实硬件。

完整启动说明、Action 请求字段、Stage A/B/C 全身 IK 策略、碰撞范围和 Nav2 替换方式见：

- [MoveIt 配置包 README](openarmx_integrated_moveit_config/README.md)
- [全身规划中文说明](openarmx_integrated_moveit_config/WHOLE_BODY_PLANNING_CN.md)
- [PlanPregrasp Action 定义](openarmx_integrated_moveit_msgs/action/PlanPregrasp.action)

## 构建

```bash
cd ~/openflex_all/openflex_ws
colcon build --packages-select \
  openarmx_integrated_moveit_msgs \
  openarmx_integrated_moveit_config \
  --symlink-install
source install/setup.bash
```
