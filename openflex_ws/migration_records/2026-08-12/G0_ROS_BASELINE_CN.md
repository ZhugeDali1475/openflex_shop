# G0 ROS 与 owner 基线

采集时间：2026-08-12 09:33 CST。

## 当前运行图

在 source `/opt/ros/humble/setup.bash` 后执行 `ros2 node/action/service/topic list`：

```text
业务 node：无
Action：无
业务 Service：无
Topic：/parameter_events、/rosout
```

因此这是离线基线，不是运行时系统验收。当前无法采集 live TF tree、控制器状态或 Action server 数量；这些证据必须在 D0 静态 TF、G3 Action 和 G7 启动关卡重新采集。

## 从现有代码确认的 owner

- `openarmx_integrated_moveit_config/launch/integrated_robot_bringup_moveit.launch.py` 创建唯一 `move_group`。
- 同一 launch 可创建 `/openflex/plan_pregrasp` 和 `/openflex/execute_joint_posture` 的实现节点。
- `openarmx_integrated_bringup` 创建左右轨迹/夹爪 controller spawner。
- `supermarket_fullbody_runtime.launch.py` 是正式全身比赛 runtime 入口。
- 当前目标没有真实 `/detect_object` 或 `/pick_object` server；仿真包有对应 mock/sim server。

## 运行时待补证据

- 左右 `link7 -> camera_link -> color_frame -> color_optical_frame` 的发布者和 TF 唯一性
- `/controller_manager` 中左右臂、夹爪和升降控制器状态
- MoveIt planning group、唯一 `move_group` 和唯一 PlanPregrasp server
- 正式/调试抓取后端互斥与进程 owner
