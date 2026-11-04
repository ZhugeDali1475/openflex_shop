# P6 右臂现场 Plan-only 链路验收

日期：2026-08-12

## 安全边界

- 只读取右腕相机、TF、RobotState，并执行模型推理和 MoveIt 规划。
- `/openflex_pick_object.execute=false`。
- `PickObject.max_dof=8`，只允许手臂与升降参与规划，不包含底盘。
- 未调用夹爪 Action、轨迹执行、关节姿态执行、导航或 `cmd_vel`。

## 现场输入

- 右腕 D405：`260522279851`。
- RGB-D：`480x270x15`，彩色与对齐深度均约 15 Hz。
- 检测目标：`画面中央最近处的水蜜桃`，用于区分画面中两个同类实例。

## 结果

- `/detect_object`：成功，置信度 `0.7394968271`，目标中心位于
  `right_camera_color_optical_frame`，约 `(0.0151, 0.0444, 0.1670) m`。
- GraspNetBaseline：成功，夹爪需求宽度约 `0.057 m`。
- PlanPregrasp：成功，使用 TCP link，允许范围为 `ARM_WITH_LIFT=8`。
- CartesianApproach：成功，使用预抓取规划末态作为 plan-only 起始状态。
- `/pick_object` 最终结果：`success=true`、`object_attached=false`、
  `error_code=0`、`stage=1`、`candidates=49`。
- 返回消息明确为 `no hardware command was sent`。

## 本次修复

- Pick adapter 不再用 `asyncio.wait_for()` 包装 `rclpy` Future。
- GraspNet 接近轴和闭合轴从 `base_link` 经 TF 转换到点云帧。
- 按负责人决定，`enable_corridor_check=false`，取消 GraspNet 前置点云走廊计数；
  GraspNet 场景推理、夹爪宽度检查、MoveIt 碰撞规划和 Cartesian 路径检查仍保留。
- plan-only Cartesian 接收 PlanPregrasp 末态，只传左右臂实际关节和可选 `lift_joint`。
- PlanPregrasp 与 CartesianApproach 统一使用左右 `openarmx_*_hand_tcp`。
- 重建 `CartesianApproach.action` 安装态，并修复 Humble 下预测起始状态校验。

## 未覆盖

- 左腕/左臂 plan-only。
- 任何 `execute=true` 真机轨迹、夹爪动作和底盘运动。
