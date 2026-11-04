# openarmx_joint_slider_panel

[English](./README.md) | 中文

---

通过滑块控制 OpenArmX 双臂关节和夹爪的 RViz2 面板插件。

## 概述

本包提供一个 RViz2 面板，允许通过滑块直接进行关节级控制。向 `forward_position_controller` 话题发布位置指令，支持可配置步长的插值运动，确保平滑安全。

## 功能特性

- 14 个机械臂关节（左臂 7 + 右臂 7）和 2 个夹爪的独立滑块
- 从 URDF 自动加载关节限位（通过 robot_description 参数或话题）
- 可配置关节和夹爪步长的分步运动执行
- "从 /joint_states 同步" 按钮，使滑块匹配当前机器人姿态
- "Home 回零" 按钮，将所有关节回到零位
- 航点记录/回放（存储在 RViz 配置中）
- 基于 TF 的实时半透明模型预览，显示目标姿态
- 首次接收到 joint_states 时自动同步

## 使用方法

### 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_joint_slider_panel
source install/setup.bash
```

### 在 RViz 中添加

1. 打开 RViz2
2. 进入 Panels > Add New Panel
3. 从 openarmx_joint_slider_panel 插件中选择 "JointSliderPanel"

### 话题

#### 订阅

| 话题 | 类型 | 描述 |
|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 用于同步和初始化的当前关节位置 |
| `/robot_description` | `std_msgs/String` | 用于关节限位和预览模型的 URDF（transient_local） |

#### 发布

| 话题 | 类型 | 描述 |
|------|------|------|
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 左臂 + 夹爪指令 |
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 右臂 + 夹爪指令 |
| `/tf` | `tf2_msgs/TFMessage` | 预览模型变换（前缀为 "preview_"） |

## 控件说明

- **关节步长滑块**（1-200 mrad）：控制每周期的机械臂关节步长
- **夹爪步长滑块**（0.1-10.0 mm）：控制每周期的夹爪步长
- **同步按钮**：读取当前 /joint_states 并应用到滑块
- **Home 按钮**：将所有关节移到零位（需确认）
- **新增记录**：保存当前机器人姿态为航点
- **运动到此点**：执行分步运动到已保存的航点

## 关节名称

左臂：`openarmx_left_joint1` 到 `openarmx_left_joint7`  
右臂：`openarmx_right_joint1` 到 `openarmx_right_joint7`  
夹爪：`openarmx_left_finger_joint1`、`openarmx_right_finger_joint1`

## 前置条件

- 机器人以 `forward_position_controller` 模式运行
- `/joint_states` 话题活跃

## 依赖

- `rclcpp`、`sensor_msgs`、`std_msgs`、`geometry_msgs`
- `rviz_common`、`pluginlib`
- `tf2_ros`、`kdl_parser`、`urdf`
- Qt5（Core、Widgets）

## 许可证

OpenArmX 研究和教育许可证 - 成都长树机器人有限公司
