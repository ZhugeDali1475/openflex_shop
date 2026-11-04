# openarmx_gripper_panel

[English](./README.md) | 中文

---

通过 GripperCommand 动作接口控制 OpenArmX 夹爪的 RViz2 插件面板。

## 概述

本包提供一个 RViz2 面板插件，允许用户通过图形界面控制 OpenArmX 机器人的左右夹爪。使用 `control_msgs/action/GripperCommand` 动作向夹爪控制器发送位置指令。

## 功能特性

- 选择左夹爪、右夹爪或双夹爪（同步）
- 位置滑块（0-44 mm 范围）
- 快捷预设按钮：关闭（0 mm）、半开（22 mm）、打开（44 mm）
- 应用按钮执行指令
- 连接状态指示
- 夹爪选择跨 RViz 会话持久保存

## 使用方法

### 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_gripper_panel
source install/setup.bash
```

### 在 RViz 中添加

1. 打开 RViz2
2. 进入 Panels > Add New Panel
3. 从 openarmx_gripper_panel 插件中选择 "GripperPanel"
4. 面板出现在侧边栏

### 需要的动作服务器

面板连接以下动作服务器：

| 动作服务器 | 类型 | 描述 |
|-----------|------|------|
| `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 左夹爪控制 |
| `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 右夹爪控制 |

## 前置条件

- OpenArmX 机器人以 `joint_trajectory_controller` 模式启动（该模式会启动夹爪控制器）
- 夹爪动作服务器可用

## 依赖

- `rclcpp`、`rclcpp_action`
- `control_msgs`
- `rviz_common`、`rviz_default_plugins`
- `pluginlib`
- Qt5（Core、Widgets）

## 许可证

CC-BY-NC-SA-4.0 - 成都长树机器人有限公司
