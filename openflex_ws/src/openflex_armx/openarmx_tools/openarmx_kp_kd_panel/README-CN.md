# openarmx_kp_kd_panel

[English](./README.md) | 中文

---

动态调整 OpenArmX 机器人关节 KP（刚度）和 KD（阻尼）参数的 RViz2 面板插件。

## 概述

本包提供一个 RViz2 面板，允许在运行时调整 OpenArmX 硬件接口的 KP 和 KD 电机控制参数。通过 ROS 2 参数服务与硬件参数节点通信，为每臂的全部 8 个关节（7 个臂关节 + 1 个夹爪）设置参数值。

## 功能特性

- 统一的臂关节（1-7）滑块控制，自动按电机类型映射
- 夹爪（关节 8）独立的 KP/KD 滑块
- 手臂选择：仅右臂、仅左臂或双臂同时
- 自动电机类型映射：
  - 关节 1-2（RS04）：KP 范围 0-5000，KD 范围 0-100
  - 关节 3-4（RS03）：KP 范围 0-5000，KD 范围 0-100
  - 关节 5-7（RS00）：KP 范围 0-500，KD 范围 0-5
  - 关节 8/夹爪（RS00）：KP 范围 0-500，KD 范围 0-5
- 每个参数组的恢复默认按钮
- 仿真模式检测（fake hardware 激活时禁用控件）
- 参数应用时的可视化确认对话框

## 使用方法

### 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_kp_kd_panel
source install/setup.bash
```

### 在 RViz 中添加

1. 打开 RViz2
2. 进入 Panels > Add New Panel
3. 从 openarmx_kp_kd_panel 插件中选择 "KpKdPanel"

### 目标参数节点

面板向以下节点设置参数：

| 节点 | 描述 |
|------|------|
| `/openarmx_right_hardware_params` | 右臂硬件参数节点 |
| `/openarmx_left_hardware_params` | 左臂硬件参数节点 |

### 设置的参数

对每臂设置：
- `kp_joint1` 到 `kp_joint8` - 位置增益（刚度）
- `kd_joint1` 到 `kd_joint8` - 速度增益（阻尼）

## 前置条件

- OpenArmX 机器人以真实硬件运行（MIT 控制模式）
- 硬件参数节点活跃（`openarmx_right_hardware_params`、`openarmx_left_hardware_params`）
- 不支持仿真/fake hardware 模式

## 依赖

- `rclcpp`
- `rviz_common`、`rviz_rendering`
- `pluginlib`
- Qt5

## 许可证

CC-BY-NC-SA-4.0 - 成都长树机器人有限公司
