# openarmx_gripper_panel

English | [中文](./README-CN.md)

---

RViz2 plugin panel for controlling OpenArmX grippers via the GripperCommand action interface.

## Overview

This package provides an RViz2 panel plugin that allows users to control the left and right grippers of the OpenArmX robot through a graphical interface. It uses the `control_msgs/action/GripperCommand` action to send position commands to the gripper controllers.

## Features

- Select left gripper, right gripper, or both (synchronized)
- Position slider (0-44 mm range)
- Quick preset buttons: Close (0 mm), Half (22 mm), Open (44 mm)
- Apply button to execute commands
- Connection status indicator
- Persistent gripper selection across RViz sessions

## Usage

### Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_gripper_panel
source install/setup.bash
```

### Add to RViz

1. Open RViz2
2. Go to Panels > Add New Panel
3. Select "GripperPanel" from the openarmx_gripper_panel plugin
4. The panel appears in the sidebar

### Action Servers Required

The panel connects to these action servers:

| Action Server | Type | Description |
|---------------|------|-------------|
| `/left_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Left gripper control |
| `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | Right gripper control |

## Prerequisites

- OpenArmX robot launched with `joint_trajectory_controller` mode (which spawns gripper controllers)
- Gripper action servers available

## Dependencies

- `rclcpp`, `rclcpp_action`
- `control_msgs`
- `rviz_common`, `rviz_default_plugins`
- `pluginlib`
- Qt5 (Core, Widgets)

## License

CC-BY-NC-SA-4.0 - Chengdu Changshu Robot Co., Ltd.
