# openarmx_joint_slider_panel

English | [中文](./README-CN.md)

---

RViz2 panel plugin with sliders to control OpenArmX bimanual joints and grippers.

## Overview

This package provides an RViz2 panel that allows direct joint-level control of the OpenArmX bimanual robot via sliders. It publishes position commands to `forward_position_controller` topics with configurable step-size interpolation for smooth, safe motion.

## Features

- Individual sliders for all 14 arm joints (7 left + 7 right) and 2 grippers
- Automatic joint limit loading from URDF (via robot_description parameter or topic)
- Stepped motion execution with configurable joint and gripper step sizes
- "Sync from /joint_states" button to match sliders to current robot pose
- "Home" button to return all joints to zero
- Record/playback of waypoints (stored in RViz config)
- Real-time TF-based ghost model preview showing target pose
- Auto-sync on first joint_states reception

## Usage

### Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_joint_slider_panel
source install/setup.bash
```

### Add to RViz

1. Open RViz2
2. Go to Panels > Add New Panel
3. Select "JointSliderPanel" from the openarmx_joint_slider_panel plugin

### Topics

#### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions for sync and initialization |
| `/robot_description` | `std_msgs/String` | URDF for joint limits and preview model (transient_local) |

#### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Left arm + gripper commands |
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | Right arm + gripper commands |
| `/tf` | `tf2_msgs/TFMessage` | Preview ghost model transforms (prefixed with "preview_") |

## Controls

- **Joint Step slider** (1-200 mrad): Controls per-cycle arm joint step size
- **Gripper Step slider** (0.1-10.0 mm): Controls per-cycle gripper step size
- **Sync button**: Reads current /joint_states and applies to sliders
- **Home button**: Moves all joints to zero position (with confirmation)
- **Add Record**: Saves current robot pose as waypoint
- **Move to Record**: Executes stepped motion to a saved waypoint

## Joint Names

Left arm: `openarmx_left_joint1` through `openarmx_left_joint7`  
Right arm: `openarmx_right_joint1` through `openarmx_right_joint7`  
Grippers: `openarmx_left_finger_joint1`, `openarmx_right_finger_joint1`

## Prerequisites

- Robot running with `forward_position_controller` mode
- `/joint_states` topic active

## Dependencies

- `rclcpp`, `sensor_msgs`, `std_msgs`, `geometry_msgs`
- `rviz_common`, `pluginlib`
- `tf2_ros`, `kdl_parser`, `urdf`
- Qt5 (Core, Widgets)

## License

OpenArmX Research and Education License - Chengdu Changshu Robot Co., Ltd.
