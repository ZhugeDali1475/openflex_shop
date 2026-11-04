# openarmx_kp_kd_panel

English | [中文](./README-CN.md)

---

RViz2 panel plugin for dynamically adjusting KP (stiffness) and KD (damping) parameters of OpenArmX robot joints.

## Overview

This package provides an RViz2 panel that allows runtime adjustment of the KP and KD motor control parameters for the OpenArmX hardware interface. It communicates with the hardware parameter nodes via ROS 2 parameter services to set values for all 8 joints (7 arm + 1 gripper) per arm.

## Features

- Unified slider control for arm joints (1-7) with automatic per-motor-type mapping
- Separate KP/KD sliders for the gripper (joint 8)
- Arm selection: Right only, Left only, or Both arms simultaneously
- Automatic motor type mapping:
  - Joint 1-2 (RS04): KP range 0-5000, KD range 0-100
  - Joint 3-4 (RS03): KP range 0-5000, KD range 0-100
  - Joint 5-7 (RS00): KP range 0-500, KD range 0-5
  - Joint 8/Gripper (RS00): KP range 0-500, KD range 0-5
- Reset-to-default buttons for each parameter group
- Simulation mode detection (disables controls when fake hardware is active)
- Visual confirmation dialog on parameter application

## Usage

### Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_kp_kd_panel
source install/setup.bash
```

### Add to RViz

1. Open RViz2
2. Go to Panels > Add New Panel
3. Select "KpKdPanel" from the openarmx_kp_kd_panel plugin

### Target Parameter Nodes

The panel sets parameters on:

| Node | Description |
|------|-------------|
| `/openarmx_right_hardware_params` | Right arm hardware parameter node |
| `/openarmx_left_hardware_params` | Left arm hardware parameter node |

### Parameters Set

For each arm, the panel sets:
- `kp_joint1` through `kp_joint8` - Position gain (stiffness)
- `kd_joint1` through `kd_joint8` - Velocity gain (damping)

## Prerequisites

- OpenArmX robot running with real hardware (MIT control mode)
- Hardware parameter nodes active (`openarmx_right_hardware_params`, `openarmx_left_hardware_params`)
- Not supported in simulation/fake hardware mode

## Dependencies

- `rclcpp`
- `rviz_common`, `rviz_rendering`
- `pluginlib`
- Qt5

## License

CC-BY-NC-SA-4.0 - Chengdu Changshu Robot Co., Ltd.
