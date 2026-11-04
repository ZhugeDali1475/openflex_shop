# openarmx_teach

English | [中文](./README-CN.md)

---

Utility package for recording and playing back OpenArmX joint trajectories.

## Overview

This package provides two command-line tools for teach-and-playback workflows:

1. **record_joint_states_always** - Continuously records `/joint_states` to a YAML file with keyboard controls for start/stop/save
2. **play_joint_trajectory** - Plays back recorded YAML trajectories using action interfaces or topic publishers, supporting multi-controller simultaneous execution

## Tools

### record_joint_states_always

Interactive recorder that streams joint states to memory and saves to YAML on demand.

```bash
ros2 run openarmx_teach record_joint_states_always [options]
```

**Options:**
| Option | Default | Description |
|--------|---------|-------------|
| `--topic` | `/joint_states` | Joint states topic to record |
| `--outfile` | auto-generated timestamp | Output YAML file path |
| `--rate` | `10.0` | Recording sample rate (Hz) |

**Keyboard Controls:**
- `SPACE` or `p`: Start/stop recording toggle
- `c`: Clear all recorded data (with confirmation)
- `w`: Save to file and quit (with confirmation)
- `q`: Quit without saving

### play_joint_trajectory

Plays recorded YAML trajectories through action clients or topic publishers.

```bash
ros2 run openarmx_teach play_joint_trajectory <file.yaml> [options]
```

**Options:**
| Option | Default | Description |
|--------|---------|-------------|
| `--rate-scale` | `1.0` | Playback speed (>1.0 faster, <1.0 slower) |
| `--left-arm` | - | Play only left arm joints |
| `--right-arm` | - | Play only right arm joints |
| `--both-arms` | - | Play both arm joints (no grippers) |
| `--all-joints` | - | Play all joints with all controllers |
| `--action` | - | Specify a single action server name |
| `--sync-feedback` | - | Use arm trajectory feedback to sync gripper timing |
| `--sync-margin` | `0.0` | Time margin for gripper synchronization |
| `--arm-prefix` | `` | Namespace prefix for multi-robot setups |

**Examples:**
```bash
# Play all joints (arms + grippers) simultaneously
\nEnglish | [中文](./README-CN.md)

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --all-joints

# Play at half speed
\nEnglish | [中文](./README-CN.md)

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --rate-scale 0.5

# Play only left arm
\nEnglish | [中文](./README-CN.md)

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --left-arm

# Synchronized gripper playback
\nEnglish | [中文](./README-CN.md)

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --all-joints --sync-feedback
```

## YAML Format

```yaml
joint_names:
  - openarmx_left_joint1
  - openarmx_left_joint2
  # ... all recorded joints
points:
  - positions: [0.1, 0.2, ...]
    time_from_start: 0.1
  - positions: [0.15, 0.22, ...]
    time_from_start: 0.2
```

## Action Servers Used (Playback)

| Action/Topic | Type | Description |
|--------------|------|-------------|
| `/left_joint_trajectory_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Left arm trajectory |
| `/right_joint_trajectory_controller/follow_joint_trajectory` | `FollowJointTrajectory` | Right arm trajectory |
| `/left_gripper_controller/gripper_cmd` | `GripperCommand` | Left gripper |
| `/right_gripper_controller/gripper_cmd` | `GripperCommand` | Right gripper |

## Usage

### Build

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_teach
source install/setup.bash
```

## Dependencies

- `rclpy`
- `sensor_msgs`
- `trajectory_msgs`
- `control_msgs`
- `python3-yaml`

## License

CC-BY-NC-SA-4.0 - Chengdu Changshu Robot Co., Ltd.
