# openarmx_teach

[English](./README.md) | 中文

---

OpenArmX 关节轨迹录制与回放工具包。

## 概述

本包提供两个命令行工具，用于示教-回放工作流：

1. **record_joint_states_always** - 持续录制 `/joint_states` 到 YAML 文件，支持键盘控制启停/保存
2. **play_joint_trajectory** - 通过动作接口或话题发布器回放录制的 YAML 轨迹，支持多控制器同步执行

## 工具

### record_joint_states_always

交互式录制器，将关节状态流式写入内存，按需保存为 YAML。

```bash
ros2 run openarmx_teach record_joint_states_always [选项]
```

**选项：**
| 选项 | 默认值 | 描述 |
|------|--------|------|
| `--topic` | `/joint_states` | 录制的关节状态话题 |
| `--outfile` | 自动生成时间戳 | 输出 YAML 文件路径 |
| `--rate` | `10.0` | 录制采样率（Hz） |

**键盘控制：**
- `空格` 或 `p`：开始/停止录制切换
- `c`：清除所有录制数据（需确认）
- `w`：保存文件并退出（需确认）
- `q`：不保存退出

### play_joint_trajectory

通过动作客户端或话题发布器回放录制的 YAML 轨迹。

```bash
ros2 run openarmx_teach play_joint_trajectory <file.yaml> [选项]
```

**选项：**
| 选项 | 默认值 | 描述 |
|------|--------|------|
| `--rate-scale` | `1.0` | 回放速度（>1.0 加速，<1.0 减速） |
| `--left-arm` | - | 仅播放左臂关节 |
| `--right-arm` | - | 仅播放右臂关节 |
| `--both-arms` | - | 播放双臂关节（无夹爪） |
| `--all-joints` | - | 使用所有控制器播放全部关节 |
| `--action` | - | 指定单个动作服务器名称 |
| `--sync-feedback` | - | 使用臂轨迹反馈同步夹爪时序 |
| `--sync-margin` | `0.0` | 夹爪同步的时间余量 |
| `--arm-prefix` | `` | 多机器人场景的命名空间前缀 |

**示例：**
```bash
# 同步播放所有关节（臂 + 夹爪）
\n[English](./README.md) | 中文

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --all-joints

# 半速播放
\n[English](./README.md) | 中文

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --rate-scale 0.5

# 仅播放左臂
\n[English](./README.md) | 中文

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --left-arm

# 同步夹爪回放
\n[English](./README.md) | 中文

---
ros2 run openarmx_teach play_joint_trajectory recording.yaml --all-joints --sync-feedback
```

## YAML 格式

```yaml
joint_names:
  - openarmx_left_joint1
  - openarmx_left_joint2
  # ... 所有录制的关节
points:
  - positions: [0.1, 0.2, ...]
    time_from_start: 0.1
  - positions: [0.15, 0.22, ...]
    time_from_start: 0.2
```

## 使用的动作服务器（回放时）

| 动作/话题 | 类型 | 描述 |
|----------|------|------|
| `/left_joint_trajectory_controller/follow_joint_trajectory` | `FollowJointTrajectory` | 左臂轨迹 |
| `/right_joint_trajectory_controller/follow_joint_trajectory` | `FollowJointTrajectory` | 右臂轨迹 |
| `/left_gripper_controller/gripper_cmd` | `GripperCommand` | 左夹爪 |
| `/right_gripper_controller/gripper_cmd` | `GripperCommand` | 右夹爪 |

## 使用方法

### 编译

```bash
cd ~/openflex_ws
colcon build --packages-select openarmx_teach
source install/setup.bash
```

## 依赖

- `rclpy`
- `sensor_msgs`
- `trajectory_msgs`
- `control_msgs`
- `python3-yaml`

## 许可证

CC-BY-NC-SA-4.0 - 成都长树机器人有限公司
