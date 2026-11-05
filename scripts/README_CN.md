# 项目脚本说明

所有脚本默认使用：

```text
/home/wgb/openflex_shop/openflex_ws/install_v2
```

## 环境与健康检查

| 脚本 | 用途 |
|---|---|
| `check_care_environment.sh` | 检查 MuJoCo、ROS 包和核心单元测试 |
| `verify_moveit_basic.sh` | 检查 MoveIt、Pick-IK、描述包和 launch 文件 |
| `rebuild_mujoco_assets.sh` | 从 ROS 网格重建 MuJoCo 资源 |

## 仿真启动

| 脚本 | 用途 |
|---|---|
| `run_thesis_offline.sh` | 启动无 MoveIt 的离线 MuJoCo 栈 |
| `run_thesis_moveit_mission.sh` | 启动低内存 MuJoCo + MoveIt 单次 mission |
| `start_moveit_sim.sh` | 单独启动连接已有仿真的 MoveIt |

所有带 MuJoCo 渲染的启动都应保持：

```bash
export MUJOCO_GL=egl
```

## MoveIt 验证与诊断

| 脚本 | 用途 |
|---|---|
| `test_moveit_ik.sh` | 验证 `/compute_ik` |
| `test_moveit_move.sh` | 验证 `/move_action` 能驱动 MuJoCo |
| `probe_moveit_workspace.sh` | 采样右臂工作空间 |
| `diagnose_moveit_candidate.sh` | 输出候选 IK 和碰撞接触对 |
| `check_moveit_current_state.py` | 检查当前整机状态及左臂姿态 |
| `search_moveit_left_park.py` | 搜索左臂停放姿态 |
| `probe_left_park_contact.py` | 离线复现左臂停姿压入地面、顶高 `lift_joint` 的缺陷（ROS-free） |

## 数据与清理

| 脚本 | 用途 |
|---|---|
| `clean_generated_artifacts.sh` | 清理可再生缓存和历史运行日志 |

清理脚本不会删除源码、构建产品、模型、地图、实验数据和数据库。

