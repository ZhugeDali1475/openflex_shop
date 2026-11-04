# swerve_navigation

[English](./README.md) | 中文

---

`swerve_navigation` 是 OpenFlex 面向用户的建图、定位、导航和导航辅助工具主入口包。

建图、定位、导航等完整流程建议从本包启动，不建议直接把 `pgo`、`icp_registration`、`pointcloud_to_laserscan`、`nmpc_controller` 等底层包作为主入口。

## 包作用

主要目录：

| 目录 | 作用 |
|---|---|
| `launch/` | 建图、完整导航、VLA 回放相关启动文件 |
| `config/` | Nav2、碰撞监控、回放等配置 |
| `behavior_trees/` | Nav2 行为树 |
| `rviz/` | RViz 调试布局 |
| `scripts/` | 里程计复用、点云转发、安全门、VLA 辅助和监控工具 |

## 主要启动文件

| Launch | 作用 |
|---|---|
| `launch/mapping.launch.py` | 启动 Livox、FAST-LIVO 兼容桥接、PGO、底盘控制和 RViz 建图链路 |
| `launch/full_system.launch.py` | 启动定位、Nav2、碰撞监控、点云投影和 RViz 完整导航链路 |
| `launch/vla_replay.launch.py` | 启动 VLA 策略回放所需辅助节点 |
| `launch/vla_base_path_tracking.launch.py` | 将 VLA 底盘路径输出桥接到 Nav2 `FollowPath` |

## 建图流程

以下示例统一使用地图名 `your_map`：

```bash
export MAP_NAME=your_map
```

### 1. 建图准备

建图前可先单独检查底盘运动是否正常：

```bash
ros2 launch swerve_bringup swerve_drive.launch.py
```

另开终端启动键盘控制：

```bash
ros2 run swerve_bringup swerve_teleop.py
```

检查完成后停止 `swerve_drive.launch.py`，避免和 `mapping.launch.py` 重复启动底盘控制链路。正式建图时，`mapping.launch.py` 已经会启动底盘运控节点；如果需要键盘遥控，只需另开终端运行 `swerve_teleop.py`。

创建 3D 地图目录：

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"
```

注意：保存地图前目录必须已经存在，否则 `/pgo/save_maps` 会返回失败。

### 2. 启动建图节点

```bash
ros2 launch swerve_navigation mapping.launch.py
```

常用可选参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `use_camera` | `false` | 是否启动 D435 相机，用于视觉融合流程 |
| `steering_can_interface` | `can5` | 转向电机 CAN 接口 |
| `driving_can_interface` | `can4` | 驱动电机 CAN 接口 |
| `mapping_max_wheel_speed` | `2.0` | 建图时轮速上限 |
| `mapping_wheel_accel_limit` | `1.2` | 建图时轮速变化率上限 |

实时建图时，RViz 中主要关注 `/pgo/global_cloud` 和 `/fastlio2/body_cloud`。

### 3. 保存 3D 地图

建图完成后调用 PGO 保存服务：

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"

ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME', save_patches: true}"
```

成功后会生成：

```text
~/MID_360_nav/openflex_maps/3d/your_map/
├── map.pcd
├── poses.txt
├── patches/
└── sc_data/
```

### 4. 查看 3D 点云地图

查看已保存的 `map.pcd`：

```bash
ros2 launch pgo view_saved_map.launch.py \
  pcd_path:="$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd"
```

如果只看实时建图点云，`mapping.launch.py` 已经会打开 RViz，主要看 `/pgo/global_cloud` 或 `/fastlio2/body_cloud`。

### 5. 3D 地图转 2D 导航地图

这里是 **3D PCD 转 2D Nav2 地图**，不是 2D 转 3D。

```bash
mkdir -p "$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME"

ros2 run pgo pcd_to_nav2_map \
  --pcd "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd" \
  --out-dir "$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME" \
  --poses "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/poses.txt" \
  --resolution 0.05 \
  --z-min 0.00 \
  --z-max 2.30 \
  --obstacle-z-min 0.10 \
  --obstacle-z-max 1.40 \
  --inflation-radius 0.10
```

输出：

```text
~/MID_360_nav/openflex_maps/2d/your_map/
├── map.yaml
└── map.pgm
```

### 6. 可选修图

可以使用图形界面生成和修图：

```bash
ros2 run pgo pcd_to_nav2_map_gui
```

## 完整导航

启动导航：

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:="$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd" \
  map_yaml:="$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME/map.yaml" \
  initial_pose:="[0.0,0.0,0.0,0.0,0.0,0.0]"
```

### supermarket_test_03 比赛基准模式

用于商超比赛前的底盘定位、航点、全向移动、动态避障和精停验收：

```bash
ros2 launch swerve_navigation supermarket_test_03_benchmark.launch.py
```

该入口默认绑定：

- `~/openflex_all/openflex_maps/2d/supermarket_test_03/map.yaml`；
- `~/openflex_all/openflex_maps/3d/supermarket_test_03/map.pcd`；
- `config/nav2_params_supermarket_test_03.yaml`；
- `config/collision_monitor_supermarket_test_03.yaml`；
- 无自动倒车、无自动旋转的比赛安全行为树。

地图和 PCD 仍可用 `map_yaml:=... pcd_path:=...` 覆盖，方便实赛场地测量后复用同一
安全与精度基线。专用参数把正常终点目标设为 `0.05 m / 0.05 rad`，但该值必须通过
GUI 多轮真机验收，而不能仅凭参数文件视为已经达到。

必填输入：

| 参数 | 含义 |
|---|---|
| `pcd_path` | ICP 定位使用的 3D PCD 地图 |
| `map_yaml` | Nav2 使用的 2D 栅格地图 |

常用可选参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `use_rviz` | `true` | 是否启动 RViz |
| `initial_pose` | `[0,0,0,0,0,0]` | ICP 初始位姿 `[x,y,z,roll,pitch,yaw]`；保持默认值时 WS 会尝试受严格分数和多帧稳定门控保护的全局搜索 |
| `nav_controller_max_wheel_speed` | `1.2` | 导航时轮速上限 |
| `nav_controller_wheel_accel_limit` | `1.0` | 导航时轮速变化率上限 |

启动后可在 RViz 中使用 `2D Pose Estimate` 指定初始位姿，也可以直接通过 `initial_pose` 参数提供初始猜测。若没有人工位姿，ScanContext 歧义不会被直接接受，而是回退到严格 ICP 全局网格；验证期间 RViz 会显示实时点云，但 `map→odom` 仅为诊断用 identity，状态栏和 `/localization_ok` 会明确保持未就绪。

导航调试 RViz 默认显示全局/局部 costmap，并隐藏未经过导航去噪的原始青色点云。定位恢复前不得发送目标；确认 `/localization_ok=true` 后，应根据完整 footprint 周围的代价而不是单个空白像素选择终点。

底盘在 `/localization_ok` 变为 `true` 前保持停车。全局重定位候选还会经过连续局部 ICP 稳定性验证；若定位连续失败或出现被拒绝的跳变，速度门会保持零速度，定位节点自动重新执行全局 ScanContext/NDT/ICP，验证稳定后才恢复导航。

## 主要运行话题

| 话题 | 发布者 | 订阅者 |
|---|---|---|
| `/fastlio2/lio_odom` | FAST-LIVO 兼容桥接 | `safe_odom_mux`、NMPC |
| `/fastlio2/body_cloud` | FAST-LIVO 兼容桥接 | ICP、安全点云转发节点 |
| `/odom` | `swerve_drive_controller` | `safe_odom_mux` |
| `/odom_safe` | `safe_odom_mux.py` | Nav2 |
| `/livox/body_cloud_safety` | 安全点云转发节点 | Collision Monitor |
| `/livox/body_cloud_nav2` | 规划点云滤波节点 | `pointcloud_to_laserscan`、Nav2 代价地图 |
| `/scan` | `pointcloud_to_laserscan_node` | Nav2 局部代价地图 |
| `/localization_ok` | `icp_registration_node` | `cmd_vel_safety_gate.py`，定位健康状态；`false` 时持续停车 |
| `/competition/post_pick_reverse_authorized` | 比赛任务执行器 | `cmd_vel_safety_gate.py`，抓取后短距直线后退授权 |
| `/cmd_vel_safe` | `cmd_vel_safety_gate.py` | `swerve_drive_controller` |
| `/initialpose` | RViz 或 launch 初始位姿参数 | `icp_registration_node` |

## 辅助脚本

| 脚本 | 作用 |
|---|---|
| `safe_odom_mux.py` | 在 LIO 里程计和轮式里程计之间安全切换，并发布 `/odom_safe` 和 TF |
| `cmd_vel_safety_gate.py` | 执行定位停车与默认禁倒车策略，仅在短时授权内允许抓取后低速直线后退 |
| `pointcloud_qos_relay.py` | 分别发布保守安全点云，以及经过半径离群点和体素滤波的规划点云 |
| `laser_scan_qos_relay.py` | 使用适合 Nav2 的 QoS 转发 LaserScan |
| `local_window_marker.py` | 发布 RViz 局部窗口 marker |
| `nav_monitor_gui.py` | 监控导航命令链路和安全话题 |
| `future_odom_pose_tracker.py` | 跟踪 VLA future-odom 位姿目标 |
| `vla_path_to_nav2_follow_path.py` | 将 VLA 底盘路径消息桥接到 Nav2 `FollowPath` |

## 检查命令

```bash
ros2 topic list
ros2 topic hz /odom_safe
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /localization_ok
ros2 topic echo --once /cmd_vel_safe
```

如果导航不运动，优先检查 `/cmd_vel`、`/cmd_vel_safe_in`、`/cmd_vel_safe`、定位状态，以及 `collision_monitor` 是否正在保持停车。
