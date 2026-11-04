# icp_registration

[English](./README.md) | 中文

---

`icp_registration` 用于基于已有地图的点云配准和重定位。它把实时激光/车体点云与保存的 PCD 地图对齐，发布 `map -> odom`，并结合 ScanContext、NDT、ICP、轮式里程计和可选 GPS 约束恢复定位。

## 包作用

主要可执行文件：

```bash
icp_registration_node
```

主要启动文件：

```bash
launch/icp.launch.py
```

## 输入

| 话题 | 类型 | 说明 |
|---|---|---|
| `/livox/lidar` 或配置的 `pointcloud_topic` | `sensor_msgs/msg/PointCloud2` | 用于配准的实时点云 |
| `/odom` | `nav_msgs/msg/Odometry` | 轮式里程计，用于 yaw 跳变检测和恢复门控 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz 手动初始位姿 |
| `/gps/map_position` | `geometry_msgs/msg/PointStamped` | 可选 GPS 辅助 ScanContext 搜索先验 |

## 输出

| 输出 | 类型 | 说明 |
|---|---|---|
| `map -> odom` TF | TF | 主要定位结果 |
| `/localization_ok` | `std_msgs/msg/Bool`（可靠、Transient Local） | 只有全局候选后连续局部 ICP 稳定时才为 `true`；底盘速度门以此解锁 |
| `/localization_hold` | `std_msgs/msg/Empty` | 定位需要保持/拒绝更新时发布 |

## 地图输入

`pcd_path` 必须指向已有的地图 PCD。如果要启用 ScanContext 重定位，地图目录通常需要包含 PGO 保存出的文件：

```text
map_dir/
├── map.pcd
├── poses.txt
├── patches/
└── scan_context/
```

## 配置

主要配置文件：

```bash
config/icp.yaml
```

关键参数：

| 参数 | 含义 |
|---|---|
| `pcd_path` | 保存地图 PCD 路径，使用前必须设置 |
| `pointcloud_topic` | 实时 PointCloud2 话题 |
| `map_frame_id` / `odom_frame_id` / `base_frame_id` | TF 坐标系名 |
| `laser_frame_id` | 输入点云坐标系，当前常用 `base_link` |
| `rough_leaf_size` / `refine_leaf_size` | 粗/精 ICP 下采样尺寸 |
| `xy_offset`, `yaw_offset`, `yaw_resolution` | 初始配准搜索网格 |
| `continuous_realign_interval_sec` | 周期性 ICP 纠偏间隔 |
| `continuous_icp_max_correspondence_distance` | 连续跟踪的最大对应距离；应小于全局搜索距离，避免跳到相邻货架 |
| `localization_stable_samples` | 全局候选后需要连续通过的局部 ICP 样本数 |
| `localization_stable_pos_thresh` / `localization_stable_yaw_thresh` | 局部 ICP 稳定性门限 |
| `max_continuous_failures` | 连续失败达到此数时撤销定位并自动重新执行 ScanContext/NDT 全局重定位 |
| `post_rotation_weak_quality_max_duration` / `post_rotation_weak_quality_max_travel` | 原地转向后驶离弱可观区域时，在 wheel/LIO 一致性约束下保留最后可信变换的最大时间/里程 |
| `fine_alignment_guard_*` | 交付台近场精调期间，在活动租约、wheel/LIO 一致性、ICP 支撑和时间/里程边界内丢弃弱 ICP 解并保留最后可信变换 |
| `allow_ambiguous_sc_grid_fallback` | ScanContext 歧义时是否允许无人工位姿执行全局网格搜索；开启时还必须满足 `global_grid_score_thresh` 并通过多帧稳定门控 |
| `global_grid_score_thresh` | 歧义全局网格候选的严格精配准分数上限，防止重复货架局部极小值被放行 |
| `global_grid_candidate_separation` / `global_grid_score_margin` | 全局网格最优与空间独立次优极小值的距离/分数差门限；近似并列时保持未定位 |
| `reject_ambiguous_global_candidates` | 是否硬拒绝存在空间独立近似同分极值的自动全局候选；商超重复环境应保持 `true` |
| `trusted_recovery_anchor_timeout_sec` | 普通掉定位后允许使用最后可信 map/wheel 轨迹自动恢复的最长时长；超时需人工 `/initialpose` |
| `publish_identity_until_localized` | 定位未就绪时暂时发布 identity `map -> odom`，仅用于 RViz 显示实时点云；`/localization_ok=false` 时运动门仍保持锁定 |
| `constrain_to_2d` | 是否将 `map -> odom` 约束为平面变换 |
| `preserve_3d_structure_for_planar_icp` | 输出仍为 SE(2) 时保留点云 z 参与对应搜索，避免不同高度货架层投影重叠 |
| `wheel_odom_topic` | 用于 yaw 检查的轮式里程计 |
| `localization_state_topic` | 定位健康状态话题，默认 `/localization_ok` |

## 前置条件

先启动实时点云来源。在 OpenFlex 导航链路中通常是 `fast_livo` 桥接发布的 `/fastlio2/body_cloud`，但只要配置正确，本包可以订阅任意 `sensor_msgs/msg/PointCloud2` 点云。

## 启动

先编辑 `config/icp.yaml`，或通过参数传入正确的 `pcd_path` 和 `pointcloud_topic`，然后运行：

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch icp_registration icp.launch.py
```

直接运行示例：

```bash
ros2 run icp_registration icp_registration_node --ros-args \
  -p pcd_path:=/path/to/map.pcd \
  -p pointcloud_topic:=/fastlio2/body_cloud
```

## 检查

```bash
ros2 topic hz /fastlio2/body_cloud
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /localization_ok
ros2 topic echo --once /localization_hold
```

RViz 中可用 `2D Pose Estimate` 手动设置初始位姿，该工具会发布 `/initialpose`。

## 注意事项

- `icp.launch.py` 默认加载 `config/icp.yaml`；仓库默认 `pcd_path` 为空，实车使用前必须设置地图路径。
- 如果实时点云 frame 与 `laser_frame_id` 不一致，节点会尝试通过 TF 做变换。
- 导航中应确保只有一个节点负责发布 `map -> odom`。
- `/localization_ok` 在启动、全局候选刚产生、连续配准失败或跳变被拒绝时为 `false`；导航链路会持续停车，而不是在固定延时后自行恢复。ScanContext 歧义时 WS 默认尝试严格全局网格搜索，仍需连续稳定样本和轮式门控才能变为 `true`。
- 连续失败达到 `max_continuous_failures` 后，节点停止使用旧 TF 并从最新点云重新进行 ScanContext/NDT/ICP 全局重定位；稳定通过后才再次放行底盘。
