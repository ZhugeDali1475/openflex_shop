# icp_registration

English | [中文](./README-CN.md)

---

`icp_registration` performs map-based point cloud registration and relocalization. It aligns live LiDAR/body clouds against a saved PCD map, publishes `map -> odom`, and can use ScanContext, NDT, ICP, wheel odometry, and optional GPS constraints to recover localization.

## Package Role

Main executable:

```bash
icp_registration_node
```

Main launch:

```bash
launch/icp.launch.py
```

## Inputs

| Topic | Type | Description |
|---|---|---|
| `/livox/lidar` or configured `pointcloud_topic` | `sensor_msgs/msg/PointCloud2` | Live point cloud to align |
| `/odom` | `nav_msgs/msg/Odometry` | Wheel odometry for yaw-jump detection and recovery gating |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Manual initial pose from RViz |
| `/gps/map_position` | `geometry_msgs/msg/PointStamped` | Optional GPS-assisted ScanContext search prior |

## Outputs

| Output | Type | Description |
|---|---|---|
| `map -> odom` TF | TF | Main localization result |
| `/localization_ok` | `std_msgs/msg/Bool` (reliable, transient local) | `true` only after a global candidate has several stable local ICP refinements; used to unlock chassis motion |
| `/localization_hold` | `std_msgs/msg/Empty` | Published when localization should be held/rejected by safety logic |

## Map Inputs

`pcd_path` must point to an existing map PCD. For ScanContext relocalization, the map directory should also contain the files produced by PGO map saving:

```text
map_dir/
├── map.pcd
├── poses.txt
├── patches/
└── scan_context/
```

## Configuration

Main config file:

```bash
config/icp.yaml
```

Important parameters:

| Parameter | Meaning |
|---|---|
| `pcd_path` | Path to saved map PCD; must be set before use |
| `pointcloud_topic` | Live PointCloud2 topic |
| `map_frame_id` / `odom_frame_id` / `base_frame_id` | TF frame names |
| `laser_frame_id` | Frame of the incoming cloud, currently commonly `base_link` |
| `rough_leaf_size` / `refine_leaf_size` | Downsample sizes for rough/refined ICP |
| `xy_offset`, `yaw_offset`, `yaw_resolution` | Search grid for initial alignment |
| `continuous_realign_interval_sec` | Periodic ICP correction interval |
| `continuous_icp_max_correspondence_distance` | Tighter correspondence distance for continuous tracking so ICP cannot jump to an adjacent shelf |
| `localization_stable_samples` | Number of stable local ICP samples required after a global candidate |
| `localization_stable_pos_thresh` / `localization_stable_yaw_thresh` | Local ICP stability gates |
| `max_continuous_failures` | Consecutive failures before revoking localization and restarting ScanContext/NDT global relocalization |
| `post_rotation_weak_quality_max_duration` / `post_rotation_weak_quality_max_travel` | Bounded wheel/LIO-verified hold of the last trusted transform while leaving weak geometry after an in-place rotation |
| `fine_alignment_guard_*` | Bounded near-field lease that discards weak ICP results and retains the last trusted transform during delivery-table fine alignment, subject to wheel/LIO and quality gates |
| `reject_ambiguous_global_candidates` | Hard-reject automatic global candidates when an independent near-tied minimum exists |
| `trusted_recovery_anchor_timeout_sec` | Time after localization loss during which automatic recovery must agree with the last trusted map/wheel trajectory |
| `constrain_to_2d` | Keep `map -> odom` planar |
| `preserve_3d_structure_for_planar_icp` | Preserve point height for correspondence search while estimating/publishing a planar transform |
| `wheel_odom_topic` | Wheel odometry topic used for yaw checks |
| `localization_state_topic` | Localization health topic, `/localization_ok` by default |

## Prerequisites

Start a live point cloud source first. In the OpenFlex navigation stack this is usually `/fastlio2/body_cloud` from `fast_livo` bridge, but this package can consume any `sensor_msgs/msg/PointCloud2` cloud if configured.

## Start

Edit `config/icp.yaml` or pass parameters so `pcd_path` and `pointcloud_topic` are correct, then run:

```bash
cd ~/openflex_all/openflex_ws
source install/setup.bash
ros2 launch icp_registration icp.launch.py
```

Direct run example:

```bash
ros2 run icp_registration icp_registration_node --ros-args \
  -p pcd_path:=/path/to/map.pcd \
  -p pointcloud_topic:=/fastlio2/body_cloud
```

## Check

```bash
ros2 topic hz /fastlio2/body_cloud
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /localization_ok
ros2 topic echo --once /localization_hold
```

Manual initial pose can be set in RViz using `2D Pose Estimate`, which publishes `/initialpose`.

## Notes

- `icp.launch.py` uses `config/icp.yaml`; the checked-in default has an empty `pcd_path`, so a map path must be provided before real use.
- If the live cloud frame differs from `laser_frame_id`, the node tries to use TF to transform it.
- For navigation, keep only one node responsible for `map -> odom`.
- `/localization_ok` is `false` during startup, while a global candidate is being validated, after rejected jumps, and after continuous registration failures. The navigation gate remains stopped; it does not reopen on a fixed timer.
- After `max_continuous_failures`, the node stops trusting the previous TF and restarts ScanContext/NDT/ICP global relocalization from the latest cloud. Motion is released only after the local refinements become stable again.
