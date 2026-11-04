# openflex_navigation_gui

OpenFlex 建图、定位、导航、商超比赛验收和诊断录制桌面工作台。该包是一个
`ament_python` ROS 2 包，使用 PyQt5 构建界面，通过一个名为
`/openflex_navigation_gui` 的 ROS 2 节点与底盘、PGO、定位和 Nav2 系统通信。

> ROS 2 中订阅的是“话题”，不是“节点”。话题的发布节点可随 launch 配置变化，
> GUI 不会把订阅绑定到某个固定发布节点。本文分别列出 GUI 直接使用的 ROS 接口、
> 期望出现的系统节点以及仅用于 rosbag 录制的话题。

## 1. 主要能力

- 一键启动/停止 `swerve_navigation mapping.launch.py`；
- GUI 按压遥控与 `W/S/A/D/Q/E` 键盘遥控，向 `/cmd_vel` 发布速度；
- 调用 `/pgo/save_maps` 保存 PCD、poses、patches 和 ScanContext 数据；
- 调用 `pgo pcd_to_nav2_map` 将三维 PCD 转换为 Nav2 PGM/YAML 地图；
- 内置二维地图编辑器，支持画笔、直线、连通区域、撤销、重做和原图备份；
- 一键启动/停止 `swerve_navigation full_system.launch.py`；
- 发布 `/initialpose`，通过 `/navigate_to_pose` Action 发送和取消 Nav2 目标；
- 显示 `map` 位姿、目标误差、路径横差、ICP 单步修正、速度链和四个舵轮状态；
- 导航时自动录制 rosbag，并保存 ROS 图、Action 列表和关键节点参数；
- 提供商超比赛专用界面，维护语义站点、执行巡游并按精度门槛自动判定；
- 整机模式的“关节控制”页支持双臂拖动示教、逐关节 Kp/Kd 调节和当前位姿记录；
- 检测外部底盘控制栈，防止重复启动 `/controller_manager`。

## 2. 包结构与文件关系

```text
openflex_navigation_gui/
├── __init__.py                 Python 包标记
├── main_window.py              通用 GUI、ROS 节点、进程管理和诊断录制
├── model.py                    地图路径、角度/四元数、路径误差和录制话题
├── map_editor_core.py          不依赖 Qt 的 PGM 读写与栅格编辑算法
├── map_editor.py               Nav2 二维地图编辑窗口
├── benchmark.py                比赛基准档案解析与精度判定
├── competition_mission.py      每张地图的语义站点持久化和坐标转换
├── waypoint_picker.py          在 PGM 地图上选择位置与朝向的窗口
├── joint_control_page.py       整机关节、拖动示教、Kp/Kd 和姿态记录
├── freedrive_runner.py         拖动模式入口，退出时恢复原 Kp/Kd 与控制模式
└── competition_window.py       商超比赛导航、巡游和验收界面
```

主要调用关系如下：

```text
main_window.py
├── model.py
├── map_editor.py ─────────── map_editor_core.py
└── PathPlot ──────────────── map_editor_core.py

competition_window.py
├── main_window.py（继承 MainWindow，复用 NavigationRosNode）
├── benchmark.py
├── competition_mission.py ── benchmark.py + map_editor_core.py + model.py
└── waypoint_picker.py ────── competition_mission.py + map_editor_core.py
```

## 3. `openflex_navigation_gui` 中每个文件的功能

### 3.1 `__init__.py`

Python 包初始化文件。目前只包含包级说明，不创建节点、话题或界面。它使目录可以被
`setuptools.find_packages()` 识别，并允许其他文件使用相对导入。

### 3.2 `main_window.py`

这是通用工作台的入口和核心文件，包含 ROS 通信、界面、工作流子进程和状态计算。

主要数据类和界面类：

- `WheelTelemetry`：保存单个舵轮的请求角、命令角、实际角、角度误差、目标/命令/
  实际速度及转向缩放比例；
- `Telemetry`：集中保存地图位姿、`map -> odom` 修正、轮式/LIO 里程计、四级速度链、
  全局路径、轨迹、节点列表、话题最后到达时间、定位状态和 Action 状态；
- `NavigationRosNode`：真正创建 ROS 2 发布者、订阅者、服务客户端、Action 客户端和
  TF 监听器，节点名为 `openflex_navigation_gui`；
- `StatusDot`、`MetricWidget`：状态灯和指标卡组件；
- `PathPlot`：加载 PGM/YAML 地图，绘制全局路径、机器人轨迹、当前位姿和参考站点，
  支持在图上按下并拖动选择目标位置和朝向；
- `MainWindow`：组装“建图 / 导航 / 系统”页面，启动与停止外部进程，定时刷新状态；
- `main()`：初始化 `rclpy` 和 Qt，创建 `NavigationRosNode` 与 `MainWindow`，退出时发布
  零速度、销毁节点并关闭 ROS。

运行时有四个 Qt 定时循环：

| 周期 | 工作 |
|---:|---|
| `20 ms` | 非阻塞执行一次 `rclpy.spin_once()`，同时检查保存地图的异步响应 |
| `50 ms` | 在遥控允许且建图可用时发布当前遥控速度，即约 `20 Hz` |
| `200 ms` | 更新位姿、误差、速度、舵轮表格和 Action 状态，即约 `5 Hz` |
| `1000 ms` | 采样 ROS 节点图与 TF，并刷新启动前检查和当前工作模式 |

#### 建图流程

“开始建图”实际启动：

```bash
ros2 launch swerve_navigation mapping.launch.py \
  use_rviz:=true \
  use_camera:=false \
  use_gps:=false \
  mapping_max_wheel_speed:=<界面值> \
  mapping_wheel_accel_limit:=<界面值>
```

GUI 会先检查 ROS 图中是否已有外部 `/controller_manager`。新地图的三维目录会提前创建，
但 `map.pcd` 要在保存服务成功后才产生。

商超整机模式下，“启动全身控制”只启动硬件、MoveIt、任务接口和统一里程计，不启动
FAST-LIVO、PGO 或 Nav2。此时再点“开始建图”，GUI 会追加
`start_robot_hardware:=false`，复用已有整机控制器，避免第二个 `controller_manager` 争用 CAN。

“关节控制”页中的拖动示教使用本包内的 `freedrive_node`：进入时先释放 GUI 手动关节发布者，
暂停 MoveIt 命令桥，把所选手臂切到 MIT 并将 Kp 置零。拖动过程中可分别调整左右臂
J1-J7 的 Kp/Kd；退出时会恢复进入前的 Kp/Kd、控制模式和 MoveIt。拖动状态点击
“Home（双臂优先）”时不切换 MIT，节点只临时恢复定位 Kp/Kd，让双臂先回零并恢复拖动
增益；控制台确认双臂实际到零后，才让升降柱和头部回零。姿态记录按钮直接读取新鲜的
`/joint_states`，不会保存拖动前残留的滑块目标。

遥控只有在以下条件同时满足时才会发布非零速度：

1. 已勾选“启用遥控”；
2. GUI 自己启动的建图进程仍在运行，或 ROS 图中存在 `/pgo`/`/pgo/*` 节点；
3. 用户正按住界面按钮或相应键盘按键。

键位为 `W/S` 前后、`A/D` 左右平移、`Q/E` 左右旋转。松开按键、取消遥控、停止建图
或退出界面时会发布零速度。

#### 保存三维地图与生成二维地图

“保存 3D 地图”调用 `/pgo/save_maps`；“保存并生成 2D 地图”会在服务成功后停止建图，
然后执行：

```bash
ros2 run pgo pcd_to_nav2_map \
  --pcd <3d/map_name/map.pcd> \
  --out-dir <2d/map_name> \
  --resolution ... --z-min ... --z-max ... \
  --obstacle-z-min ... --obstacle-z-max ... \
  --inflation-radius ... --obstacle-min-hits ... --free-min-hits ... \
  --sor-mean-k ... --sor-stddev ... --min-obstacle-cluster ... \
  --raycast-max-range ...
```

如果 `poses.txt` 存在，还会增加 `--poses` 和 `--crop-to-poses-margin`。转换进程退出后，
GUI 会同时检查 `map.yaml` 与 `map.pgm`，成功时可直接进入地图编辑器。

#### 导航流程

“启动定位导航”要求同名的 `map.pcd`、`map.yaml`、`map.pgm` 都存在，然后启动：

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:=<3d/map_name/map.pcd> \
  map_yaml:=<2d/map_name/map.yaml> \
  initial_pose:='[x,y,0.0,0.0,0.0,yaw]' \
  use_rviz:=true \
  nav_controller_max_wheel_speed:=<界面值> \
  nav_controller_wheel_accel_limit:=<界面值>
```

商超整机模式下，定位导航是独立进程，并以 `start_robot_hardware:=false` 和
`start_safe_odom_mux:=false` 复用全身运行时。建图和定位导航互斥，但二者切换时不需要重启
全身硬件与 MoveIt；停止全身控制前必须先停止当前建图或定位导航工作流。

勾选“自动录制”时，GUI 在启动导航前先开始 rosbag；导航进程启动约 8 秒后再次导出
关键节点参数。停止导航会取消当前 Action、发布零速度、停止 launch，并停止自动录制。

发送目标前必须已经收到 `/localization_ok=true`，且 `/navigate_to_pose` Action 服务端
已就绪。GUI 使用 Action 下发目标，不向 RViz 常用的 `/goal_pose` 发布目标，避免某些 RViz
Nav2 面板重复提交；只向 `/openflex_navigation_gui/goal_pose` 发布一份诊断镜像。

#### 进程管理

建图、导航、转换和录制均由 `QProcess` 启动，并通过 `setsid` 放入独立进程组。停止时先向
该进程组发送 `SIGINT`，6 秒后仍未退出再发送 `SIGTERM`。GUI 只管理自己启动的进程；发现
外部系统时只显示状态或阻止重复启动，不会杀死外部进程。

### 3.3 `model.py`

不依赖 ROS 节点和 Qt 的通用模型与数学工具：

- `MapPaths`：描述一张地图对应的根目录、2D/3D 目录和 PCD、poses、YAML、PGM 路径；
- `validate_map_name()`：地图名只允许字母、数字、点、下划线和短横线，防止名称成为
  任意路径；
- `resolve_map_paths()`：把根目录与地图名解析为统一的 `MapPaths`；
- `discover_map_names()`：从 `3d/*/map.pcd` 和 `2d/*/map.yaml` 合并发现地图名；
- `normalize_angle()`：把角度归一化到 `[-pi, pi]`；
- `yaw_from_quaternion()` / `quaternion_from_yaw()`：偏航角与四元数互转；
- `nearest_path_error()`：找到路径中离机器人最近的离散位姿，返回欧氏横差和航向差；
- `command_text()`：仅用于把启动命令安全、可读地打印到 GUI 日志；
- `RECORD_TOPICS`：诊断 rosbag 的固定话题清单。注意它不等于 GUI 的订阅列表。

### 3.4 `map_editor_core.py`

纯 Python 栅格地图算法，不依赖 PyQt5 或 ROS，便于单元测试和复用。

- 定义 Nav2 地图常用灰度：`OCCUPIED=0`、`UNKNOWN=205`、`FREE=254`；
- `GridMap` 保存宽、高和一维像素数组，并提供复制、索引、取值和数量统计；
- `read_pgm()` 支持二进制 `P5` 和文本 `P2` PGM，最大值不是 255 时缩放到 8 bit；
- `write_pgm()` 统一写为 8-bit `P5`；
- `save_with_backup()` 保存前将原图复制到 `backups/`，再使用同目录临时文件和
  `os.replace()` 原子替换；
- `parse_map_yaml()` 提取 `resolution` 和三元素 `origin`；
- `paint_circle()` 负责圆形画笔，`line_cells()` 使用整数直线算法，`paint_line()` 将两者
  组合；
- `fill_component()` 对同一灰度的八连通区域进行填充；
- `edit_summary()` 统计总改动数、障碍转空闲、未知转空闲、新障碍和新未知格数。

### 3.5 `map_editor.py`

在 `map_editor_core.py` 上构建的 PyQt5 二维地图编辑器，本文件不创建 ROS 节点，也不直接
订阅话题。

- `MapCanvas`：显示栅格地图并处理鼠标事件；支持画笔、直线、八连通区域三种工具；
- `MapEditorDialog`：提供工具/目标灰度/半径/缩放设置、100 步撤销栈、重做、恢复到
  已保存版本、修改统计和保存确认；
- 连通区域一次超过 `2000` 格会撤销本次操作，防止误清整片未知区；
- 把障碍或未知改为空闲时必须二次确认；
- 保存时再次调用 `save_allowed`。如果 GUI 导航进程正在运行，或 ROS 图中出现
  `/map_server`、`/planner_server`、`/controller_server`、`/bt_navigator`，则拒绝保存；
- 保存成功后原图位于 `2d/<map_name>/backups/map_<timestamp>.pgm`；已启动的
  `map_server` 不会自动重载，仍需重启导航系统。

### 3.6 `benchmark.py`

商超比赛基准档案的数据模型、YAML 加载和结果判定，不创建 ROS 节点。

- `BenchmarkWaypoint`：航点 ID、名称、`x/y/yaw`、净空和语义角色；
- `BenchmarkAcceptance`：终点位置/朝向、路径横差、ICP 跳变、停车速度与保持时间、
  成功率等门槛；
- `BenchmarkTour`：巡游 ID、航点序列和航点间稳定等待时间；
- `BenchmarkProfile`：场景信息、地图根目录、地图名、launch 文件、初始位姿、门槛、
  航点和巡游；
- `load_benchmark_profile()`：加载 `schema_version: 1` 的 YAML，校验正数门槛、成功率、
  空航点以及巡游对未知航点的引用；
- `expand_tour()`：按重复次数展开巡游；
- `evaluate_goal()`：按终点位置误差与朝向误差给出通过/超差；
- `waypoint_landmarks()`：把同坐标站点合并为地图标记；
- `find_waypoint_by_role()`：查找唯一语义角色，例如 `start_position`。

当前 `competition_window.py` 的单目标最终判定实际使用 `goal_position_m`、
`goal_yaw_rad`、`path_cross_track_m`、`max_icp_step_m` 和 `max_icp_step_yaw_rad`。
档案中的 `max_goal_position_m`、`max_goal_yaw_rad`、`minimum_success_rate` 和
`required_zero_collision_tours` 会被解析和保留，但当前界面尚未据此生成整场汇总结论。

### 3.7 `competition_mission.py`

管理每张地图自己的语义导航站点，不创建 ROS 节点。

- 站点文件固定为 `2d/<map_name>/competition_mission.yaml`；
- 支持起点、交付台、货架接近点、补货点、巡查点、任务判定区、终点和通用站点角色；
- `validate_station_id()` 限制 ID 字符，`save_mission()` 检查 ID 唯一且最多一个起点；
- `save_mission()` 先写临时文件再用 `os.replace()` 替换，避免中途写坏 YAML；
- `load_mission()` 校验 `schema_version` 和 `map_name`，防止把一张地图的站点用于另一张；
- `grid_to_world()` / `world_to_grid()` 根据 PGM 行列、地图高度、分辨率和 origin 进行
  图像坐标与 ROS `map` 坐标转换。

坐标转换目前使用 `origin[0]`、`origin[1]`，没有应用 `origin[2]` 的地图旋转；因此地图
YAML 应使用常见的零旋转原点，否则选点和显示坐标会产生偏差。

### 3.8 `waypoint_picker.py`

用于比赛站点和临时目标的地图选点窗口，不创建 ROS 节点。

- `WaypointCanvas`：显示 PGM、已有站点、当前选中位置和朝向箭头；
- `WaypointPickerDialog`：提供 1～12 倍缩放、坐标/角度显示和合法性检查；
- 鼠标左键按下的位置是目标中心，拖动方向是车头方向；拖动不足约 2 个栅格时保留原
  朝向；
- 只有像素值大于等于 `FREE` 的栅格可以确认，障碍和未知栅格会禁用“确定”；
- 通过 `competition_mission.py` 的转换函数保证 PGM 向下的 Y 轴与 ROS 地图向上的 Y 轴
  正确互换。

### 3.9 `competition_window.py`

商超比赛专用入口。`CompetitionMainWindow` 继承 `MainWindow`，复用同一个
`NavigationRosNode`，因此它的直接 ROS 发布、订阅、服务和 Action 与通用界面完全相同，
不会再创建第二个 ROS 节点。

主要扩展：

- 将普通“导航”页替换为“比赛导航”页，保留“建图”和“系统检查”；
- 从比赛档案加载地图、初始位姿、launch 文件、验收门槛、默认航点和巡游；
- 在 `competition_mission.yaml` 中新增、更新、删除、排序语义站点；
- 支持地图选点、使用当前可信 `map -> base_link` 位姿建立站点、返回起点并对正；
- 支持单航点、临时地图目标、按站点顺序巡游和档案预定义巡游；
- 在整机页显示真实抓取、模拟抓取和安全退避接口状态，并提供
  “模拟抓取成功 -> 安全退避 -> 下一站导航”联调链；
- 记录每个目标的最终位置/角度误差、耗时、最大路径横差和最大 ICP 单步修正；
- 检测速度链明显被压低的次数并写入日志，作为碰撞预测限速/停车提示。

比赛目标下发前的额外检查：

1. 当前没有正在发送、执行或取消的 `NavigateToPose` 目标；
2. 已收到 `/localization_ok=true`；
3. `/scan` 和 `/odom_safe` 数据年龄都不超过 `0.5 s`；
4. ROS 图中存在 `/controller_server`、`/collision_monitor`、
   `/cmd_vel_safety_gate`；
5. `/navigate_to_pose` Action 服务端就绪。

Action 成功后不会立即判定。机器人还必须满足档案中的线速度/角速度停车门槛，并持续
`stopped_hold_sec`，之后才计算终点位置、朝向、全程最大横差与 ICP 修正。巡游任一目标
Action 失败或精度超差时会停止后续队列。

模拟抓取链使用 `/competition/mock_pick_object`，只假定抓取成功和物体已附着，不启动
真实 2D 检测、SAM2、GraspNetBaseline、MoveIt 抓取或夹爪。GUI 仅在该结果成功后调用
`/retreat_from_station`，退避成功并停稳后才下发下一段 `/navigate_to_pose`。当前地图
必须先保存至少两个不同且经过安全掩膜和现场确认的语义站点；停止定位导航会同步取消
正在执行的模拟抓取或退避。

## 4. GUI 节点的 ROS 2 接口

### 4.1 直接订阅的话题

以下订阅都由 `NavigationRosNode` 创建。表中的“常见来源/用途”是按系统命名和代码用途
说明，实际发布节点请在运行时用 `ros2 topic info -v <topic>` 确认。

| 话题 | 消息类型 | QoS | GUI 中的用途 |
|---|---|---|---|
| `/odom_safe` | `nav_msgs/msg/Odometry` | depth 20 | 安全里程计、底盘线/角速度、停车稳定判定、数据新鲜度检查 |
| `/odom` | `nav_msgs/msg/Odometry` | depth 20 | 轮式里程计位姿，用于和 LIO 里程计比较 |
| `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | sensor data | LIO 位姿，用于和轮式里程计比较 |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | depth 20 | Nav2 导航控制器输出速度 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | depth 20 | 原始/汇合后的速度；也会收到 GUI 自己发布的遥控速度 |
| `/cmd_vel_safe_in` | `geometry_msgs/msg/Twist` | depth 20 | 安全链中间速度，比赛界面用于识别预测限速 |
| `/cmd_vel_safe` | `geometry_msgs/msg/Twist` | depth 20 | 最终安全速度，显示在“最终速度”指标中 |
| `/joint_states` | `sensor_msgs/msg/JointState` | depth 20 | 读取四个转向关节的实际角度 |
| `/swerve_drive_controller/debug_states` | `std_msgs/msg/Float64MultiArray` | depth 20 | 四舵轮请求角、实际角、误差、速度和缩放比例 |
| `/scan` | `sensor_msgs/msg/LaserScan` | sensor data | 不解析扫描内容，只记录最后到达时间，判断障碍感知是否在线 |
| `/localization_hold` | `std_msgs/msg/Empty` | depth 10 | 统计定位保持/保护事件次数 |
| `/localization_ok` | `std_msgs/msg/Bool` | reliable + transient local，depth 1 | 定位许可门；为 false 或从未收到时禁止发目标 |
| `/plan` | `nav_msgs/msg/Path` | reliable + volatile，depth 1 | 绘制 Nav2 全局路径，计算目标误差和最近离散路径横差 |

`/swerve_drive_controller/debug_states` 至少需要 45 个元素。GUI 使用：

- `data[3]`：全局驱动缩放；`data[4]`：最大转向角误差；
- 每个车轮占 10 个值，从 `5 + wheel_index * 10` 开始；
- 块内偏移 `0/1/2/3`：请求角、命令角、实际角、角误差；
- 块内偏移 `5/6/7/9`：目标速度、命令速度、实际速度、转向缩放。

车轮顺序固定为 `FL`、`FR`、`BL`、`BR`；`/joint_states` 中查找的关节名为
`fl_steering_joint`、`fr_steering_joint`、`bl_steering_joint`、
`br_steering_joint`。

### 4.2 直接发布的话题

| 话题 | 消息类型 | QoS | 发布条件与内容 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | depth 10 | 建图遥控约 20 Hz；停止、松键和退出时主动发布零速度 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | reliable + transient local，depth 1 | `frame_id=map`；位置协方差 X/Y 为 `0.25`，Yaw 为 `0.0685` |
| `/openflex_navigation_gui/goal_pose` | `geometry_msgs/msg/PoseStamped` | depth 10 | 每次发送 Action 目标时发布诊断镜像，`frame_id=map` |

### 4.3 服务客户端

| 服务 | 类型 | 请求 | 响应处理 |
|---|---|---|---|
| `/pgo/save_maps` | `interface/srv/SaveMaps` | `file_path=<3d 地图目录>`，`save_patches=true` | 检查 `success` 和 `message`；成功后可继续二维转换 |

GUI 不会等待服务无限上线。点击保存时如果 `service_is_ready()` 为 false，会提示先启动建图。

### 4.4 Action 客户端

| Action | 类型 | 目标 | 反馈/结果 |
|---|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `PoseStamped`，`frame_id=map` | 使用 `distance_remaining`；跟踪 accepted/rejected/succeeded/error，并支持取消 |
| `/shelf_fine_align` | `openarmx_task_msgs/action/ShelfFineAlign` | 站点、5 cm 净距、角度/超时门槛 | 显示净距、角差和稳定帧；成功后才完成货架站判定 |
| `/competition/mock_pick_object` | `openarmx_task_msgs/action/PickObject` | 模拟物品、手臂标签 | 只显示模拟进度与假定附着结果；不控制抓取硬件 |
| `/retreat_from_station` | `openarmx_task_msgs/action/RetreatFromStation` | 站点、距离、低速和超时 | 显示已退距离；受定位、里程计、漂移和授权门保护，并支持取消 |

代码把 Action 状态码 `4` 记为 `succeeded`，其他完成状态显示为 `finished:<status>`。

### 4.5 TF 与 ROS 节点图

`TransformListener` 会通过标准 `/tf` 和 `/tf_static` 接收 TF，但代码没有手工创建这两个
订阅。每秒查询：

- `map -> base_link`：机器人在地图中的位置与朝向，用于轨迹、目标误差、站点捕获和验收；
- `map -> odom`：全局定位对里程计系的修正；相邻采样的平移/偏航变化作为“ICP 单步修正”。

同一周期还调用 `get_node_names_and_namespaces()` 采样整个 ROS 图。节点图用于：

- 发现外部 `/controller_manager`，阻止重复启动硬件控制栈；
- 显示 `/pgo/pgo_node`、`/icp_registration`、`/planner_server`、
  `/controller_server`、`/bt_navigator`、`/collision_monitor` 等关键节点；
- 判断外部建图或导航模式；
- 禁止在 Nav2 使用静态地图时保存编辑后的 PGM；
- 在比赛目标下发前检查 controller、collision monitor 和速度安全门。

这些是“观察到的节点”，不是 GUI 对节点建立的订阅。

## 5. 诊断录制

默认录制根目录为：

```text
~/openflex_nav_records/nav_<YYYYmmdd_HHMMSS>/
├── bag/                    rosbag2 数据
├── params/                 关键节点 ros2 param dump
├── recorded_at.txt         带时区的录制开始时间
├── manifest.json           地图、初始位姿和录制话题
├── topics.txt              ros2 topic list -t
├── nodes.txt               ros2 node list
├── actions.txt             ros2 action list -t
└── bag_info.txt            停止后生成的 ros2 bag info
```

可用环境变量修改目录：

```bash
export NAV_RECORD_ROOT=/data/openflex_nav_records
```

录制命令使用 `--include-hidden-topics`、50 MiB 缓存，并录制 `model.py` 中的
`RECORD_TOPICS`：

```text
/tf  /tf_static  /map  /initialpose  /goal_pose
/openflex_navigation_gui/goal_pose
/plan  /plan_smoothed  /local_plan  /predicted_path  /astar_detour_path
/odom  /odom_safe  /fastlio2/lio_odom  /fastlio2/raw/odom
/fastlio2/body_cloud  /fastlio2/world_cloud
/livox/body_cloud_safety  /livox/body_cloud_nav2  /livox/imu
/cmd_vel_nav  /cmd_vel  /cmd_vel_safe_in  /cmd_vel_safe
/joint_states  /dynamic_joint_states  /swerve_drive_controller/debug_states
/scan  /local_costmap/costmap_raw  /global_costmap/costmap_raw
/global_costmap/voxel_marked_cloud
/dyn_obj  /dyn_obj_removed  /dyn_obj_dbg_hist  /cloud_effected  /Laser_map
/localization_ok  /localization_hold  /localization_costmap_cleared
/collision_monitor/footprint_approach  /speed_limit  /diagnostics  /rosout
/behavior_tree_log
/navigate_to_pose/_action/feedback  /navigate_to_pose/_action/status
/follow_path/_action/feedback  /follow_path/_action/status
```

录制某个当前不存在的话题不会让 GUI 订阅它，也不代表系统必须提供全部话题；rosbag 只会
保存录制期间实际出现的数据。

参数导出目标包括：

```text
/controller_server
/swerve_drive_controller
/icp_registration
/safe_odom_mux
/cmd_vel_safety_gate
/localization_costmap_guard
/fastlio2/lio_bridge
/collision_monitor
/local_costmap/local_costmap
/global_costmap/global_costmap
/bt_navigator
```

## 6. 地图目录与文件

默认根目录为 `~/openflex_all/openflex_maps`，界面也预置
`~/MID_360_nav/openflex_maps` 作为可选项。结构为：

```text
<map_root>/
├── 3d/<map_name>/
│   ├── map.pcd
│   ├── poses.txt                         可选；控制二维转换的位姿裁剪
│   └── ...                               PGO patches/ScanContext 等输出
└── 2d/<map_name>/
    ├── map.yaml
    ├── map.pgm
    ├── competition_mission.yaml          比赛界面保存的语义站点
    └── backups/
        └── map_<timestamp>.pgm           地图编辑器保存的旧版本
```

地图名称只能包含 `A-Z`、`a-z`、`0-9`、`.`、`_`、`-`。地图下拉列表由已有
`3d/*/map.pcd` 和 `2d/*/map.yaml` 的父目录名合并生成，也可以手工输入新名称。

## 7. 二维地图参数

工作台默认使用 `supermarket_test_02` 筛选后的参数组：

| 参数 | 默认值 | 传给转换程序的参数 |
|---|---:|---|
| 分辨率 | `0.05 m` | `--resolution` |
| 投影高度 | `-0.35` 至 `2.80 m` | `--z-min` / `--z-max` |
| 障碍高度 | `-0.18` 至 `1.80 m` | `--obstacle-z-min` / `--obstacle-z-max` |
| 地图膨胀 | `0.00 m` | `--inflation-radius` |
| 障碍最少命中 | `2` | `--obstacle-min-hits` |
| 空闲最少命中 | `3` | `--free-min-hits` |
| SOR 邻居数 | `16` | `--sor-mean-k` |
| SOR 标准差倍数 | `1.5` | `--sor-stddev` |
| 最小障碍簇 | `3` | `--min-obstacle-cluster` |
| 位姿裁剪边界 | `2.5 m` | `--crop-to-poses-margin`，仅 `poses.txt` 存在时 |
| 射线最大距离 | `4.5 m` | `--raycast-max-range` |

静态地图默认不膨胀，安全距离由 Nav2 costmap inflation layer 处理。不同场地、雷达安装
高度和地面高度需要重新标定 Z 范围。参数默认锁定；勾选“解锁参数编辑”只影响当前会话，
重新打开 GUI 后恢复锁定。

## 8. 商超比赛基准台

### 8.1 启动

安装后可直接运行入口：

```bash
ros2 run openflex_navigation_gui openflex_competition_navigation_gui
```

项目上层如果提供启动脚本，也可使用：

```bash
cd ~/openflex_all/openflex_ws
./start_supermarket_competition_gui.sh
```

全身 `supermarket_test_07` 联调入口为：

```bash
./start_supermarket_fullbody_competition_gui.sh
```

该入口默认加载 `supermarket_test_07_competition.yaml` 与
`stations_supermarket_test_07.yaml`，并由“启动全身控制”启动模拟抓取和退避服务。
真实 `/detect_object` 和 `/pick_object` 仍需由 Qwen/OWLv2、SAM2、
GraspNetBaseline 与 MoveIt 适配器提供。

默认档案由 `ament_index` 查找：

```text
<swerve_navigation share>/config/supermarket_test_05_competition.yaml
```

可覆盖为其他场地档案：

```bash
export OPENFLEX_COMPETITION_PROFILE=/path/to/profile.yaml
ros2 run openflex_navigation_gui openflex_competition_navigation_gui
```

档案中的 `map_root`、`map_name` 和 `launch_file` 决定比赛界面实际使用的地图和 launch。
每张地图首次没有 `competition_mission.yaml` 时：若地图名等于档案地图名，界面使用档案
航点作为初始值；否则站点列表为空。编辑站点后会保存为该地图自己的任务文件。

### 8.2 比赛精度数据来源

| 指标 | 数据来源 | 计算方式 |
|---|---|---|
| 最终位置/朝向误差 | TF `map -> base_link` 与目标位姿 | 欧氏距离与归一化偏航差 |
| 路径横差 | `/plan` 与 TF | 当前位姿到最近离散路径点的距离，记录单目标最大值 |
| ICP 单步修正 | TF `map -> odom` | 相邻 1 秒采样间的平移和偏航变化，记录最大值 |
| 停车状态 | `/odom_safe` | 线速度模长和 `abs(angular.z)` 低于档案阈值并保持指定时间 |
| 安全链介入提示 | `/cmd_vel` 与 `/cmd_vel_safe_in` | 原始速度明显非零且后者范数小于前者 75% |

路径横差是到 `/plan` 离散位姿的最近点距离，不是线段投影距离；ICP 指标受 1 Hz TF
采样周期影响。它们适合工作流验收和回归对比，不应替代离线高精度真值系统。

## 9. 安装、构建与启动

### 9.1 依赖

`package.xml` 声明的主要依赖包括：

- ROS：`rclpy`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`std_msgs`、
  `diagnostic_msgs`、`action_msgs`、`nav2_msgs`、`tf2_ros`；
- 项目接口：`interface` 中的 `SaveMaps` 服务；
- Python/GUI：`python3-pyqt5`、`python3-yaml`、`ament_index_python`；
- 工作流：`swerve_navigation`、`pgo`、`rosbag2`。

### 9.2 构建

```bash
cd ~/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openflex_navigation_gui --symlink-install
source install/setup.bash
```

### 9.3 通用工作台

```bash
ros2 run openflex_navigation_gui openflex_navigation_gui
```

`setup.py` 注册了两个控制台入口：

| 命令 | Python 入口 |
|---|---|
| `openflex_navigation_gui` | `openflex_navigation_gui.main_window:main` |
| `openflex_competition_navigation_gui` | `openflex_navigation_gui.competition_window:main` |

GUI 使用 `QSettings("OpenArmX", ...)` 保存地图根目录、地图名、初始位姿、自动录制开关和
比赛巡游重复次数。具体配置文件位置由 Qt 和桌面环境决定。

## 10. 运行时检查与排障

### 10.1 查看 GUI 实际订阅和发布

```bash
ros2 node info /openflex_navigation_gui
ros2 topic info -v /odom_safe
ros2 topic info -v /localization_ok
ros2 action info /navigate_to_pose
ros2 service type /pgo/save_maps
```

### 10.2 “没有发现 `/pgo/save_maps`”

- 确认建图 launch 没有提前退出；
- 用 `ros2 service list | grep pgo` 检查服务；
- 确认 GUI 与建图系统使用相同的 `ROS_DOMAIN_ID` 和 RMW；
- 查看 GUI “运行日志”页中的建图进程最后输出。

### 10.3 无法发送导航目标

- `ros2 topic echo --once /localization_ok` 应返回 `data: true`；
- `ros2 action list -t` 中应存在 `/navigate_to_pose`；
- 比赛界面还要求 `/scan`、`/odom_safe` 在 0.5 秒内更新，并存在安全链关键节点；
- 系统页的“地图定位 TF”应持续正常，TF 必须能查询 `map -> base_link`。

### 10.4 位姿或地图显示异常

- 检查 `map.yaml` 的 `resolution` 和 `origin` 是否有效；
- 当前地图显示与选点假设 `origin[2] == 0`；
- 确认 PGM 宽高和数据长度正确，支持的格式为 8-bit `P5`/`P2`；
- 运行 `ros2 run tf2_ros tf2_echo map base_link` 检查定位 TF。

### 10.5 无法编辑或保存二维地图

- 必须先停止 GUI 导航进程；
- ROS 图中存在 `map_server`、planner、controller 或 BT navigator 时也会禁止保存；
- 确认 `map.pgm` 与 `map.yaml` 同时存在且当前用户可写；
- 备份位于同一地图目录下的 `backups/`，可手工复制旧 PGM 恢复。

## 11. 安全约束与已知边界

- “停止速度”只发布零 `Twist`，不能代替机器人硬件急停；
- GUI 只停止自己启动的进程，不会强制终止外部 launch；
- 检测到外部 `/controller_manager` 时禁止重复启动建图或导航；
- 手动遥控仅用于建图流程，且必须显式勾选“启用遥控”；
- 清除静态障碍会改变全局规划结果。实时 `/scan`、局部代价地图和
  `collision_monitor` 仍可能保护机器人，但玻璃等物体可能无法被激光可靠检测；
- 地图编辑器只改写 PGM，不修改 YAML；已启动的 `map_server` 不会自动重载；
- GUI 本身没有声明 ROS 参数，界面设置通过 Qt 保存，launch 参数通过命令行传给外部包；
- 目标和初始位姿固定使用 `map` 坐标系，TF 查询固定使用 `base_link` 与 `odom`；若系统
  使用其他 frame 名称，需要修改代码或在 TF 树中提供相应变换。
