# OpenArmX 商超完整比赛真值仿真

> 换机续做请先阅读 [CONTINUATION_HANDOFF_CN.md](CONTINUATION_HANDOFF_CN.md)。该文件记录
> 2026-07-29 的准确完成边界、最后修改、验证结果和下一步优先级。

本包用正式 ROS 2 接口运行三环节比赛。`CompetitionCoordinator` 和
`ExecuteTransport` 不做仿真特判；仿真节点替代它们依赖的外部能力：

- `/navigate_to_pose`、`/dock_to_station`
- `/read_delivery_order`、`/inspect_retail_scene`
- `/detect_object`、`/pick_object`、`/place_object`
- `/verify_placement`、`/set_perception_mode`

每次复位随机抽取两个清单 SKU、两个缺货位和两个乱放商品。虚拟裁判按正式规则一次
只发送一个环节开始指令，并统计 40/55/55 分以及碰撞、越界、掉落扣分。

## 构建

```bash
cd /home/openarmx/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to openarmx_supermarket_sim
source install/setup.bash
```

## 自动跑完整场

```bash
ros2 launch openarmx_supermarket_sim competition_truth_sim.launch.py \
  auto_run:=true random_seed:=20260729
```

默认 `time_scale:=0.02`，完整比赛会被加速；第三环节结束并写完报告后，launch 会自动
正常退出。传入 `exit_on_complete:=false` 可在赛后保持节点运行。结束报告写入：

```text
/tmp/openarmx_supermarket_sim_report.json
```

查看实时状态和成绩：

```bash
ros2 topic echo /sim_competition/status --once
```

## 手动模拟裁判

启动时传入 `auto_run:=false`，然后每次裁判确认后调用一次：

```bash
ros2 service call /sim_competition/start_next_phase std_srvs/srv/Trigger '{}'
```

复位并生成下一随机场：

```bash
ros2 service call /sim_competition/reset std_srvs/srv/Trigger '{}'
```

故障和扣分注入：

```bash
ros2 service call /sim_competition/inject_collision std_srvs/srv/Trigger '{}'
ros2 service call /sim_competition/inject_drop std_srvs/srv/Trigger '{}'
ros2 service call /sim_competition/inject_out_of_bounds std_srvs/srv/Trigger '{}'
```

## MuJoCo 商超模型

仓库同时提供 OpenFlex 整机与完整商超场景的 MJCF 合并器。它会处理两份模型的相对
资源路径、角度单位和重名资源，移除机器人演示用小地板，加入 `base_x`、`base_y`、
`base_yaw` 平面底盘关节以及头部、左右腕部相机：

```bash
ros2 run openarmx_supermarket_sim build_competition_model
```

默认输出：

```text
/tmp/openarmx_supermarket_competition.xml
```

也可以显式指定输入和输出：

```bash
ros2 run openarmx_supermarket_sim build_competition_model \
  --robot-model /home/openarmx/openflex_all/openflex_ws/src/openflex_mujoco/openflex_mujoco.xml \
  --supermarket-model /home/openarmx/openflex_all/supermarket_scene/supermarket_scene_from_db.xml \
  --output /tmp/openarmx_supermarket_competition.xml
```

该入口生成可供 MuJoCo 加载的物理场景，但本机还需要安装 `mujoco` Python 运行时，
并接入 ROS 关节控制、RGB-D 发布和抓取接触约束后，才能把真值能力服务器逐项替换为
物理能力服务器。

## 场景地图与货架注册

仿真以比赛全场的下半场作为本队赛区：`map` 坐标与 MuJoCo 世界坐标完全一致，范围
为 X `[-3.15, 3.15]`、Y `[-4.15, 0.05]`。场景中的货架组 3/4 分别注册为四个
货架面 A/B/C/D，左侧操作台为交付台，右侧操作台为补货台。

生成并由 Nav2 `map_server` 发布地图：

```bash
ros2 launch openarmx_supermarket_sim supermarket_map.launch.py
```

只生成地图并验证全部站位：

```bash
ros2 run openarmx_supermarket_sim build_supermarket_map \
  --layout $(ros2 pkg prefix openarmx_supermarket_sim)/share/openarmx_supermarket_sim/config/scene_layout_bottom.yaml \
  --stations $(ros2 pkg prefix openarmx_task_executor)/share/openarmx_task_executor/config/stations_supermarket_mujoco.yaml \
  --output-directory /tmp/openflex_supermarket_map
```

地图会在每次启动时从 MJCF 静态几何重新投影，因此货架或操作台发生移动时不会留下
过期地图。生成过程还会使用真实底盘足迹检查每个导航站位，发生几何穿透时拒绝启动。

## 两级验证与证据边界

- `truth_mode`：当前已完整运行。证明三环节状态机、裁判门控、接口组合、任务顺序、
  路线、工单、失败闭锁和计分逻辑能够闭环。
- `mujoco_physics`：当前完成整机与商超模型合并入口；尚未完成 ROS 物理适配器，不能
  作为真实局部检测、GraspNetBaseline、MoveIt、相机真实性或接触物理成功率证据。

后续高保真模式应保持本包的裁判和计分节点，只逐项替换真值能力服务器，并确保机器人
侧节点无法读取 `CompetitionWorld` 隐藏状态。

## MuJoCo 物理与 RGB-D 桥

安装 MuJoCo 运行时后，可启动完整整机、250 个商品和三路相机的物理桥：

```bash
python3 -m pip install --user 'mujoco>=3.3,<4'
ros2 launch openarmx_supermarket_sim mujoco_physics.launch.py
```

桥接接口包括：

- Nav2：`/cmd_vel`、`/odom_safe`、`map -> odom -> base_link`；
- MoveIt：升降、左右臂 `FollowJointTrajectory` 和左右夹爪 `GripperCommand`；
- 感知/GraspNetBaseline：`/camera`、`/cam_left`、`/cam_right` 三路对齐 RGB-D；
- MoveIt Octomap：`/head_mount_d435/depth_registered/image_raw`；
- 物理事件：`/sim_competition/physics_events`。

商品使用独立碰撞位：商品与机器人、货架、地面碰撞，但相邻商品之间不互撞，以避免
250 个高精 mesh 在货架密集摆放时产生数值爆炸。当前抓取附着仍使用“接近距离 + 夹爪
闭合”的稳定约束代理；它比真值瞬移更接近物理执行，但不能替代夹爪接触参数标定。
