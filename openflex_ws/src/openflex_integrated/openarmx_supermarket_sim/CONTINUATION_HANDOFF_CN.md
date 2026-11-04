# OpenArmX 商超比赛仿真续做交接（2026-07-29）

> **历史快照，不是当前比赛运行说明。** 本文记录 2026-07-29 时的 AnyGrasp 方案，
> 其中相关包、路径、启动参数和 GPU 策略均已退出当前比赛链路。当前迁移以
> `GRASP_CONSOLE_MIGRATION_EXECUTION_PLAN_CN.md` 为准，只使用 GraspNetBaseline；
> 不得照本文恢复 AnyGrasp SDK、worker、checkpoint 或 launch。

本文是迁移到另一台电脑继续开发的唯一断点说明。当前目标是：比赛全场 MuJoCo、注册地图、
Nav2、DREAM、AnyGrasp、MoveIt 和三阶段比赛执行器最终形成不读取隐藏真值的物理闭环。

## 1. 当前结论

项目已经完成“真值模式完整比赛”和“物理模式基础设施”，但**物理模式还不能完整自动跑比赛**。
不要把当前状态描述成 DREAM/AnyGrasp/MoveIt 已经端到端通过。

当前最明确的断点如下：

- 真值模式：三阶段、裁判门控、随机任务、计分和报告已经闭环。
- MuJoCo：整机、商超、约 250 个商品、底盘、双臂、夹爪和三路 RGB-D 已能合并、加载。
- 地图：比赛下半场地图和货架/操作台/导航站位已经注册到同一 `map == MuJoCo world` 坐标。
- Nav2、MoveIt、DREAM observation bridge、AnyGrasp worker 已有统一启动骨架。
- 尚缺物理比赛高层适配器：`/detect_object`、`/pick_object`、`/place_object`，以及真实版
  `ReadOrder`、`InspectRetailScene`、`VerifyPlacement`、`SetPerceptionMode`。
- 因为上述 Action 服务器缺失，新总启动文件的 `start_task_api` 默认保持 `false`。

## 2. 已完成内容

### 2.1 真值比赛闭环

主要文件：

- `openarmx_supermarket_sim/openarmx_supermarket_sim/sim_world.py`
- `openarmx_supermarket_sim/openarmx_supermarket_sim/sim_node.py`
- `openarmx_supermarket_sim/launch/competition_truth_sim.launch.py`
- `openarmx_supermarket_sim/config/world.yaml`
- `openarmx_supermarket_sim/config/simulation.yaml`
- `openarmx_task_executor/openarmx_task_executor/competition_coordinator_node.py`
- `openarmx_task_executor/openarmx_task_executor/task_executor_node.py`
- `openarmx_task_executor/config/competition.yaml`

该模式可以验证三阶段流程、任务顺序、失败闭锁和计分，但不能作为物理感知/抓取证据。

### 2.2 MuJoCo 完整场景

`competition_model_builder.py` 已实现：

- 合并 `/home/openarmx/openflex_all/openflex_ws/src/openflex_mujoco/openflex_mujoco.xml`；
- 合并 `/home/openarmx/openflex_all/supermarket_scene/supermarket_scene_from_db.xml`；
- 资源绝对路径化、场景资源命名空间化、角度单位统一；
- 增加 `base_x/base_y/base_yaw` 平面底盘关节；
- 保留商品与机器人/货架/地面的碰撞，抑制商品之间的密集互撞；
- 增加底盘 D435、左腕和右腕三个 MuJoCo camera；
- 最新修改给 camera 增加 `quat="0 1 0 0"`，用于对齐 MuJoCo `-Z/+Y` 与 ROS optical
  `+Z/-Y` 约定，避免点云镜像。

快速检查结果：

```text
MODEL_OK nbody=407 njnt=282 ncam=7
```

### 2.3 地图、货架和站位注册

主要文件：

- `openarmx_supermarket_sim/openarmx_supermarket_sim/map_builder.py`
- `openarmx_supermarket_sim/config/scene_layout_bottom.yaml`
- `openarmx_task_executor/config/stations_supermarket_mujoco.yaml`
- `openarmx_supermarket_sim/launch/supermarket_map.launch.py`
- `openarmx_supermarket_sim/launch/supermarket_nav2.launch.py`
- `openarmx_supermarket_sim/config/nav2_sim.yaml`

坐标范围为 X `[-3.15, 3.15]`、Y `[-4.15, 0.05]`。货架组 3/4 注册为 A/B/C/D，
左右操作台注册为交付台和补货台。地图由 MJCF 几何投影，并用真实底盘 footprint 验证站位。

快速检查结果：

```text
MAP_OK 126x84 resolution=0.05
```

### 2.4 MuJoCo 物理桥

主要文件：

- `openarmx_supermarket_sim/openarmx_supermarket_sim/physics_bridge_node.py`
- `openarmx_supermarket_sim/config/physics.yaml`
- `openarmx_supermarket_sim/launch/mujoco_physics.launch.py`

已提供：

- `/cmd_vel` 到平面底盘，发布 `/odom_safe` 和 `map -> odom -> base_link`；
- 升降、左右臂 `FollowJointTrajectory`；
- 左右夹爪 `GripperCommand`；
- `/joint_states`、`/clock`；
- 三路渲染 RGB-D；
- 商品“近距离 + 夹爪闭合”的稳定附着代理；
- 碰撞事件 `/sim_competition/physics_events`。

最新修改统一了相机契约：

- 深度改成 RealSense/AnyGrasp 可直接读取的 `16UC1` 毫米；
- DREAM 主相机：`/camera/...`；
- 左腕别名：`/cam_left/...` 与 `/camera/left_camera/...`；
- 右腕别名：`/cam_right/...` 与 `/camera/right_camera/...`；
- DREAM 深度兼容别名：`/camera/depth/image_raw`；
- MoveIt Octomap：`/head_mount_d435/depth_registered/image_raw`；
- 消息 frame 改用机器人描述中已有的 `d435_color_optical_frame`、
  `openarmx_left_link7_pico`、`openarmx_right_link7_pico`。

### 2.5 MoveIt

`openarmx_integrated_moveit_config/launch/sim_moveit.launch.py` 已能接到 MuJoCo 的
`/joint_states`、TF、轨迹 Action 和夹爪 Action。

完整栈中必须传：

```text
start_base_navigation_bridge:=false
start_command_bridge:=false
```

原因：Nav2 是唯一 `/navigate_to_pose` 所有者，MuJoCo 是唯一控制器 Action 所有者。

### 2.6 AnyGrasp 仿真契约

新增：

- `openarmx_supermarket_sim/config/anygrasp_sim.yaml`

它使用右腕渲染 RGB-D：

```text
/camera/right_camera/color/image_raw
/camera/right_camera/aligned_depth_to_color/image_raw
/camera/right_camera/color/camera_info
frame = openarmx_right_link7_pico
depth_unit_m = 0.001
```

该配置把结果标记为 rendered physical RGB-D，而不是 oracle/truth grasp。仍须用 RViz、TF 和
已知物体姿态验证 `graspnet_to_tcp_rpy_rad` 后，才能把它作为最终执行标定证据。

### 2.7 新的物理栈总启动入口

新增：

- `openarmx_supermarket_sim/launch/competition_physics_stack.launch.py`

它可以按顺序启动：

1. MuJoCo 场景和 RGB-D；
2. 注册地图和 Nav2；
3. MoveIt；
4. 可选 RGB-D-only RTAB-Map + DREAM observation server；
5. 可选 AnyGrasp worker + `/thesis/generate_grasps` gateway；
6. 可选比赛 task API（默认关闭，因为高层物理 Action 尚缺）。

重要：`start_dream:=true` 当前启动的是 RGB-D/里程计接入与 observation server，不等于已经
启动完整的动态语义查询客户端。DREAM 的目标查询仍需接成比赛 Action，不能靠固定环境变量写死 SKU。

### 2.8 启动前自检

新增：

- `openarmx_supermarket_sim/openarmx_supermarket_sim/preflight.py`
- console entry：`competition_preflight`

它检查模型加载、商品数量、地图/站位、Nav2、MoveIt、AnyGrasp SDK/checkpoint/Python、
DREAM workspace 和 NVIDIA 设备。

## 3. 本次最后修改的文件

```text
openarmx_supermarket_sim/openarmx_supermarket_sim/competition_model_builder.py
openarmx_supermarket_sim/openarmx_supermarket_sim/physics_bridge_node.py
openarmx_supermarket_sim/openarmx_supermarket_sim/preflight.py                 (new)
openarmx_supermarket_sim/config/anygrasp_sim.yaml                              (new)
openarmx_supermarket_sim/launch/competition_physics_stack.launch.py            (new)
openarmx_supermarket_sim/test/test_competition_model_builder.py
openarmx_supermarket_sim/setup.py
openarmx_supermarket_sim/CONTINUATION_HANDOFF_CN.md                             (new)
```

这些最后修改尚未在当前机器重新 `colcon build` 到 install space；换机后必须先构建。

## 4. 当前验证状态与已知环境问题

已经通过：

- `python3 -m compileall`；
- 所有 `openarmx_supermarket_sim/launch/*.launch.py` 导入检查；
- 合并模型由 MuJoCo 3.11.0 成功加载；
- 地图生成和站位 footprint 验证。

尚未通过/未执行：

- 当前系统 Python、DREAM conda、AnyGrasp conda 都没有 `pytest`，因此没有重新跑 pytest；
- 当前机器 `nvidia-smi` 无法连接 NVIDIA driver，`/dev/nvidia0` 不可用；
- 未在最后修改后执行完整 `colcon build`；
- 未运行物理全栈端到端比赛；
- 未验证 EGL 渲染帧率、AnyGrasp 实际推理、DREAM 语义查询和 MoveIt 真正抓取成功率。

当前顶层 `.git` 元数据不可用/为空，不能依赖 `git status` 找出上述改动。迁移时应复制整个相关
源码目录，或者在有正常 Git 元数据的机器手工纳入这些文件。

## 5. 换机后第一轮操作

必须同时迁移或重新准备以下外部目录：

```text
/home/openarmx/openflex_all/openflex_ws/src/openflex_integrated
/home/openarmx/openflex_all/openflex_ws/src/openflex_mujoco
/home/openarmx/openflex_all/openflex_ws/src/openflex_thesis
/home/openarmx/openflex_all/supermarket_scene
/home/openarmx/openflex_all/DREAM-realtime
/home/openarmx/openflex_all/anygrasp_sdk_v3
```

构建：

```bash
cd /home/openarmx/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to openarmx_supermarket_sim
source install/setup.bash
```

如果要启用 DREAM，还要准备并 source：

```bash
source /home/openarmx/openflex_all/DREAM-realtime/install/setup.bash
```

先跑自检：

```bash
ros2 run openarmx_supermarket_sim competition_preflight
```

仅启动物理、Nav2、MoveIt（第一轮推荐）：

```bash
ros2 launch openarmx_supermarket_sim competition_physics_stack.launch.py \
  start_dream:=false start_anygrasp:=false start_task_api:=false \
  use_moveit_rviz:=true
```

核对基础接口：

```bash
ros2 topic hz /joint_states
ros2 topic hz /odom_safe
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/right_camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/right_camera/color/camera_info --once
ros2 action list
ros2 node list
```

再逐项开启 GPU 模块：

```bash
ros2 launch openarmx_supermarket_sim competition_physics_stack.launch.py \
  start_dream:=true start_anygrasp:=true start_task_api:=false
```

不要在 `/detect_object`、`/pick_object`、`/place_object` 适配器完成前启用
`start_task_api:=true`，否则比赛执行器会在 preflight 阶段正确失败。

## 6. 下一步开发顺序（按优先级）

### P0：验证 RGB-D 和 TF，不允许跳过

1. 用 RViz 同时显示右腕 RGB、深度点云、`openarmx_right_link7_pico` 和机器人模型；
2. 放置一个已知尺寸/姿态的商品，验证点云左右、上下、前后方向；
3. 验证 `base_link -> openarmx_right_link7_pico` 在机械臂运动时连续更新；
4. 验证主 D435 的 frame 和 MuJoCo 相机视点一致；
5. 如画面方向错误，只修 `competition_model_builder.py` 的 camera quat 或相机 frame，
   不要在 AnyGrasp 结果后面用经验旋转掩盖错误。

### P0：补物理比赛能力适配器

建议在 `openarmx_supermarket_sim` 新建 `physical_competition_capabilities.py`，提供比赛要求的
高层 Action，且禁止读取 `CompetitionWorld` 隐藏状态：

- `/detect_object`：DREAM/检测器输出转 `DetectObject`；
- `/pick_object`：目标内存 -> `/thesis/generate_grasps` -> MoveIt 抓取执行；
- `/place_object`：MoveIt 规划到站位的 place pose、张开夹爪、撤离并确认释放；
- `/read_delivery_order`：真实/渲染 OCR 或比赛输入；
- `/inspect_retail_scene`：DREAM 语义记忆转缺货/乱放工单；
- `/verify_placement`：渲染感知或独立物理观测验证稳定性；
- `/set_perception_mode`：串行化 DREAM 与 AnyGrasp GPU 所有权。

适配器必须满足：超时、取消、Action 状态、TF 时间戳、置信度阈值和 fail-closed。不能为了让
比赛“跑通”而直接返回真值成功。

### P1：MoveIt 抓取闭环

1. 明确选择 `/openflex/plan_pregrasp` 还是论文栈 `/thesis/execute_moveit_grasp` 作为唯一执行链；
2. 用一个固定商品先完成：AnyGrasp -> pregrasp -> grasp -> 夹爪 -> attach -> transport pose；
3. 让 MuJoCo 附着事件独立反馈抓取成功，而不是由请求本身宣告成功；
4. 实现 place、detach、稳定等待和独立验证；
5. 再扩展左右臂和不同货架层。

### P1：DREAM 动态比赛查询

1. 当前 DREAM observation server 已能接 RGB-D/odom，下一步要增加按 `object_id` 动态请求的
   ROS Action/Service；
2. 不要用 `TARGET_OBJECT` 环境变量写死整场目标；
3. 输出至少包含 object id、`PoseStamped`、confidence、observation id、时间戳和可选 mask；
4. mask 应接到 AnyGrasp `target_mask_topic`，并验证与 RGB-D 同步；
5. 第 2/3 阶段的场景巡检必须生成 `CompetitionWorkItem`，不能读取真值工单。

### P1：完整比赛 E2E

完成上述 Action 后：

1. 将能力节点加入 `competition_physics_stack.launch.py`；
2. 把 `start_task_api` 默认值改为 `true`；
3. 加入物理虚拟裁判，只负责发 phase start 和计分，不给机器人隐藏物体状态；
4. 依次跑 phase 1、2、3，再跑随机种子批量回归；
5. 保存 rosbag、DREAM observation id、AnyGrasp request/grasp id、MoveIt trajectory、碰撞/掉落事件和成绩。

## 7. 完成判据

只有同时满足以下条件，才能称为“完整比赛仿真”：

- 单一 `map -> odom -> base_link` TF 链，无重复发布者；
- Nav2 真正在注册地图上避障到全部站位；
- DREAM 使用渲染 RGB-D 生成动态语义结果；
- AnyGrasp 使用目标 RGB-D/mask 生成抓姿，不使用 oracle pose；
- MoveIt 真实规划并通过 MuJoCo 控制器执行；
- 抓取、放置、稳定性和碰撞由物理/独立观测确认；
- 三阶段均由同一比赛执行器完成，裁判只发送阶段开始；
- 至少多个随机种子和多次重复运行无 TF、Action、GPU 所有权冲突；
- 报告明确区分 truth、rendered perception、physics execution 和真实硬件证据。
