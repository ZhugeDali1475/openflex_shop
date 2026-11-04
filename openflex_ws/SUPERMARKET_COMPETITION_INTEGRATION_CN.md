# OpenFlex 商超比赛全流程与整机 GUI 教学说明

本文面向当前工作区：

```text
/home/openflex/old/openflex_ws
```

覆盖以下三个入口：

```text
src/openflex_integrated/openarmx_task_executor
src/openflex_integrated/openarmx_task_msgs
start_supermarket_fullbody_competition_gui.sh
```

目标是让操作员和各模块开发者从同一份说明中理解：比赛三阶段怎样编排、单件商品怎样搬运、ROS 2 Action 怎样衔接、当前 GUI 能执行到哪一步，以及现场应按什么顺序启动和停止。

## 1. 先看当前可用边界

当前代码同时存在“正式比赛架构”和“test_08 整机联调 GUI”，两者不能混为一谈。

| 能力 | 当前状态 | 说明 |
| --- | --- | --- |
| 三阶段裁判门控状态机 | 已实现 | `/competition/execute_phase` 强制按第一、第二、第三阶段顺序执行 |
| 单件商品完整搬运编排 | 已实现 | `/execute_transport` 编排导航、精停、检测、抓取、退避、导航、放置 |
| Nav2、货架激光精停、安全退避 | 已实现 | 当前 test_08 GUI 可以直接联调 |
| 货架观察姿态 | 已实现并会运动真实硬件 | GUI 会通过 MoveIt 驱动升降台和右臂到第 2/3 层示教姿态；这不是基于物体 TCP 的真实预抓取 |
| GUI 模拟抓取 | 已实现 | `/competition/mock_pick_object` 只返回“假定附着成功”，不运动夹爪 |
| GUI 第一阶段快捷流程 | 部分实现 | 实际执行“选 SKU -> 到货架 -> 精停 -> 真实预抓取 -> 人工确认模拟抓取 -> 退避 -> 到交付台” |
| GUI 真实检测、真实抓取、真实放置 | 未接入快捷流程 | 需要 `/detect_object`、`/pick_object`、`/place_object` 等外部服务 |
| GUI 第二阶段、第三阶段按钮 | 未实现 | 页面中明确显示“待实现” |
| test_08 正式三阶段配置 | 未完成 | 默认 `competition.yaml` 的站点 ID 与 test_08 站点库不匹配 |

特别注意：GUI“比赛流程”页的说明文字提到了识别、放置、交付台退避和返回起点，但当前代码在到达 `delivery_counter` 后就把第一阶段联调标记为完成，尚未执行真实放置、交付台退避或返回 `START_CORRIDOR`。

因此，当前 GUI 适合整机链路和安全流程联调，不等于已经具备无人值守完成三阶段正式比赛的能力。

## 2. 系统职责与唯一所有权

比赛模式采用单一所有权，避免多个节点同时控制同一硬件或同一 ROS 接口。

```text
整机运行时 supermarket_fullbody_runtime.launch.py
  |
  +-- 唯一 robot_state_publisher
  +-- 唯一 controller_manager
  +-- 底盘、升降、双臂、头部硬件
  +-- MoveIt 与货架姿态执行器
  +-- safe_odom_mux，唯一发布 odom -> base_link
  +-- openarmx_task_executor 任务接口

定位导航 supermarket_test_05_benchmark.launch.py
  |
  +-- Livox / LIO / ICP 定位
  +-- Nav2，唯一 /navigate_to_pose 服务端
  +-- collision_monitor
  +-- cmd_vel_safety_gate
  +-- 不再启动第二套底盘硬件
```

速度命令必须经过完整安全链：

```text
Nav2 / GUI 建图遥控 / 精停 / 退避
                  |
                  v
              /cmd_vel
                  |
                  v
          collision_monitor
                  |
                  v
          /cmd_vel_safe_in
                  |
                  v
       localization safety gate
                  |
                  v
           /cmd_vel_safe
                  |
                  v
      swerve_drive_controller
```

禁止把精停或退避直接重映射到 `/cmd_vel_safe_in` 或 `/cmd_vel_safe`，否则会绕过部分安全层。

## 3. 两个 ROS 包分别负责什么

### 3.1 `openarmx_task_msgs`

该包只定义接口，不执行机器人动作。它是导航、感知、MoveIt 和任务编排之间的稳定边界。

主要消息：

| 消息 | 用途 |
| --- | --- |
| `OrderLine.msg` | OCR/VLM 输出的一条已校验订单，包含 SKU、规格、数量和置信度 |
| `CompetitionWorkItem.msg` | 一个可独立执行的拣选、补货或归位工单 |
| `TaskError.msg` | 全系统统一错误码，例如导航失败、底盘未停稳、抓取失败、超时和取消 |

Action 按层级分组如下：

| 层级 | Action | 默认接口 |
| --- | --- | --- |
| 整场阶段 | `ExecuteCompetitionPhase` | `/competition/execute_phase` |
| 单件搬运 | `ExecuteTransport` | `/execute_transport` |
| 导航局部动作 | `DockToStation`、`ShelfFineAlign`、`RetreatFromStation` | `/dock_to_station`、`/shelf_fine_align`、`/retreat_from_station` |
| 感知与任务理解 | `ReadOrder`、`InspectRetailScene`、`DetectObject`、`SetPerceptionMode` | `/read_delivery_order`、`/inspect_retail_scene`、`/detect_object`、`/set_perception_mode` |
| MoveIt/操作 | `MoveLiftToLevel`、`MoveToShelfView`、`PickObject`、`PlaceObject`、`VerifyPlacement` | `/competition/move_lift_to_level`、`/competition/move_to_shelf_view`、`/pick_object`、`/place_object`、`/verify_placement` |

所有物体、抓取和放置位姿都使用 `geometry_msgs/PoseStamped`。提供方必须填写 `header.frame_id`；精停后的首轮集成优先使用 `base_link`。

### 3.2 `openarmx_task_executor`

该包负责任务顺序和安全门控，不直接发布手臂关节命令，也不实现 MoveIt 规划。

| 可执行节点 | 作用 |
| --- | --- |
| `task_executor` | 提供 `/execute_transport`，执行一件商品的完整搬运链 |
| `competition_coordinator` | 提供 `/competition/execute_phase`，执行裁判门控三阶段流程 |
| `station_docking` | 基于站点 marker TF 的局部闭环靠站 |
| `shelf_fine_align` | 基于 `/scan` 左右前角距离的货架前向精停 |
| `station_retreat` | 抓取或放置后的受控直线后退 |
| `shelf_view` | 把站点层号映射为实测升降位置和手臂姿态，再调用 MoveIt |
| `lift_level_test` | 单独测试升降层号映射 |
| `competition_mock_pick` | 无硬件动作的模拟抓取成功服务 |

关键配置：

| 文件 | 作用 |
| --- | --- |
| `config/competition.yaml` | 三阶段时间、站点、巡查路线、SKU 路由和感知模式策略 |
| `config/stations_supermarket_test_08.yaml` | test_08 的导航位姿、精停策略、退避距离、放置位姿和层级示教姿态 |
| `config/inventory_supermarket_scene.example.yaml` | 正式协调器接入只读货架 SQLite 数据库的示例片段 |
| `launch/competition_task_api.launch.py` | 启动执行器、精停、退避、模拟抓取、协调器等任务接口 |
| `launch/supermarket_fullbody_runtime.launch.py` | GUI 使用的整机运行时，不含定位和 Nav2 |
| `launch/supermarket_full_system.launch.py` | 命令行一次性启动整机、导航和任务接口的替代入口 |

## 4. 正式比赛三阶段总流程

正式比赛的公共入口是：

```text
/competition/execute_phase
openarmx_task_msgs/action/ExecuteCompetitionPhase
```

一次 Action goal 只允许启动一个阶段。状态机规则如下：

1. `run_id` 不能为空，同一场比赛的三个阶段必须使用同一个 `run_id`。
2. 初始只接受 `phase=1`，第一阶段成功后才接受第二阶段，第二阶段成功后才接受第三阶段。
3. 同一时刻只能有一个阶段运行。
4. 阶段失败或取消后会锁存失败阶段，不再接受后续阶段。
5. 只有在没有活动阶段时，才能调用 `/competition/reset_run` 回到第一阶段。
6. `dry_run=true` 只检查所需 Action Server，不发送运动命令，也不推进阶段。
7. 默认阶段预算为 420 秒、420 秒、360 秒，总预算为 1200 秒。
8. 第一、第二阶段成功后返回 `waiting_for_referee=true`；第三阶段成功后整场结束。

总流程：

```text
裁判第一次开始
  -> 第一阶段：商品拣选
  -> 到 phase_1_judging
  -> 硬等待裁判

裁判第二次开始
  -> 第二阶段：货架补货
  -> 到 phase_2_judging
  -> 硬等待裁判

裁判第三次开始
  -> 第三阶段：巡查归位
  -> 到 finish
  -> 整场结束
```

### 4.1 第一阶段：商品拣选

配置目标是读取两种不同 SKU、总数量为 2，并逐件送到交付台。

```text
Nav2 到 delivery_counter
  -> 可选 DockToStation
  -> 确认 /odom_safe 新鲜且底盘持续停稳
  -> SetPerceptionMode(TARGET_REFINE)
  -> ReadOrder
  -> 校验 SKU 种类数、总数量、置信度和确定性路由
  -> SetPerceptionMode(LOCAL_GRASP)
  -> 对每个 CompetitionWorkItem 调用 /execute_transport
  -> VerifyPlacement
  -> SetPerceptionMode(IDLE)
  -> Nav2 到 phase_1_judging
  -> 等待下一次裁判指令
```

订单校验要求：

- 不同 SKU 数必须等于 `required_items`；
- 所有行的数量总和必须等于 `required_items`；
- 每行置信度不得低于 `min_order_confidence`；
- SKU 必须能从 `sku_routes` 或启用后的货架库存数据库解析出来源站点；
- 自然语言和 VLM 输出不能直接成为底盘或机械臂命令。

### 4.2 第二阶段：货架补货

```text
SetPerceptionMode(DREAM_OBSERVE)
  -> 按 scan_tour 逐站 Nav2 巡查
  -> 每站停稳后调用 InspectRetailScene(MISSING_STOCK)
  -> 合并多站证据，按 work_id/evidence_id 去重
  -> 按置信度排序，并保证目标货位不重复
  -> 得到恰好 required_items 个工单
  -> SetPerceptionMode(LOCAL_GRASP)
  -> 对每个工单调用 /execute_transport
  -> VerifyPlacement
  -> SetPerceptionMode(IDLE)
  -> Nav2 到 phase_2_judging
  -> 等待裁判
```

DREAM 负责跨视角记忆和候选召回，不直接控制 AnyGrasp、MoveIt 或底盘。

### 4.3 第三阶段：巡查归位

```text
SetPerceptionMode(DREAM_OBSERVE)
  -> 按第三阶段 scan_tour 巡查交付区、通道和补货区
  -> 每站调用 InspectRetailScene(MISPLACED_ITEMS)
  -> 将发现位置与货架数据库期望位置比较
  -> 生成两个“当前位置 -> 正确货位”工单
  -> SetPerceptionMode(LOCAL_GRASP)
  -> /execute_transport 逐件归位
  -> VerifyPlacement
  -> SetPerceptionMode(IDLE)
  -> Nav2 到 finish
  -> 比赛结束
```

## 5. 单件商品 `/execute_transport` 完整流程

三阶段协调器不会直接控制底盘或手臂，而是把每个工单交给：

```text
/execute_transport
openarmx_task_msgs/action/ExecuteTransport
```

执行顺序如下：

```text
PREFLIGHT
  -> Nav2 到来源站
  -> 等待底盘停稳
  -> 来源站精停
  -> 再次等待底盘停稳
  -> 可选 MoveToShelfView 货架观察姿态
  -> DetectObject 获取新鲜物体位姿
  -> TF 转到 base_link
  -> PickObject，要求 success=true 且 object_attached=true
  -> 按站点配置安全退避
  -> 等待底盘停稳
  -> Nav2 到目的站
  -> 等待底盘停稳
  -> 目的站精停
  -> 再次等待底盘停稳
  -> PlaceObject，要求 success=true 且 object_released=true
  -> 按站点配置安全退避
  -> DONE
```

### 5.1 预检

在底盘移动前，执行器会检查：

- `/navigate_to_pose`；
- `/pick_object`；
- `/place_object`；
- 来源/目的站需要的 `/shelf_fine_align` 或 `/dock_to_station`；
- 启用检测时的 `/detect_object`；
- 启用货架观察姿态时的 `/competition/move_to_shelf_view`；
- 站点启用退避时的 `/retreat_from_station`。

执行器一次只接受一个搬运任务，并接受取消请求。取消或超时会继续取消当前子 Action。

### 5.2 精停模式

站点 YAML 的 `alignment.mode` 决定精停方式：

| 模式 | 行为 |
| --- | --- |
| `shelf_scan` | Nav2 到达后调用 `/shelf_fine_align`，使用激光左右前角闭环前进 |
| `marker` | 调用 `/dock_to_station`，使 `station_marker_<marker_id>` 到达目标 TF 位姿 |
| `none` | 不执行局部精停，只使用 Nav2 终点精度 |

test_08 的八个货架站使用 `shelf_scan`；交付台和补货台使用 `none`，因为台面高度不在当前激光扫描平面内。

### 5.3 重试规则

`max_retries` 只用于：

- 去来源站的 Nav2；
- 来源站精停；
- 去目的站的 Nav2；
- 目的站精停。

物理抓取和放置不会自动重试。自动重复夹爪或手臂动作可能造成二次碰撞、掉落或重复夹持，必须由更高层根据现场状态重新规划。

### 5.4 抓取约束

任务执行器向 `PickObject` 发送：

- 新鲜检测得到的 `object_pose`；
- `source_slot` 和可选 `shelf_level`；
- 可选的实测 `nominal_lift_position`；
- `max_dof=8`，表示固定底盘的“手臂加升降”；
- `retreat_to_transport=true`。

库存数据库中的 `world_z` 不是升降电机位置，只能作为感知提示。真实抓取仍必须使用检测位姿、MoveIt IK、关节限位和附着验证。

## 6. test_08 GUI 的默认数据

启动脚本当前默认选择：

```text
比赛档案:
src/openflex_chassis/navigation_layer/swerve_navigation/config/
  supermarket_test_08_competition.yaml

任务站点库:
src/openflex_integrated/openarmx_task_executor/config/
  stations_supermarket_test_08.yaml

地图:
/home/openflex/old/openflex_maps/3d/supermarket_test_08/map.pcd
/home/openflex/old/openflex_maps/2d/supermarket_test_08/map.yaml
/home/openflex/old/openflex_maps/2d/supermarket_test_08/map.pgm

GUI 语义站点:
/home/openflex/old/openflex_maps/2d/supermarket_test_08/
  competition_mission.yaml
```

默认初始位姿：

```text
x = -0.100 m
y =  0.700 m
yaw = 1.5708 rad，约 90 deg
```

开机前应把机器人实际摆到该位置和朝向。该值不仅用于 GUI 显示，也是 ICP 重定位搜索初值；机器人实际位置偏差过大时，可能出现定位错误或起点落入障碍。

当前语义站点包括：

```text
START_CORRIDOR
SHELF_A ... SHELF_H
delivery_counter
restock_counter
```

test_08 站点库中的货架层示教值会驱动真实硬件：

| 层号 | `lift_joint` 实测位置 | 用途 |
| --- | ---: | --- |
| 第 2 层 | `0.1066645` | 预抓取，也是当前安全运输姿态 |
| 第 3 层 | `0.29981` | 第 3 层预抓取，模拟抓取后会先回第 2 层再退避 |

目前只配置了右臂第 2/3 层姿态。GUI 会拒绝左臂预抓取联调，也会拒绝没有示教姿态的库存层号。

## 7. 构建、预检和启动

### 7.1 首次构建或接口修改后重建

```bash
cd /home/openflex/old/openflex_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-up-to \
  openarmx_task_executor \
  openflex_navigation_gui

source install/setup.bash
```

修改 `.action` 或 `.msg` 后必须重新构建并重新 `source install/setup.bash`。旧的生成接口可能导致 GUI 按钮存在，但 Action 永远无法连接。

### 7.2 查看脚本参数

```bash
cd /home/openflex/old/openflex_ws
./start_supermarket_fullbody_competition_gui.sh --help
```

支持参数：

```text
--profile PATH
--station-config PATH
--record-root PATH
--check
-- ROS_ARGS...
```

### 7.3 先执行无运动预检

```bash
cd /home/openflex/old/openflex_ws
./start_supermarket_fullbody_competition_gui.sh --check
```

预检会验证：

- ROS 2 Humble 环境；
- 工作区已经构建；
- 比赛档案和站点 YAML 存在且可解析；
- PCD、PGM、YAML 三个地图文件存在；
- 三个关键 launch 文件已经安装；
- `PickObject` 和 `RetreatFromStation` 生成接口可以导入；
- 当前地图是否至少有两个语义站点；
- 哪些站点允许抓取后或放置后退避。

当前工作区的预检结果应包含：

```text
Profile: supermarket_test_08_competition
Map: supermarket_test_08
Mock grasp endpoint: /competition/mock_pick_object (no hardware motion)
Retreat endpoint: /retreat_from_station
Full-body competition GUI preflight passed.
```

该预检不会启动硬件，也不会验证正式 `competition.yaml` 是否与 test_08 站点库完全匹配。

### 7.4 启动 GUI

```bash
cd /home/openflex/old/openflex_ws
./start_supermarket_fullbody_competition_gui.sh
```

脚本会自动 `source` ROS 和本工作区，不需要在同一终端提前执行。

脚本使用文件锁保证同一登录会话只运行一个比赛控制台。第二次启动会提示已有窗口并以退出码 3 结束。GUI 只管理自己启动的进程，因此不要同时从旧 GUI 或其他终端启动另一套整机/导航系统。

如果终端没有 `DISPLAY` 或 `WAYLAND_DISPLAY`，脚本会警告，Qt 窗口可能无法打开。

### 7.5 自定义档案、站点库和录制目录

```bash
./start_supermarket_fullbody_competition_gui.sh \
  --profile /absolute/path/to/profile.yaml \
  --station-config /absolute/path/to/stations.yaml \
  --record-root /absolute/path/to/records
```

正式三阶段配置通过环境变量传入：

```bash
export OPENFLEX_COMPETITION_CONFIG=/absolute/path/to/competition.yaml
./start_supermarket_fullbody_competition_gui.sh
```

脚本当前没有 `--competition-config` 参数；该环境变量由 GUI 在点击“启动全身控制”时传给整机运行时。

## 8. GUI 总览

全身比赛模式下，左侧页签顺序为：

```text
建图 | 整机控制 | 比赛流程 | 比赛导航 | 系统检查
```

右侧始终显示：

- 地图位姿；
- 目标误差；
- 路径横向误差；
- ICP 单步修正；
- 最终安全速度；
- LIO 与轮式里程计差异；
- 2D 地图、全局路径、轨迹和语义站点；
- 舵轮状态与运行日志。

窗口顶部的“停止速度”只发布零速度，不能代替硬件急停。任何真实运动测试都必须保证人员可触及实体急停。

## 9. 标准 GUI 操作顺序

### 9.1 第一步：确认地图选择

进入“比赛导航”，确认“当前比赛场景”中的地图是：

```text
supermarket_test_08
```

GUI 会记住上一次选择，保存的 Qt 设置可能覆盖档案默认地图。不要只依赖脚本预检输出，必须在窗口中再次确认。

如果切换地图，必须同时保证以下三者使用相同站点 ID 和同一物理坐标系：

1. 比赛档案 `profile.yaml`；
2. 地图目录中的 `competition_mission.yaml`；
3. 启动脚本传入的任务站点库 `stations.yaml`。

只在 GUI 中换地图、却继续使用 test_08 任务站点库，会造成导航目标和操作站点不一致。

### 9.2 第二步：整机控制页

按以下顺序操作：

1. 确认机器人周围清空，实体急停可用。
2. 点击“启用全部 CAN”，等待显示 `CAN：6/6 已启用`。
3. 点击“检查全部电机”，必须显示“检查通过”。
4. 正式自主联调选择“全自主比赛”。
5. “启动 MoveIt RViz”按调试需要选择；比赛运行可保持关闭以减少图形负载。
6. 点击“启动全身控制”。
7. 等待显示“整机：全身硬件与 MoveIt 在线”。

该按钮实际启动 `supermarket_fullbody_runtime.launch.py`，包括：

- 唯一全身硬件和 `controller_manager`；
- MoveIt；
- `safe_odom_mux`；
- `/execute_transport`；
- `/dock_to_station`；
- `/shelf_fine_align`；
- `/retreat_from_station`；
- `/competition/mock_pick_object`；
- `/competition/move_to_shelf_view`；
- 三阶段协调器进程。

分层接口状态中，以下项目应可在当前联调栈变为在线：

```text
/controller_manager
/move_group
/competition/move_to_shelf_view
/competition/mock_pick_object
/retreat_from_station
/execute_transport
```

以下项目需要外部团队适配器，显示“等待接入”不一定是 GUI 故障：

```text
/detect_object
/pick_object
/place_object
```

### 9.3 可选：VR 调试 / 遥控

正式全自主比赛不需要启动 VR。需要联调双臂、升降或头部时：

1. 在“整机控制”中把控制方式切换为“VR 调试 / 遥控”。
2. 先启动全身控制，并等待 `/controller_manager` 和 `/move_group` 在线。
3. 只调试上身时，保持“允许 VR 控制底盘”关闭。
4. 需要 VR 控制底盘时，必须先启动定位导航和完整速度安全链，再勾选“允许 VR 控制底盘”。
5. 点击“启动 VR 遥操作”。
6. 结束后点击“停止 VR”。切回“全自主比赛”也会自动停止当前 VR 进程。

即使允许 VR 控制底盘，速度仍必须经过碰撞监控和定位安全门。建图、定位导航和 VR 的互斥提示必须按 GUI 要求处理，不能从其他终端绕过。

### 9.4 第三步：启动定位导航

进入“比赛导航”：

1. 确认初始位姿为 `-0.100, 0.700, 1.5708`，或填写现场真实初值。
2. 保持“启动导航 RViz”开启；全身比赛模式下该选项被强制开启。
3. 建议保持“自动录制”开启。
4. 点击“启动定位导航”。
5. 等待 `/scan`、`/odom_safe`、`map -> base_link` 和 `/localization_ok=true`。
6. 确认“定位：稳定”和“安全链：正常”。

该按钮只启动定位、Nav2 和速度安全链，并传入：

```text
start_robot_hardware:=false
start_safe_odom_mux:=false
```

因此不会创建第二个控制器管理器或第二个 `odom -> base_link` 发布者。

如果定位初值错误，可使用：

- “地图选择初始位姿”：在地图中按下并拖动选择位置和朝向；
- “发送输入的初始位姿”：发布输入框中的 `/initialpose`。

### 9.5 第四步：先做单航点测试

正式运行联调链前，建议依次验证：

1. 选中 `START_CORRIDOR`，点击“发送选中航点”或“返回起点并对正”。
2. 选中一个货架站，例如 `SHELF_A`，点击“发送选中航点”。
3. 观察 Nav2 到站后是否自动进入“货架精停中”。
4. 查看最终净距、精停角差、最大横差和最大 ICP。

下发比赛航点前，GUI 强制检查：

- 没有其他 Nav2 goal 正在发送、运行或取消；
- `/localization_ok=true`；
- `/scan` 和 `/odom_safe` 的数据年龄不超过 0.5 秒；
- `/controller_server`、`/collision_monitor`、`/cmd_vel_safety_gate` 存在；
- `/navigate_to_pose` 可用；
- 货架站还要求 `/shelf_fine_align` 可用；
- 目标处完整车体足迹与障碍保持档案规定净距。

货架站的 GUI 精停目标为：

```text
物理净距 0.05 m
净距容差 0.01 m
请求偏航容差 1 deg
超时 20 s
```

精停节点还会拒绝或中止以下情况：定位失效、扫描/里程计超过 0.5 秒、缺少左右角点、起始角差超过 8 度、任一前角小于 3.5 cm、横向/角向状态不安全或超时。

### 9.6 第五步：自动验收巡游

“自动验收巡游”支持：

- 当前 `competition_mission.yaml` 的站点顺序；
- 档案定义的 `shelf_alignment` 巡游；
- 1 到 20 次重复。

当前 test_08 档案的巡游顺序为：

```text
START_CORRIDOR
  -> SHELF_A -> SHELF_B -> SHELF_C -> SHELF_D
  -> SHELF_E -> SHELF_F -> SHELF_G -> SHELF_H
  -> START_CORRIDOR
```

任一目标 Action 失败或精度超差，后续队列会停止。当前 GUI 统计单次目标门槛，但不会自动把 `minimum_success_rate=0.95` 和“10 次零碰撞巡游”汇总为整场最终认证结论，需要人工依据结果记录判断。

### 9.7 系统检查页

点击“立即检查”可查看：

- 驱动 CAN `can4` 和转向 CAN `can5`；
- `~/.openflex/lidar_config.yaml`；
- 当前 3D PCD 和 2D YAML；
- `/scan` 数据年龄；
- `/odom_safe` 数据年龄；
- `map -> base_link` TF 数据年龄；
- controller、PGO、ICP、Nav2 和碰撞监控等关键节点。

这里的 CAN 检查只显示 `can4/can5`；六路 CAN 是否全部启用，应以“整机控制”页的 `6/6` 状态为准。

## 10. “比赛流程”页的实际使用

### 10.1 手动联调链

手动区域标题为：

```text
导航精停 -> 货架观察姿态 -> 模拟抓取 -> 运输姿态 -> 退避导航
```

参数填写：

| 控件 | 当前要求 |
| --- | --- |
| 当前货架站 | 必须是 `shelf_approach` 类型，例如 `SHELF_A` |
| 下一导航站 | 必须与货架站不同，可选 `delivery_counter` 或其他安全站点 |
| 货架层 | 当前只支持第 2 层或第 3 层 |
| 手臂标签 | 当前必须选择 `right` |
| 模拟物品 ID | 仅作为模拟 Action 的对象 ID |
| 后退距离 | 0.05 到 0.30 m，默认 0.15 m |
| 后退速度 | 0.01 到 0.06 m/s，默认 0.04 m/s |

操作步骤：

1. 点击“导航、精停并进入预抓取”。
2. GUI 向来源站发送 Nav2 goal。
3. Nav2 成功并停稳后，货架站自动调用 `/shelf_fine_align`。
4. 精停完成后，GUI 调用 `/competition/move_to_shelf_view`。
5. 任务适配器把站点 YAML 中的实测升降和右臂关节值转换为 `right_arm_with_lift` 完整关节目标，再调用 `/openflex/execute_joint_posture`。
6. 等待“观察姿态结果：第 N 层到位”。
7. 点击“模拟抓取成功并执行后续链路”。
8. `/competition/mock_pick_object` 等待约 0.5 秒并返回“假定物体已附着”。
9. 如果来源是第 3 层，GUI 先把升降和右臂移回第 2 层安全运输姿态。
10. GUI 调用 `/retreat_from_station` 直线后退。
11. 退避停稳后，GUI 向“下一导航站”发送 Nav2 goal。
12. 到达下一站并通过精度判定后，联调链完成。

这里的“模拟抓取”不会执行：

- DREAM 或目标检测；
- AnyGrasp；
- 真实 `/pick_object`；
- MoveIt 抓取轨迹；
- 夹爪闭合；
- 物体附着传感器验证；
- 真实 `/place_object`。

货架观察和回运输姿态会真实运动升降台和右臂，不能把整条链理解成纯软件模拟。真正基于物体 TCP 的预抓取由 `/openflex/plan_pregrasp` 承担。

代码允许一种“流程联调放行”：如果货架局部精停 Action 已完成，但严格的横差或 ICP 验收结果超差，GUI 会保留超差警告，同时仍允许继续进入预抓取。正式比赛前不能把这种放行当作精度合格。

### 10.2 第一阶段 SKU 快捷流程

第一阶段区域会只读打开：

```text
/home/openflex/old/supermarket_scene/shelf_inventory.db
```

数据库不存在时，商品下拉框无法使用。GUI 会读取 `shelf_inventory` 中非 `unknown` 的 SKU，并将货架槽位映射到 test_08 的 A 到 H 货架站。

当前硬编码映射只覆盖货架组 1 和 3 的两个面：

| 数据库货架面 | 候选导航站 |
| --- | --- |
| shelf 1, face 0 | `SHELF_D` / `SHELF_C` |
| shelf 1, face 1 | `SHELF_B` / `SHELF_A` |
| shelf 3, face 0 | `SHELF_H` / `SHELF_G` |
| shelf 3, face 1 | `SHELF_F` / `SHELF_E` |

GUI 根据槽位的世界 Y 坐标选择同一货架面上最近的接近站。同款商品有多个实例时，依次按“能否映射到站点、与候选站的 Y 向距离、较低层号、较小 `y_cm`”选择。数据库第 2/3 层映射到同号示教姿态；其他层显示“未示教”并禁止启动。

操作步骤：

1. 确认整机控制和定位导航均已启动。
2. 等待 `/localization_ok=true`。
3. 从“选择商品”中选择一个 SKU。
4. 检查界面显示的货架 ID、face、数据库层号、世界坐标和目标导航站。
5. 点击“开始第一阶段流程”。
6. GUI 自动填写手动联调区的来源站、`delivery_counter`、层号、右臂和对象 ID。
7. 等待货架导航、精停和预抓取完成。
8. 当状态显示“预抓取完成，请切换抓取成功开关”时，勾选“模拟抓取成功”。
9. GUI 执行模拟抓取、必要的回第 2 层运输姿态、货架退避和到交付台导航。
10. 到达 `delivery_counter` 后显示“第一阶段完成”。

当前第一阶段快捷流程每次只处理一个 SKU，不读取比赛订单中的两个商品，也不调用三阶段协调器。它直接复用手动联调状态机。

### 10.3 第二、第三阶段

GUI 页面中的“第二阶段：补货”和“第三阶段：归位错放商品”目前是禁用占位区，不能点击执行。

正式的第二、第三阶段逻辑已经存在于 `competition_coordinator_node.py`，但还需要：

- GUI 或裁判端发送 `/competition/execute_phase`；
- 完整且与 test_08 匹配的 `competition.yaml` 和站点库；
- `/inspect_retail_scene`；
- `/set_perception_mode`；
- 真实检测、抓取、放置和复核接口。

## 11. 货架精停和安全退避

### 11.1 货架精停

`/shelf_fine_align` 使用 `/scan` 在 `base_link` 中独立测量左右前角的货架边缘，比较两侧距离得到货架相对偏航误差。

当前关键限制：

```text
前向速度：0.030 到 0.040 m/s
最大角速度：0.10 rad/s
目标附近迟滞：3 mm
稳定样本：10 帧
最小安全净距：0.035 m
最小起始净距：0.040 m
最大起始地图偏航误差：8 deg
扫描和里程计超时：0.50 s
```

偏航误差大于 3 度时先停止平移，只做角度修正。该动作只允许前向精停，不使用倒车修正。

### 11.2 安全退避

`/retreat_from_station` 是任务层唯一允许的直线倒车原语，限制为：

```text
最大距离：0.30 m
最大速度：0.06 m/s
横向漂移：不超过 0.03 m
偏航漂移：不超过 3 deg
里程计最大年龄：0.50 s
停车阈值：0.02 m/s、0.03 rad/s
停车保持：0.30 s
```

退避前必须已经收到 `/localization_ok=true`。执行期间节点持续发布 `/competition/post_pick_reverse_authorized`；授权撤销、定位失效、里程计过期、横向漂移或偏航漂移超限都会立即发布零速度并失败。

test_08 的货架站、交付台和补货台均配置了 0.15 m 退避，但只有调用方真正请求退避时才会运动。当前 GUI 第一阶段只在货架模拟抓取后请求退避，没有在交付台请求放置后退避。

## 12. 建图页的使用

已有 test_08 地图时，正式联调可跳过建图页。

需要重新建图时：

1. 确保定位导航和 VR 已停止。
2. 可以先启动全身控制，再点击“开始建图”；此时建图 launch 使用 `start_robot_hardware:=false` 复用现有硬件。
3. 勾选“启用遥控”后，按住 GUI 按钮或 `W/S/A/D/Q/E` 才会发布非零速度。
4. 点击“保存 3D 地图”调用 `/pgo/save_maps`。
5. 点击“保存并生成 2D 地图”后，GUI 停止建图并运行 `pcd_to_nav2_map`。
6. 必要时点击“编辑 2D 地图”，保存前会自动备份原 PGM。
7. 建图停止后才能启动定位导航。

建图和定位导航互斥。停止全身控制前，也必须先停止正在运行的建图或定位导航流程。

## 13. 语义站点管理

GUI 语义站点保存在每张地图自己的：

```text
2d/<map_name>/competition_mission.yaml
```

可使用：

- “新建站点”；
- “地图选点”；
- “使用当前位姿”；
- “新增/更新站点”；
- “删除站点”；
- “上移/下移”调整巡游顺序。

站点 ID 只能包含字母、数字、点、下划线和短横线，每张地图最多一个 `start_position`。

需要区分两套站点数据：

| 数据 | 使用方 | 内容 |
| --- | --- | --- |
| `competition_mission.yaml` | GUI | 语义航点、地图显示、巡游顺序和角色 |
| `stations_supermarket_test_08.yaml` | 任务执行器 | Nav2 位姿、精停模式、退避、抓取/放置位姿和层级示教 |

同名站点的 `x/y/yaw` 必须一致。GUI 保存语义站点不会自动回写任务执行器的站点 YAML。

## 14. 正式 Action 操作示例

以下命令只适用于正式三阶段配置和外部 Action Server 已全部接入之后。当前默认 test_08 配置不满足该条件。

### 14.1 阶段预检

```bash
ros2 action send_goal --feedback \
  /competition/execute_phase \
  openarmx_task_msgs/action/ExecuteCompetitionPhase \
  "{run_id: match_001, phase: 1, dry_run: true, timeout_sec: 30.0}"
```

### 14.2 正式按顺序执行三个阶段

```bash
ros2 action send_goal --feedback \
  /competition/execute_phase \
  openarmx_task_msgs/action/ExecuteCompetitionPhase \
  "{run_id: match_001, phase: 1, dry_run: false, timeout_sec: 420.0}"

ros2 action send_goal --feedback \
  /competition/execute_phase \
  openarmx_task_msgs/action/ExecuteCompetitionPhase \
  "{run_id: match_001, phase: 2, dry_run: false, timeout_sec: 420.0}"

ros2 action send_goal --feedback \
  /competition/execute_phase \
  openarmx_task_msgs/action/ExecuteCompetitionPhase \
  "{run_id: match_001, phase: 3, dry_run: false, timeout_sec: 360.0}"
```

必须等待前一个命令结束并获得裁判许可后，才能发送下一个阶段。不能同时打开三个终端并行发送。

### 14.3 失败后重置整场状态

```bash
ros2 service call /competition/reset_run std_srvs/srv/Trigger "{}"
```

只允许在没有活动阶段时调用。正式比赛是否允许重置必须服从裁判规则。

### 14.4 单件搬运示例

```bash
ros2 action send_goal --feedback \
  /execute_transport \
  openarmx_task_msgs/action/ExecuteTransport \
  "{task_id: demo_001, object_id: demo_object, source_station: SHELF_A, destination_station: delivery_counter, source_slot: '*', destination_slot: '', use_source_level: true, source_level: 2, use_nominal_object_pose: false, max_retries: 1, timeout_sec: 180.0}"
```

该命令需要真实 `/detect_object`、`/pick_object` 和 `/place_object`。GUI 的模拟抓取服务位于不同接口 `/competition/mock_pick_object`，不会自动替代 `/pick_object`。

## 15. 当前正式比赛配置缺口

默认 `competition.yaml` 引用的站点包括：

```text
phase_1_judging
phase_2_judging
finish
shelf_a_pick ... shelf_d_pick
shelf_a_scan ... shelf_d_scan
delivery_scan
aisle_1_scan
aisle_2_scan
replenishment_scan
```

这些 ID 不在当前 `stations_supermarket_test_08.yaml` 中。若不提供匹配的 `OPENFLEX_COMPETITION_CONFIG`，三阶段协调器会因引用未知站点而无法正常初始化，但任务执行器、模拟抓取和 GUI 其他联调接口仍可能继续运行。

正式运行前必须完成：

1. 为 test_08 实测并添加第一/第二判定区、终点和所有巡查站；
2. 统一 `competition.yaml`、任务站点 YAML 和 GUI mission 的 ID；
3. 配置 SKU 路由，或启用库存数据库并填写 `shelf_face_stations`；
4. 接入 `/read_delivery_order`；
5. 接入 `/inspect_retail_scene`；
6. 接入 `/set_perception_mode`；
7. 接入真实 `/detect_object`；
8. 接入真实 `/pick_object` 和 `/place_object`；
9. 接入 `/verify_placement`；
10. 若正式 `/execute_transport` 需要自动分层预抓取，启动时设置 `enable_shelf_view:=true`；GUI 当前只启动了服务端，执行器钩子默认仍为关闭；
11. 为 GUI 第二、第三阶段增加正式 Action 操作入口，或由独立裁判端发送 `/competition/execute_phase`。

还需注意：GUI 第一阶段会直接读取 SQLite 数据库；正式协调器只有在 `competition.inventory.enabled=true` 且面到站点映射完整时才会使用该数据库。这是两条独立的数据路径，GUI 能显示 SKU 不代表正式协调器已经启用库存路由。

## 16. 录制与运行日志

脚本默认把比赛诊断记录放在：

```text
~/openflex_nav_records/fullbody_competition
```

可通过 `--record-root` 或 `NAV_RECORD_ROOT` 修改。

勾选“自动录制”后，GUI 在启动导航前创建 rosbag，并保存：

- `/tf`、`/tf_static`；
- 地图、路径和目标；
- `/odom`、`/odom_safe`、LIO 里程计；
- 四级速度链；
- `/scan` 和代价地图；
- `/localization_ok`、定位保护事件；
- Nav2 Action 状态；
- ROS 节点、话题、Action 列表和关键节点参数。

排障时先查看右下角“运行日志”，再检查最新录制目录中的 `nodes.txt`、`actions.txt`、`topics.txt`、参数转储和 `bag_info.txt`。

## 17. 常见问题

### 17.1 GUI 无法打开

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
```

两者都为空时，当前终端没有图形会话。远程使用还需正确配置 X11/Wayland 转发。

### 17.2 提示比赛控制台已经在运行

先查找现有窗口。锁文件用于防止两个 GUI 各自拥有不同进程句柄，不应直接删除锁并强开第二套控制台。

### 17.3 “启动全身控制”被拒绝

检查：

- `can0` 到 `can5` 是否全部启用；
- 是否已有外部 `/controller_manager`；
- 建图或定位导航是否仍在运行；
- 运行日志中的 launch 最后输出。

### 17.4 “启动定位导航”按钮不可用

必须满足：

- 全身运行时仍由当前 GUI 启动并存活；
- `/controller_manager` 在线；
- `/move_group` 在线；
- 建图未运行；
- 导航尚未运行。

### 17.5 无法发送航点

```bash
ros2 topic echo --once /localization_ok
ros2 topic hz /scan
ros2 topic hz /odom_safe
ros2 action info /navigate_to_pose
ros2 node list | rg 'controller_server|collision_monitor|cmd_vel_safety_gate'
```

任何一项不满足，GUI 都会拒绝下发目标。

### 17.6 货架精停失败

重点检查：

- `/scan` 的 `frame_id` 是否能通过 TF 转到 `base_link`；
- 左右前角是否都能看到货架立柱/前沿；
- Nav2 初始交接角差是否小于 8 度；
- 起始距离是否大于 4 cm；
- `/localization_ok` 是否保持 true；
- `/odom_safe` 和 `/scan` 是否持续新鲜。

### 17.7 预抓取失败或按钮不可用

检查：

- 来源站角色必须是 `shelf_approach`；
- 手臂必须选择 `right`；
- 只能选择第 2/3 层；
- `/competition/move_to_shelf_view` 是否在线；
- `/joint_states` 和 `/odom_safe` 是否新鲜；
- 底盘是否已持续停稳；
- MoveIt 内部 `/openflex/execute_joint_posture` 是否在线；
- 实测升降位置是否在 `[-0.650, 0.300]` 范围内。

### 17.8 第一阶段按钮一直灰色

按钮只有在以下条件同时满足时启用：

- 当前没有第一阶段联调正在执行；
- GUI 自己启动的定位导航进程正在运行；
- `/localization_ok=true`；
- 已选择并成功解析一个可达 SKU。

### 17.9 `/competition/execute_phase` 不存在

先查看整机 launch 日志。默认正式配置与 test_08 站点库不匹配时，协调器节点可能启动后立即报未知站点并退出。应提供匹配的 `OPENFLEX_COMPETITION_CONFIG`，不是反复点击 GUI 启动按钮。

### 17.10 桌面图标无法启动

当前工作区中的 `OpenFlex商超比赛控制台.desktop` 仍硬编码了旧目录 `/home/openarmx/openflex_all/openflex_ws`。在修正桌面文件前，请从本说明给出的 `/home/openflex/old/openflex_ws` 终端命令启动。

## 18. 正确停止顺序

正常退出按以下顺序：

1. 在“比赛流程”中停止当前流程，或在“比赛导航”中取消当前目标/巡游。
2. 如已启动 VR，点击“停止 VR”。
3. 点击“停止定位导航”。GUI 会同时取消模拟抓取、货架精停和退避 Action，并停止自动录制。
4. 如建图仍在运行，先停止建图。
5. 点击“停止全身控制”。
6. 等待 `/controller_manager` 和 `/move_group` 退出。
7. 确认机器人已经断使能且现场允许后，再点击“禁用全部 CAN”。
8. 关闭 GUI。

GUI 只停止自己启动的进程。其他终端或旧 GUI 启动的节点不会被当前窗口自动关闭。

## 19. 现场快速操作清单

```text
赛前：
[ ] 机器人摆到 START_CORRIDOR 初始位姿
[ ] 实体急停可用，运动区域清空
[ ] ./start_supermarket_fullbody_competition_gui.sh --check 通过
[ ] 启动 GUI，确认地图为 supermarket_test_08

启动：
[ ] 启用全部 CAN，显示 6/6
[ ] 检查全部电机通过
[ ] 选择全自主比赛
[ ] 启动全身控制
[ ] 等待 controller_manager 和 MoveIt 在线
[ ] 启动定位导航
[ ] 等待 /scan、/odom_safe、map->base_link、/localization_ok=true

联调：
[ ] 先验证 START_CORRIDOR
[ ] 再验证单个 SHELF 航点和货架精停
[ ] 再运行手动预抓取/模拟抓取/退避链
[ ] 最后运行 SKU 第一阶段快捷流程

停止：
[ ] 停止当前流程或取消目标
[ ] 停止定位导航
[ ] 停止全身控制
[ ] 确认硬件进程退出后禁用 CAN
```

正式比赛前，必须完成第 15 节列出的配置和外部 Action 接入；不能把模拟抓取联调通过等同于正式三阶段闭环通过。
