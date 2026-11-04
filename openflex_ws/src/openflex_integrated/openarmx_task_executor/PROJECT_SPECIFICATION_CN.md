# OpenFlex 商超任务执行系统 项目说明书

## 0. 文档范围

本说明书覆盖两个 ROS2 包：

- **`openarmx_task_msgs`** —— 接口包（消息/动作定义），只声明契约，不含任何可执行逻辑。
- **`openarmx_task_executor`** —— 执行包，包含全部运行时节点、纯算法模块、配置文件与启动文件，是"商超场景赛"三环节自主比赛框架的落地实现。

两者关系：`openarmx_task_executor` 中的所有 Action Server/Client 都以 `openarmx_task_msgs` 定义的类型通信，`openarmx_task_msgs` 本身不依赖 `openarmx_task_executor`，可被其它包（如 MoveIt 侧的抓放服务器、感知包）单独引用来实现同一套接口。这是典型的"接口与实现分离"设计，允许不同团队各自实现 `/pick_object`、`/place_object`、`/detect_object` 等服务器而不用共享代码。

---

## 1. `openarmx_task_msgs` —— 接口包

### 1.1 包元数据

- **`package.xml`**：`ament_cmake` 构建类型，许可证 CC-BY-NC-SA-4.0，属于 `rosidl_interface_packages` 组（即被 ROS2 识别为"仅接口"包，其它包可以 `find_package` 它而不引入执行逻辑依赖）。
- **`CMakeLists.txt`**：调用 `rosidl_generate_interfaces`，列出全部 3 个消息 + 13 个动作，`geometry_msgs` 作为依赖（因为很多字段用到了 `geometry_msgs/Pose`）。

### 1.2 消息定义（`msg/`）

**`TaskError.msg`** —— 全系统统一错误码表（`uint16` 常量，0–99）。这是整个系统里最关键的"契约"文件之一：所有动作的 `result` 里都带一个 `TaskError error_code` 字段，执行器和总控通过它做确定性的失败分支，而不是解析字符串。数值分区大致如下：

| 范围 | 含义 |
|---|---|
| 0 | `OK` |
| 1–9 | 通用/前置检查类（如服务器未就绪、超时、取消） |
| 10–16 | 导航/对接类失败 |
| 17 | `PHASE_ORDER_VIOLATION`（环节顺序错误，如跨序开始） |
| 18–19 | `ORDER_READ_FAILED` / `ORDER_INVALID`（订单读取/校验失败） |
| 20 | `INSPECTION_FAILED`（货架巡检失败） |
| 21 | `WORK_PLAN_INCOMPLETE`（工单数量不足，比如巡检没凑够 2 个候选） |
| 22 | `PLACEMENT_VERIFY_FAILED` |
| 23 | `REFEREE_GATE_LOCKED`（失败锁存后拒绝重新开始） |
| ...99 | `INTERNAL_ERROR` |

**`CompetitionWorkItem.msg`** —— 描述"一件要被运输的商品"的结构化工单，是 `ReadOrder`、`InspectRetailScene` 的输出、也是 `ExecuteTransport` 的输入。字段：
- `kind`（枚举：`PICK_ORDER=1` 拣选 / `RESTOCK=2` 补货 / `RETURN_MISPLACED=3` 归位）
- `work_id`、`object_id`、`sku`、`specification`
- `source_station`、`destination_station`、`source_slot`、`destination_slot`
- `observed_pose`、`confidence`、`evidence_id`（用于乱放判定的证据追踪）

**`OrderLine.msg`** —— OCR/VLM 解析出的一行订单：`sku`、`display_name`、`specification`、`quantity`、`confidence`。这是"感知输出"和"业务校验"之间的边界：感知只吐 `OrderLine[]`，不允许直接输出导航点或机械臂指令，校验逻辑全部在 `competition_coordinator_node.py::_read_order` 里。

### 1.3 动作定义（`action/`）

13 个动作按用途分组：

**导航/靠站类**
- `DockToStation`：视觉标记精靠站（最后 0.3~0.7m）。
- `ShelfFineAlign`：无标记的货架扫描精对准（用激光雷达间隙测量代替视觉标记）。
- `RetreatFromStation`：低速直线退避。

**感知类**
- `DetectObject`：局部目标检测/定位，供抓取前复核。
- `InspectRetailScene`：DREAM 驱动的巡检（缺货识别 `MISSING_STOCK` / 乱放识别 `MISPLACED_ITEMS`）。
- `ReadOrder`：OCR/VLM 订单读取。
- `SetPerceptionMode`：GPU 显存串行调度（DREAM_OBSERVE / TARGET_REFINE / LOCAL_GRASP / IDLE）。

**操作类**
- `PickObject` / `PlaceObject`：MoveIt 侧抓/放执行器实现的接口，`max_dof` 字段约束规划自由度（7=仅手臂，8=手臂+升降）。
- `VerifyPlacement`：放置后位置/稳定性/SKU复核。
- `MoveLiftToLevel`：升降轴独立测试接口（`FollowJointTrajectory` 封装）。

**顶层编排类**
- `ExecuteTransport`：单件商品"导航→靠站→检测→抓→[退避]→导航→靠站→放→[退避]"的确定性执行器。
- `ExecuteCompetitionPhase`：三环节总控入口，裁判每次开始只调这一个动作。

每个 `.action` 文件的 goal/result/feedback 字段已逐一核对，具体字段含义在下文对应节点的说明中结合使用场景解释（脱离实现单独罗列字段意义不大）。

---

## 2. `openarmx_task_executor` —— 执行包

### 2.1 包结构

- **`setup.py`**：`ament_python` 构建，注册 `data_files`（`config/*.yaml`、`launch/*.py`、README/框架文档/检查清单）和 **7 个 console_scripts 入口**：
  `task_executor`、`station_docking`、`shelf_fine_align`、`station_retreat`、`lift_level_test`、`competition_mock_pick`、`competition_coordinator`。
- **`setup.cfg`**：`pytest` 配置里显式加了 `-p no:anyio`，注释说明是因为宿主机装了一个要求更新版 pytest 的 anyio 插件，而本包没有异步 anyio 测试，加这行是为了让测试环境与宿主机隔离，避免插件冲突导致测试跑不起来。
- **`package.xml`**：`ament_python` 构建，依赖 `rclpy`、`geometry_msgs`、`nav2_msgs`、`tf2_ros`、`control_msgs`、`openarmx_task_msgs` 等。

### 2.2 运行时节点（7 个）

#### 2.2.1 `task_executor_node.py`（824 行）—— `/execute_transport`

系统的核心执行器，`TaskExecutorNode` 类。职责是把"运一件东西"拆成确定性的 9 步：

```
Nav2 到源站 → 靠站/货架对准 → 底盘静止确认 → 局部检测(可选)
  → 抓取 → [可选退避] → Nav2 到目的站 → 靠站 → 放置 → [可选退避]
```

关键方法：

| 方法 | 作用 |
|---|---|
| `_sleep` | 不用 `asyncio.sleep`，而是用 rclpy `create_timer` + `Future` 实现的异步让出控制权。因为 `execute_callback` 跑在 rclpy 的执行器线程里，用标准 asyncio 会跟 rclpy 的事件循环冲突，所以全项目所有 action server 都统一用这个自造的"定时器+Future"模式做协程内的等待。 |
| `_call_action` | 通用子动作调用封装，负责把父目标的取消/超时正确传导给子动作（父被取消时子动作也要被取消，不能留孤儿目标）。 |
| `_preflight` | 在底盘真正开始移动前，检查这次任务需要的所有 Action Server（Nav2、pick、place、docking 或 shelf_align、retreat、detect）是否都已就位。**这是刻意的设计**：宁可任务还没开始就原地失败，也不要导航到货架跟前才发现 MoveIt 抓取服务器没启动。 |
| `_wait_for_base_stop` | 复用的"底盘静止"判定：要求 `/odom_safe` 数据够新，且线速度/角速度低于阈值（默认 0.02 m/s / 0.03 rad/s）并连续保持 `base_stop_hold_sec`（默认 0.5s）才算真的停稳，而不是瞬时速度过零就误判。 |
| `_dock` | 按站点配置的 `alignment_mode` 分支：`"marker"` 调 `DockToStation`，`"shelf_scan"` 调 `ShelfFineAlign`，`"none"` 跳过。 |
| `_detect` | 检测失败或未启用时，若站点配置了 `allow_nominal_detection_fallback`，退化为使用配置文件里的名义位姿而不是直接失败——这是给调试/演示模式用的降级路径，正式比赛应关闭。 |
| `_pick` | 校验升降标定范围（`lift_joint_min/max_position`），设置 `max_dof`（7 或 8）、`arm_name`、`grasp_strategy`，并把 `retreat_to_transport=True` 传给抓取服务器（抓完后手臂应先收回"运输姿态"再走）。 |
| `_place` | 用 `exact_slot`（不允许通配符回退）解析目的槽位精确位姿，在 `base_link` 下算出放置目标。 |
| `_phase_with_retries` | 导航/靠站阶段允许按配置重试；**抓取动作永远不重试**——抓失败要先重新观测确认物体状态，不能盲目重来一次抓取。 |
| `execute_callback` | 主状态机，按 9 步顺序推进，feedback 的 `progress` 从 0.02 递增到 1.0。 |

大量声明参数控制阈值，例如 `base_stop_linear_speed=0.02`、`base_stop_angular_speed=0.03`、`base_stop_hold_sec=0.5`、`lift_joint_min/max_position=-0.650/0.300`，`max_manipulation_dof` 强制只能是 7 或 8。

`goal_callback`/`cancel_callback` 实现单活跃目标模式：有任务在跑时拒绝新目标，保证同一时刻只有一次运输在执行。

#### 2.2.2 `competition_coordinator_node.py`（811 行）—— `/competition/execute_phase`

三环节比赛总控，`CompetitionCoordinatorNode` 类。启动时加载 `CompetitionConfig` + `StationDatabase` + 可选的 `ShelfInventoryDatabase`，并立刻用 `_validate_station_references` 检查配置里引用的所有站点是否都在 `stations.yaml` 里真实存在（防止 typo 导致运行到一半才报错）。

核心私有方法：

- **`_read_order`**：校验 OCR/VLM 输出的 `OrderLine[]`——SKU/规格是否在 `required_items` 白名单内、总件数是否匹配、每行置信度是否达标（`min_order_confidence=0.80`）。然后解析 SKU 路由：**优先用库存数据库解析出的槎位**，除非 `prefer_inventory=false`，或数据库没查到该 SKU 的槽位，或配置的路由里没设 `source_slot`——这三种情况才退回配置文件里写死的路由。
- **`_scan`**：DREAM 驱动的巡检动作封装，按 `scan_tour` 逐点导航+观察，用 evidence key 去重、按置信度排序，强制要求恰好凑够 `required_items` 个不同目的槽位的候选。
- **`_transport`**：把 `CompetitionWorkItem` 包装成 `ExecuteTransport` 目标，`source_slot` 通过 `ShelfInventoryDatabase.parse_slot_id` 解析出层号。
- **`_verify`**：调用 `VerifyPlacement`。
- **`_run_phase`**：完整环节时序——preflight → (读订单 或 巡检) → 感知模式切换 → 逐件运输+复核 → 切回 IDLE → 导航到判定区。
- **`execute_callback`**：区分 `dry_run`（只做接口 preflight，不动、不推进正式状态）与正式运行；总超时预算 = `min(该环节配置超时, 剩余总超时)`；环节成功推进 `CompetitionRunState`，失败则**锁存**，必须裁判批准复位。

`/competition/reset_run` 是一个 `Trigger` 服务，只有在没有环节正在跑时才允许复位。

#### 2.2.3 `docking_node.py`（238 行）—— `/dock_to_station`

`StationDockingNode`，基于视觉标记 TF 帧（`station_marker_<marker_id>`）的比例控制精靠。查 TF 得到标记相对 `base_link` 的位姿，误差乘比例增益（`kp_x=0.8`、`kp_y=1.2`、`kp_yaw=1.5`）转成 `/cmd_vel`，速度限幅（`max_linear/lateral/angular_speed` 默认 0.08/0.08/0.25）。目标位姿可以由请求里的 `target_marker_pose` 指定，否则用节点参数里的默认值。无论成功、取消、超时还是异常退出，`finally` 块都会发一个零速度 `Twist`，保证不会留下"最后一帧非零速度"这种隐患。

#### 2.2.4 `shelf_fine_align_node.py`（556 行）—— `/shelf_fine_align`

`ShelfFineAlignNode`，无标记场景下用激光雷达（稀疏 Livox 风格点云/LaserScan）做货架精对准的 Action Server。这是算法最密集的运行时节点，其核心几何/滤波逻辑都委托给 `shelf_fine_align_core.py`（见第 3 节详细算法说明）。

关键行为：
- `_scan_callback`：若激光帧与 `base_link` 不一致，用 TF 查询 + `_planar_transform`（四元数旋转的平面简化版）把点云转换到 `base_link`，再交给 `measure_scan_shelf_corners` 提取左右角点间隙，喂给 `ShelfCornerFilter` 做时间滤波。
- `localization_state_allows_alignment`：解释的是**锁存状态**而不是心跳——一旦定位置信丢失就一直禁止对准，直到显式恢复，而不是"最近一帧没收到就临时禁止"。
- `execute_callback`：大量安全前置检查（定位/里程计/激光数据新鲜度），要求启动前姿态本身就满足"低速稳定"才允许开始运动；一旦测到 `TOO_CLOSE` 立即中止；必须连续 `settle_samples`（默认10）帧都在容差内才判定成功，防止抖动误判。

大量参数决定了几何与控制律的具体数值（`half_length/width`、ROI 范围、`filter_quantile=0.25`、各种速度上下限），这些数值同时对应 `shelf_fine_align.launch.py` 里覆写的一份"标定后"的默认值集合。

#### 2.2.5 `station_retreat_node.py`（342 行）—— `/retreat_from_station`

`StationRetreatNode`，抓/放完成后的**低速直线退避**。订阅 `/odom_safe` 和 `TRANSIENT_LOCAL` 的 `/localization_ok`，发布 `/cmd_vel` 和一个持续重发的 `/competition/post_pick_reverse_authorized`（`Bool`）授权心跳。

安全设计要点：
- `goal_callback` 校验请求参数合理（`0<distance≤0.30`、`0<speed≤0.06`、`timeout>0`），拒绝任何试图后退更远/更快的请求。
- 开始前要求 `localization_ok` 锁存为真且里程计够新。
- 每周期用 `measure_retreat`（见第3节）算出已走距离/侧偏/航向误差，一旦侧偏超 0.03m 或航向误差超 3° 就中止。
- 到达目标距离后要求"停车并保持稳定 `stopped_hold_sec`(0.30s)"才算成功，而不是瞬间到距离就返回。
- `finally` 块永远把命令清零并**撤销**授权——这个授权 Bool 只在退避动作真正跑着的窗口内为真，其它任何时候安全链应把倒车当成禁止行为。

这是全系统里唯一被允许发出"倒车"速度的节点，其它所有导航/对接节点（包括 `compute_command`里显式提到）都被要求"宁可失败也不倒车"。

#### 2.2.6 `lift_level_test_node.py`（571 行）—— `/competition/move_lift_to_level`

`LiftLevelTestNode`，独立的升降轴（prismatic lift）标定/联调用 Action Server，**故意与生产环境的 MoveIt 手臂+升降联合规划分离**，避免测试时和 MoveIt 抢占同一个控制器所有权。

流程：校验 `lift_joint_min/max_position` 参数合法（min<max）→ 要求底盘已停稳（复用同款"新鲜度+速度阈值+保持时间"判定）且 `/joint_states` 数据新鲜 → 用 `FollowJointTrajectory` 命令 `/lift_joint_controller/follow_joint_trajectory`，轨迹时长按 `max(minimum_trajectory_sec, |target-current|/maximum_lift_speed)` 算出（保证不会用不合理的短时间硬冲到目标）→ 到位后校验最终位置在容差 `joint_position_tolerance=0.005` 内，且整个过程持续复查底盘没有意外移动 → 父目标被取消/超时时通过 `_cancel_child`/`_cancel_pending_send` 干净地取消底层 `FollowJointTrajectory` 目标。

#### 2.2.7 `mock_pick_node.py`（118 行）—— `/competition/mock_pick_object`

`MockPickNode`，最简单的一个节点：按参数 `succeed` 决定延时后固定返回成功/失败，只用于验证"抓取成功→退避→下一段导航"这条接口链路是否衔接得上。日志里有明确警告：**它不做任何真实感知或抓取**，不能作为正式抓取判据，也不占用未来真实 `/pick_object` 接口名（它自己是独立的 `mock_pick_object` 接口，不是 `pick_object` 的假实现）。

### 2.3 纯算法/数据模块（无 ROS 依赖，方便单测）

#### 2.3.1 `shelf_fine_align_core.py`（702 行）—— 见第 3 节完整算法说明

#### 2.3.2 `post_pick_retreat_core.py`（43 行）

`PlanarPose`、`RetreatProgress` 数据类；`normalize_angle`；核心函数 `measure_retreat(start, current)`：把里程计位移投影到**起始朝向**定义的坐标轴上，分解成：
- `backward_distance`：沿起始朝向反方向的位移（应为正，表示确实在后退）
- `lateral_drift`：垂直于起始朝向的侧偏（应接近0，侧偏过大说明退偏了）
- `yaw_error`：当前朝向与起始朝向的差（应接近0，说明退避过程中没有意外转向）

这是一个典型的"局部坐标投影"算法：设起始位姿朝向为 `θ₀`，位移向量 `Δ=(current.x-start.x, current.y-start.y)`，则：
```
backward_distance = -(Δx·cosθ₀ + Δy·sinθ₀)
lateral_drift     =  -Δx·sinθ₀ + Δy·cosθ₀
yaw_error         = normalize_angle(current.yaw - start.yaw)
```
（符号取法保证正常后退时 `backward_distance` 为正。）

#### 2.3.3 `station_db.py`（245 行）

站点配置的数据模型层。`PoseSpec`（不可变 dataclass，`from_mapping` 支持"只给 yaw"或"给完整四元数"两种写法，`.yaw` 属性统一转换）、`SlotSpec`（槎位：位姿+手臂+抓取策略+层号，默认层号 -1 表示未分层）、`StationSpec`（站点：导航位姿、marker id、是否对接、对齐模式 `marker|shelf_scan|none`、位置/角度容差、货架目标间隙及其容差和超时、抓/放后是否退避及退避距离、槎位字典、放置位姿与策略、每层的升降标定位置字典）。

`StationDatabase.from_yaml` 解析 `stations.yaml`，做强校验：`alignment_mode` 必须是三者之一；若配置了退避，`retreat.distance` 必须 >0。

`.slot(...)` 查询槎位时若找不到精确匹配会**回退到通配符 `"*"`**（用于扫描站点等"先到位再由感知决定具体物体"的场景）；但 `.exact_slot(...)`（用于目的地/放置）**不允许**回退，防止把货物放到一个"随便凑的"槎位上。`.level_lift_position` 只返回显式标定过的层高，绝不做插值猜测——没标定就是没有，逼着使用者去真实测量。

#### 2.3.4 `shelf_inventory_db.py`（245 行）

对外部 SQLite 只读数据库 `shelf_inventory.db` 的适配层。以 URI 模式 `mode=ro` 打开，杜绝任何写入风险。

`_validate_schema` 检查 `shelf_groups`/`shelf_types`/`shelf_inventory` 三张表及必需列是否存在（同时兼容新版 `height_cm` 列名和旧版 `z_offset_cm`）。

几何常量（货架几何的默认假设，当数据库缺某些字段时使用）：
```
DEFAULT_SHELF_LENGTH=1.86m   DEFAULT_SHELF_WIDTH=0.80m
DEFAULT_BOTTOM_CLEARANCE=0.05m   DEFAULT_LEVEL_SPACING=0.40m
DEFAULT_PANEL_THICK=0.02m   DEFAULT_BACK_THICK=0.005m
DEFAULT_SHELF_DEPTH_NORMAL=0.30m   DEFAULT_SHELF_DEPTH_BOTTOM=0.40m
```

`find_sku_slots`：SQL 联表查询后，用货架几何算出该 SKU 在**货架局部坐标**下的 `(local_x, local_y, local_z)`（层号→高度用 `底部间隙 + 层号×层间距` 换算，横向用数据库存的 `y_cm`），再按货架的世界朝向（yaw）做一次 2D 旋转，转换成 `(world_x, world_y)`：
```
world_x = shelf_origin_x + local_x·cos(yaw) - local_y·sin(yaw)
world_y = shelf_origin_y + local_x·sin(yaw) + local_y·cos(yaw)
```
这是标准的"局部坐标系→世界坐标系"刚体变换，本质上和 `measure_retreat` 里的投影是同一族数学操作的逆过程。

`select_sku_slot`：先按"允许的货架+面"白名单过滤，再优先选**层号最低**的、层号相同时选**离货架中心最近**的槎位（更靠中间更容易抓取，靠边的槎位可能超出手臂工作空间）。

`format_slot_id`/`parse_slot_id`：`"shelf-face-level-y_cm"` 格式字符串的双向转换，是 `source_slot` 字段在整个系统里流转时的标准编码。

#### 2.3.5 `competition_config.py`（209 行）

比赛业务规则的配置模型。`PHASE_PICKING=1`/`PHASE_RESTOCKING=2`/`PHASE_RETURN_MISPLACED=3` 常量；`PhasePolicy`（每环节的入口/判定站、巡检路线、超时、是否用 DREAM）；`SkuRoute`（一个 SKU 对应的源/目的站点及可选槎位）；`InventoryPolicy`（含 `station_for(shelf_id, face)`，把"货架+面"映射到已标定的靠站点）；顶层 `CompetitionConfig`。

`from_yaml` 做的校验非常细：`control_mode` 必须是 `"autonomous"`；`required_items>0`；超时 `>0`；入口/判定站必须存在；第2、3环节必须配置 `scan_tour`；库存功能若启用，必须同时配好 `database_path`、`shelf_face_stations`、`default_destination_station`；数据库相对路径按配置文件所在目录解析（不是按运行时 CWD，避免"在哪个目录启动就找不到文件"的坑）。

`route_for`/`try_route_for`：查路由时优先匹配 `"sku|specification"` 精确键，找不到再退回裸 `sku` 键——同一 SKU 不同规格（比如不同容量的牛奶）可以路由到不同货架。

#### 2.3.6 `competition_state.py`（60 行）

`CompetitionRunState` dataclass：`run_id`、`expected_phase`（初始为1）、`active_phase`、`failed_phase`。这是"裸的状态机数据"，不含 I/O：

- `can_start(run_id, phase)`：只有当 `run_id` 匹配（或首次运行）、`phase == expected_phase`、当前没有失败锁存、当前没有其它环节在跑时才允许开始。
- `complete(phase)`：`expected_phase = min(4, phase+1)`（第3环节完成后 `expected_phase=4`，代表"全部结束"，不再接受任何开始请求）。
- `fail(phase)`：锁存 `failed_phase`，此后 `can_start` 永远拒绝，直到 `reset()`。
- `reset()`：只有在没有 `active_phase` 时才允许，防止"边跑边被裁判重置"这种竞态。

这套状态机精确实现了 CN 框架文档里描述的 `READY_PHASE_1 → RUNNING_PHASE_1 → JUDGE_GATE_1 → ... → FINISHED`，以及"任一环节失败 → FAILED_LATCHED"。

---

## 3. 核心算法详解：`shelf_fine_align_core.py`

这是全项目算法密度最高的文件（702行），纯函数+dataclass，无 ROS 依赖，专为可单测设计。核心问题：**没有视觉标记时，如何仅用稀疏激光点云让机器人精确停在货架前指定间隙、并对正货架朝向？**

### 3.1 几何测量层

**`rectangular_clearance(point, half_length, half_width)`**：给定一个点（在 base_link 下），算它到"以原点为中心的矩形机器人轮廓"边缘的欧氏距离，而不是简单地到中心点的距离——因为机器人是矩形底盘，不同方向上真实碰撞边界不同。

**`measure_scan_clearance`**：在感兴趣区域（ROI，纵向 `roi_forward_min~max`，横向 `roi_half_width`）内找**最近点**，作为当前"货架间隙"的原始测量值。支持传入 `point_transform` 回调，用于处理雷达安装帧不等于 `base_link` 的情况（先转换坐标再筛选）。

**`measure_scan_shelf_corners`**：把 ROI 内的点按 `minimum_lateral_offset` 分成左右两侧"角点带"，每侧要求至少 `minimum_points_per_side` 个点才认为该侧测量有效。核心洞察：**如果货架平面与机器人朝向不完全平行，左右两侧到货架的间隙会不对称**，利用这个不对称量反算航向误差：
```
yaw_error = asin((right_clearance - left_clearance) / (2 · half_lateral_span))
```
（本质是用两点间隙差 + 已知横向跨距，反解出小角度近似下的夹角——类似双目测距的思路，只是这里"两目"是雷达点云里的左右两簇点。）

**`_fit_line_normal`**（内部函数）：给定一组 2D 点，做**总体最小二乘（Total Least Squares）直线拟合**，返回法线方向。做法是对点集去均值后算协方差矩阵的二阶矩 `xx, yy, xy`，用 `atan2(2·xy, xx-yy)/2` 得到主轴角度（这是 2D 数据 PCA/TLS 拟合的经典闭式解，等价于对协方差矩阵做特征值分解取最小特征值对应的特征向量），然后转 90° 得到法线方向——即货架表面的朝向。比普通"最小二乘拟合 y=kx+b"更稳健，因为不会在直线接近垂直（货架正对机器人时法线沿x轴）时退化。

**`fit_shelf_plane`**：这是最复杂的函数，思路借鉴 **RANSAC**：
1. 从左右两侧点里两两配对，只保留横向跨度 `≥ pair_span` 的候选点对（跨度太小的点对连线方向噪声会被严重放大，是 TLS/RANSAC 类算法的常见陷阱，这里主动过滤掉）。
2. 对每个候选点对，用它定义一条候选直线，统计有多少原始点落在 `inlier_tolerance` 距离内——这就是 RANSAC 的"打分"步骤，候选点对本身充当了"最小样本"角色。
3. 取得票最高（inlier 最多）的候选，再用**所有 inlier 点**重新跑一次 `_fit_line_normal` 做精修（这是 RANSAC 标准的"最终用全部内点重新拟合"步骤，比只用两点更抗噪声）。
4. 拒绝条件：左右两侧点数不足 `minimum_points_per_side`、总横向跨度不足 `minimum_lateral_span`、拟合出的法线航向误差 `|yaw|` 超过 `maximum_normal_yaw`——任一条件不满足直接判定本次拟合失败，宁可这一帧没有测量结果，也不要用一个不可靠的平面误导控制环。

**`measure_scan_shelf_plane`**：先按 ROI 筛点，再调用 `fit_shelf_plane`，是节点侧真正调用的入口。

### 3.2 时序滤波层

激光测量逐帧抖动很大（尤其稀疏点云），三个滤波器都用**滑动窗口 + 分位数**而不是均值，理由是分位数对离群点更鲁棒：

- **`ClearanceFilter`**：`deque` 滑窗，默认取 `quantile=0.25`（即取"较近的一侧分位点"而不是中位数）。选 0.25 而非 0.5 是**保守偏置**：宁可把间隙估计得偏小（更靠近障碍），也不要因为个别偏远的噪声点让估计偏大导致真的撞上货架——这是安全优先于精度的设计取舍。
- **`ShelfPlaneFilter`**：对左右两侧间隙各自做分位数滤波，航向误差取**中位数**（航向误差本身没有"保守方向"，中位数是标准的鲁棒中心估计），重建出滤波后的 `ShelfPlaneMeasurement`。
- **`ShelfCornerFilter`**：结构与 `ShelfPlaneFilter` 相同，复用其 `_quantile` 静态方法，处理角点测量。

`filter_window`/`filter_minimum_samples` 控制窗口长度和"凑够多少样本才输出有效结果"，避免刚启动时用不足的数据就给出误导性的滤波值。

### 3.3 控制律层

**`FineAlignCommand`**：`(linear_x, angular_z, phase, within_tolerance)`，phase 是个字符串枚举，用于对外暴露"当前处于哪个控制阶段"（供 feedback 和日志）。

**`compute_shelf_corner_command`**（主控制器，`compute_shelf_plane_command` 是它的别名）是一个**有限状态机 + 分段比例控制**：

| 阶段 | 触发条件 | 动作 |
|---|---|---|
| `STABILIZING` | 间隙已在目标附近（`upper_target` 到 `lower_target` 的带内）且航向已对齐 | 零速度，等待 `settle_samples` |
| `TOO_CLOSE` | 最小间隙 < `lower_target - hysteresis` | **立即中止**（不倒车，见下文说明） |
| `ALIGN_SHELF_FACE`（或 `ALIGN_HEADING`） | 航向误差超出 `yaw_align_gate` 或间隙 ≤ `upper_target` | 只转向修正航向，`angular_z = rotate_kp · yaw_error`（限幅在 `maximum_angular_speed`） |
| `ALIGN_HEADING_CLOSE` | 航向已对齐但间隙误差仍超 `tolerance` | 小范围内精修 |
| `APPROACH`（默认兜底） | 其余情况 | 前进接近，速度带**航向门控的地板值**：`linear_x = clamp(max(minimum_linear_speed, 1 - |yaw_error|/scale_denominator), min, max) · approach_kp` |

几个设计要点特别值得记录：

1. **迟滞带（hysteresis）**：`target_clearance_hysteresis` 防止在目标间隙附近反复抖动切换 `TOO_CLOSE`/`ALIGN` 状态——典型的施密特触发器（Schmitt trigger）思路，用于避免控制器在边界值附近震荡。
2. **航向门控的前进速度**：不是简单地"要么走要么不走"，而是让前进速度随航向误差**连续衰减**，误差越大走得越慢（但设了地板值 `minimum_linear_speed`，不会衰减到底盘的死区以下——注释里明确提到要遵守实际部署的差速轮控制器 0.015 m/s 底盘级 / 0.020 m/s 单轮级死区，速度低于死区电机根本不会响应,还不如干脆停下）。
3. **`compute_command`**（更老的角度优先单间隙变体，用于纯平面法向对准场景）明确**禁止倒车**："fail safely instead of backing up"——即使测量表明离得太远或角度太偏，也宁可让上层判失败重新规划，绝不在这个精细控制环节里自主后退（倒车是 `station_retreat_node` 的专属职责，而且需要独立授权）。

---

## 4. 配置文件（`config/`）

| 文件 | 用途 |
|---|---|
| `stations.yaml` | **默认/主站点数据库**，205行，注释明确写着"必须替换为实测的地图位姿和站点局部槎位位姿"——这是一份带真实结构的**占位配置**：既有早期 demo 用的 `shelf_A`/`drop_A`，也有完整对齐 CN 框架文档的比赛站点集（`delivery_counter`、`replenishment_box`、`shelf_[a-d]_pick/scan/place`、`phase_1/2_judging`、`finish`）。每个货架站点都区分了 `_pick`（有明确槎位，用于按 SKU 抓取）、`_scan`（通配符槎位，先到位再由感知决定）、`_place`（补货/归位目的地，`place_strategy: shelf_insert`）三种角色，共享同一个 `nav_pose` 附近的位置但语义不同。 |
| `stations_supermarket_mujoco.yaml` | MuJoCo 仿真专用站点配置。特点：直接引用仿真场景实体名 `scene_entity`（如 `counter_bottom_left`、`shelf_group_3`），用 YAML anchor/alias（`&shelf_a`/`*shelf_a`）让同一个货架的 `_pick`/`_scan`/`_place` 三个站点复用完全相同的位姿定义（因为仿真里三种角色实际就是同一个物理位置），减少重复。所有站点 `dock.enabled: false`——仿真环境暂不需要视觉标记精靠。 |
| `stations_supermarket_test_07.yaml` | 单站点真机测试配置（`SHELF_TABLE_APPROACH`），使用 `shelf_scan` 对齐模式，抓/放后都退避 0.15m,槎位用通配符 `"*"` —— 注释明确说明这只用于 mock 测试。 |
| `competition.yaml` | 比赛业务规则：三环节 `PhasePolicy`（入口/判定站、巡检路线、超时、`use_dream` 开关）；`sku_routes` 覆盖 milk/chips/juice/bottled_water 四种商品；库存功能默认**关闭**；`perception_mode_control=true`；`total_timeout_sec=1200.0`（对应CN文档"硬上限20分钟"）；`max_transport_retries=1`；`min_order_confidence=0.80`；`min_inspection_confidence=0.65`。 |
| `inventory_supermarket_scene.example.yaml` | 一段**合并片段**（不是完整配置），展示如何注册 `shelf_face_stations`，并显式警告"在完成地图标定前不要启用"。 |

---

## 5. 启动文件（`launch/`）

8 个 launch 文件构成清晰的分层组合关系：

```
supermarket_full_system.launch.py          (真机/带地图导航的完整比赛栈)
  ├─ integrated_robot_bringup_moveit.launch.py   (外部包：机身+MoveIt，唯一硬件/TF所有者)
  ├─ supermarket_test_05_benchmark.launch.py     (外部包：定位+Nav2+安全速度链)
  └─ competition_task_api.launch.py              (本包：任务API集合)
       ├─ task_executor.launch.py
       ├─ station_docking.launch.py
       ├─ shelf_fine_align.launch.py
       ├─ station_retreat.launch.py
       ├─ competition_mock_pick (Node，非独立launch)
       ├─ lift_level_test (Node，非独立launch，条件启动)
       └─ competition_coordinator.launch.py

supermarket_fullbody_runtime.launch.py     (无建图/定位/Nav2，只跑全身+任务API，配 safe_odom_mux 独立发布odom)
  ├─ integrated_robot_bringup_moveit.launch.py
  ├─ safe_odom_mux.py (Node，融合 LIO 里程计和轮式里程计)
  └─ competition_task_api.launch.py
```

`supermarket_full_system.launch.py` 的文档字符串明确写了"单一所有者"原则：全身 bringup 独占 `robot_state_publisher`/`ros2_control`/所有硬件；导航 include 只管感知/定位/Nav2/安全速度链；task_api 只组合队友已有的动作服务器，**不会**再启动第二套机器人栈。它显式把 MoveIt 侧的 `enable_odom_tf` 设为 `false`（因为导航侧 `safe_odom_mux` 是唯一的 `odom→base_link` 发布者），底盘命令话题固定成 `chassis_cmd_vel_topic="/cmd_vel_safe"`（即硬件只接收经过完整安全链过滤后的速度）。

`supermarket_fullbody_runtime.launch.py` 用于没有地图/定位/Nav2 时的全身联调，自己起一个 `safe_odom_mux` 节点融合 `/fastlio2/lio_odom` 和 `/odom`（轮式）成 `/odom_safe`。

`competition_task_api.launch.py` 是任务层的组合入口，把 5 个子 launch + 2 个独立节点串起来，统一暴露 `station_config`/`competition_config`/各种 `start_xxx_server` 开关，是 `supermarket_full_system` 和 `supermarket_fullbody_runtime` 共同复用的"任务API层"。

各单节点 launch（`task_executor.launch.py`、`station_docking.launch.py`、`shelf_fine_align.launch.py`、`station_retreat.launch.py`、`competition_coordinator.launch.py`）分别是对应节点的最小可运行封装，声明了该节点全部可调参数的 `DeclareLaunchArgument` 及默认值，供上层组合复用或单独调试某一个节点时使用。

---

## 6. 安全机制汇总

这套系统里反复出现的几个安全设计模式，值得单独提出来看：

1. **速度链拓扑（硬约束）**：`/cmd_vel → collision_monitor → /cmd_vel_safe_in → 定位安全门 → /cmd_vel_safe → 差速/摆动轮控制器`。文档反复强调对接/货架对准/退避这几个任务层节点**只允许发到 `/cmd_vel`**，绝不允许绕过安全链直接写 `/cmd_vel_safe`。
2. **底盘静止判定的统一模式**：`task_executor_node`、`competition_coordinator_node`、`lift_level_test_node` 三处都复用同一套"里程计新鲜度 + 线/角速度阈值 + 保持时间"的静止判定，而不是各自发明一套逻辑——这是隐性的代码重复但设计意图一致，值得作为一个可提炼的共享组件记录下来（目前是分散实现的）。
3. **退避授权心跳**：`/competition/post_pick_reverse_authorized` 只在 `station_retreat_node` 真正执行退避的窗口内为真，且是持续重发（心跳）而不是一次性置位，这样如果退避节点本身崩溃，授权会随着心跳停止自然失效，而不是停留在"永久授权"的危险状态。
4. **定位状态锁存 vs 心跳的区分**：`shelf_fine_align_node` 和 `station_retreat_node` 都用的是"锁存状态"（TRANSIENT_LOCAL QoS 的 `/localization_ok`），一旦丢失置信度就一直禁止相关动作直到显式恢复，而不是靠"最近有没有收到消息"这种瞬时判断——避免了消息延迟或丢帧被误判为"定位丢失后又恢复"的抖动。
5. **抓取/放置不做盲目重试**：区别于导航/靠站类操作允许按配置重试，物理抓取动作失败后必须先重新观测确认物体状态，绝不自动重来一次——因为抓取失败往往意味着物体位置/姿态已经和预期不一致，重试同样的动作参数没有意义甚至危险。
6. **裁判闸门锁存**：`CompetitionRunState.fail()` 之后 `can_start` 永远拒绝新的开始请求，必须裁判批准后走复位流程，防止软件自动重试绕过裁判监管。

---

## 7. 部署/标定清单（HARDWARE_INTEGRATION_CHECKLIST.md 内容整理）

真机联调前必须解决的**已知配置冲突**：

1. `swerve_navigation/full_system.launch.py` 和 `openarmx_integrated_bringup` 都会各自启动一套 `robot_state_publisher`/`controller_manager`，不能同时原样运行——必须只保留一个。
2. 集成 bringup 的 `swerve_drive_controller.enable_odom_tf` 默认是 `true`，与导航侧 `safe_odom_mux` 冲突，联合运行时前者必须显式设为 `false`。
3. `openarmx_integrated_moveit_config/demo.launch.py` 也会再起一个 `robot_state_publisher`，联合运行时只能保留 bringup 里的那一个。
4. MoveIt 执行映射期望的控制器名（`left/right_joint_trajectory_controller`、`lift_joint_controller`、两个 `GripperCommand`）与集成 bringup 实际加载的控制器名（`left/right_forward_position_controller`、`lift_manual_position_controller`）**不匹配**——文档明确说这意味着"不能据此宣称 MoveIt 真机执行已打通"，需要单独对齐。
5. MoveIt SRDF 用 `world→base_link` 固定虚拟关节（适合仿真固定底盘），导航侧是 `map→odom→base_link` 移动底盘——联合运行需要单独设计移动底盘下的规划场景坐标策略，且不能再发布静态的 `world/map→base_link`。

**建议的首次真机测试顺序**（8 步，从"只起集成 bringup 确认唯一控制器"到"最后才起 competition_task_api.launch.py"），逐层加码验证，任何一步没打通都不进入下一步。其中第 8 步特别强调：每个用到的货架层都要在 `stations.yaml` 里补上实测的 `level_lift_positions`（`lift_joint` 坐标），且没做标定时**绝不能**把数据库里的 `world_z` 直接当成升降电机指令。

---

## 8. 端到端运行流程总览

### 8.1 单件运输（`ExecuteTransport`，`task_executor_node`）

```
preflight（检查本次任务需要的全部子服务器）
 → Nav2 导航到源站
 → 靠站/货架对准（marker 或 shelf_scan，取决于配置）
 → 底盘静止确认
 → 局部检测（可选，禁用/失败时可降级为名义位姿）
 → 抓取（含升降标定校验、max_dof 设置）
 → [可选：退避到安全距离]
 → Nav2 导航到目的站
 → 靠站
 → 放置（精确槎位，不允许通配符回退）
 → [可选：退避]
```

### 8.2 三环节比赛总控（`ExecuteCompetitionPhase`，`competition_coordinator_node`）

```
裁判发 phase=1
 → CompetitionRunState.can_start 校验（run_id/顺序/无锁存/无并发）
 → preflight
 → 环节1：Nav2→delivery_counter → 靠站+静止 → ReadOrder(OCR两次+规则校验)
    → SKU路由解析(库存优先) → 逐件 ExecuteTransport → VerifyPlacement
    → Nav2→phase_1_judging → 硬等待裁判
裁判发 phase=2（run_id相同）
 → GPU模式→DREAM_OBSERVE → 按 scan_tour 巡检货架 → InspectRetailScene(缺货)
 → GPU模式→LOCAL_GRASP → 逐工单：Nav2→补货箱→抓→Nav2→目标货架→靠站→放→复核
 → Nav2→phase_2_judging → 硬等待裁判
裁判发 phase=3
 → GPU模式→DREAM_OBSERVE → 巡查全场 → InspectRetailScene(乱放)
 → GPU模式→LOCAL_GRASP → 逐工单抓取归位
 → Nav2→finish → 全场结束
```

任一环节失败 → `FAILED_LATCHED`，必须裁判批准后走 `/competition/reset_run` 才能重新开始。

---

## 9. 已知落地状态与差距（截至代码现状）

**已落地**：环节 Action 接口、裁判闸门状态机、配置强校验、Nav2 巡检路线、工单去重排序、GPU 模式切换契约、单件运输调用链、放置复核调用、判定区收尾、失败锁存。

**尚未实现/未真机验证**（会导致 preflight 直接失败，属于设计上的"宁可拒绝启动"）：
1. `/read_delivery_order` 的真实 OCR/规格解析服务器。
2. `/inspect_retail_scene` 的真实 DREAM + 货架数据库规则服务器。
3. `/set_perception_mode` 的真实进程/GPU资源管理服务器。
4. 完整闭环的 `/pick_object`、`/place_object`（当前"预抓取能力"不等同于抓放闭环）。
5. `/verify_placement`、夹持/掉落检测、PlanningScene 货架碰撞几何同步。
6. `stations.yaml` 里全部比赛站点/marker/抓放位姿的实测替换（当前是结构完整但数值占位的配置）。
7. 与裁判确认预建地图使用规则、联网API许可、故障重入和阶段间重新布置的具体规则。

---

说明书覆盖了两个包里全部 Python 模块、消息/动作定义、配置文件、启动文件和随附文档的作用、含义与算法细节。测试目录（`test/*.py`，6 个单测文件 + 1 个 bag 回放脚本）暂未逐一读取源码，如需补充"每个测试具体验证了算法的哪些边界情况"，可在此文档后续追加一节。
