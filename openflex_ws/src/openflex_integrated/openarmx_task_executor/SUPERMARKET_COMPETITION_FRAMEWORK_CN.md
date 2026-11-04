# 商超场景赛三环节自主控制框架

本文依据《世界人形机器人运动会 场景赛 商超 解析和方案》整理当前 OpenFlex
比赛控制流程。规则基线为：总时长不超过 20 分钟、三个环节必须顺序执行、每个环节
完成后必须进入判定区并等待裁判、自主模式下每个环节开始前只能发送一次开始指令。

## 1. 总体控制原则

```text
裁判开始指令
  -> CompetitionCoordinator（一次只运行一个环节）
       -> Nav2（通道级移动）
       -> DockToStation（最后 0.3~0.7 m 精靠）
       -> OCR / DREAM（生成结构化工单，不直接控制机器人）
       -> ExecuteTransport（单件确定性执行）
            -> Qwen/OWLv2 + SAM2 检测 -> GraspNetBaseline
            -> MoveIt 固定底盘抓取/放置
       -> VerifyPlacement（位置、稳定性、SKU 复核）
  -> Nav2 进入判定区
  -> 硬等待下一次裁判指令
```

职责必须保持单一：

| 场景 | 使用模块 | 不应使用的模块 |
| --- | --- | --- |
| 起点到交付台、货架、补货箱、判定区 | Nav2 | MoveIt 全身 Stage C |
| 距货架最后几十厘米的重复定位 | DockToStation / 视觉伺服 | Nav2 全局规划器 |
| 移动中建立全局语义记忆、巡查大区域 | DREAM observe-only | GraspNetBaseline |
| 已知 SKU 在近距离 RGB-D 中精定位 | Qwen3.5-4B（默认）或 OWLv2 + SAM2 | DREAM 全图检索 |
| 从目标实例点云生成抓取姿态 | GraspNetBaseline | Nav2 |
| 手臂、升降、夹爪轨迹 | MoveIt，默认 7/8 DOF | 底盘控制器 |
| 缺货/乱放判断 | 货架数据库 + DREAM 证据 + 规则引擎 | VLM 直接下发动作 |

比赛时 Nav2 是常规点到点导航的唯一底盘运动所有者。唯一例外是抓取成功且机械臂
回到运输位后，任务层调用导航侧 `/retreat_from_station`，按站点配置以不超过
`0.06 m/s` 的速度闭环直线后退，停车稳定后才发送下一段 Nav2 目标。只有显式配置
`retreat.after_pick/after_place` 的近距离站点才会退避。该动作通过短时授权进入原有
碰撞与定位安全链，Nav2 本身仍禁止倒车。MoveIt 的 11 DOF 能力保留给离线规划和诊断；正式流程先由 Nav2 到站
并精靠，再以 `base_link` 下目标执行 7 DOF 手臂或 8 DOF 手臂加升降，避免
`/navigate_to_pose` 与 MoveIt 同时争夺底盘。

## 2. 开赛前状态

开赛前允许做的只有设备就绪和规则允许的先验加载，不应提前观察当场随机任务物料。

1. 只启动一个全身 `robot_state_publisher`、`controller_manager` 和
   `odom -> base_link` 发布者。
2. 检查急停、底盘安全速度链、升降回零、双臂安全运输姿态和夹爪力限位。
3. 加载官方允许的静态 2D 地图、3D PCD、货架几何和 SKU 目录。
4. 如果裁判禁止预建地图，必须换成在线 SLAM 配置；当前
   `supermarket_full_system.launch.py` 默认需要预建地图，不能在规则未确认时直接使用。
5. 启动 DREAM observe-only、任务接口和 GPU 模式管理器，但不触发目标检索或抓取。
6. 对所有 Action Server 做 preflight；正式开始后不能靠第二次开始命令补救缺失节点。

建议时间预算：第一环节 420 秒、第二环节 420 秒、第三环节 360 秒，总计 1200 秒。
这是硬上限，不是期望耗时；正常目标应留出至少 120 秒异常余量。

## 3. 第一环节：商品拣选

第一环节目标明确、SKU 有限，不需要 DREAM 做全场搜索。

```text
裁判第一次开始
 -> Nav2 到 delivery_counter
 -> 靠站 + 底盘静止保持 0.5 s
 -> ReadOrder：OCR 两次读取，规则校验名称/规格/数量
 -> SKU 路由表解析两个取货站
 -> 对每件商品：
      Nav2 到货架站 -> 精靠 -> 局部 SKU 复核 -> SAM mask
      -> GraspNetBaseline -> MoveIt 抓取 -> 夹持/附着验证
      -> 任务层调用站点安全退避 -> 停稳
      -> Nav2 到交付台 -> MoveIt 放置 -> VerifyPlacement
 -> Nav2 到 phase_1_judging
 -> 返回 waiting_for_referee=true，停止所有自主动作
```

OCR 或 VLM 只输出 `OrderLine[]`。总控必须检查：恰好两个不同 SKU、总件数为 2、
每行置信度达标、SKU 和规格存在确定的 `sku_routes`。自然语言不得直接成为导航点或
机械臂命令。

当前基线逐件送回交付台，路径更简单且掉落风险较低。后续若篮子机构完成可靠验证，
可增加两个机载货位，把两件商品先放入篮子再一次送达，但不能只在总控里改路线；还
需要机载放置、在手状态和篮中物体稳定性检测。

## 4. 第二环节：货架补货

DREAM 在本环节用于连续巡检和跨视角记忆，不用于最终抓取控制。

```text
裁判第二次开始
 -> GPU 模式切换 DREAM_OBSERVE
 -> Nav2 按 shelf_a_scan ... shelf_d_scan 巡检
 -> 每个点停稳后头部扫视，DREAM 融合语义关键帧
 -> InspectRetailScene(MISSING_STOCK)
      货架数据库期望槽位 - 实际观测 = 缺货候选
      复核标签/邻近商品/槽位几何，输出恰好两个 CompetitionWorkItem
 -> GPU 模式切换 LOCAL_GRASP
 -> 对每个工单：
      Nav2 到 replenishment_box -> SKU 复核 -> GraspNetBaseline -> 抓取
      -> Nav2 到目标货架 -> 精靠 -> MoveIt 模板化插入
      -> VerifyPlacement 检查正确槽位、姿态和稳定性
 -> Nav2 到 phase_2_judging
 -> 硬等待裁判
```

缺货不能只靠“画面中有空白”。至少要同时满足槽位存在、数据库有期望 SKU、当前
观测中该槽位为空、遮挡质量足够。两个候选必须使用不同目的槽位，代码会按置信度
排序并去重。

## 5. 第三环节：巡查归位

```text
裁判第三次开始
 -> GPU 模式切换 DREAM_OBSERVE
 -> Nav2 按 delivery_scan、aisle_1_scan、aisle_2_scan、
    replenishment_scan 巡查全场可放物区域
 -> InspectRetailScene(MISPLACED_ITEMS)
      当前 SKU 位置与 shelf_inventory 期望位置比对
      输出两个“发现位置 -> 正确货架槽位”工单
 -> GPU 模式切换 LOCAL_GRASP
 -> 局部复核 -> GraspNetBaseline -> MoveIt 抓取
 -> Nav2 到正确货架 -> 精靠 -> 模板化放置 -> 视觉复核
 -> Nav2 到 finish
 -> 第三环节和整场结束
```

乱放判定必须同时保存 `evidence_id`、观测时间、来源站和目标槽位。DREAM 的粗语义
命中只用于候选召回，最终 SKU 和位置应由近距离检测与数据库规则确认。

## 6. GPU 模式和数据流

### 6.1 外部货架数据库与层级路由

`/home/openarmx/openflex_all/supermarket_scene/shelf_inventory.db` 是货架
SKU、货架面、层号和名义 `map` 商品坐标的只读来源。总控按
`competition.inventory.shelf_face_stations` 将 `(shelf_id, face)` 映射到已
实测的货架 approach 站点，再把数据库生成的
`source_slot=shelf-face-level-y_cm`、`source_level` 和名义位姿透传给运输
执行器。层号不改变 Nav2 目标的 `z`；同一货架面的各层复用同一个水平靠站点，
只有不同横向货格/货架面在经过安全掩膜验证后才使用不同站点。

数据库当前版本使用 `height_cm`，旧版本使用 `z_offset_cm`，任务包适配器
两者均支持并始终以只读方式打开。名义世界坐标只用于检测 ROI/初值，不能替代
到站后的局部 3D 检测。

抓取契约将层号和可选的实测 `lift_joint` 初值传给 `/pick_object`，并固定
`max_dof=8`。真实适配器应在底盘精靠、停车后使用
`left/right_arm_with_lift` 联合规划；没有回零或层高标定时，不能把数据库的
`world_z` 直接作为升降电机指令。

比赛抓取使用两个相互隔离、常驻显存的模型环境：一套 Qwen+SAM2 worker 和一套
GraspNetBaseline worker，左右臂共享这两套 worker。`/set_perception_mode` 只表达比赛
流程状态，不卸载或替换这两个 worker，也不静默切换 2D 框 provider：

```text
DREAM_OBSERVE   移动巡检、SigLIP 语义记忆
TARGET_REFINE   默认 Qwen3.5-4B + SAM2；可由调试 GUI 显式选择 OWLv2
LOCAL_GRASP     GraspNetBaseline 使用 detection_id 对应的同帧实例点云
IDLE            判定区等待
```

Qwen 路线失败必须直接报错，只能由操作员在 GUI 手动改选 OWLv2；DINO 仅可在 OWLv2
路线中显式开启，Qwen 路线不使用 DINO。模式服务器必须等所需资源真正就绪后才返回
成功。DREAM 保持 `observe-only`，禁止其导航和操作代码发送任何底盘、手臂或夹爪命令。

## 7. ROS 接口和状态机

总控 Action 为 `/competition/execute_phase`，类型
`openarmx_task_msgs/action/ExecuteCompetitionPhase`。正式比赛依次发送 phase 1、2、3，
`run_id` 必须相同。跨序、并发、不同 run ID 或失败后的重复开始会被拒绝。

```text
READY_PHASE_1
 -> RUNNING_PHASE_1 -> JUDGE_GATE_1
 -> RUNNING_PHASE_2 -> JUDGE_GATE_2
 -> RUNNING_PHASE_3 -> FINISHED

任一正式环节失败 -> FAILED_LATCHED
```

失败锁存后不会自动接受第二次开始命令。只有裁判批准故障离场/重新开始后，才能按
现场流程重启或复位总控。导航和靠站最多按配置重试；物理抓取、夹爪闭合和放置不做
盲目自动重试，必须先重新观测和确认物体状态。

正式运行命令示例：

```bash
ros2 action send_goal --feedback /competition/execute_phase \
  openarmx_task_msgs/action/ExecuteCompetitionPhase \
  "{run_id: official_01, phase: 1, dry_run: false, timeout_sec: 420.0}"
```

裁判确认后仅将 `phase` 改为 2，再确认后改为 3。赛前可使用 `dry_run: true` 做接口
preflight；它不发送运动命令，也不推进正式环节状态。

## 8. 当前落地状态和真机前缺口

已经落地：环节 Action、裁判闸门状态机、配置校验、Nav2 巡检路线、工单排序去重、
GPU 模式切换契约、单件运输调用、放置复核调用、判定区收尾和失败锁存。

仍需实现或实测：

1. `/read_delivery_order` OCR/规格解析服务器。
2. `/inspect_retail_scene` 的 DREAM + 货架数据库规则服务器。
3. `/set_perception_mode` 的实际进程/GPU 资源管理服务器。
4. `/pick_object`、`/place_object` 完整抓放服务器；当前预抓取能力不能等同于抓放闭环。
5. `/verify_placement`、夹持/掉落检测和 PlanningScene 货架碰撞几何同步。
6. `stations.yaml` 中全部比赛站点、marker、抓取和放置位姿的实测替换。
7. 与裁判确认预建地图、联网 API、故障重入和阶段间重新布置规则。

缺少上述任一必需服务器时，preflight 会在底盘移动前失败。

当前 GUI 中的 `/competition/mock_pick_object` 只用于验证“抓取成功 -> 退避 -> 下一段
导航”的接口衔接。它假定物体已附着，不运行真实检测、SAM2、GraspNetBaseline、
MoveIt 抓取或夹爪，不能
作为正式抓取成功判据，也不会占用未来真实 `/pick_object` 的接口名。
