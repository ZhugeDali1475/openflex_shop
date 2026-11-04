# OpenFlex 抓取能力接入商超比赛控制台：逐项讨论稿

本文是 `openflex_grasp` 迁移到目标工作空间的设计讨论稿，面向后续把真实商品抓取接入 OpenFlex 商超比赛控制台。文中的“候选方向”和“建议”不代表已经决定；只有与项目负责人逐项讨论并明确确认后，才会进入实施计划。

## 0. 本文怎样逐项讨论

每个会改变接口、包归属、启动方式或比赛行为的问题都使用独立决策编号，并采用以下状态：

| 状态 | 含义 |
| --- | --- |
| `待讨论` | 只有事实和候选方案，没有项目决定 |
| `讨论中` | 正在比较选项，尚不能据此修改核心代码 |
| `方向确认` | 目标已经确认，但实现细节仍需继续讨论 |
| `已确认` | 负责人明确选择方案，可以进入实现 |
| `已实施` | 代码、配置、测试和文档均已完成 |

讨论过程固定为：核对当前代码事实 -> 列出选项 -> 说明代价和风险 -> 记录你的选择 -> 更新本文 -> 再进入下一项。不会把一次代码审查得出的推荐自动写成项目决定。

当前决策看板：

| 编号 | 决策点 | 当前状态 |
| --- | --- | --- |
| D0 | 新左右腕手眼标定接入 | **方案 A 已确认**；右腕换算完成，左腕等待对应相机连接 |
| D1 | `PlanPregrasp`、`CartesianApproach` 等 MoveIt 消息归属 | **方案 A 已确认**；待实施和回归测试 |
| D2 | `DetectObject` 与 `PickObject` 是否传递 `detection_id` | **方案 A 已确认**；待实施和缓存策略验收 |
| D3 | 本地 Qwen3.5-4B、OWLv2 与 DINO 的 provider/复核规则 | **已确认**；默认 Qwen，DINO/回退均采用方案 B/A |
| D4 | 第一版双臂能力与任务选臂规则 | **已确认**；双臂方案 B、配置选臂方案 A |
| D5 | 抓取调试界面加入比赛控制台 | **方案 C 已确认**；正式/调试进程 owner 强制互斥 |
| D6 | Qwen3.5-4B 的部署环境、显存和精度验收 | **方向确认**；部署方案已确认，GPU/精度数据验收后置 |
| D7 | AnyGrasp legacy 清理与 GraspNet 替换 | **已确认**；删除目标 legacy 及引用，统一使用 GraspNetBaseline |
| D8 | 本阶段是否实现真机放置 | **已确认后置**；本阶段不实现，保留接口与仿真/mock |
| D9 | `supermarket_scene` 是否成为目标 workspace 的正式依赖 | **方案 C 已确认**；系统级正式外部组件，抓取侧分层依赖 |
| D10 | 中立 2D 框 provider 的进程与通信边界 | **方案 A 已确认**；worker 内 Python 接口，OWLv2 使用 ROS client 适配 |

相关文档：

- 抓取套件的完整迁移说明：[openflex_grasp/MIGRATION_GUIDE_CN.md](/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_ws/src/openflex_grasp/MIGRATION_GUIDE_CN.md)
- 比赛任务与 GUI 现状：[SUPERMARKET_COMPETITION_INTEGRATION_CN.md](/home/openflex/old/openflex_ws/SUPERMARKET_COMPETITION_INTEGRATION_CN.md)
- 场景视觉和 GUI 设计：[SUPERMARKET_SCENE_GUI_MIGRATION_DESIGN_CN.md](/home/openflex/old/openflex_ws/SUPERMARKET_SCENE_GUI_MIGRATION_DESIGN_CN.md)

## 第一项（D0）：应用新左右腕手眼标定

### 已确认方案

左右腕均采用方案 A：把 `/home/openflex/old/headeye` 中以相机光学坐标系表示的手眼标定结果，换算为机器人末端 `link7 -> camera_link` 的安装外参，并写入 URDF 固定关节。RealSense 驱动继续作为相机内部 TF 的唯一发布者，发布 `camera_link -> color_frame -> color_optical_frame`。

禁止同时发布 `link7 -> color_optical_frame` 静态 TF，否则会与 RealSense 内部 TF 形成重复或冲突的 TF 链。

新标定绑定的腕部 D405 序列号为：

| 侧别 | 相机序列号 | 当前处理状态 |
| --- | --- | --- |
| 左腕 | `260522277324` | 当前未连接；必须连接该相机后再精确换算 |
| 右腕 | `260522279851` | 序列号匹配，精确换算已完成 |

标定采样时使用的是名义右臂运动学，当时的整机 URDF 尚无 `right_poe_calibrated` 分支。因此右腕新结果只对应目标比赛启动配置中的 `right_poe_calibrated=false`，不得直接用于 `right_poe_calibrated=true` 的 POE 右臂模型。

### 右腕已确认的 URDF 固定关节

将以下外参写入 `right_poe_calibrated=false` 对应的 URDF 分支：

```xml
<link name="right_camera_link"/>
<joint name="openarmx_right_link7_to_right_camera_link" type="fixed">
  <parent link="openarmx_right_link7"/>
  <child link="right_camera_link"/>
  <origin xyz="0.0830149457953 -0.0219906437615 0.0768390078403"
          rpy="-0.181214943034 -0.999759811745 -2.97809648406"/>
</joint>
```

对应四元数 `xyzw`：

```text
[-0.482244367477, 0.0401674710428, -0.874662654214, 0.0281457924245]
```

该换算重组回原始 `openarmx_right_link7 -> right_camera_color_optical_frame` 标定结果时，平移误差和旋转误差均为 0。它与目标 URDF 中旧安装值相差约 `33.896 mm / 13.408 deg`，新旧值不得混用。

### 左腕待完成事项

左腕也采用同一方案，但当前不能写入 `left_link7 -> left_camera_link` 数值。必须先连接标定绑定的 D405 `260522277324`，读取该设备实际发布的内部 TF，再使用同一换算流程生成固定关节外参。不得使用右腕相机内部 TF近似，也不得根据右腕结果推测左腕值。

### 唯一 TF 链

左右两侧最终均应保持单一父链：

```text
openarmx_<side>_link7
  -> <side>_camera_link                    URDF 固定关节
     -> <side>_camera_color_frame          RealSense 驱动
        -> <side>_camera_color_optical_frame
```

### 实施后的验收顺序

1. 检查 TF 发布者，确认每一段变换只有一个发布源且 TF 树无环。
2. 在 RViz 中同时检查机器人模型、彩色图、深度点云和相机坐标轴方向。
3. 固定一个静止物体，让机械臂改变多个姿态，确认物体在机器人基座坐标系下的位置基本不漂移。
4. 只显示商品 mask、物体点云和抓取候选，检查其与真实商品的空间重合。
5. 使用 `plan-only` 验证预抓取与进给轨迹，不驱动真实机械臂。
6. 完成安全限位和人工急停准备后，再进行低速、单次真实执行验收。

D0 当前实施边界：右腕换算结果已经确认，可以进入 URDF 修改与上述验收；左腕必须等待对应相机连接和精确换算后才能实施。

## 1. 已确认事实与总体架构

当前系统不是“把 `openflex_grasp` 目录复制进目标工作空间”这么简单，主要存在四个边界：

1. 目标工作空间已经拥有 `openarmx_integrated_moveit_msgs/action/PlanPregrasp` 和对应服务器 `/openflex/plan_pregrasp`。抓取套件中的同名 Action 来自 `openflex_grasp_msgs`，即使字段几乎相同，也不能作为同一个 ROS Action 使用。
2. `openflex_grasp_moveit` 提供目标工作空间目前没有的 `CartesianApproach` 能力，可以作为补充；它不应再重复提供 `PlanPregrasp`。
3. 比赛任务执行器的正式入口是 `/execute_transport`，其子接口是 `/detect_object`、`/pick_object`、`/place_object`。当前抓取执行器是“商品字符串触发的独立状态机”，不是这些 Action Server 的实现，不能直接替换 `/pick_object`。
4. `supermarket_scene/find_sku` 和 Qwen 都能做“商品名到 2D 框”，但前者是货架/SKU 场景查询服务，只返回像素框；后者是腕部 RGB-D 抓取流水线的第一步。两者应统一到可插拔检测 provider，不能让任务执行器直接把一个 2D 框服务当作完整抓取服务。

基于 D1-D10 已确认决策，目标总体架构如下；具体实施与各阶段验收尚未完成：

```text
/execute_transport                         openarmx_task_executor
    -> /detect_object                      真实感知适配器（待实现）
    -> /pick_object                        真实抓取适配器（待实现）
         -> ProductPerception provider     Qwen 或 FindSku 适配器
         -> SAM2 + RGB-D                    实例 mask 和物体位姿
         -> GraspNetBaseline                抓取候选拟合
         -> /openflex/plan_pregrasp         目标 MoveIt Action
         -> /openflex/cartesian_approach    统一操作 Action（中期加入）
         -> 夹爪/升降控制器
    -> /place_object                       本阶段仅保留接口与仿真/mock；真机实现后置
```

D1 已确认：复用目标工作空间已有的 `PlanPregrasp` 和 `ExecuteJointPosture`，把 `CartesianApproach` 合并进 `openarmx_integrated_moveit_msgs`，再迁移调用方并逐步废弃 `openflex_grasp_msgs` 中的 MoveIt Action。该方案尚未实施和回归测试。

## 2. 当前端到端链路

### 2.1 比赛控制台链路

目标任务包 `openarmx_task_executor` 的 `TaskExecutorNode` 只编排动作，不直接控制手臂：

```text
GUI/比赛协调器
    |
    +-- /competition/execute_phase（若启用三阶段裁判门控）
    +-- /execute_transport
            |
            +-- /navigate_to_pose
            +-- /dock_to_station 或 /shelf_fine_align
            +-- /competition/move_to_shelf_view（可选）
            +-- /detect_object
            +-- /pick_object
            +-- /retreat_from_station（可选）
            +-- 再导航和精停
            +-- /place_object
```

`ExecuteTransport` 的实际顺序是：预检查 -> 到取货站 -> 等待底盘稳定 -> 精停 -> 等待底盘稳定 -> 可选货架观察姿态 -> 检测 -> 将物体位姿变换到 `base_link` -> 抓取 -> 可选退避 -> 到放货站 -> 精停 -> 放置。失败会转成 `openarmx_task_msgs/TaskError` 数值错误码，并通过 Action result 返回；不会依赖解析日志文本。

### 2.2 当前 `openflex_grasp` 链路

抓取套件的主入口 `grab_product.launch.py` 启动的是另一套独立流水线：

```text
商品字符串参数
    -> 右腕 RealSense（默认 /cam_right/...）
    -> ProductPointCloudNode
         Qwen3.5 文本 grounding -> 多个 2D 框
         SAM2 -> 每个框的 mask
         对齐深度 + CameraInfo -> 商品点云
    -> GraspNetBaseline -> 候选抓取
    -> grasp geometry 后处理 -> pregrasp/grasp/夹爪开口
    -> product_grasp_executor.py
         photo -> pregrasp -> Cartesian/MoveIt 进给 -> 夹爪 -> 回撤/复位
```

这个执行器自己持有状态机、右臂/升降/夹爪动作和回拍照位逻辑；它没有实现 `DetectObject`、`PickObject` 或 `PlaceObject` Action Server。因此它应迁移为“感知与抓取后端”，而不是直接作为比赛任务执行器启动。

### 2.3 当前 GUI 的真实边界

`openflex_navigation_gui` 当前会检查 ROS graph 中是否存在 `/detect_object`、`/pick_object`、`/place_object` 和 `/execute_transport`，并显示“在线/等待接入”。但当前 GUI 节点只创建了以下与抓取联调相关的 Action client：

```text
/competition/move_to_shelf_view
/competition/mock_pick_object
/retreat_from_station
```

它没有创建真实 `/detect_object`、`/pick_object`、`/place_object` 或 `/execute_transport` 的 client。比赛流程页第一阶段实际执行：选 SKU -> 导航/精停 -> 货架观察姿态 -> 操作员勾选抓取成功 -> mock pick -> 退避/运输。因此“接口在线状态”和“GUI 已经调用真实接口”是两件事。

集成真实抓取后有两个选择：

- 推荐由 GUI 提交一个 `/execute_transport` goal，只显示总任务和子阶段状态，把安全顺序继续交给 `openarmx_task_executor`。
- 保留现有 GUI 手工串联流程作为工程联调页，但明确标记为 manual/mock，不作为正式比赛入口。

不要在 GUI 中复制一份真实检测、抓取、放置和错误重试状态机，否则它会与 `TaskExecutorNode` 形成第二个任务编排 owner。

### 2.4 D5 已确认方案 C：控制台嵌入与进程所有权

比赛控制台增加“抓取调试”顶层 workspace tab，嵌入现有抓取 Web 可视化内容，显示左右腕 RGB、2D 框、mask、商品/场景点云、GraspNet 候选、抓取路径以及检测/规划/执行状态。

D5 的后端生命周期采用方案 C：

- 正式比赛模式由 `supermarket_fullbody_runtime.launch.py` 唯一启动并拥有相机、感知、`/detect_object`、`/pick_object` 和抓取可视化后端。
- 整机 fullbody 未运行时，控制台允许通过现有 `QProcess + 独立进程组` 机制启动/停止专用 `grasp_debug.launch.py`。
- fullbody 已在线或检测到对应外部 ROS owner 时，禁止启动独立调试后端；调试后端在线时也禁止启动 fullbody，必须先停止调试进程。
- 关闭控制台只停止由该控制台启动的调试进程，不停止外部启动或正式比赛拥有的 fullbody 系统。

调试页提供手臂、SKU、2D provider、OWLv2/DINO 参数、启动/停止、仅检测和仅规划等控件。调试后端默认 `execute=false`；正式任务执行期间禁止更换手臂、provider 或模型参数。

现有 `grasp_web_visualizer.py` 的 HTTP 服务继续保持只读 GET API。所有会改变 ROS 状态或驱动机械臂的命令必须由控制台 ROS client 调用正式 Action/Service，不能给本地网页增加无安全门的任意执行 POST 接口。GUI 只发请求和显示反馈，不复制 `TaskExecutorNode` 的比赛状态机。

## 3. MoveIt 与消息冲突矩阵

| 能力 | 当前抓取套件 | 目标工作空间 | 结论 |
| --- | --- | --- | --- |
| 预抓取规划 | `openflex_grasp_msgs/PlanPregrasp`，多两个 `use_exact_joint_goal`、`exact_joint_goal` 字段 | `openarmx_integrated_moveit_msgs/PlanPregrasp` | **类型冲突**。同 topic 不代表兼容，必须选一个消息包 |
| 完整关节姿态 | 通过 `PlanPregrasp.exact_joint_goal` 扩展实现 | `/openflex/execute_joint_posture`，支持 `lift_only`、左右臂及带升降组 | 目标接口更清晰，替代扩展字段 |
| 直线/受限进给 | `openflex_grasp_msgs/CartesianApproach`，支持严格 Cartesian、走廊验证、普通 MoveIt 回退 | 未发现等价 Action Server | **目标缺失能力**，应迁移但换消息归属 |
| 低层 MoveIt | `openflex_grasp_moveit` 启动独立 Cartesian Action 节点，并向节点加载目标 MoveIt 参数；它不启动 `move_group` | `openarmx_integrated_moveit_config` 已是整机唯一 `move_group` 和控制器 owner | 可并行作为 client/server 辅助节点，但禁止再启动第二套 `move_group`、robot state publisher 或 controller |
| 夹爪/升降 | `product_grasp_executor.py` 直接调用控制器 | 目标任务 API 尚无真实 `/pick_object`、`/place_object` 实现 | 需要适配器，不能让两个状态机同时执行 |

### 3.1 `PlanPregrasp` 为什么不能直接共用

当前抓取执行器导入：

```python
from openflex_grasp_msgs.action import PlanPregrasp
```

目标 C++ 服务器导入：

```cpp
#include <openarmx_integrated_moveit_msgs/action/plan_pregrasp.hpp>
```

两者生成的 ROS 类型标识分别包含不同的 package 名。ROS 2 Action client/server 通过完整类型匹配；即使 goal/result 的大部分字段顺序一致，也不会握手成功。当前两个实现还共用 `/openflex/plan_pregrasp`，所以如果同时启动，表现会是“topic 名存在但客户端等不到正确服务器”或接口诊断混乱。

### 3.2 已确认方案 A：统一到目标消息包

D1 已确认采用方案 A，接口归属如下：

- `openarmx_integrated_moveit_msgs` 是比赛系统唯一的 MoveIt Action 消息包，继续提供 `PlanPregrasp` 和 `ExecuteJointPosture`。
- 把 `CartesianApproach.action` 从 `openflex_grasp_msgs` 迁入 `openarmx_integrated_moveit_msgs`。
- 抓取执行器改用目标版 `PlanPregrasp`；原抓取版增加的 `use_exact_joint_goal`、`exact_joint_goal` 不合入目标版。
- 恢复拍照位等完整关节姿态操作改为调用 `ExecuteJointPosture`，不再通过 `PlanPregrasp` 的扩展字段实现。
- `openflex_grasp_moveit` 暂时保留为 Cartesian server 的实现包，但改为依赖 `openarmx_integrated_moveit_msgs/CartesianApproach`。
- `openarmx_integrated_moveit_config` 继续作为整机唯一 `move_group`、机器人状态和控制器配置 owner；不得由抓取启动文件启动第二套实例。

统一后的接口结构为：

```text
openarmx_integrated_moveit_msgs
├── PlanPregrasp.action
├── ExecuteJointPosture.action
└── CartesianApproach.action      # 从 openflex_grasp_msgs 迁入
```

实施时修改 `openflex_grasp_moveit` 和所有 client 的 import、`package.xml`、CMake 依赖。完成全工作空间编译、Action 类型核验以及 plan-only/执行回归后，让 `openflex_grasp_msgs` 中的 MoveIt Action 进入 deprecated 状态；确认无调用方后再决定是否删除。D1 不要求现在合并 `openflex_grasp_moveit` 与 `openarmx_integrated_moveit_config` 两个实现包。

## 4. 任务 Action 的适配缺口

### 4.1 `DetectObject` 不是 2D 框接口

目标 `DetectObject.action` 输入商品/站点/货位，输出：

```text
found, object_pose (PoseStamped), confidence, detection_id, error_code, message
```

`TaskExecutorNode` 会要求检测服务器返回带 `frame_id` 的 pose，并立即将 pose 变换到 `base_link`。所以一个只返回 2D 框的节点不能直接注册为 `/detect_object`。

### 4.2 D2 已确认方案 A：通过 `detection_id` 交接检测快照

`PickObject.action` 输入 `object_id`、`object_pose`、`slot_id`、层号、臂名、`grasp_strategy` 等，但没有检测快照 ID。当前任务执行器在 `_detect()` 中拿到 `detection_id` 后直接丢弃，只把 `object_pose` 传给 `_pick()`。

这会产生一个真实集成风险：检测返回的 RGB、深度和 mask 可能已经不是抓取时的同一帧，抓取后端也无法可靠知道该使用哪个实例。

D2 已确认采用方案 A：

1. 给 `PickObject.Goal` 增加 `string detection_id`。
2. `/detect_object` 冻结同步 RGB-D，完成框、mask、商品点云、实例选择和 `object_pose` 后，以 `detection_id` 为键保存完整检测记录。
3. `TaskExecutorNode._detect()` 同时返回 `object_pose` 和 `detection_id`；位姿仍按现有流程变换到 `base_link`，ID 不变并由 `_pick()` 原样写入 `PickObject.Goal`。
4. `/pick_object` 按 ID 取回同一次检测的 mask 和商品点云，再运行 GraspNet；不得重新运行 Qwen/SAM2，也不得按商品名或货位猜测“最近一次”记录。
5. ROS 接口和内部缓存统一使用 `detection_id`，不再另设同义的 `observation_id`。

真实检测模式下 `detection_id` 必须非空、存在且未过期；仿真或明确关闭检测的名义位姿模式可为空，并走单独的 mock/nominal 行为。大型 RGB、深度、mask 和点云保留在本机检测缓存中，不直接放入 Action goal/result。

### 4.3 `PickObject` 的职责边界

建议真实 `/pick_object` 适配器负责：

- 校验 pose 的 frame、臂名、最大自由度和站点策略。
- 从检测快照或 ROI 重新得到商品点云，并运行 GraspNet 候选拟合。
- 调用目标 `/openflex/plan_pregrasp`。
- 调用统一的 Cartesian Action 完成最后进给，并执行夹爪闭合、附着判定和安全回撤。
- 返回 `object_attached=true` 之前必须有夹爪或负载证据，不能只以 MoveIt 轨迹成功代替抓取成功。

它不应再导航底盘，也不应创建第二个 `move_group`。`retreat_to_transport` 由任务执行器决定是否需要底盘退避；手臂回撤属于抓取适配器内部安全动作，二者要在命名和日志中区分。

### 4.4 D8 已确认：真机放置整体后置

本阶段只实现并验收以下真机能力边界：

```text
/detect_object -> /pick_object -> 手臂安全回撤
```

`PlaceObject.action` 和 `/place_object` 名称继续保留，避免现在破坏 `ExecuteTransport` 的既有接口；但本阶段只允许仿真或明确标识的 mock 实现承接，不实现交付台放置、货架插入、释放轨迹或真机释放判据。真机放置作为独立后续计划重新讨论，其范围至少包括放置目标表达、左右臂策略、approach/retreat、碰撞场景、夹爪释放和 `object_released` 的真实证据。

因此，本阶段可以在 `mock place` 模式下联调完整 `/execute_transport` 编排，但这只能证明 Action 串联和错误处理正常。mock 返回的 `object_released=true` 不得作为真机任务完成或比赛放置能力通过的依据；控制台必须清楚显示当前 place backend 是 `mock`，不能把整条链路标成“真机运输完成”。

### 4.5 当前错误语义会丢失细节

目标 Action 已经为 `DetectObject`、`PickObject` 和 `PlaceObject` 定义 `error_code` 与 `message`，但 `TaskExecutorNode` 当前没有完整保留：

- 检测 Action 只要状态不是 succeeded 或 `found=false`，统一返回 `OBJECT_NOT_FOUND`，没有区分 TF、模型、相机、超时等错误。
- 抓取 Action 只要状态失败、`success=false` 或 `object_attached=false`，统一返回 `PICK_FAILED`。
- 放置 Action 只要状态失败、`success=false` 或 `object_released=false`，统一返回 `PLACE_FAILED`。
- 最终 `/execute_transport` message 通常只显示 `transport task failed with error code N`，子节点的详细 message 不会传到控制台。

这不影响确定性分支，但会影响现场定位。建议适配器先严格使用 `TaskError` 中已有错误码；随后修改任务执行器，仅透传白名单内的稳定错误码，并把子 Action message 写入最终 result/结构化诊断。未知错误仍归并为 `PICK_FAILED`、`PLACE_FAILED` 或 `INTERNAL_ERROR`，避免不同包私自扩展数值造成契约漂移。

## 5. 两套 2D 检测能力如何合并

### 5.1 对比

| 维度 | `openflex_grasp` Qwen3.5 | `supermarket_scene/find_sku` |
| --- | --- | --- |
| 输入 | 当前右/左腕 RGB 图像 + 商品字符串 | `CompressedImage` + SKU/商品查询字符串 |
| 目标 | 直接为抓取选实例 | 货架场景中查找 SKU，支持数据库参考槽位 |
| provider | 本地 Qwen3.5-4B | 本地 OWLv2，可选 DINO 参考图复核；现有云端 provider 不进入本次比赛方案 |
| 输出 | 多个归一化框，随后 SAM2 mask、深度点云和抓取位姿 | 像素 `DetectedBox[]`，带 provider/model/request_id |
| 时序 | 与 RGB-D 近似同步，面向腕部实时抓取 | 服务请求内 JPEG 解码；响应没有 image stamp/frame/depth 关联 |
| 不能替代的部分 | 无货架数据库语义 | 无 mask、深度、点云、TCP 姿态和抓取执行 |

结论是“算法可以复用，ROS 契约不能直接复用”。`FindSku` 很适合 GUI、货架盘点和场景检查；真实抓取应通过适配器把它的框与发起请求时冻结的同一组 RGB-D 绑定，再进入公共的 SAM2、深度点云和实例选择阶段。

### 5.2 D3 已确认的检测边界与 provider 方向

`/detect_object` 的职责已确认到以下边界：

```text
商品和货位上下文
  -> 冻结一组腕部 RGB + 对齐深度 + CameraInfo
  -> 2D 框 provider
  -> SAM2 实例 mask
  -> mask + 深度生成商品点云
  -> 对多个实例做语义与几何质量筛选
  -> 计算选中实例的 object_pose
  -> 返回检测结果
```

GraspNet 不属于 `/detect_object`。它由后续 `/pick_object` 使用选中商品点云生成 6D 抓取候选，然后进入 MoveIt、Cartesian 进给和夹爪执行。

本次比赛当前只考虑两个本地 2D 框 provider：

- 默认 `qwen_local`：本地 Qwen3.5-4B，根据商品文本在腕部 RGB 中输出一个或多个框。
- 可选 `owlv2_local`：复用 `/supermarket_scene/find_sku` 的本地 OWLv2 路线；SKU 必须存在可用的英文 `owlv2_prompt`。
- Ark、SiliconFlow、DashScope 等云端 API 暂不进入运行方案，也不作为自动回退。

Qwen 查询文本不应把中英文简单拼成一个无结构字符串。应始终保留数据库中的 SKU 中文原称呼，并在存在 `owlv2_prompt` 时把它作为附加外观提示，例如：

```text
目标商品（规范名称）：<sku 中文原称呼>
外观英文提示（仅帮助识别外观）：<owlv2_prompt，可选>
```

Qwen 必须仍以中文规范名称标识目标；英文提示不能覆盖或替换 SKU 身份。

D10 已确认采用方案 A：2D 框 provider 是 Qwen+SAM2 worker 内部的 Python 接口，不新增 `/detect_boxes` ROS Service 或 Action。外部可取消、可反馈的稳定边界仍是 `/detect_object`；worker 内部串行调用 provider，避免为同一张 RGB 图增加 ROS 序列化和额外进程。

统一 Python 接口的输入至少包含冻结的 RGB、SKU 中文原名、可选 `owlv2_prompt`、最大框数和本次 provider 参数；输出统一为像素坐标框、可选置信度、标签、provider/model/request ID 和耗时。Qwen provider 直接实现该接口；OWLv2 provider 作为独立集成适配模块，通过 ROS client 调用现有 `/supermarket_scene/find_sku`，再转换为相同对象。核心感知模块不得 import `robot_service.py`，也不得直接依赖数据库或 Web 包。

已确认的可插拔结构为：

```text
openflex_grasp_perception/
├── providers/
│   ├── base.py                       # BoxProvider Python 接口和统一结果对象
│   └── qwen_provider.py              # 当前腕部文本 grounding
├── segmentation.py                   # SAM2
├── rgbd_observation.py               # RGB-D 同步、深度和 CameraInfo
└── observation_store.py              # detection_id 和一次性快照

openflex_grasp_supermarket_scene_adapter/
└── owlv2_provider.py                  # ROS client 调 /find_sku；可选 DINO 复核

openflex_grasp_pick/
└── grasp_fitting.py                   # GraspNet + 几何后处理，属于 /pick_object
```

内部统一检测记录包含：

```text
DetectionObservation
  object_id / sku
  boxes[] (pixel coordinates)
  selected_box
  mask
  rgb_stamp / depth_stamp / camera_info_stamp
  image_frame / depth_frame
  object_pose_base
  confidence
  provider / model / request_id
  detection_id / expires_at
```

已确认的 ROS 边界是：

```text
/supermarket_scene/find_sku
    = 场景/SKU 查询（返回框）
/detect_object
    = 比赛感知 Action（返回稳定 PoseStamped + detection_id）
/pick_object
    = 比赛抓取 Action（消费 detection_id，返回 attached）
```

不要让 `openarmx_task_executor` 直接订阅或解析 Qwen JSON，也不要让 GUI 直接控制抓取内部 topic。GUI 只看高层 Action 状态和诊断摘要。

抓取调试界面的方向已确认：允许选择 `qwen_local` 或 `owlv2_local`，并显示/调整与当前 provider 有关的 `max_boxes` 和阈值。选择 `owlv2_local` 且 SKU 存在参考图时，才允许开启 DINO 复核并调整相似度阈值；选择 `qwen_local` 时 DINO 控件必须禁用。正式比赛流程不应在任务执行中途改变这些参数；控制台应显示本次请求实际使用的 provider、模型、提示词来源、DINO 状态和阈值，以便复现结果。

D3 的 DINO 子决策已确认采用方案 B：维持现有边界，DINO 只作为 OWLv2 provider 内部的可选参考图复核器，Qwen 路线不使用 DINO。

D3 的失败处理已确认采用方案 A：一次 `/detect_object` 请求只运行显式选定的 provider。默认 Qwen 在模型加载、推理、输出解析、超时或无有效框时直接返回带原因的检测失败，不自动切换 OWLv2。需要使用 OWLv2 时，只能由抓取调试 GUI、比赛配置或下一次明确请求切换；不得在同一次任务中静默改变 provider。SAM2、深度、点云、TF 或 `object_pose` 阶段失败同样直接返回对应错误，不触发 2D provider 回退。

### 5.3 `FindSku` 的帧同步问题

当前 `FindSku.srv` 请求只有压缩图，响应没有：

- 图像时间戳；
- 图像 `frame_id`；
- 与深度图的对应关系；
- mask 或 observation ID 的严格语义。

因此不能安全地拿一次 `/find_sku` 结果配另一时刻的腕部 depth。按 D10，worker 先冻结同步 RGB-D，再由 OWLv2 Python 适配器把其中的同一张 RGB 压缩后调用 `FindSku`；返回框只能写回这条尚未完成的检测记录，并与原 depth、CameraInfo、stamp 和 frame 一起生成 `detection_id`。请求超时、取消或帧校验失败时整条记录作废，不得按“最近一帧”补配。

## 6. D7 已确认：删除 AnyGrasp legacy，使用 GraspNetBaseline

本次迁移完全不使用 AnyGrasp。迁移起点是：

```text
/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_ws/src/openflex_grasp
```

迁移终点是：

```text
/home/openflex/old/openflex_ws
```

新的比赛抓取只使用迁移起点中的 GraspNetBaseline。已删除的 `/home/openflex/old/anygrasp_sdk_v3` 不恢复；其他目录中的 AnyGrasp SDK、checkpoint、环境和机器许可证不复制、不引用，也不作为备用后端。

目标 workspace 原有的 `src/openflex_thesis/openflex_anygrasp` ROS legacy 包及其比赛/仿真功能引用纳入迁移清理范围。实施清单为：

1. 删除 `src/openflex_thesis/openflex_anygrasp` 源码包，并从保留的 bringup/package manifest 中移除对 `openflex_anygrasp` 的依赖。
2. 清理 `openarmx_supermarket_sim` 中的 `start_anygrasp`、SDK/checkpoint/Python preflight、`anygrasp_sim.yaml`、AnyGrasp launch include 和 gateway 启动分支。
3. 不把 `/thesis/anygrasp/request`、`/thesis/anygrasp/candidates` 或 DREAM AnyGrasp topic 适配成新比赛接口；新仿真/真机适配器直接实现 `/detect_object` 和 `/pick_object`。
4. 把 `stations_supermarket_test_07/08.yaml` 的 `mock_or_anygrasp` 改为明确的 `graspnet_shelf`、`graspnet_basket` 或 `mock`；把 MuJoCo 配置中的 `anygrasp` 改为与实际仿真后端一致的 `graspnet_*` 或 `mock`。
5. 从比赛控制台、状态检查、文档和测试中删除 AnyGrasp SDK/worker 的入口与就绪条件，换成 Qwen/SAM2、GraspNet worker、`/detect_object` 和 `/pick_object` 的健康状态。
6. 执行干净构建和启动测试，确认不存在因旧 `install/` 残留造成的假依赖；不复制或沿用旧 AnyGrasp 构建产物。

`openflex_thesis_msgs` 还被其他 thesis 包用于非 AnyGrasp 接口，D7 不整包删除它。其中专属的 `AnyGraspRequest`、`GraspCandidate*` 消息以及 `openflex_tc_wam`/thesis 仿真内的 AnyGrasp gateway 是否随整个 thesis 子系统继续保留，必须以其余 thesis 功能是否仍在目标范围为依据；它们不能进入新的比赛 fullbody launch。

D7 的“替换”不是把旧 topic 名称机械映射到 GraspNet。新的链路必须按已确认的 D2-D6 契约完成同步 RGB-D、provider、SAM2、`detection_id`、共享 GraspNet worker、双臂 MoveIt/Cartesian 和夹爪执行适配。

## 7. 相机、TF 和启动冲突

### 7.1 D4 已确认方案 B：第一版同时完成左右臂

D4 已确认采用方案 B。第一版真实比赛能力必须同时支持左臂和右臂，不能把“消息中存在 `arm_name`”或“左腕能生成点云”视为已经支持左臂。

左右腕原始 optical 手眼标定结果都已存在并纳入 D0。右腕 `link7 -> camera_link` 已完成精确换算；左腕不是重新标定，而是等待连接其标定绑定的 D405 `260522277324`，读取设备内部 TF 后完成同样的 URDF 固定关节换算。

当前左腕 launch 只能启动相机、Qwen/SAM2 和 GraspNet 候选生成，尚无完整左臂执行链；`product_grasp_executor.py` 还明确限制为右臂，并写死右臂关节、TCP、夹爪 Action、负载参数和拍照姿态恢复。迁移时必须把这些差异收敛为明确的左右臂配置，而不是复制两份状态机：

```text
ArmContext
  side
  camera_serial / RGB-D topics / optical_frame
  tcp_frame / planning_group
  arm_joint_names / gripper_action
  payload parameters
  photo and transport postures
  approach and closing axes
```

`DetectObject`、`PickObject`、`PlanPregrasp` 和 `CartesianApproach` 保持同一套接口，通过明确的 `arm_name`/`arm` 选择 `ArmContext`。两侧必须分别完成 TF、多姿态点云、候选方向、plan-only、低速抓取、夹持判定和失败回撤测试。

D4 的任务选臂已确认采用方案 A：正式比赛由 station/slot 配置中的显式 `arm_name: left|right` 决定，不同时运行两侧检测后再自动比较，也不由操作员在任务执行中临时换臂。`TaskExecutorNode` 必须在进入检测阶段前解析并固定本次任务的手臂，把同一个值传给 `/detect_object` 和 `/pick_object`。

当前 `DetectObject.Goal` 没有手臂字段，需要增加：

```text
string arm_name
```

检测记录同时保存 `arm_name`、`camera_serial` 和 `camera_frame`。`/pick_object` 按 `detection_id` 取记录时，必须校验记录中的手臂与 `PickObject.Goal.arm_name` 一致，禁止使用一侧腕部点云驱动另一侧手臂。抓取调试 GUI 可以在没有正式比赛任务运行时人工选择手臂，但这只是显式生成一条带对应 `arm_name` 的调试请求，不覆盖正在执行的任务。

允许实现阶段先完成右臂切片，再按同一抽象接入左臂；但第一版发布验收必须两侧全部通过。当前 `stations_supermarket_test_08.yaml` 只有右臂示教姿态和右臂 slot，后续还必须为每个真实来源 slot 显式选择手臂，并补充左臂货架观察/运输姿态，否则即使代码支持左臂，比赛流程也不会实际选择它。真实任务配置缺少或写入非法 `arm_name` 时必须在预检查阶段拒绝，不再依赖 `default_arm_name` 猜测。

抓取套件默认用：

```text
/cam_right/color/image
/cam_right/depth/image
/cam_right/depth/camera_info
```

目标旧 AnyGrasp/仿真兼容别名主要是：

```text
/camera/right_camera/color/image_raw
/camera/right_camera/aligned_depth_to_color/image_raw
/camera/right_camera/color/camera_info
```

目标整合 URDF 已提供 `right_camera_link` 和 `right_camera_color_optical_frame`。迁移时应采用一个真实 RealSense driver，使用 remap/relay 提供兼容 topic，而不是启动两台同 serial 的 driver。硬件 launch 还必须确认：

- `publish_tf` 只由一个 driver 或 robot description owner 发布；
- color/depth 分辨率、对齐方式和 CameraInfo 与 GraspNet 深度尺度一致；
- `right_camera_color_optical_frame -> openarmx_right_link7` 手眼外参是目标机器实测值；
- 当前 `test_08` slot 使用 `arm_name: right`，而 task executor 默认参数是 `left`；D4 要求第一版支持两侧，因此必须统一臂选择规则并补齐左臂 slot/姿态配置，不能依赖互相矛盾的默认值。

仿真侧 `openarmx_supermarket_sim` 已发布 `/cam_right` 和 `/camera/right_camera` 兼容通道，并把深度转成 RealSense/AnyGrasp 使用的毫米 `16UC1` 语义；这只说明 topic 适配存在，不代表真机标定已经完成。

## 8. 目标工作空间的包结构建议

### 8.0 D6 已确认的模型资产根目录

迁移后的模型资产统一放在 workspace 外部：

```text
/home/openflex/old/
├── openflex_models/
│   ├── Qwen3.5-4B/
│   ├── sam2.1-hiera-small/
│   └── 其他经确认的本地模型资产
└── openflex_ws/
```

确认使用 `openflex_models`，不另建名称相近的 `openfx_models`。大模型权重不放入 `openflex_ws/src`、Git 仓库或 colcon 的 `build/install` 目录。

启动环境统一引用：

```text
OPENFLEX_MODEL_ROOT=/home/openflex/old/openflex_models
OPENFLEX_QWEN_MODEL=/home/openflex/old/openflex_models/Qwen3.5-4B
OPENFLEX_SAM_MODEL=/home/openflex/old/openflex_models/sam2.1-hiera-small
```

当前 `Qwen3.5-4B.zip` 仍保留在原资产目录，本项只确认目标布局；实际解压、校验和启动参数修改要在 D6 的 Python/CUDA 环境确定后实施。

D6 的 Python 环境已确认采用方案 C，拆分为两个独立模型环境：

```text
/home/openflex/old/.venvs/
├── openflex_perception/   # Qwen3.5-4B + SAM2
└── openflex_graspnet/     # GraspNet + PointNet2/KNN CUDA 扩展
```

本地 OWLv2 和可选 DINO 继续由 `supermarket_scene/.venv` 及 `/supermarket_scene/find_sku` 服务拥有。`product_grasp_executor`、MoveIt bridge 和抓取 Web 可视化等控制/展示节点使用 ROS 系统 Python，不安装进模型环境。

两个模型环境均以系统 Python 3.10 为基础并能够导入 ROS 的 `rclpy`、消息类型和 `cv_bridge`。依赖使用各自的锁定文件管理；GraspNet 的 PointNet2/KNN 必须针对目标 PyTorch、CUDA toolkit 和 GPU 重新编译，禁止复制源工作空间现有的 `.so`。

当前脚本的单一 `OPENFLEX_PRODUCT_SEG_PYTHON` 要拆分为至少：

```text
OPENFLEX_PERCEPTION_PYTHON=/home/openflex/old/.venvs/openflex_perception/bin/python
OPENFLEX_GRASPNET_PYTHON=/home/openflex/old/.venvs/openflex_graspnet/bin/python
```

Qwen/SAM2 节点与 GraspNet 节点分别使用对应解释器；其余节点不继承模型解释器。实际创建环境和选择精确版本要等待 GPU 驱动/CUDA 状态恢复并完成兼容性测试。

D6 的双臂模型进程拓扑已确认采用方案 A：左右臂共享模型 worker，不为每侧相机重复加载模型。

```text
DetectObject(arm_name)
  -> RGB-D router 选择对应腕部同步帧
  -> 单一 Qwen3.5-4B + SAM2 worker
  -> detection_id 绑定 arm_name / camera_serial / frame

PickObject(detection_id, arm_name)
  -> 单一 GraspNet worker
  -> 校验检测记录后处理对应商品点云
```

全系统最多常驻一个 Qwen/SAM2 worker 和一个 GraspNet worker；各 worker 内部串行处理任务。左右 RealSense 可以同时在线，但一次检测只冻结任务指定侧的 RGB-D。现有左右 launch 中重复启动模型节点以及共用 `/product_perception/*` 全局 topic 的方式不能直接迁移，必须改为按请求和 `detection_id` 路由。

D6 的显存驻留已确认采用方案 A：Qwen3.5-4B + SAM2 worker 与 GraspNet worker 在 fullbody 启动时完成加载，并在运行期间同时常驻显存，避免每次 Detect/Pick 切换时重复加载模型。默认 Qwen 比赛模式不启动或预加载 OWLv2/DINO 模型；调试 GUI 显式选择 OWLv2 时才启动对应视觉后端。

D6 的模型精度格式已确认采用方案 A：Qwen3.5-4B 按模型原始配置使用 BF16，SAM2 使用 BF16 autocast，GraspNet 保持其经验证的默认 FP32，不强制旧 CUDA 扩展进入半精度。第一版不引入 INT8/4-bit 量化和 `bitsandbytes` 依赖；只有常驻显存实测无法保留安全余量时，才重新讨论量化方案并重新做商品框、JSON 和端到端精度回归。

目标 GPU 已通过 PCI 识别为 RTX 3090（24 GB 规格），但当前 `nvidia-smi` 无法连接驱动、NVIDIA 设备节点缺失，并存在 590/595 驱动包混装，因此尚未完成任何 CUDA 加载或显存验证。方案 A 的验收门槛为：

- Qwen3.5-4B、SAM2 和 GraspNet 全部加载后至少保留 `2 GB` 可用显存余量；
- 连续运行至少 20 轮 Detect -> Pick 推理，无 CUDA OOM、模型重载或显存持续增长；
- 同时记录每个模型加载时间、单轮延迟、已分配/保留显存峰值和失败原因；
- 若不满足门槛，必须重新进入显存策略讨论，不能静默改为动态卸载或量化。

当前没有可用的左右腕商品 RGB-D 验收数据集，因此 2D 框召回率、误检率、SAM2/点云质量、端到端姿态稳定性和抓取成功率暂不判定。数据采集与冻结验收集后再执行这组回归；本阶段只保留验收项目和门槛，不把缺少数据解释为模型已经通过。

### 8.1 迁移后的推荐布局

```text
openflex_ws/src/
├── openflex_integrated/
│   ├── openarmx_integrated_description/
│   ├── openarmx_integrated_moveit_msgs/       # 唯一 MoveIt Action 定义
│   ├── openarmx_integrated_moveit_config/     # move_group、PlanPregrasp、JointPosture、Cartesian
│   ├── openarmx_task_msgs/                    # 比赛高层 Action 定义
│   ├── openarmx_task_executor/                # ExecuteTransport 编排
│   └── openflex_navigation_gui/               # 只调用高层 Action
├── openflex_grasp/
│   ├── openflex_grasp_perception/              # provider、SAM2、RGB-D、GraspNet
│   ├── openflex_grasp_adapters/               # DetectObject/PickObject/PlaceObject server
│   └── openflex_grasp_tools/                  # 离线评估和 RViz/Web 诊断
├── supermarket_scene_ros/                     # 先以外部 overlay 接入
└── openflex_thesis/                           # legacy，默认不启动
```

实际第一阶段可以保留当前 Python 包名 `openflex_grasp`，避免一次性改动入口；先把 `product_grasp_executor.py` 的模型和几何逻辑提取为可调用库，再添加 `detect_object_adapter`、`pick_object_adapter`。包结构重构不应与首次真机联调同时进行。

### 8.2 `supermarket_scene` 的接入方式

`/home/openflex/old/supermarket_scene/ros2` 下有三个 ROS 包，但 `supermarket_scene_ros` 当前通过 `SUPERMARKET_SCENE_ROOT` 动态寻找仓库根目录和 `robot_service.py`。因此不能只复制三个 ROS 包目录后就认为它是自包含的。

D9 已确认采用方案 C：`supermarket_scene` 是 OpenFlex 比赛系统的正式外部组件，但不是 `openflex_grasp` 可以直接调用内部 Python 模块的代码库。依赖分为三层：

1. 系统部署层通过版本固定的 `.repos` 或等价部署清单声明完整 `supermarket_scene` 仓库、模型环境和数据路径，并把其 ROS 包构建为正式 overlay。
2. 比赛控制台可以显式依赖 `supermarket_scene_interfaces` 和 `supermarket_scene_web`，继续提供场景数据库、Web 页面和 `/find_sku` 调试能力。
3. `openflex_grasp` 只依赖中立的 2D 框 provider 契约和适配器；不得 import `robot_service.py`，也不得读取 `shelf_inventory.db` 或引用 Web 实现。OWLv2/DINO 适配器负责把 `supermarket_scene` 的响应转换成该契约，Qwen provider 使用同一输出结构。

过渡阶段可以把现有仓库作为外部 overlay 构建，并设置：

```bash
colcon build --base-paths src /home/openflex/old/supermarket_scene/ros2
export SUPERMARKET_SCENE_ROOT=/home/openflex/old/supermarket_scene
```

上述环境变量只是过渡兼容方式，不是最终部署契约。正式集成需要移除代码中的开发机绝对路径，为仓库根目录、数据库和模型目录提供 launch/config 参数，并把 `cv2`、`numpy`、OWLv2、DINO 等当前未完全由 `package.xml` 表达的依赖写入部署清单。D10 已确认 provider 契约采用 worker 内 Python 接口；只有 OWLv2 集成适配模块通过 ROS client 调用现有 `/find_sku`。

## 9. 分阶段迁移计划

### 阶段 0：冻结现状和所有权

- 记录目标机器人 ROS graph、控制器、右腕相机 serial、TF、CUDA 和模型 manifest。
- 为 `openflex_grasp`、目标 MoveIt、AnyGrasp legacy 分别保存 plan-only/录包基线。
- 在比赛 launch 中明确禁止 legacy AnyGrasp 和独立 `grab_product.launch.py` 同时启动。

验收：只有一个 `/openflex/plan_pregrasp` server、一个相机 driver、一个夹爪 owner。

### 阶段 1：只迁移感知，不运动

- 目标 workspace 构建 `openflex_grasp` 三个包，不复制 `build/`、`install/`、CUDA `.so`。
- 校验并解压已提供的 `Qwen3.5-4B.zip`，同时确认 SAM2 和 GraspNet checkpoint 完整可用；不得继续使用代码中的旧 `Qwen3.5-9B` 默认路径。
- 用目标左右腕 RGB-D 录包验证 Qwen 框、SAM2 mask、深度点云、TF 和 GraspNet 候选。

验收：同一录包可重复生成 `detection_id`、目标 mask、点云和候选 JSON；不连接任何 MoveIt/夹爪控制器。

### 阶段 2：收敛 MoveIt 依赖

- 修改抓取 client 使用 `openarmx_integrated_moveit_msgs/PlanPregrasp`。
- 去掉 `exact_joint_goal` 依赖，篮筐/回拍照位改调用 `/openflex/execute_joint_posture`。
- 暂时单独运行 `CartesianApproach` 服务器，topic 使用独立命名，确认不创建第二个 `move_group`。

验收：真机或仿真中能完成 photo -> pregrasp 的 plan/execute，且不出现第二套 controller 或 Action type mismatch。

### 阶段 3：实现比赛高层适配器

- 实现 `/detect_object`：同步 RGB-D、选择 provider、运行 SAM2/深度位姿，返回带 frame 的 pose、置信度和 `detection_id`。
- 实现 `/pick_object`：消费检测快照，运行 GraspNet，调用目标 MoveIt Action、Cartesian 进给、夹爪和附着判定。
- 按 D2 给 `PickObject` 增加 `detection_id`，并修改 task executor 原样传递。
- 保留 `/place_object` 接口和仿真/mock 实现；本阶段不实现任何真机放置动作。

验收：左右臂分别通过独立 Action CLI 完成检测 -> 抓取 -> 安全回撤，错误码稳定映射到 `TaskError`；不以真机 Place 作为本阶段验收项。完整 `/execute_transport` 只在明确的 mock place 模式下做编排联调。

### 阶段 4：接入控制台和比赛流程

- 为 GUI 增加 `/execute_transport` client；只显示总任务及子 Action 的 phase/progress/error，不再以 `mock_pick_object` 作为真实成功判据。
- 先保留 mock endpoint 作为回归开关，真实 endpoint 在线时 GUI 显示真实状态。
- 按 D5 方案 C 增加“抓取调试”workspace tab；正式 fullbody 后端与控制台启动的 `grasp_debug.launch.py` 强制互斥，网页保持只读，命令通过 ROS client 发送。
- 对 test_08 站点配置逐项确认右/左臂、货架层号、lift 姿态和抓取策略；放置策略留到后续阶段。

验收：取消、超时、底盘未停稳、TF 丢失、检测不到和夹爪未附着都能停止后续阶段并显示确定性错误；mock place 仅验收接口失败分支，不代表真机放置验收。

### 阶段 5：性能和比赛可靠性

- 测量 Qwen/SAM2/GraspNet 延迟、显存峰值和抓取成功率；必要时将 detector provider 和 GraspNet 分进程或 GPU 调度器。
- 给 observation store 加过期、单消费者和失败回收机制。
- 对相机 topic、TF、控制器和动作 server 做启动前健康检查。
- 完成 D7 规定的 `openflex_anygrasp` 包、比赛/仿真/preflight/GUI 引用删除和抓取策略重命名。

## 10. 需要共同确认的决策

以下问题会改变接口或比赛策略，建议在开始阶段 3 前确认：

1. **D4 已确认**：双臂能力采用方案 B，第一版同时完成左右臂；选臂采用方案 A，由 station/slot 显式配置并在检测前固定，同值传给 Detect/Pick。
2. **D2 已确认**：沿用 `DetectObject.Result.detection_id`，给 `PickObject.Goal` 增加同名字段，并用它交接同一次检测的 mask 和商品点云。
3. **D3 已确认**：2D 框默认使用本地 Qwen3.5-4B，可在抓取调试界面显式切换到本地 OWLv2；云端 API 暂不考虑。DINO 方案 B 只复核 OWLv2；失败处理方案 A 不做自动 provider 回退。
4. **D1 已确认**：引入 `CartesianApproach.action` 到 `openarmx_integrated_moveit_msgs`，并统一 MoveIt Action 消息归属；实现和回归尚未完成。
5. **D8 已确认后置**：本阶段不实现真机放置，只保留 `/place_object` 接口与仿真/mock；当前真机边界是 Detect -> Pick -> 安全回撤。交付台和货架插入的范围、approach、碰撞与释放判据留到后续计划重新讨论。
6. **D7 已确认**：本次迁移不使用 AnyGrasp；删除目标 `openflex_anygrasp` legacy 包及比赛/仿真引用，相关能力由新 GraspNetBaseline Detect/Pick 链替换。
7. **D9 方案 C 已确认**：`supermarket_scene` 作为比赛系统版本固定的正式外部组件和 overlay；控制台可依赖其 interfaces/Web，抓取链只通过中立 2D provider 适配层接入，不直接依赖数据库、Web 或 `robot_service.py`。
8. **D10 方案 A 已确认**：中立 2D 框 provider 使用 Qwen+SAM2 worker 内 Python 接口；Qwen 直接实现，独立 OWLv2 适配模块通过 ROS client 调 `/find_sku`，不新增内部检测 ROS Service/Action。

## 11. 建议的首个实现切片

不要一次改完整比赛流程。先完成“真实取货、仿真/mock 放置”的开发切片，并按 D4 分别完成左右臂 Detect/Pick 验收：

```text
固定 station/slot
  -> /detect_object（本地 Qwen + SAM2 + RGB-D）
  -> /pick_object（GraspNet + 目标 PlanPregrasp + Cartesian + 夹爪）
  -> 返回 object_attached
```

先不接 GUI 第一阶段快捷按钮；用 `ros2 action send_goal`、录包和仿真验证 Action 契约。这个切片通过后再把同一个适配器挂到 `/execute_transport`，最后才替换 GUI 中的 mock 流程。AnyGrasp 已按 D7 从目标链路移除，不再作为此切片的开关或回退。

## 12. 当前明确的禁止事项

- 不要在目标工作空间再放一个同名 `openflex_grasp_msgs/PlanPregrasp` 并连接 `/openflex/plan_pregrasp`。
- 不要复制、启动或恢复任何 AnyGrasp SDK、checkpoint、环境或 worker；按 D7 清理旧入口。
- 不要让 `openarmx_task_executor` 解析 Qwen JSON、GraspNet JSON 或内部 topic。
- 不要以 MoveIt 规划成功代替商品已经夹住的判定。
- 不要把当前 `build/`、`install/` 或 CUDA 扩展二进制复制到目标机。
- 不要在没有目标机器人手眼标定、夹爪标定和安全限位验证前开启 `execute=true`。

## 13. 后续计划：真机放置

D8 将真机放置整体移出当前迁移阶段。后续启动该项前，需要单独确认比赛要求是交付台放置、货架插入还是两者都需要，并据此设计 `PlaceObject` 的目标表达、双臂动作、碰撞场景、释放与回撤轨迹，以及可验证的 `object_released` 判据。在这些决策和真机验收完成前，mock place 不得升级为正式比赛能力。

D0-D10 的架构与阶段边界均已由项目负责人确认，可以转入正式执行计划；代码、配置、模型环境、硬件和数据验收仍须逐项实施并留存证据，不能因设计已确认而标记为已完成。
