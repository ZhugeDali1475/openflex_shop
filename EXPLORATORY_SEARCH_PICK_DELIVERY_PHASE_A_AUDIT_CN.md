# 阶段 A：基线和真值泄漏初审

审计日期：2026-11-19  
范围：只读静态审查 + 已保存的仿真运行证据；本次没有启动仿真、ROS 控制栈或任何运动。  
结论：**阶段 A 未通过，阶段 B 不得开始。** 当前搜索代码存在已登记槽位先验，抓取候选链路使用 MuJoCo Oracle；两者与纯视觉未知位置目标不相符。

## 1. 冻结基线

- 当前 Git `HEAD`：`2a0e9db8bcb6ee264ce6606fd52b33f349d1d777`（`Implement multi-shelf retail delivery contracts`）。
- 工作区不是干净状态：本任务计划书为新增文件；另有用户工作区中的 `openflex_ws/` 文档删除项。本轮未恢复、暂存或覆盖这些删除项。
- 场景：`local_dependencies/openflex_thesis/retail_fixture_single_red_runtime_20261116_left_v9/`。
- `scene.xml` SHA-256：`6db3a9bfad3fa6f937856521ece259e06fe1efc578db858251cb630a82257995`。
- `runtime.yaml` SHA-256：`44f372b5f08bb9ba584cfc96cab90b483e0fe2901816db0fbff8ab949fbe8e5d`。
- `shelf_scene.yaml` SHA-256：`9c6083e952e5f040a637572fc34f353f0fe88f35af56c9590a34e74bb85f495c`。
- Manifest 拓扑：两组货架、一个活动商品、左台 `sim_delivery_top_left` 是唯一范围内目的地；场景 `execution_enabled=false`。
- ROS 运行隔离：`scripts/openflex_runtime.py` 固定使用 `ROS_DOMAIN_ID=83`，且必须显式传入 `--enable-simulation-motion`；此开关只允许 MuJoCo 仿真链，不授权实机。
- 当前审计时没有发现 18552/18553/18556 监听服务或活动仿真进程；所以本轮没有实际读取在线相机帧、节点列表或运行时订阅关系。

## 2. 当前数据流（源码静态追踪）

```text
`/thesis_sim/head/color/image_raw`
`/thesis_sim/head/aligned_depth_to_color/image_raw`
`/thesis_sim/head/color/camera_info` + TF map<-camera
        │
        ▼
SearchSkill._fresh_frame / analyze_once
        │
        ├─ registry.objects[*].nominal_pose_xyzyaw → expected_slot（目标登记槽位先验）
        ├─ HSV 颜色连通域 + 像素/深度过滤 → image candidate
        └─ resolve_search_target(..., object_id=registry ID)
                    │
                    ▼
        ManipulationSkill.execute(object_id="thesis_item_01")
                    │
                    ▼
        /thesis/generate_grasps → AnyGrasp gateway
                    │
                    ▼
        oracle_anygrasp → /thesis/anygrasp/candidates
        （读取请求中由 object_memory/MuJoCo 真值产生的目标位姿）
                    │
                    ▼
        TC-WAM / IK / MoveIt → 抓取候选评估与轨迹
```

放置、附着、落稳等 MuJoCo 状态可作为仿真物理结果/评测证据；但这些状态不可再被用作图像识别或抓取候选的输入。

## 3. 已确认的边界问题

### A1. 搜索不是未知槽位搜索

`openflex_roboos_bridge/search_skill.py`：

- `_expected_slot()` 从 `node.registry["objects"][object_id]["nominal_pose_xyzyaw"]` 读取商品名义位置。
- `analyze_once()` 将该坐标投影到相机图像，并传入 `analyze_frame(expected_point_map=...)`。
- `target_search_contract.py` 以投影的 expected pixel、expected depth 和槽位半径筛选候选。
- 候选被授权时，`resolve_search_target()` 把候选绑定回传入的登记 `object_id`。

因此，已有 P4.3 `search_and_pick` 证明的是：在登记槽位附近用图像候选做颜色/可见性判定；它**没有证明**机器人不知道槽位位置，也没有主动搜索整个货架或未知地图。

### A2. 视觉实例 ID 与仿真对象 ID 尚未解耦

`search_skill.py` 的 `execute()` 从唯一 registry 商品中取出 `object_id`，最终把 `{"object_id": "thesis_item_01"}` 交给 `ManipulationSkill`。抓取接口和部分工具 schema 也强制单一注册 ID。当前不存在端到端的视觉 `perception_track_id` 身份链路。

### A3. 抓取候选不是实际 AnyGrasp 相机/点云推理

`scripts/openflex_manipulation_preflight.launch.py` 同时启动 `anygrasp_gateway` 与 `oracle_anygrasp`。`oracle_anygrasp_node.py` 的说明明确是 MuJoCo ground-truth grasp backend；它用请求中的物体真值位姿构造候选，并将 `physical_perception=false`、`source=mujoco_ground_truth_oracle_NOT_PHYSICAL` 写入结果。

因此当前保存的 AnyGrasp 命名/候选协议，不等于部署并运行了真实 AnyGrasp 模型。阶段 D 需要单独确认实际推理 SDK、模型/权重、数据输入和许可证；不能以 Oracle 的候选成功代替视觉抓取算法验收。

### A4. 旧任务语义依赖“固定目的地”和“预登记对象”

本计划第一验收层将 `sim_delivery_top_left` 固定为已知目的地是允许的，但这只能证明目标位置未知的自主搜索，不证明交付台位置也未知。任务解析目标（例如“红色商品”）、视觉实例身份、MuJoCo object ID、交付台语义 ID 必须保持不同字段。

## 4. 已有证据与本轮检查

- 已有搜索批次说明：`local_dependencies/openflex_thesis/multiskill_execution_20261114/p4_search_and_pick_20261117/notes.md`。记录的 `oracle_used_for_selection=false` 表示 oracle 不参与图像候选选择；它不表示抓取候选阶段不使用 Oracle。二者必须分别审计。
- 该批次保留相机帧、深度、候选和选择理由：`local_dependencies/openflex_roboos/runs/search_and_pick_p43_20261117_v2/`。
- 搜索/目标解析相关离线测试：`29 passed`（`test_openflex_target_search.py`、`test_openflex_target_resolution_contract.py`、`test_openflex_retail_search.py`、`test_openflex_mcp_target_compat.py`）。这证明当前契约测试通过，不证明未知槽位能力。
- 静态语法编译：运行时启动器、预检 launch、SearchSkill、目标搜索契约、ManipulationSkill、Oracle 节点均通过 `py_compile`。
- `git diff --check` 通过。

## 5. 阶段 A 条目状态

- (✓) 固定 v9 双货架/单红/左台场景并核对 `scene.xml` 哈希。
- (✓) 绘制当前搜索、抓取、Oracle、TC-WAM/MoveIt 的静态数据流图。
- (✓) 完成第一轮源码泄漏审计，明确找到登记槽位先验和 Oracle 抓取候选输入。
- (✓) 核对 ROS domain、仿真启动门、运行目录和安全停机代码入口。
- ( ) 完成身份解耦：视觉 `perception_track_id`、任务语义目标与 MuJoCo `object_id` 分离。
- ( ) 建立纯视觉抓取运行模式，保证该模式不启动/调用 Oracle；Oracle 对照使用独立开关和批次。
- ( ) 建立感知日志与评测真值分离的 schema，并为本阶段生成运行证据。
- ( ) 在线运行时复核节点、话题、输入来源及相机帧；本轮没有启动 ROS，因此尚未完成。

## 6. 下一步（仍属于阶段 A）

1. 先设计并测试任务目标/视觉实例/仿真 ID 分层，不直接改写运行数据流。
2. 在抓取接口层增加“仅视觉观测输入”的显式契约；缺目标深度、点云、时间戳或 TF 时必须拒绝抓取。
3. 将 MuJoCo Oracle 变为显式的仅对照后端，纯视觉 profile 默认拒绝启动 Oracle 和读取真值定位话题。
4. 明确实际抓取模型可用性：检查仓库依赖清单、模型资产路径和许可证，不擅自下载/恢复被移除的资产。
5. 补齐上述静态检查与测试后，再进行隔离域 83 的只观察在线审计；不发运动指令。在线链路通过后才可将阶段 A 标为通过并开始阶段 B。

本报告是当前已知实现的审计记录，不是能力通过报告。任何修复后应追加新审计版本和新的代码/配置哈希，不覆盖本记录。

## 7. v2 执行进度追加（2026-11-19）

依据执行计划 v2，当前仅完成契约/启动边界的第一步，不代表阶段 A 通过：

- 新增 `openflex_roboos_bridge/target_observation.py`：定义 ROS-free `TargetObservation`，携带任务语义、视觉 track ID、RGB-D 坐标、有效深度比例、时间戳、TF、置信度和来源；拒绝无效 TF、零有效深度、非有限坐标及 Oracle/MuJoCo 来源。输出中明确 `mujoco_object_id=null`、`physical_verification=false`。
- 新增契约、搜索未知槽位用例、抓取输入校验用例和 MCP tool-surface 约束；阶段 A 相关回归最新为 `90 passed`。命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q scripts/tests/test_target_observation_contract.py scripts/tests/test_openflex_target_search.py scripts/tests/test_openflex_target_resolution_contract.py scripts/tests/test_openflex_retail_search.py scripts/tests/test_openflex_mcp_target_compat.py scripts/tests/test_openflex_tool_surface.py scripts/tests/test_openflex_delivery_contract.py scripts/tests/test_openflex_grasp_plan_contract.py`。禁用自动插件是因为系统 pytest 与用户目录 anyio 插件版本不兼容（插件导入报 `No module named '_pytest.scope'`）。
- `scripts/openflex_manipulation_preflight.launch.py` 现默认不启动 `object_memory` 和 `oracle_anygrasp`；仅显式 `OPENFLEX_ENABLE_ORACLE_GRASP=1` 才加入这两个对照组件。该启动文件已通过 `py_compile`，尚未启动 ROS 栈验证实际节点图。
- `git diff --check` 通过；新增契约文件和启动器语法编译通过。

进一步修复：`SearchSkill` 已不再读取 registry nominal slot；无 expected slot 的整帧解析会返回帧级视觉候选 token（当前字段名为 `perception_track_id`，尚不是跨帧 tracker）；并在当前阶段故意拒绝把候选交给仍要求仿真 ID 的抓取器，因此不会发运动命令。旧 slot-bound 解析仅保留给历史回归测试，不能作为未知槽位能力证据。

进一步修复：MCP/tool surface 和 `ManipulationSkill.submit` 已改为接收 `perception_track_id`；`grasp_plan_contract.validate_visual_grasp_input` 会拒绝字符串 `object_id`、无效 TF/深度和 Oracle 来源。SearchSkill 会把 RGB-D 候选保存为 `TargetObservation`，但当前执行器在视觉规划后端接入前明确返回失败且不发运动。

仍未完成且保持阻塞：`TargetObservation` 尚未接入真实抓取规划动作；启动器改动尚未做 ROS 运行时节点/订阅审计。故不进入阶段 B。原有 `openflex_ws/` 删除项保持原样。

## 8. 现有 RGB-D/GraspNet 组件可复用性复核（2026-11-19）

本次只读检查 `openflex_ws/src/openflex_grasp`，没有启动节点、下载模型或发出运动：

- `detect_object_server.py` 同步订阅左右腕部 RGB、深度、CameraInfo，调用 Qwen 定位和 SAM2 分割，再从同帧深度构造商品点云及场景点云。点云坐标属于相机帧，消息携带该帧与时间戳。
- `FitGrasp.action` 的输入是 `detection_id`、`arm_name`、策略及 `product_points`/`scene_points`；没有 MuJoCo `object_id` 字段。`graspnet_worker.py` 对这两份点云做 GraspNet 推理和抓姿拟合，再用 TF 变换抓取轴。
- 因此，GraspNet worker 的“抓姿候选拟合”边界可以消费视觉点云，是可复用算法候选；它不等同于当前毕业设计链已经调用了该 worker。
- 现有公共链仍有身份与执行耦合：`PickObject` 从 `GetDetectionSnapshot` 取快照后要求 `snapshot.object_id == goal.object_id`，其 Detect 入口也保留注册商品/站位语义；Pick 后续连接的是这套腕部相机、MoveIt 及控制器接口。毕业设计当前 `SearchSkill` 输出 `TargetObservation`，而 `ManipulationSkill` 的 MuJoCo 轨迹规划器仍依赖仿真物体记忆。因此不能直接把 `TargetObservation` 填成 `ObjectMemory`，也不能仅凭 FitGrasp 的点云接口就跳过身份、TF、规划场景、执行器和评测真值隔离的适配。
- 外部模型路径默认指向 `openflex_models` 下的 GraspNet checkpoint、Qwen、SAM2；静态代码/文档检查未验证这些资产在本机存在、许可证、CUDA 扩展或推理环境可用性。本阶段不下载或恢复资产。

结论：下一项应设计 `TargetObservation/RGB-D point cloud → FitGrasp` 的无 ROS 纯数据适配契约和仿真规划边界；FitGrasp 仅返回抓姿，不应负责执行。只有在视觉位姿能进入不读取 MuJoCo 目标真值的仿真 MoveIt 场景/规划链、且日志把感知输入和真值评测分开后，才可做在线只观察审计。阶段 A 仍未通过。

离线检查说明：本轮曾尝试把 `test_openflex_manipulation_submission.py` 加入 pytest 批次，但该环境虽有 `rclpy`，缺少构建生成的 `openflex_thesis_msgs` Python 模块，收集阶段报 `ModuleNotFoundError`；该测试未运行，不将其计入通过数。应使用前述 ROS-free 契约测试批次作本阶段有效回归。

## 9. ROS-free 点云适配契约（2026-11-19）

新增 `openflex_roboos_bridge/visual_grasp_input.py` 的 `VisualGraspPointCloudInput`，作为视觉观测进入未来 FitGrasp transport adapter 前的唯一数据边界：

- 只接受已经通过 `TargetObservation.validate_for_grasp()` 的视觉观测；保留 `perception_track_id`，没有 `object_id` 字段。
- 商品点云和场景点云必须是同一相机帧、同一时间戳，XYZ 有限且每份默认至少 100 点；带颜色的商品点云会在边界处裁成 XYZ，不改变来源。
- `to_fit_grasp_payload()` 只返回 ROS-free 字典和 NumPy XYZ 数组，明确 `oracle_input=false`、`physical_verification=false`；它不是 `FitGrasp.Goal`，不会发 action 或运动。
- 新增 `test_visual_grasp_input_contract.py`，覆盖身份隔离、帧/时间戳一致性、点数和非有限值拒绝。

这一步固定了“感知点云可以被抓姿后端消费”的接口，但仍没有解决相机/TF 配置、GraspNet 模型资产、仿真 MoveIt 场景碰撞、执行器身份和评测真值分离。因此阶段 A 继续保持未通过。

继续静态复核发现并修复一处数据契约断点：`SearchSkill` 使用头部相机观察，却没有给 `TargetObservation` 标出之后负责抓取的手臂，无法满足 FitGrasp 的左右臂参数。现在候选转观测 helper 接受可选 `arm_name`，当前任务显式填 `right`；新增端到端 ROS-free 测试确认搜索候选观测能通过点云抓姿输入契约。这里的 `right` 是任务执行配置，不是从视觉推断出的物体信息。

MuJoCo bridge 静态发布 `/thesis_sim/head/{color/image_raw,aligned_depth_to_color/image_raw,color/camera_info}`，三者采用 `thesis_head_color_optical_frame` 和同一仿真时间戳；并静态发布 `map→thesis_head_color_optical_frame`。`camera_spec.py` 明确相机位姿来源是 `simulation_mesh_estimated_not_physically_calibrated`。这说明静态接口形状可满足 RGB-D 回投影和坐标变换输入，但不构成 ROS 运行时新鲜度/TF 查询验证，也不等同于真实手眼标定。当前抓姿适配将继续限定为右臂和头部相机候选；在线规划前必须验证该 frame 到执行器规划坐标系的 TF、时间一致性和碰撞场景，不能把 MuJoCo 商品真值用于规划。

本轮回归：阶段 A ROS-free 相关测试 `97 passed`；新增搜索观测→点云抓姿契约贯通用例。Python 语法编译和 `git diff --check` 通过；没有运行 ROS 或仿真。

## 10. 感知证据/评测真值分层（2026-11-19）

新增 ROS-free `evidence_schema.py`，并接入 `SearchSkill.save_evidence()`：

- 每次尝试分别生成 `attemptNN_perception.json` 和 `attemptNN_evaluation.json`，两者由 `evidence_layer` 明确区分，并从原分辨率记录引用 RGB、深度和帧哈希。
- 感知记录明确 `physical_verification=false`，递归拒绝非空的 `mujoco_object_id`、`object_id`、`selected_object_id`、`truth_object_id`、Oracle/MuJoCo ID 字段；其中显式 null 可用于表达“无真值关联”。
- 评测记录默认 `mujoco_object_id=null`、`physical_verification=false`；只有评测侧显式给出仿真身份后才允许标为物理验证，感知记录本身永远不接受该身份。
- operation journal 中的 observation 仍可包含状态和证据引用；每条事件载荷含有可独立解析的感知记录，而两类原始 JSON 文件分别持久化。该 schema 尚未接入物理评测器的真值关联流程。

阶段 A 本地 ROS-free 回归现为 `100 passed`；Python 语法编译和 `git diff --check` 通过。本轮没有启动 ROS/仿真或发运动。此项完成日志契约，不代表已有在线运行批次或评测器隔离已经运行时验证。

## 11. TC-WAM、AnyGrasp 与 MoveIt 目标接口复核（2026-11-19）

本轮只读检查 Action 定义、gateway、Oracle、TC-WAM server/evaluator、MoveIt executor 和规划探针；没有启动 ROS/仿真，也没有发出运动。当前仓库 HEAD 为 `acc7d26`（`Separate perception and evaluation evidence`）；此前第 1 节中的 `2a0e9db` 是该次初审的历史基线，不是当前 HEAD。工作区仅保留原有 `openflex_ws/` 删除项。

### 11.1 现有接口为什么不能直接接视觉观测

- `GenerateGrasps.action` 的 `target` 类型是 `ObjectMemory`。它必须携带 `object_id`、带坐标的 pose、`map_version`、可见/授权状态；请求虽有 `require_physical_perception`，但该布尔值只作为生成策略提示/结果验收门，不能把视觉点云变成该 Action 的目标输入。
- `AnyGraspGatewayNode` 将完整 `goal.target` 原样复制到 `AnyGraspRequest`，再以 `object_id + map_version + stage` 关联响应。Gateway 本身没有 RGB-D、点云、检测 ID 或视觉位姿转换逻辑。
- 仓库中的 `OracleAnyGraspNode` 直接读取 `request.target.pose.pose` 构造规则化候选，并明确写 `physical_perception=false`、`source=mujoco_ground_truth_oracle_NOT_PHYSICAL`。所以当前 thesis Action 链上由 Oracle 回答时，候选是由目标真值位姿构造，而非视觉推理。
- `ComputeTcWam.action` 同样以 `ObjectMemory target` 加 `GraspCandidateArray` 为输入。TC-WAM server/evaluator 以 `object_id/map_version` 检查同一目标，并从该 memory 的 pose、可见性和抓取授权状态建立目标状态、进行稳健性扰动和基座/手臂候选评估。它是候选/可达性规划链，不是视觉目标适配器。
- `ExecuteMoveItGrasp.action` 及 `moveit_grasp_executor_node.py` 继续要求 target、粗规划和精化候选的 object ID/map version 一致；执行器还订阅 `/thesis/object_memory`，以该记忆身份/位姿做新鲜度确认、抓取验证和附着身份管理。货架碰撞物则可从独立 `shelf_scene_file` 静态载入并应用到 MoveIt PlanningScene。这意味着“静态货架几何”与“仿真目标身份/目标位姿”是两类依赖，前者有可复用入口，后者仍被现有执行器硬编码为 `ObjectMemory`。
- `scripts/probe_openflex_grasp_planning.py` 明确订阅 `/thesis/object_memory`，从最新 `thesis_item_01` memory 调 GenerateGrasps 和 ComputeTcWam，并设置 `require_physical_perception=False`。该探针只验证仿真真值候选规划/碰撞检查，不是视觉输入链路。

### 11.2 可复用部分与必须隔离/改造部分

可复用：现有 RGB-D 检测/分割产出的商品点云和场景点云；`FitGrasp.action` 的 `product_points + scene_points + arm_name` 输入形态；`graspnet_worker` 的点云抓姿候选拟合；MoveIt executor 中货架静态碰撞场景加载、当前机器人状态、IK/碰撞校验、轨迹执行及夹爪控制的实现思路。

不能直接复用为纯视觉链：以 `ObjectMemory` 为目标的 `GenerateGrasps → ComputeTcWam → ExecuteMoveItGrasp` 目标身份闭环；依赖 `object_id/map_version` 的响应关联；从 MuJoCo memory pose 创建候选、扰动目标或授权附着的路径。也不能把 `TargetObservation` 伪造为 `ObjectMemory`，或把 `FitGrasp` 的抓姿结果直接当作已通过全身碰撞检查的 MoveIt 轨迹。

下一步接口边界建议（设计建议，不是已实现能力）：

1. 纯视觉目标包以 `TargetObservation`/视觉 track ID、RGB-D 点云、时间戳、相机 frame 和已验证 TF 为唯一输入，调用 `FitGrasp` 或其明确标注的视觉抓姿 backend；返回候选抓姿及其 frame/stamp/track ID，不含仿真 object ID。
2. 视觉规划适配器把候选抓姿转换到 MoveIt planning frame，并使用独立目标碰撞几何（来自分割点云的保守包围体或经验证的目标模型）与静态货架场景联合检查；规划输入只含机器人实测状态、候选位姿和碰撞场景，不订阅 `/thesis/object_memory`。
3. 执行身份使用 perception track/request ID 与候选序号；仿真桥仅在执行后通过独立评测通道把接触、保留/附着、释放结果关联到仿真对象真值。评测 ID 不回流到候选生成、规划或在线目标选择。
4. 若现有 executor 无法在不读取 ObjectMemory 的条件下提供独立抓取验证/附着状态，则新建视觉授权的执行边界或先抽离运动/场景原语；不得通过伪造 memory 绕过 fail-closed 检查。

此接口仍需后续实现并补 ROS-free 伪造真值注入测试、场景碰撞集成测试和 ROS 运行时订阅图核验。阶段 A 的“视觉点云接入 MuJoCo 相机/MoveIt/评测”条目及阶段门槛均未完成；阶段 B 仍不得开始。
