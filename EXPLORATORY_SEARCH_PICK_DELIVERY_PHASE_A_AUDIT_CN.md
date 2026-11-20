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
- (✓) 在线运行时复核节点、话题、输入来源及相机帧（隔离域 83，只读 MuJoCo 桥；见第 18 节）。纯视觉抓姿 Skill 链本身未启动，在线抓取输入仍 fail-closed。

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

## 12. 视觉抓姿→MoveIt 规划请求的 ROS-free 契约（2026-11-19）

针对第 11 节给出的接口边界建议 1–2，本轮新增纯数据契约 `openflex_roboos_bridge/visual_grasp_planning.py`，把“视觉点云抓姿候选”与“仿真真值规划链”在数据层显式分开。本轮只读/离线实现与测试，没有启动 ROS/仿真，也没有发出运动。

- `FrameTransform`：以四元数/平移表示相机光学帧在规划帧中的位姿；拒绝非有限值和非单位四元数，提供点变换。
- `Aabb`：单一坐标系下的轴对齐包围盒；拒绝反向或退化（`max<=min`）盒。`aabb_from_pointcloud()` 从分割商品点云（相机帧）取保守包围盒并可加 padding，`transform_aabb()` 将 8 个角点变换后重取包围盒，得到规划帧下的独立目标碰撞体。
- `GraspCandidatePose`：拟合得到的抓姿，只有 `candidate_id`、相机帧位置/姿态、`score`、`required_width_m`；`candidate_id` 若形如 `mujoco*`/`thesis_item*` 或含 `object_id` 直接拒绝。
- `VisualGraspPlanRequest`：由 `VisualGraspPointCloudInput`、规划帧、相机→规划 `FrameTransform`、静态货架障碍 AABB 列表、候选抓姿列表和机器人实测关节状态构造。`to_planning_payload()` 输出仅含 `robot_state`、规划帧候选位姿、`collision_scene{target_aabb, static_obstacles}`，并显式写入 `oracle_input=false`、`object_memory_used=false`、`physical_verification=false`；序列化前有递归守卫，任何非空仿真身份键（`object_id`/`mujoco_object_id`/…）都会报错。

失败即关闭（fail-closed）覆盖：空静态障碍、空候选、`camera_frame` 与观测不一致、Oracle/MuJoCo 来源观测、非单位四元数、退化 AABB、关节名/位置长度不一致、候选 ID 携带仿真身份、传入字符串 `object_id` 而非视觉输入。新增 `scripts/tests/test_visual_grasp_planning.py`（13 例）验证相机→规划帧位姿变换、包围盒保守性、目标碰撞体的相对/旋转两种情形，以及上述拒绝路径和“工造真值注入”用例。

阶段 A ROS-free 相关回归最新为 `126 passed`；`py_compile` 与 `git diff --check` 通过。

仍未完成、因此阶段 A 与计划第 56 行条目保持未通过：

- 该契约尚未接到真实 MuJoCo 头部相机 TF/时间戳运行时查询，也未验证货架障碍 AABB 与实际 `shelf_scene` 的运行时一致性。
- 未实现 ROS action transport 到 MoveIt、未载入 GraspNet 模型资产，也未做节点/订阅图运行时审计；`ManipulationSkill.execute()` 仍对视觉后端 fail-closed，不发运动。
- 评测真值仅在执行后由独立通道关联这一环尚未实现；本契约只保证规划输入不含仿真身份，不代表闭环已接通。

## 13. 静态货架场景与头部相机 TF 的 ROS-free 绑定（2026-11-19）

第 12 节的 `VisualGraspPlanRequest` 只接受“已经给定的规划帧障碍与相机→规划变换”。本轮新增 `openflex_roboos_bridge/visual_grasp_scene.py`，用纯数据方式把仓库现有静态货架场景与 MuJoCo 头部相机 TF 接到该契约，仍不启动 ROS/仿真、不发运动。

- `load_shelf_scene()` 解析货架场景 YAML（路径或映射）：要求非空 `frame_id`、非空 `collision_objects`、每个对象 `shape=box`、尺寸为正、位置/姿态有限、`id` 唯一；把每个带姿态的盒子按 8 角点变换取保守 AABB，得到规划帧下的独立障碍列表。`calibrated` 仅作为来源信息记录（与预检启动器 `require_calibrated_scene=False` 的仿真规划口径一致），不表态为物理标定。
- `frame_transform_from_tf()` 从实测 TF 读数构造相机→规划 `FrameTransform`，并强制 `parent_frame==planning_frame`、`child_frame==观测相机帧`，对应 MuJoCo bridge 的 `map→thesis_head_color_optical_frame` 广播；非单位四元数/非有限值拒绝。
- `build_visual_grasp_plan_request()` 把视觉点云输入、实测 TF、货架场景和机器人关节状态组合成 `VisualGraspPlanRequest`，并强制 `场景 frame == 规划帧 == 观测 map_frame`（镜像 `moveit_evaluator_node.shelf_objects_in_candidate_base` 的“scene frame 必须等于目标帧”检查）。
- 新增 `scripts/tests/test_visual_grasp_scene.py`（8 例）：直接读取仓库跟踪的 `openflex_thesis_mujoco/config/sim_shelf_scene.yaml`，核对帧名/标定标志/6 个障碍与后挡板保守 AABB；验证旋转盒保守性、畸形场景拒绝、TF 帧角色与四元数拒绝、端到端构建请求、场景帧/规划帧/`object_id` 拒绝路径。

静态事实澄清：头部相机在 `map` 中的位姿由 `bridge_node._mujoco_body_pose` 在运行时逐帧发布（`camera_spec.HEAD_CAMERA` 只是相对 `head_yaw_link` 的挂载偏移），所以该变换确实是运行时读数，不能离线写死；本模块只定义“给定一次 TF 读数如何安全绑定”，不替代运行时新鲜度/TF 查询。

阶段 A ROS-free 相关回归最新为 `134 passed`；`py_compile` 与 `git diff --check` 通过。

仍未完成，阶段 A 与计划第 56 行条目继续未通过：未查询运行时 TF/时间戳新鲜度、未把请求接到 MoveIt action transport、未载入 GraspNet 资产、未做节点/订阅图运行时审计，评测真值仍只在执行后由独立通道关联。`ManipulationSkill.execute()` 保持对视觉后端 fail-closed。

## 14. 视觉点云↔FitGrasp 传输适配契约（2026-11-19）

第 12/13 节分别固定了“点云→规划请求”与“场景/TF 绑定”，中间的抓姿拟合环节仍无 ROS-free 契约。本轮新增 `openflex_roboos_bridge/visual_grasp_fit_adapter.py`，把“视觉点云→FitGrasp→抓姿候选”闭合为纯数据链路：`TargetObservation → VisualGraspPointCloudInput → FitGraspRequestPayload →（未来 FitGrasp server / GraspNet worker）→ FitGraspResult → GraspCandidatePose → VisualGraspPlanRequest`。不调用 ROS action、不载入模型、不发运动。

- `FitGraspRequestPayload.from_visual_input()`：从视觉输入生成 `FitGrasp.Goal` 的 ROS-free 类比（`detection_id=perception_track_id`、`arm_name`、`grasp_strategy`、相机帧/时间戳、商品与场景 XYZ、`timeout_sec`），并标 `oracle_input=false`；拒绝字符串 `object_id`、空策略或非法超时。
- `FitGraspResult`：`FitGrasp.action` 结果的 ROS-free 类比（成功标志、帧/时间戳、抓姿位置/四元数、`required_width_m`、`score`、候选计数、可选预抓姿、来源）。
- `grasp_candidates_from_result()`：结果必须成功、来源非 Oracle/MuJoCo、帧与时间戳与点云一致、候选数 ≥1、宽度/得分有限、四元数为单位四元数；通过后产出相机帧 `GraspCandidatePose`。

新增 `scripts/tests/test_visual_grasp_fit_adapter.py`（7 例）：请求载荷不含仿真身份、结果转候选、失败/空结果/帧/时间戳/Oracle/四元数/`object_id` 拒绝，以及“点云→FitGrasp 载荷→结果→候选→场景约束规划请求”的端到端 ROS-free 贯通。

阶段 A ROS-free 相关回归最新为 `141 passed`；`py_compile` 与 `git diff --check` 通过。

仍未完成，阶段 A 与计划第 56 行条目继续未通过：未接真实 GraspNet 模型资产与 `FitGrasp` ROS action、未做运行时 TF/时间戳新鲜度与订阅图审计、未接 MoveIt action transport、评测真值仍只在执行后由独立通道关联。`ManipulationSkill.execute()` 保持对视觉后端 fail-closed。

## 15. 抓取/感知模型资产与许可证静态核查（2026-11-19）

对应执行计划第 6 节"明确实际抓取模型可用性"。本轮只读核查依赖清单、资产路径与许可证，不下载/恢复任何资产，不启动 ROS/仿真，不导入 torch/transformers。新增只读脚本 `scripts/audit_openflex_model_assets.py` 与测试 `scripts/tests/test_openflex_model_assets.py`（6 例，含"资产缺失即跳过"的真实环境集成用例）。

资产事实（本机 `/home/wgb/openflex_shop/openflex_models`，`.gitignore` 第 21 行忽略 `/openflex_models/`，未纳入 git）：

- 四个必需文件全部在位：`Qwen3.5-4B/config.json`、`sam2.1-hiera-small/config.json`、`graspnet-baseline/checkpoint-rs.tar`、`SHA256SUMS`；与 `openflex_grasp/.../backend_health.REQUIRED_MODEL_FILES` 一致。
- `SHA256SUMS` 共 27 条，逐文件 sha256 校验 27/27 通过（0 不匹配、0 缺失）。即本机资产与清单完整一致；此前第 8 节"未验证本机存在"的表述现更新为"存在且哈希一致"。
- 清单路径不可移植：条目使用另一台机器的绝对前缀 `/home/openflex/old/openflex_models/`（`path_portable=false`）；`backend_health.validate_model_root` 仅查存在性、不校验哈希，故哈希校验目前只有本审计脚本执行。
- 无 AnyGrasp SDK/权重残留（`anygrasp_assets_present=[]`）；仓库仅存 AnyGrasp 的迁移记录文档和文献 html，与第 1.2 节"不恢复 AnyGrasp"一致。

许可证（脚本内 `LICENSE_TABLE` 记录，供论文引用核对）：

- `Qwen3.5-4B`：Apache-2.0（目录内含 LICENSE，README 元数据 license: apache-2.0）。
- `sam2.1-hiera-small`：README 元数据 license: apache-2.0，但目录内未随附独立 LICENSE 文件（来源为 Meta segment-anything-2，Apache-2.0）。
- `graspnet-baseline`（代码与 checklist 权重）：SJTU "Academic or Non-Profit Organization Noncommercial Research Use Only" —— 非商业研究用途；论文化使用与再分发需遵守。
- ROS 包：`openflex_grasp`/`openflex_grasp_interfaces`/`openflex_grasp_supermarket_scene_adapter` 为 Apache-2.0；`openflex_grasp_moveit` 为 CC-BY-NC-SA-4.0（非商业）。

推理环境事实（只读探测，未运行模型）：本机 Python 3.10.12，`torch 2.12.0+cu130` 且 `cuda_available=True`；`transformers` **不可导入**（感知 worker 依赖未装）；`graspnet-baseline` 下无编译好的 `pointnet2`/`knn` CUDA `.so`。依赖清单 `requirements-graspnet.lock.txt`/`requirements-perception.lock.txt` 固定 `torch==2.7.1+cu126` 与 nvcc 12.6，与本机 2.12.0+cu130 不一致。即：**资产在位且哈希可信，但感知/GraspNet 推理运行时在本环境尚未就绪**，本任务不安装/不下载。

结论：资产可用性的"存在性 + 完整性 + 许可证"已确认，但"CUDA 扩展/推理运行时可用"未满足，因此本阶段仍不能宣称视觉抓取后端可运行；阶段 A 与计划第 56 行条目继续不勾选。

> 更新（2026-11-19）：经用户明确授权后，本机已完成 GraspNet 推理依赖校验与 CUDA 扩展重建，"推理运行时可用"结论见第 17 节；本节记录的"未就绪/不安装"是当轮事实。

## 16. 视觉抓姿规划链的证据分层接入（2026-11-19）

第 12–14 节把纯视觉抓姿规划链打通为不含仿真身份的数据链路，但该链尚未把结果写进第 10 节的"感知/评测分离"证据层。本轮新增 `openflex_roboos_bridge/visual_grasp_evidence.py`，把 `VisualGraspPlanRequest` 接入该 schema，仍不启动 ROS/仿真、不发运动。

- `planning_perception_evidence()`：把规划请求载荷包进 `PerceptionEvidence.record`（`boundary=visual_grasp_planning`、`perception_track_id`、`task_target`、`planning_frame`、`candidate_count`、`visual_grasp_plan`），并即时调用 `to_dict()` 触发 schema 校验；构造即失败，不延迟到写出。
- `planning_evaluation_evidence()`：默认生成 `mujoco_object_id=null`、`physical_verification=false` 的评测占位；仅当评测侧显式给出仿真身份才允许标 `physical_verification=true`。
- `planning_evidence_pair()`：返回 `perception`/`evaluation` 两条记录；任一非法（如载荷被注入非空 `object_id`）立即抛 `VisualGraspEvidenceError`。

新增 `scripts/tests/test_visual_grasp_evidence.py`（4 例）：感知/评测层分离、载荷注入仿真身份被拒、评测层缺身份不得标物理验证、缺 `VisualGraspPlanRequest`/帧引用被拒；并确认证据可 JSON 序列化。

阶段 A ROS-free 相关回归最新为 `151 passed`；`py_compile` 与 `git diff --check` 通过。

仍未完成，阶段 A 与计划第 56 行条目继续未通过：这是 ROS-free 证据契约，不代表已有在线运行批次；运行时 TF/时间戳新鲜度、节点/订阅图审计、MoveIt transport、GraspNet 推理运行时（见第 15 节）与执行后真值关联均未完成。`ManipulationSkill.execute()` 保持对视觉后端 fail-closed。

## 17. GraspNet 推理依赖与 CUDA 扩展就绪核查（2026-11-19）

经用户明确授权后，本轮在本机完成迁移 venv 的抓取/感知推理依赖校验与 CUDA 扩展重建。改动只落在仓库外的环境目录（`/home/wgb/.cuda/`）与已被 gitignore 的 `.venvs/`，不触碰任何跟踪源码或资产，不启动 ROS/仿真、不发运动。

### 17.1 迁移 venv 修复与运行时事实

- 迁移 venv `.venvs/openflex_graspnet`、`.venvs/openflex_perception` 的 `bin/*` 脚本 shebang 仍指向旧部署根 `/home/openflex/openflex_shop/...`，在本机不可执行。已用脚本把 56 个脚本（graspnet 32、perception 24）的 shebang 改指 `/home/wgb/openflex_shop/...`，其余字节不变；`pip --version` 现可正常工作。
- `.venvs/` 已被 `.gitignore` 忽略，不进入提交；`pyvenv.cfg` 保持 `home=/usr/bin`、`version=3.10.12`。
- 运行时事实：graspnet venv `torch 2.7.1+cu126`（`cuda_available=True`，设备算力 `(8,9)`）、`numpy 1.26.4`、`scipy 1.15.3`、`open3d 0.19.0`；perception venv `torch 2.7.1+cu126`、`transformers 5.15.0`。

### 17.2 本机 CUDA 12.6 工具链与扩展重建

- 系统无 nvcc/CUDA；`nvidia-cuda-nvcc-cu12` pip wheel 不含 `nvcc`。本轮从 NVIDIA 官方 apt 仓库本地解包 `cuda-nvcc-12-6`/`cuda-nvvm-12-6`/`cuda-cudart-dev-12-6`/`cuda-cudart-12-6`/`cuda-cccl-12-6` 到仓库外 `/home/wgb/.cuda/cuda-12.6-local/usr/local/cuda-12.6`（`CUDA_HOME`），合并 pip 头文件并软链 `libcudart.so`；`nvcc --version` = 12.6.85，最小 CUDA 程序 `-arch=sm_86` 编译+运行通过。
- 按 `COMPATIBILITY_MATRIX_CN.md` 要求在目标环境重编译（禁止复制 `.so`）：`TORCH_CUDA_ARCH_LIST="8.6;8.9"`、`pip wheel . --no-build-isolation --no-deps` 重建 `pointnet2` 与 `knn_pytorch`，卸载旧 sm_86 egg（备份到 `/home/wgb/.cuda/backup_eggs/`）后装入新 wheel。新 `.so` 位于 venv `site-packages`，引用本机 `/home/wgb/.cuda/...`，确认是本机产物。

### 17.3 可复现校验脚本与结果

- 新增只读脚本 `scripts/audit_openflex_graspnet_runtime.py`（+ `scripts/tests/test_openflex_graspnet_runtime.py`，10 例）：用系统 Python 通过 subprocess 调用 venv 解释器做探针，绝不在系统解释器里 import torch；venv 缺失即 `skipped`，探针内强制 `import torch` 先于扩展。
- 检查覆盖：venv `bin` shebang 无残留外部根；`pointnet2._ext` + `knn_pytorch` 加载并在本机 GPU 上跑通 fps/gather/ball_query/group_points/cylinder_query/three_nn/three_interpolate/knn；`GraspNetBackend` 加载 `checkpoint-rs.tar` 并在仓库自带 `doc/example_data` 上推理；perception venv 导入 torch/transformers。
- 实跑结果：`cuda_ops` 全 PASS；`checkpoint_inference` 用 513,688 点得 `(326,17)` 全有限，score 0.0144..1.3318（与迁移记录 P4-09 一致）；`perception_import` PASS；`overall_ok=true`。脚本不写文件、不起 ROS/仿真、不发运动。

### 17.4 与阶段 A 门槛的关系

- 计划第 6 节"明确实际抓取模型可用性"的**推理运行时可用性**本轮已满足：资产哈希/许可（第 15 节）+ CUDA 扩展重建 + checkpoint 推理 PASS + 感知 venv 可导入。环境产物（`.venvs/`、`/home/wgb/.cuda/`、`openflex_models/`）均在仓库外或已 gitignore，本轮提交只含脚本、测试与本文档。
- 阶段 A **仍未通过**：计划书阶段 A 的另一条门槛仍是隔离域 83 下的只读运行时审计（节点/话题/TF 新鲜度/订阅图），本轮未启动 ROS/仿真、未做该审计、未发运动。视觉抓姿链的在线 transport（`FitGrasp` ROS action、MoveIt transport）与执行后真值关联也仍未接。

## 18. 隔离域 83 只读运行时审计（2026-11-19）

经用户授权，本轮首次实际启动 ROS 2（Humble，`ROS_DOMAIN_ID=83`）做只观察在线审计：只运行仓库既有 `scripts/run_openflex_readonly_sim.sh`（仅 MuJoCo 桥节点，无 mission/Nav2/MoveIt/Oracle/自动客户端），不发任何运动指令，审计后立即关停并确认图为空。环境使用仓库既有 `openflex_ws/install_v2`。

### 18.1 观测到的运行时图

- 节点（启动后）：`/openflex_thesis_mujoco_bridge`、`/mujoco_control_mode_receiver`。无 Oracle、AnyGrasp gateway、MoveIt、Nav2 或 mission 节点/进程（`ps` 复核一致）。
- 真值/运动入口被切断：`/thesis/object_memory` 话题在图中不存在；桥节点改为订阅 `/openflex_readonly_disabled/object_memory`，其发布者数为 0。`cmd_vel`、`place_control` 同样重映射到 `openflex_readonly_disabled/*`（发布者 0）；`reset_nominal`/`randomize`/`drop_object` 服务重映射为 `openflex_readonly_disabled/*`。
- 相机与 TF：`/thesis_sim/head/color/image_raw` 约 4 Hz；`/tf` 含 `map→odom→base_link` 及 `map→thesis_head_color_optical_frame`（平移 `[0.253, 0, 1.312]`，四元数 `[-0.5, 0.5, -0.5, 0.5]`），与第 13 节“头部相机在 `map` 的位姿是运行时读数”一致。
- 安全审计：桥节点在约 30 s（约 15005 次采样）内持续输出 `MUJOCO_SAFETY_AUDIT ... "violation": false`。
- 关停后复核：域 83 节点列表与话题列表（除 `parameter_events`/`rosout`）均为空，无残留仿真进程。

### 18.2 新发现并已修复：只读 profile 的运动 action 重映射在 Humble 下失效

- 首次捕获发现：`-r /right_joint_trajectory_controller/follow_joint_trajectory:=/openflex_readonly_disabled/right_arm` 等规则未生效——隐藏 `_action/*` 话题仍是 `/right_joint_trajectory_controller/follow_joint_trajectory/_action/status` 等规范名。用隔离最小 rclpy `ActionServer` 实验复现：`-r probe_action:=remapped_action` 后 `ros2 action list` 仍显示 `/probe_action`。原因是 ROS 2 中 action 的图名是隐藏的 `<action>/_action/{feedback,status,send_goal,cancel_goal,get_result}` 话题/服务，精确名重映射规则不匹配 action 名本身。
- 影响：只读 profile 下臂/升降/夹爪 action 仍以规范名对外暴露，"禁用"只由“不启动客户端”保证，而非由重映射强制。
- 修复（`scripts/run_openflex_readonly_sim.sh`）：把每个待禁用 action 展开为其 5 个 `_action/*` 隐藏名再重映射（enabled 路径为 identity 时跳过）。隔离实验确认重映射隐藏名可真正改名。修复后复跑只读仿真：5 个运动 action 变为 `/openflex_readonly_disabled/{lift,right_arm,right_gripper,left_arm,left_gripper}`，规范名 action 不存在；其余图事实不变。

### 18.3 结论与仍未覆盖的部分

- 已核对：纯只读桥 profile 可启动；Oracle 不在进程/订阅图中；真值/运动通道在图上无发布者；头部相机与 `map→head` TF 在线可用；运动 action 现已真正改名隔离。计划阶段 A"只读运行时审计（节点/话题/输入来源/相机帧）"条目录入证据。
- 仍未覆盖（阶段 A 不能因此标为通过）：本轮未启动视觉抓姿 Skill 链、`FitGrasp` ROS action、MoveIt transport 与评测真值关联，故"将现有视觉点云抓姿链适配到 MuJoCo 相机/场景坐标/TF/仿真 MoveIt/评测接口"条目仍未完成；桥节点发布的 `/thesis/object_observation` 是仿真真值派生观测（noise=0，confidence=0.99），只作为仿真观测口，不是物理感知证据。

## 19. 视觉抓姿链接入本任务 MuJoCo 相机与仿真 MoveIt 规划（只规划，2026-11-20）

第 11–16 节把视觉抓姿链闭合为 ROS-free 契约，第 18 节确认了只读运行时可启动。本轮把该链真正接到本任务的在线接口：MuJoCo 头部相机、`map`→头部光学的实测 TF、货架静态碰撞场景、仿真 MoveIt 规划服务与感知/评测证据层。全部只规划，不执行运动。

### 19.1 新增传输适配与只读探针

- 新增 ROS-free `openflex_roboos_bridge/visual_grasp_moveit.py`：把 `VisualGraspPlanRequest` 转成 MoveIt 规划接口所需的两类载荷——(1) 规划帧下的世界碰撞体（独立目标保守体积 + 货架障碍盒）；(2) 每个视觉候选一个“实测关节位置”机器人状态。模块把 `plan_only` 固定为真、`trajectory_execution` 固定为假，并拒绝 `allow_trajectory_execution`/`moveit_manage_controllers` 为真的 MoveIt 参数集；不读 `ObjectMemory`、不自调 MoveIt、不发运动。
- 新增只读探针 `scripts/probe_openflex_visual_grasp_moveit.py`（需 `ROS_DOMAIN_ID=83`）：读实时头部相机信息/深度、`/joint_states`、`/swerve_drive_controller/odom` 与 `map→thesis_head_color_optical_frame` TF；把深度反投影为目标点云并走第 12–16 节 ROS-free 链；再经 `/apply_planning_scene` 应用视觉碰撞场景、`/get_planning_scene` 回读、`/check_state_validity` 检查实测起始状态；最后写 `planning_evidence_pair()` 的感知/评测两份记录。探针不调用 `/execute_kinematic_path`、不发控制器目标、不授权抓取。
- 新增 `scripts/tests/test_visual_grasp_moveit.py`（9 例）：碰撞体中心/尺寸换算、目标与货架盒 ID、候选状态与实测关节一致、任意载荷不含 `object_id`/`mujoco`/`oracle`、plan-only 参数守卫（拒绝允许执行）、非单位四元数与跨帧拒绝。

### 19.2 在线只读运行证据（隔离域 83，两次复现）

- 组合栈：仓库既有 `scripts/run_openflex_readonly_sim.sh`（MuJoCo 桥，真值/运动通道重映射为 `openflex_readonly_disabled/*`）+ `scripts/run_openflex_manipulation_preflight.sh`（`move_group` 以 `allow_trajectory_execution:=false`、`moveit_manage_controllers:=false` 只规划；另含 `anygrasp_gateway`、`moveit_evaluator`、`tc_wam_server`、`robot_state_publisher`）。
- 进程/图事实：无 oracle 节点；`/thesis/object_memory` 话题不存在（仅有 `/openflex_readonly_disabled/object_memory`，发布者 0、订阅者 1）；`/thesis/anygrasp/request` 发布者 1（`anygrasp_gateway`）、订阅者 0。另观察到图中存在两个同名 `/move_group` 节点（MoveIt 自身），记录备查。
- 探针结果（两次一致）：对 320×240、`16UC1`(mm)、`fx≈171.4` 的实时头部深度反投影并按货架层窗口筛选得到 13687 个目标点，目标保守体积中心落在 `map` 约 `(1.169, 0.002, 0.891)`；经 `/apply_planning_scene` 应用 1 个目标体 + 6 个货架体共 7 个碰撞体，`/get_planning_scene` 回读 7 个 ID 全部匹配；`/check_state_validity` 对实测起始状态返回 `valid=true`、接触数 0；写出感知/评测证据对；`trajectory_execution=false`、`robot_motion_executed=false`、`object_memory_used=false`。
- 关停后域 83 图（除 `parameter_events`/`rosout`）为空，无残留仿真进程。

### 19.3 结论与仍未覆盖的部分

- 结论：计划阶段 A“将现有视觉点云抓姿链适配到 MuJoCo 相机、场景坐标/TF、仿真 MoveIt 规划和评测接口”条目已有在线只读证据；阶段 A 的门槛（在线输入无真值泄漏、纯视觉 profile 可启动、Oracle 不在进程/订阅图中）在在线运行层面得到支持。
- 仍未覆盖（属阶段 B/D，不在本轮范围）：尚未有分割/GraspNet 感知前端（本轮用深度窗口 + 由观测质心构造的夹具候选位姿代替，非训练模型输出）；候选末端状态尚未经 IK 校验；未做连续路径与附着物间隙规划；未执行任何轨迹。目标体积因窗口包含货架层板而偏保守（y 向约 1.35 m）。`ManipulationSkill.execute()` 仍对视觉后端 fail-closed，且未启动。

## 20. 阶段 B：单视点 RGB-D 红色商品搜索（2026-11-20）

阶段 B 的目标是把一帧实拍（仿真）头部 RGB-D 变成“可抓取或具名拒绝”的视觉目标，并证明五类条件都有可复现证据、搜索授权不读槽位坐标。图像派生候选与 fail-closed 解析原本已在 `target_search_contract.py`；本轮补齐整帧无槽位授权、`TargetObservation` 输出、深度有效率与随机位置回归，并在隔离域 83 跑通在线审计。

### 20.1 契约层新增

- `depth_valid_fraction(depth_mm)`：帧内可用深度返回占比，写入 `FrameAnalysis.model_output`，用于“深度有效率”判定。
- `observation_from_resolution(...)` + `TargetSearchError`：只有 `authorized`（唯一可见、有深度、有 `map` 位姿、具视觉 track id）的解析才能变成 `TargetObservation`；其余原因（`ambiguous_candidates`/`wrong_class`/`target_occluded`/`empty_slot`/`target_absent`/`below_confidence`）一律抛错。观测只带视觉 `perception_track_id`，不带仿真身份，并显式记录 `slot_coordinates_used`/`mujoco_object_id_used=false`。
- 收紧颜色门限：新增 `DEFAULT_MIN_SATURATION=0.5`。原先货架暗部（RGB 均值约 (85,106,144)、饱和比约 0.47）被误判为“蓝色物体”，整帧模式下会让移除目标后报 `wrong_class`；收紧后该面不再成为候选，与模块“中性/暗面不参与候选”的设计一致（真实红色物体饱和比约 0.83，安全通过）。
- 新增 `scripts/tests/test_openflex_search_observation.py`（11 例）：深度有效率、授权观测、五类拒绝都不产出观测、无效深度不授权、以及 24 次随机位置回归（图像内随机位置、随机深度，断言定位误差 < 3 px 且搜索不使用槽位先验）。

### 20.2 在线只读审计（隔离域 83）

- `scripts/audit_openflex_target_search.py` 改为双模式，并在真实头部帧上运行：
  - **整帧授权模式（`full_frame_search`）**：完全不提供注视先验，只按图像候选授权。覆盖 `full_visible`（授权）、`full_wrong_class`（`wrong_class`）、`full_ambiguous`（`ambiguous_candidates`）、`full_absent`（`target_absent`）。
  - **区域锚定诊断模式（`region_anchored_diagnostics`）**：遮挡与空位在单帧里必须有一个“注视点”才能与“本来就没有”区分开，故由调用方给出锚点；本轮锚点**取自图像本身**（可见候选的质心/深度，`diagnostic_anchor_source=image_derived_visible_candidate_not_registered_slot`），绝不读登记槽位，且该模式从不授权。覆盖 `anchored_visible`/`anchored_occluded`/`anchored_empty`。
- 采集记录（`capture.json`）：RGB/深度 SHA-256、深度有效率、宽高、内参 K、相机在 `map` 的位姿、锚点候选 id/像素/深度、`oracle_used_for_selection=false`，以及可选的 oracle 交叉核对（本轮快照端点不可用，记为 unavailable）。
- 本轮结果：7 个用例全部符合预期（`passed: true`）；授权用例产出 `TargetObservation`（`track_...`、`task_target`、`depth_valid_fraction=1.0`、`position_map_xyz≈(1.085, 0.003, 0.840)`，与仿真物体名义位姿一致但由图像反投影得到）。`authorization_reads_slot_coordinates=false` 写入报告。全程无运动。

### 20.3 结论与仍未覆盖

- 结论：计划阶段 B 条目与门槛（可见/遮挡/空位/错类/歧义五类可复现证据、授权不读槽位坐标）在本轮在线只读审计与 ROS-free 回归中得到支持。
- 仍未覆盖：没有跨帧持久 tracker（`perception_track_id` 是帧内 token）；`yaw` 仍是未观测默认值，不是物体朝向；遮挡/空位依赖调用方锚点，纯整帧模式只能给出 `target_absent`；“随机槽位”回归目前是图像内随机位置（只读 profile 下 `/thesis_sim/randomize` 被禁用，未做场景级随机化）；只有单一“单红”场景，未覆盖多货架/多实例。以上属阶段 C 及以后。

## 21. 阶段 C：已知地图多视点主动搜索（2026-11-20）

阶段 C 的目标是在已知货架地图上，仅由货架几何派生合法观察位，用可取消状态机把单视点搜索串成多视点搜索，并保证停稳、视野覆盖、过期帧与终止条件都 fail-closed。本轮全部为 ROS-free 离线实现与可复现审计；没有启动 ROS/仿真，也没有发出运动。

### 21.1 新增契约（ROS-free）

- `openflex_roboos_bridge/shelf_viewpoints.py`：`Viewpoint`、`shelf_front_geometry(obstacles)→(front_x,y_min,y_max)`、`visible_span_at(standoff,fov)`（含 `0<fov<π` 与正 standoff 校验）与 `generate_shelf_viewpoints(...)`。站位数量是覆盖 `[y_min,y_max]` 的最小值、步长上限为相机可见跨度；生成后断言覆盖连续、且站位不侵入货架 clearance 体积。输入只有静态 `Aabb` 障碍，**不接收任何目标槽位或 `ObjectMemory` 字段**。
- `openflex_roboos_bridge/active_search.py`：`ActiveSearch` 状态机 + `SearchDecision`。终态为 `found/not_found/ambiguous/stale_frame/invalid_depth/not_settled/cancelled/timeout`，只有 `found` 授权下游。`submit()` 的判定顺序为：视点匹配 → 记历史 → **停稳**（`settled=False`→`not_settled`）→ **过期帧**（age>阈值→`stale_frame`）→ **深度有效率**（低于阈值→`invalid_depth`）→ **截止时间**（→`timeout`）→ 授权（`authorized`→`found`）→ 歧义（`ambiguous_candidates`→`ambiguous`）→ 视点用尽（→`not_found`），否则推进下一视点。`__post_init__` 增加**视野覆盖不变量**：若给定视点窗口之间存在横向盲区则拒绝构造（`ActiveSearchError`）。
- 输入 `resolution` 是 `target_search_contract.SearchResolution` 同形对象（duck-typed：`outcome/reason/selection_reason/selected_perception_track_id`），因此状态机不耦合图像实现。

### 21.2 离线回归

- `scripts/tests/test_openflex_active_search.py`（16 例）：仅由几何生成视点、可见跨度与 FOV 一致、拒退化输入/货架内站位、唯一可见目标授权、逐视点扫描后安全 `not_found`、歧义立即停、过期帧/无效深度 fail-closed、取消与截止终态（过期帧优先于截止）、错误视点被拒、停稳失败 fail-closed、视点盲区被拒、决策可序列化、随机槽位 120 次重复（全部命中且**不改变视点计划**）。
- `scripts/tests/test_openflex_active_search_audit.py`（5 例）：审计脚本严格离线、四元数为单位、视点仅来自几何且窗口连续、合成帧经真实契约得到授权/歧义/错类、扫描确实跨多个视点。

### 21.3 离线可复现审计（`scripts/audit_openflex_active_search.py`）

- 仅从真实 `sim_shelf_scene.yaml` 障碍生成 4 个视点（standoff 0.75、步长 0.30、H_FOV≈86°、clearance 0.05），再为每个视点用真实头部内参（320×240、fx≈171.38）与"相机 +z 沿 map +x"位姿合成帧，帧经**真实** `analyze_frame`+`resolve_search_target` 得到授权/拒绝，再喂给状态机。
- 13 个场景全部符合预期（`passed: true`）：
  - 隐藏横向位置 ±0.6/±0.2/0/0.3/0.6 均在首个能覆盖的视点授权（`found`），其中 `+0.60` 在 `shelf_view_00` 判 `target_absent`、到 `shelf_view_01` 才授权——证明搜索确实逐视点推进。
  - 货架外（y=1.5）在 4 个视点全 `target_absent` → `not_found`；两个红目标 → `ambiguous`；只有错类 → `not_found`。
  - 过期帧 → `stale_frame`；无效深度 → `invalid_depth`；未停稳 → `not_settled`；截止超时 → `timeout`；取消 → `cancelled`。
- 报告显式写入 `robot_motion_executed=false`、`ros_started=false`、`viewpoint_source=static_shelf_geometry_only`、`authorization_reads_slot_coordinates=false`、`physical_verification=false`。

### 21.4 结论与仍未覆盖

- 结论：计划阶段 C 条目（几何派生观察位、可取消状态机、停稳/覆盖/过期帧/终止条件、随机槽位重复测试与安全失败）在本轮 ROS-free 回归与离线可复现审计中得到支持。离线套件本轮新增后为 `426 passed`（新增前为 `405 passed`）（另有 9 例因本机缺 `mcp` 等可选依赖在收集期报错，属既有环境缺口，与本任务无关）。
- 仍未覆盖（如实记录）：无跨帧持久 tracker，`perception_track_id` 仍是帧内 token；`yaw` 仍是未观测默认值；观察位只由货架几何派生，未接入实时 MoveIt 碰撞与导航；"随机槽位"目前是合成帧内隐藏位置，不是场景级随机化（只读 profile 下 `/thesis_sim/randomize` 被禁用）；只有单一货架、单一"单红"语义，未覆盖多货架/多实例/遮挡；未做隔离域 83 在线多视点运行批次。阶段 C2（未知地图建图与探索）不得开始。

## 22. 阶段 C2 前置能力盘点：建图/定位/导航（静态，2026-11-20）

阶段 C2 要求"用 RGB-D/激光建立地图并记录地图版本、定位质量"，再用 frontier/信息增益自主选点。开始写代码前先只读盘点仿真栈到底提供了什么。本轮纯静态源码/配置检查，**未启动 ROS/仿真**。

### 22.1 结论速览

- **没有激光、没有地图、没有点云话题**：全仓库（`openflex_thesis` 各级 `*.py/*.yaml/*.xml` 与 `scripts/`）无 `LaserScan`、`/map`、`OccupancyGrid` 发布者，也没有深度转点云的节点。在线几何只有头部（以及双手）RGB-D 相机话题。
- **定位是"构造真值"、不是估计器**：`bridge_node.py` 把 `map→odom` 作为近似静态变换广播（只带 `base_link_height≈0.17` 的 z），`odom→base_link` 直接取仿真里 `base_x/base_y/base_yaw` 的真值，`map→thesis_head_color_optical_frame` 也由真值直接广播；`/swerve_drive_controller/odom` 同样是真值里程计。⇒ 仿真里**没有漂移、没有协方差、没有定位估计**，"定位质量"无法作为真实估计器输出被测量。
- **导航是预设站点、不是探索**：`sim_navigation_node.py` 自述"collision-unaware navigation baseline; not Nav2"，只做比例控制；更高层 MCP 提供 `navigate_to_target(target_id=...)`，目标名是 `pickup_zone`/`delivery_zone` 这类**预设站点路线**——正是 C2 第 2 条"不得读取预设货架路线"要避免的东西。
- **中间件不启 Nav2/SLAM**：`thesis_system.launch.py` 明确注释"localization and Nav2 are intentionally external"；`run_openflex_readonly_sim.sh` 也声明"No Nav2"。不过系统侧 `/opt/ros/humble` **已安装** `nav2_*`、`slam_toolbox`、`amcl`、`octomap`、`pcl_*`、`tf2` 等，工具链可用但未接入本栈。
- **世界是一个开放地面**：上游 MJCF 只有一张 `floor` 平面（半边 2.4 m，约 4.8 m×4.8 m），加派生的货架（x≈1.24，y∈±0.65）与交付台（≈(-0.85,-0.80)）；**没有墙壁/房间**。底座是 3 自由度平面关节（`base_x/base_y/base_yaw`），由 `/thesis/cmd_vel_selected` 驱动。

### 22.2 对 C2 的含义（需要用户决策的点）

- 第 1 条"建图并记录地图版本/定位质量"：地图可以从头部深度（配合同步 TF）投影成占用栅格来建；但由于定位是真值，**"定位质量"只能是仿真真值衍生的量**，除非给里程计注入噪声/漂移并引入真正的定位估计器（AMCL/slam_toolbox 或小型 EKF）。这是一个必须先定的范围选择：
  - 方案 (a)：接受真值定位，L2 结论如实标注为"地图未知、但定位为仿真真值（无估计器）"；
  - 方案 (b)：给 `odom` 加噪并在本栈内跑定位估计器，使"定位质量"有真实含义（改动更大，需设计并复核 MoveIt/抓取链对 `odom` 的依赖）。
- 第 2 条"frontier/信息增益选点"：可实现且与仿真解耦，但**必须另起实现，不能复用 MCP 站点导航**。
- 第 3 条"定位丢失/地图不完整/未发现/超时"：可在契约与状态机里 fail-closed 实现；但"定位丢失"在真值定位下不会自然发生，需要方案 (b) 或仅在契约层模拟触发。
- 第 4 条"L1/L2 分开统计"：需要额外批次运行器，属在线阶段。

### 22.3 建议顺序

1. 先做与两方案解耦的 ROS-free 地基层：占用栅格/地图快照与定位质量契约 + frontier/信息增益选点 + fail-closed 探索状态机（合成都市栅格测试 + 离线可复现审计），不启动 ROS。
2. 方案 (a)/(b) 定下后，再做隔离域 83 的**只读**建图/探索审计（先从头部深度建占用栅格、验证 frontier 选点、不发运动）。
3. 通过后才考虑带运动的探索批次与 L1/L2 统计。

本轮未改变任何任务完成项状态；C2 条目继续不勾选。

## 23. 阶段 C2 地基层：占用建图 + frontier 探索契约（离线，2026-11-20）

按第 22 节的建议顺序，先做与定位方案解耦的 ROS-free 地基层。用户已确认采用**方案 (a)：接受仿真真值定位**，故 L2 结论须如实标注为"地图未知、定位为仿真真值（无估计器）"。本轮无 ROS/仿真、无运动。

### 23.1 新增契约（ROS-free）

- `openflex_roboos_bridge/mapping_contract.py`：
  - `LocalizationQuality`（`converged`/`source`/`position_rmse_m`/`yaw_rmse_rad`/`stamp_sec`/`covariance_trace`，构造即校验）与 `ground_truth_localization()`（`source=simulation_ground_truth_not_an_estimator`，误差 0）；`is_usable(max_position_rmse_m, max_yaw_rmse_rad)`。
  - `MapSnapshot`（帧、分辨率、原点、宽高、占用元组、`map_version`、定位质量、时间戳）与 `OccupancyGridBuilder`：`integrate_ray` 沿射线标空闲、命中标占用并单调递增版本；`integrate_points_2d`；`integrate_depth_frame` 用 `target_search_contract.camera_to_map` 把深度反投影为地图点后逐条射线积分。
- `openflex_roboos_bridge/exploration_contract.py`：
  - `find_frontier_clusters`（空闲↔未知的 frontier 聚类）+ `information_gain`（质心邻域未知格面积）+ `select_frontier`（按信息增益取最优、距离近者破平、可选 `max_reach_m`）。
  - `ExplorationSearch.plan(snapshot, robot_xy, now_sec, target_observed)` 状态机，终态 `target_found/target_not_found/map_incomplete/localization_lost/timeout/steps_exhausted/cancelled`，非终态 `next_position`。判定顺序：帧一致 → 定位质量（不可用→`localization_lost`）→ 目标观测（→`target_found`）→ 截止时间（→`timeout`）→ 步数预算（→`steps_exhausted`）→ 选 frontier：无 frontier 且覆盖率低于阈值→`map_incomplete`；无 frontier 且地图完整→`target_not_found`；否则给出 `next_position` 目标（含朝向、增益、`map_version`）。**下一位置只来自当前地图快照，不读任何预设站点/路线。**

### 23.2 离线回归与可复现审计

- `scripts/tests/test_openflex_mapping_contract.py`（8 例）与 `scripts/tests/test_openflex_exploration_contract.py`（9 例）：射线/深度积分、版本、覆盖、快照校验、frontier 聚类与信息增益、距离破平、reach 排除、定位质量门、`map_incomplete` 与 `target_not_found` 区分、超时/步数/取消终态、帧不匹配与复用拒绝、确定性。
- `scripts/audit_openflex_exploration.py`（+ `test_openflex_exploration_audit.py` 6 例）：合成真值栅格世界 + 带遮挡的射线传感器模型（经 `mapping_contract`），由 `ExplorationSearch` 逐视点驱动。7 个场景全部符合预期（`passed: true`）：
  - `room_target_absent`：4 步扫完房间（覆盖率 0.70→0.95→0.99→1.00）→ `target_not_found`；
  - `room_target_present`：3 步在 (7,3) 观测到目标格 → `target_found`（覆盖率 0.97）；
  - `sealed_pocket_map_incomplete` → `map_incomplete`；`localization_lost`、`deadline_timeout`、`steps_exhausted`、`cancelled` 各归其位。
  - 报告显式写入 `robot_motion_executed=false`、`ros_started=false`、`preset_route_used=false`、`next_position_source=current_map_frontier_information_gain_only`、`localization_source=simulation_ground_truth_not_an_estimator`。
- 全套离线回归本轮为 `449 passed`（新增 23 例；同一批 9 例因本机缺 `mcp` 等可选依赖在收集期报错，属既有环境缺口）。

### 23.3 结论与仍未覆盖

- 计划阶段 C2 第 2、3 条（frontier/信息增益自主选点、不读预设路线；定位丢失/地图不完整/未发现/超时 fail-closed）在本轮 ROS-free 回归与离线可复现审计中得到支持，条目勾选。
- 仍未覆盖（C2 不通过）：第 1 条"用 RGB-D/激光**建立**地图并记录地图版本/定位质量"尚未接真实传感器——目前建图只在合成栅格与深度反投影契约层验证，未在隔离域 83 从实时头部深度建占用栅格；第 4 条 L1/L2 分开统计需要在线批次运行器。定位真实性采方案 (a)：真值定位、无估计器，L2 结论必须如实标注。下一步是隔离域 83 的**只读**建图/探索审计（从实时头部深度积占用栅格、验证 frontier 选点，不发运动），通过后再考虑带运动的探索批次与 L1/L2 统计。

## 24. 阶段 C2 只读在线建图与选点审计（隔离域 83，2026-11-20）

在合成栅格审计（第 23 节）之后，做真正接实时传感器的只读验证：启动仓库既有 `scripts/run_openflex_readonly_sim.sh`（域 83，真值/运动通道重映射到 `openflex_readonly_disabled/*`），只订阅、只建图、只选点，**节点不发布任何话题、不发运动**。

### 24.1 做法

- 新脚本 `scripts/audit_openflex_exploration_live.py`：订阅头部 `/thesis_sim/head/color|aligned_depth_to_color|color/camera_info` 与 TF，取一帧头部深度，用 `mapping_contract.OccupancyGridBuilder.integrate_depth_frame` 反投影积分成 2-D 占用栅格（0.05 m、覆盖 4.8 m 见方的地面、`map_version` 记录积分射线数），再以仿真真值 TF 构造 `LocalizationQuality`（`simulation_ground_truth_not_an_estimator`）生成 `MapSnapshot`，交给 `exploration_contract.ExplorationSearch.plan` 选出下一观察位。并发落 `head_color.png`/`head_depth.png` 和 `exploration_live.json`。
- 采样帧确认是真实头部视图：货架三层横板 + 下层红色商品（见 `head_color.png`）。

### 24.2 结果

- `decision.outcome = next_position`，目标 `(0.62, 0.00)`，1 个 frontier 簇（137 格、增益 1.878，位于货架前沿的已知/未知边界）；栅格 269 个占用格、`coverage_fraction≈0.051`、`map_version=8129`（积分射线数）。
- **占用格包围盒与货架几何一致**：x∈[1.00,1.45]、y∈[-0.65,0.65]（第 13 节货架 front_x=0.99、back 1.49、y=±0.65）。
- 报告显式写入 `node_publishes=[]`、`robot_motion_executed=false`、`preset_route_used=false`、`next_position_source=current_map_frontier_information_gain_only`、`localization_source=simulation_ground_truth_not_an_estimator`。仿真运行期间 `MUJOCO_SAFETY_AUDIT` 无违规；审计后按 `pkill -INT` 关停。

### 24.3 在线暴露并修复的问题

- 首轮直接把整帧深度投影到 2-D 栅格时，**俯视相机把地面也标成障碍**：占用格包围盒一度扩到 x≤2.3、y=±1.95。修复：给 `integrate_depth_frame` 增加地图帧 z 高度裁剪（`min_height_m`/`max_height_m`，默认 `0.0`/不裁剪），在线取 `min_height_m=0.30` 后包围盒收敛到货架本体。该裁剪是 2-D 占用投影的标准地面过滤，已补单元测试（`test_depth_frame_height_clip_filters_floor_points`）。

### 24.4 结论与仍未覆盖

- 计划阶段 C2 第 1 条"用 RGB-D/激光建立地图并记录地图版本、定位质量"在本轮只读在线审计中得到支持（从实时头部深度建立占用地图、记录 `map_version` 与定位质量），条目勾选。
- 仍未覆盖：本轮只是**单帧快照**建图，没有多帧融合/SLAM 回路，也没有在移动中连续建图与避障；定位仍是仿真真值（方案 (a)，无估计器），L2 结论必须如此标注；第 4 条"L1/L2 分开统计"需要在线批次运行器，仍未做。故阶段 C2 整体不判通过。

## 25. 阶段 C2 第 4 条：L1/L2 分开统计契约（离线，2026-11-20）

计划要求"分别统计 L1 与 L2，禁止合并成功率"。本轮新增 ROS-free 统计契约与批次运行器，并强调：**任何合并 L1/L2（或合并纯视觉/Oracle）的请求都会被显式拒绝**。无 ROS、无运动。

### 25.1 新增契约（`openflex_roboos_bridge/trial_statistics.py`）

- `TrialRecord`（`trial_id`/`layer`∈{L1,L2}/`evidence_source`∈{pure_vision,oracle_control}/`condition`/`success`/`failure_stage`/`duration_sec`/`localization_error_m`/`safety_stop`）：构造即校验——**成功不得带失败阶段，失败必须具名归因阶段**（`perception/localization/mapping/exploration/grasp/navigation/placement/harness`）。
- `slice_metrics(records, layer=, evidence_source=)` → `Metrics`（`trials/successes/failures/success_rate/failure_rate/safety_stop_rate/mean_duration/mean_localization_error/attribution/conditions`），**失败计入分母**，归因计数和等于失败数。
- `layer_report(records)`：按 layer、再按 evidence_source 分栏输出，**只输出存在的分栏，从不跨层合并**。
- `combine_layers(...)`/`combine_evidence_sources(...)`：永远抛 `TrialStatisticsError`——这是"禁止合并"的可执行表达。
- `run_batch(specs, execute)`：按序运行；`execute` 抛异常或返回身份不符的记录时，保留该 spec 的身份并记为 `harness` 失败——**没有试验会被丢弃**。

### 25.2 离线回归与可复现审计

- `scripts/tests/test_openflex_trial_statistics.py`（6 例）：记录校验、失败入分母与归因、空分栏拒绝、L1/L2 与来源分栏且合并被拒、`run_batch` 计入 harness 错误并保身份、报告确定性。
- `scripts/audit_openflex_experiment_stats.py`（+4 例）：确定性合成批次（种子 20261120，8 条件×2 层×8 次 + 2 次注入 harness 异常）。
  - 结果：共 130 次试验；**L1 成功率 0.769**（65 次，归因 exploration 1/grasp 6/harness 1/perception 7），**L2 成功率 0.477**（65 次，exploration 3/grasp 8/harness 1/perception 22）；两层分栏、`combined_rate=null`，7 项检查全为真。
- 全套离线回归本轮为 `460 passed`（新增 10 例；同一批 9 例因本机缺 `mcp` 等可选依赖在收集期报错，属既有环境缺口）。

### 25.3 结论与仍未覆盖

- 计划阶段 C2 第 4 条的**统计能力**（L1/L2 分栏、禁止合并、失败入分母、按阶段归因、harness 计入）在本轮契约与合成批次审计中得到支持，条目勾选；支持 Oracle 对照分栏但尚无 oracle 试验。
- 仍未覆盖：本轮数字来自**合成批次**（种子驱动的确定性生成器），不是物理或在线结果；真实 L1/L2 数字需要在线批次（属阶段 F）。因此阶段 C2 作为**整阶段仍不判通过**：L2 尚未做带运动、多帧融合的调查批次，L2 结论未成立。
