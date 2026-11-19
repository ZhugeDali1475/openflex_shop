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
- 新增 `scripts/tests/test_target_observation_contract.py` 和“未知槽位、偏离旧注册位置”的整帧搜索测试；相关契约、搜索、MCP/tool surface 回归 `80 passed`。命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q scripts/tests/test_target_observation_contract.py scripts/tests/test_openflex_target_search.py scripts/tests/test_openflex_target_resolution_contract.py scripts/tests/test_openflex_retail_search.py scripts/tests/test_openflex_mcp_target_compat.py scripts/tests/test_openflex_tool_surface.py scripts/tests/test_openflex_delivery_contract.py`。禁用自动插件是因为系统 pytest 与用户目录 anyio 插件版本不兼容（插件导入报 `No module named '_pytest.scope'`）。
- `scripts/openflex_manipulation_preflight.launch.py` 现默认不启动 `object_memory` 和 `oracle_anygrasp`；仅显式 `OPENFLEX_ENABLE_ORACLE_GRASP=1` 才加入这两个对照组件。该启动文件已通过 `py_compile`，尚未启动 ROS 栈验证实际节点图。
- `git diff --check` 通过；新增契约文件和启动器语法编译通过。

进一步修复：`SearchSkill` 已不再读取 registry nominal slot；无 expected slot 的整帧解析会返回帧级视觉候选 token（当前字段名为 `perception_track_id`，尚不是跨帧 tracker）；并在当前阶段故意拒绝把候选交给仍要求仿真 ID 的抓取器，因此不会发运动命令。旧 slot-bound 解析仅保留给历史回归测试，不能作为未知槽位能力证据。

仍未完成且保持阻塞：MCP 和 `ManipulationSkill` 仍以 `object_id` 为抓取接口；`TargetObservation` 尚未接入规划器；启动器改动尚未做 ROS 运行时节点/订阅审计。故不进入阶段 B。原有 `openflex_ws/` 删除项保持原样。
