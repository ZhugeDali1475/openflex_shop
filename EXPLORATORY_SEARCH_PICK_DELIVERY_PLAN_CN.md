# 自主探索、视觉抓取与交付：执行任务计划书

版本：v2.0（2026-11-19）  
目标：机器人只从命令获得目标语义（第一验收批次为“红色商品”），不获得商品坐标或仿真 `object_id`，通过相机观察、主动探索、视觉定位、抓取和交付完成闭环。

## 1. 先冻结范围，防止偏离

### 1.1 本轮必须交付

- 第一验收层：已知货架/交付台地图，商品位置未知；先实现单红商品，再做随机槽位、多视点和失败案例。
- 第二验收层：地图和货架布局也未知；增加建图、定位和 frontier/信息增益探索。没有完成第二层，不得宣称“完全未知环境”。
- 所有在线感知输入只能是 RGB、深度、CameraInfo、TF 和机器人自身状态。
- 在线目标身份只能是任务语义和视觉 `perception_track_id`；MuJoCo `object_id` 只允许出现在物理桥/评测器。
- 失败必须 fail-closed：目标不唯一、深度/TF 无效、抓取或放置未验证时停止并记录失败。

### 1.2 明确不做

- 不恢复或下载被清理的 AnyGrasp SDK、私有权重或不明许可证资产。
- 不启动或订阅 Oracle 来生成视觉候选；Oracle 只做独立对照实验。
- 不改造真实硬件、实机标定、力控或生产部署。
- 不把颜色阈值结果描述成通用识别；第一批次仅证明红色目标。
- 不在阶段门禁未通过时扩展多商品、开放词汇或 GUI/MCP 大重构。

### 1.3 实验分层

| 层级 | 已知信息 | 能宣称的结论 |
|---|---|---|
| L1 | 货架地图、合法观察位、交付台语义已知；商品槽位未知 | 已知地图中的自主搜索取放 |
| L2 | 货架布局、商品位置、观察路线均未知 | 传感器建图和主动探索下的自主闭环 |
| Oracle 对照 | 可读取仿真真值 | 仅用于误差/上限对比，不计入纯视觉成绩 |

## 2. 总体数据流和身份边界

`RGB-D → 质量检查 → 目标候选/分割 → TargetObservation → 3D 位姿 → 抓取候选后端 → IK/TC-WAM/MoveIt → 抓取验证 → 携物导航 → 放置验证`。

- `task_target`：命令中的语义，如 `red`。
- `perception_track_id`：视觉跟踪产生的临时实例 ID，如 `track_red_001`。
- `mujoco_object_id`：仿真物理对象 ID，只能在评测/接触验证边界使用。
- 抓取接口必须接收 `TargetObservation` 或其 ID，不得要求预登记商品 ID。
- 每条观测必须包含图像/深度引用、mask/bbox、相机和地图坐标、深度有效率、TF 状态、时间戳、置信度和来源。

## 3. 分阶段执行清单

### 阶段 A：基线、契约与真值隔离（当前阶段）

- (✓) 重写并冻结本计划书 v2，明确 L1/L2、边界和门禁；场景哈希、ROS domain、启动/停机命令沿用阶段 A 审计基线。
- (✓) 新增 ROS-free `TargetObservation` 数据契约与拒绝 Oracle/无效深度/无效 TF 的单元测试；尚未接入在线抓取主链，故身份解耦整体仍未完成。
- (✓) 移除 `SearchSkill` 的 `expected_slot`/`nominal_pose_xyzyaw` 调用和搜索时目标登记深度/像素先验。
- (✓) `resolve_search_target` 在无 expected slot 时扫描整帧 RGB-D，输出帧级视觉 token 且不返回仿真 `object_id`；旧槽位模式仅保留作回归对照（当前 token 不是跨帧追踪 ID）。
- (✓) 抓取工具面改为 `perception_track_id`，新增视觉抓取输入校验；缺深度/时间戳/TF、Oracle 来源或传入 `object_id` 时拒绝。当前执行器仍在未接入视觉后端时 fail-closed，不发运动。
- (✓) 预检启动器默认不启动 `oracle_anygrasp`、`object_memory`；Oracle 仅由显式 `OPENFLEX_ENABLE_ORACLE_GRASP=1` 加入对照进程。
- (✓) 将通过 RGB-D 搜索的帧级候选序列化为 `TargetObservation` 并写入搜索证据；尚无跨帧持久 tracker，不能将帧级 token 冒称为稳定跟踪器。
- (✓) 静态核查仓库现有 `openflex_grasp`：Qwen/SAM2 同帧 RGB-D 分割与点云、GraspNet `FitGrasp` 可作为视觉抓姿候选技术基础；其接口边界和未接通原因已记入阶段 A 审计。此项只确认代码能力，不代表模型资产可用或闭环已集成。
- (✓) 新增 ROS-free `VisualGraspPointCloudInput` 契约：校验 `TargetObservation`、同相机帧/时间戳、商品与场景 XYZ 点云、有限值和最少点数，并序列化为不含 `object_id` 的 FitGrasp 适配载荷；该契约不调用 ROS、不执行运动。
- (✓) 搜索观测现在显式记录拟执行手臂（本阶段固定右臂），可通过上述视觉点云契约；MuJoCo 当前头部 RGB-D 话题及 `map→head optical` TF 已静态核对，头部相机标定来源仍标记为仿真估计、非物理标定。
- (✓) 将现有视觉点云抓姿链适配到本任务的 MuJoCo 相机、场景坐标/TF、仿真 MoveIt 规划和评测接口（见审计第 19 节）：新增 ROS-free `visual_grasp_moveit.py` 与只读探针 `probe_openflex_visual_grasp_moveit.py`，在隔离域 83 用实时头部深度 + 实测 TF + 货架场景向只规划 `move_group` 应用 7 个碰撞体并回读、对实测起始状态做 `/check_state_validity`（valid、0 接触）、写出感知/评测证据对；未执行任何运动。未借用 `PickObject` 的注册 `object_id` 校验。分割/GraspNet 前端、候选 IK、连续路径与轨迹执行仍属阶段 B/D。
- (✓) 建立感知证据与评测真值分离的日志 schema：每次搜索尝试独立保存 `*_perception.json` 与 `*_evaluation.json`；感知层拒绝非空仿真 ID，评测层默认 `physical_verification=false`。
- (✓) 完成静态测试和只读运行时审计（隔离域 83 只读 MuJoCo 桥；见阶段 A 审计第 18 节）：Oracle/真值/运动通道在图上无发布者，头部相机与 `map→head` TF 在线可用，运动 action 重映射失效问题已修复并复验；不发运动指令。

门槛：源码、配置和日志都能证明在线输入无真值泄漏；纯视觉 profile 可启动且 Oracle 不在进程/订阅图中。未满足则停留在 A。

### 阶段 B：单视点 RGB-D 找红色商品

- (✓) 采集同步 RGB、深度、CameraInfo、TF，并保存哈希和帧引用：`audit_openflex_target_search.py` 记录 RGB/深度 SHA-256、深度有效率、内参 K、相机位姿与锚点，并落盘 `frame.png`/`depth.png`/`case_*.png`。
- (✓) 完成整帧红色候选、深度有效率、几何过滤、遮挡/空位/错类/歧义判定（见审计第 20 节）：新增 `depth_valid_fraction`；整帧授权模式覆盖可见/错类/歧义/缺失，遮挡与空位由“图像派生锚点”的诊断模式区分，并收紧饱和度门限以排除货架暗部误判。
- (✓) 输出唯一 `TargetObservation`；唯一且可定位时才授权抓取：新增 `observation_from_resolution`，非授权原因一律抛 `TargetSearchError`；观测只带视觉 `perception_track_id`、显式记录 `slot_coordinates_used=false`。
- (✓) 用随机槽位和无目标/错类/无效深度案例回归：新增 11 例测试含 24 次随机位置回归与无效深度 fail-closed；在线审计 7 用例全通过。随机化目前在图像空间（只读 profile 禁用 `/thesis_sim/randomize`），未做场景级随机槽位。

门槛：可见、遮挡、空位、错类、歧义五类均有可复现证据，搜索不读槽位坐标。（2026-11-20 达成，证据见审计第 20 节；遮挡/空位用图像派生锚点，授权路径零槽位读取。）

### 阶段 C：已知地图中的多视点主动搜索

- (✓) 仅由货架几何生成合法观察位和朝向，不使用目标槽位。
- (✓) 实现观察→质量检查→候选→下一视点的可取消状态机。
- (✓) 检查停稳、视野覆盖、过期帧和搜索终止条件。
- (✓) 随机商品槽位重复测试；找不到或歧义时安全失败。

### 阶段 C2：未知地图建图与探索（完整毕业设计必做）

- ( ) 用 RGB-D/激光建立地图并记录地图版本、定位质量。
- ( ) 用 frontier/信息增益自主选择下一位置，不读取预设货架路线。
- ( ) 处理定位丢失、地图不完整、目标未发现和超时。
- ( ) 分别统计 L1 与 L2，禁止合并成功率。

### 阶段 D：视觉抓取候选与执行

- ( ) 将 `TargetObservation` 点云/位姿接入抓取后端。
- ( ) 先核验真实 AnyGrasp SDK、模型和许可证；不可用时使用明确标注的几何/GraspNet 后端。
- ( ) Oracle 只保留对照批次，不能进入纯视觉成绩。
- ( ) 用 TC-WAM、IK、MoveIt 做碰撞和轨迹检查，记录失败原因。

### 阶段 E：抓取—携物—交付闭环

- ( ) 完成预抓取、接近、闭爪、撤出和独立附着验证。
- ( ) 使用已知交付台语义导航，不读取商品真值坐标。
- ( ) 预放置、释放、稳定等待、撤臂和独立终态验证。
- ( ) 任一失败都安全停稳，不写成功。

### 阶段 F：正式批次和论文证据

- ( ) 覆盖随机槽位、观察顺序、遮挡、空位、错类、歧义、无效深度和不可达抓取。
- ( ) 统计发现率、定位误差、抓取率、放置率、端到端成功率、耗时和安全失败率。
- ( ) 失败计入分母，按感知/定位/抓取/导航/放置归因。
- ( ) 分开报告纯视觉和 Oracle 对照，不合并统计。

## 4. 每阶段的停止条件

出现真值泄漏、无效深度/TF 仍运动、抓取/放置失败未停稳、删除失败记录、许可证不清或范围扩张，立即停止扩展并回到当前阶段修复。阶段完成必须同时具备：代码测试、可复现日志、失败样本、安全停机证据和 Git 提交。

## 5. 交付物

- ( ) 本计划及范围变更记录。
- ( ) 真值隔离审计、数据流图和身份契约。
- ( ) 可复现仿真启动/重置/停机/批测脚本。
- ( ) RGB-D 搜索与 `TargetObservation` 模块。
- ( ) AnyGrasp/替代后端适配器及 Oracle 对照说明。
- ( ) 视觉授权的 `search_and_pick → deliver` 证据和正式统计。

## 6. 当前状态

- (✓) 已冻结目标、边界、L1/L2 分层和停止条件。
- (✓) 已完成阶段 A 初审并确认两个阻塞：登记槽位先验、Oracle 抓取候选；并识别到可复用的 RGB-D/GraspNet 组件，但它尚未接入本任务执行链。
- (✓) 已新增观测契约和预检 Oracle 显式开关；相关离线契约/搜索/tool 回归 90 项通过。
- (✓) 阶段 B 通过（2026-11-20）：单视点 RGB-D 红色商品搜索在隔离域 83 只读在线审计中 7 用例全部符合预期，授权路径不读槽位坐标；跨帧 tracker、物体朝向与场景级随机槽位属阶段 C 及以后。
- (✓) 阶段 C 通过（2026-11-20）：多视点主动搜索。观察位仅由货架几何派生（`shelf_viewpoints.py`），`active_search.py` 为可取消、fail-closed 状态机（停稳/过期帧/无效深度/歧义/超时/取消/视点用尽均具名终止，并拒绝有横向盲区的视点计划）；离线回归 16 例、审计脚本 5 例、可复现离线审计 13 场景全部符合预期，全套离线 `426 passed`。细节见阶段 A 审计第 21 节。仍未覆盖：跨帧 tracker、物体朝向、场景级随机化、多货架/遮挡与在线多视点批次，属阶段 C2 及以后。
- (✓) 阶段 A 通过（2026-11-20）：计划阶段 A 条目全部勾选，门槛证据（在线输入无真值泄漏、纯视觉 profile 可启动、Oracle 不在进程/订阅图中）见审计第 18–19 节。该结论仅限数字仿真与规划层；感知前端、抓取后端、连续路径与执行属阶段 B–E，任何执行仍 fail-closed。
- ( ) 阶段 C2 及以后。

### 6.1 阶段 A 最新审计补充（2026-11-19）

- 当前 HEAD：`acc7d26`（感知证据与评测真值分离）；本轮提交之前已无本任务未提交代码改动，工作区只存在既有 `openflex_ws/` 删除项。
- 已静态复核 `GenerateGrasps → AnyGrasp gateway/Oracle → ComputeTcWam → MoveIt grasp executor`：现有链路从 `ObjectMemory` 取得目标 pose/身份，并用 `object_id + map_version` 做请求、候选和执行关联；MoveIt 执行器也订阅 `/thesis/object_memory` 做新鲜度/身份验证和仿真附着管理。因此 ROS-free `TargetObservation/VisualGraspPointCloudInput` 还没有可直接调用的 ROS 规划入口。
- 可复用边界：`FitGrasp` 接收视觉商品/场景点云并返回抓姿候选；MoveIt 执行器可单独装载静态货架碰撞场景。不可复用为纯视觉证据：当前 `GenerateGrasps/ComputeTcWam/ExecuteMoveItGrasp` 所需 ObjectMemory 目标语义及 Oracle pose 候选。
- 推荐先定义不含仿真 ID 的视觉候选/规划接口，再隔离验证“视觉候选 + 独立目标碰撞几何 + 静态货架场景 + MoveIt 规划”；评测真值只允许由仿真桥在动作后关联，不能进入目标选择或规划输入。详细字段依赖和边界见阶段 A 审计第 11 节。
- 阶段 A 仍未通过；本轮未启动 ROS/仿真、未做运行时订阅审计、未发出运动，也未改变任何任务完成项状态。
- 已按第 11 节接口建议新增 ROS-free `VisualGraspPlanRequest` 契约（`visual_grasp_planning.py`）：从视觉点云推导独立目标碰撞体、把候选抓姿变换到规划帧、并与静态货架障碍和实测机器人状态组合成不含仿真身份的规划请求；具备 13 例拒绝路径/变换测试，离线批次最新 `126 passed`。此契约尚未接入 MuJoCo TF/时间戳运行时、GraspNet 资产、MoveIt transport 与评测真值关联，故计划“视觉点云抓姿链适配”条目不勾选。
- 已新增 ROS-free 场景/TF 绑定 `visual_grasp_scene.py`：解析货架场景 YAML 为保守障碍 AABB、强制 `场景 frame==规划帧==观测 map_frame`、从实测头部相机 TF 构造相机→规划变换，并组装不含仿真身份的规划请求；新增 8 例测试（含真实 `sim_shelf_scene.yaml`），离线批次最新 `134 passed`。头部相机在 `map` 的位姿仍是运行时读数，运行时 TF 新鲜度/订阅图审计与 MoveIt transport 仍未做，故计划条目继续不勾选。
- 已新增 ROS-free 抓姿拟合传输适配 `visual_grasp_fit_adapter.py`：把视觉点云输入塑造成 `FitGrasp` 请求载荷，并校验拟合结果（成功、非 Oracle、同帧同时间戳、单位四元数）后转为规划候选，串起“点云→FitGrasp→候选→规划请求”的纯数据链路；新增 7 例测试并含端到端贯通，离线批次最新 `141 passed`。真实 GraspNet 资产、`FitGrasp` ROS action、运行时 TF/订阅审计与 MoveIt transport 仍未做，故计划条目继续不勾选。
- 已完成计划第 6 节"模型资产可用性"静态核查：新增只读脚本 `audit_openflex_model_assets.py` 与 6 例测试。结论：本机 `openflex_models` 四个必需文件在位、`SHA256SUMS` 27/27 哈希一致、无 AnyGrasp 残留；许可证已记录（Qwen/SAM2 为 Apache-2.0，GraspNet 为 SJTU 非商业研究，moveit 包 CC-BY-NC-SA-4.0）。但 `transformers` 未装、GraspNet CUDA 扩展未编译、本机 torch 版本与依赖锁不一致——推理运行时未就绪，不下载/不安装，阶段 A 继续不勾选。
- 已把视觉抓姿规划链接入感知/评测分离证据层 `visual_grasp_evidence.py`：规划请求写入 `PerceptionEvidence`（构造即校验、拒绝注入仿真身份），评测层默认真值无关，并新增 4 例测试；离线批次最新 `151 passed`。仍无在线运行批次，运行时 TF/订阅审计、MoveIt transport 与 GraspNet 推理运行时未就绪，计划条目继续不勾选。
- 已完成计划第 6 节"推理运行时可用性"本机核查（经用户授权后安装）：修复迁移 venv 失效 shebang（56 个），用仓库外 `/home/wgb/.cuda/cuda-12.6-local`（nvcc 12.6.85）按兼容矩阵在目标环境重编 `pointnet2`/`knn_pytorch` CUDA 扩展（`sm_86;sm_89`），并新增只读脚本 `scripts/audit_openflex_graspnet_runtime.py`（+ 10 例测试）用 subprocess 探针验证：CUDA 算子全 PASS、checkpoint 推理 513,688 点得 `(326,17)` 全有限、perception venv 可导入 torch/transformers、`overall_ok=true`。环境产物在仓库外或已 gitignore，提交只含脚本/测试/文档；阶段 A 的"隔离域 83 只读运行时审计（节点/话题/TF/订阅图）"与在线 transport、执行后真值关联仍未做，故阶段 A 与计划第 56 行条目继续不勾选。
- 已完成隔离域 83 只读运行时审计（见阶段 A 审计第 18 节）：先启动仓库既有 `scripts/run_openflex_readonly_sim.sh`，只观察节点/话题/服务/action/TF 再关停。证实 ≤2 节点、无 Oracle/AnyGrasp/MoveIt/Nav2/mission；`/thesis/object_memory` 在图中不存在、真值/运动通道发布者为 0；头部相机约 4 Hz 且 `map→thesis_head_color_optical_frame` TF 在线广播；约 30 s 内 `MUJOCO_SAFETY_AUDIT` 无违规。审计发现只读 profile 的运动 action 重映射在 Humble 下失效（action 图名是隐藏 `_action/*`，精确名规则不匹配），已把 `run_openflex_readonly_sim.sh` 改为展开 5 个隐藏名重映射并复验：运动 action 现为 `/openflex_readonly_disabled/*`、规范名消失。阶段 A 仍不勾选：视觉抓姿链接入 `FitGrasp` ROS action、MoveIt transport 与评测真值关联仍未做。
- 已完成阶段 A 收尾：视觉抓姿链接入本任务在线接口（审计第 19 节）。新增 ROS-free `openflex_roboos_bridge/visual_grasp_moveit.py`（把规划请求转为只规划 MoveIt 载荷：规划帧下目标保守体积 + 货架障碍盒 + 每候选一个实测关节状态；固定 plan-only 并拒绝允许执行的参数集）与 9 例测试；新增只读探针 `scripts/probe_openflex_visual_grasp_moveit.py`。隔离域 83 组合栈（只读 MuJoCo 桥 + 只规划预检 MoveIt）两次复现：无 oracle 节点、`/thesis/object_memory` 不存在、`/thesis/anygrasp/request` 订阅者 0；探针由 320×240 `16UC1` 实时头部深度反投影得 13687 目标点，向 `/apply_planning_scene` 应用 7 个碰撞体并回读一致，`/check_state_validity` 实测起始状态 valid、0 接触，写出感知/评测证据对；全程 `trajectory_execution=false`、无运动。阶段 A 至此通过；阶段 B 起（感知前端、抓取后端、连续路径、执行）与物理验证仍未开始。
