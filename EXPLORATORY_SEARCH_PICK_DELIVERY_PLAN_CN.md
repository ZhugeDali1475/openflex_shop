# 自主探索、视觉抓取与交付：执行任务计划书

版本：v2.1（2026-11-20）
目标：机器人只从命令获得目标语义（第一验收批次为“红色商品”），不获得商品坐标或仿真 `object_id`，通过相机观察、主动探索、视觉定位、抓取和交付完成闭环。

**范围决议（2026-11-20）：**论文核心聚焦“任务条件化站位选择”和“分层自由度释放的全身求解”；未知地图建图与探索是支撑端到端系统演示的次级能力，不列为第三项核心研究贡献。核心算法先在受控的 L1 条件下完成公平对照与消融；L2 用于展示 SLAM/探索与操作系统的集成，单独报告，不与 L1 合并，也不作为开始或完成核心算法实验的前置门槛。只有在 L2 在线建图、定位和探索确实使用传感器估计而非 MuJoCo 真值后，才可声称完成未知地图闭环。当前不要求引入 VLN 模型。

## 1. 先冻结范围，防止偏离

### 1.1 本轮必须交付

- 第一验收层（L1，核心受控实验）：已知货架/交付台地图，商品位置未知；先实现单红商品，再做随机槽位、多视点和失败案例。用于验证站位选择与全身求解，不以探索难度混淆算法效果。
- 第二验收层（L2，次级系统演示）：地图和货架布局也未知；以仿真传感器建图、定位和基础 frontier 探索支撑完整闭环。它不是论文核心算法贡献，也不要求先于 L1 核心实验完成。没有完成真实估计器驱动的在线 L2 调查，不得宣称“完全未知环境”。
- 商品识别与抓取授权只使用 RGB-D、CameraInfo、TF 和机器人自身状态；L2 建图定位可使用 MuJoCo 仿真 MID-360/IMU 与 FAST-LIO2 估计结果，但不得用仿真 `object_id` 或真值位姿驱动在线目标选择、建图或导航。
- 在线目标身份只能是任务语义和视觉 `perception_track_id`；MuJoCo `object_id` 只允许出现在物理桥/评测器。
- 失败必须 fail-closed：目标不唯一、深度/TF 无效、抓取或放置未验证时停止并记录失败。

### 1.2 明确不做

- 不恢复或下载被清理的 AnyGrasp SDK、私有权重或不明许可证资产。
- 不启动或订阅 Oracle 来生成视觉候选；Oracle 只做独立对照实验。
- 不改造真实硬件、实机标定、力控或生产部署。
- 不把颜色阈值结果描述成通用识别；第一批次仅证明红色目标。
- 不在阶段门禁未通过时扩展多商品、开放词汇或 GUI/MCP 大重构。
- 不把未知地图探索、信息增益优化或 VLN 模型扩成第三项核心研究；L2 先采用可解释、可复现的基础 frontier 策略。

### 1.3 实验分层

| 层级 | 已知信息 | 本计划中的用途 | 能宣称的结论 |
|---|---|---|---|
| L1 | 货架地图、合法观察位、交付台语义已知；商品槽位未知 | 站位选择与全身求解的主要受控实验及消融 | 已知地图中的自主搜索取放；核心算法收益须由同条件对照支持 |
| L2 | 货架布局、商品位置、观察路线均未知 | 次级系统集成演示；与 L1 分开统计，不作为核心算法效果的替代证据 | 仅在传感器估计驱动在线建图、定位和探索并通过重复调查后，才可称未知地图闭环 |
| Oracle 对照 | 可读取仿真真值 | 调试或误差/上限对比 | 仅作对照，不计入纯视觉或 SLAM 驱动成绩 |

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

### 阶段 C1：L1 核心方法实验（论文最高优先级，不依赖 C2）

- ( ) 冻结 L1 场景、目标/抓取条件、起始状态、随机种子、规划预算和统一成功判据；真值只用于离线评测，不进入候选选择或在线规划。
- ( ) 对“任务条件化站位选择”做同条件基线与消融，检查条件身份保持是否减少“底盘可达但操作不可行”的候选/失败，并记录导航与验证代价。
- ( ) 对“分层自由度释放全身求解”做同总预算基线与消融，至少报告求解成功率、末端误差、碰撞/约束拒绝、计算时间和底盘/升降/手臂运动代价。
- ( ) 记录每次失败阶段和拒绝原因，重复不同目标槽位/起始状态；不得只挑成功案例或按测试结果回调参数后重报。

门槛：两项方法均有可复现的基线、消融与失败样本；在 L1 上形成核心结论后即可进入论文主结果整理，不等待 C2/L2 完成。细化实验定义与理论论证以《商超轮臂机器人_研究主线与算法实验方案_CN.md》为准。

### 阶段 C2：未知地图建图与探索（L2 次级系统集成，不是论文核心贡献）

- (✓) 完成 RGB-D/激光建图、地图版本和定位质量契约及单帧只读验证；**多帧融合、运动中建图与估计器驱动的定位仍待完成**。（2026-11-20：隔离域 83 只读在线审计从实时头部深度建立占用栅格并记录 `map_version`/定位质量，占用包围盒与货架几何一致；仍为单帧快照、真值定位，见审计第 24 节。）
- (✓) 完成基础 frontier/信息增益选点契约与离线验证，不读取预设货架路线；L2 在线实现先采用简单 frontier，信息增益优化不作为必做项或核心贡献。
- (✓) 处理定位丢失、地图不完整、目标未发现和超时。
- (✓) 完成 L1/L2 分栏统计契约，禁止合并成功率；真实在线批次仍待完成。（2026-11-20：`trial_statistics.py` 分栏统计 + 合并显式拒绝 + 失败入分母 + 按阶段归因；合成批次审计通过。真实 L1/L2 数字待在线批次，见审计第 25 节。）

范围与门禁：C2 的完成只决定是否能展示 L2 未知地图闭环，不阻塞 L1 下的站位选择/全身求解核心实验。L2 演示的最低可信门槛是：仿真 MID-360/IMU 输入 FAST-LIO2，地图与导航使用估计位姿，定位质量异常能安全停止或恢复，并完成多帧、带运动的重复探索批次。若暂时使用 MuJoCo 真值，只能作为调试或 Oracle 对照，不能计作 L2 成绩。长距离、大转向、回环/`map→odom`、质量信号接入等仍按当前实际状态推进；不为追求 L2 而扩大成新的探索算法研究。

前置核查（2026-11-20，静态，见阶段 A 审计第 22 节）：**数字实验（仿真）栈**无激光/`/map`/点云话题，定位是构造真值（`map→odom` 近似静态、`odom`/头部 TF 均取仿真真值，无估计器/协方差），导航是预设站点基线而非探索（不得复用）。故第 1 条的"定位质量"在定下方案 (a) 接受真值定位并如实标注，或方案 (b) 给里程计加噪并引入定位估计器之前，不具备真实语义；第 2 条需另起 frontier/信息增益实现。
补充核查（见审计第 26 节）：**同一仓库自带实机 MID-360 LiDAR + 完整 SLAM 栈**（`openflex_chassis/mapping_localization_layer`：fastlio2/pgo/hba/icp_registration，及 `navigation_layer`：Nav2/pointcloud_to_laserscan），并有真实定位质量信号（`RegistrationQuality`/低支撑守卫）。该栈未编译进仿真 `install_v2`、依赖物理 `/livox/lidar`+`/livox/imu`，仿真原本无雷达/IMU 传感器，故不能直接喂入；可选路径为实机运行、给仿真加仿真雷达+IMU 桥接、或回放实机 bag。许可证 CC BY-NC-SA 4.0（非商业），需按第 15 节口径记录。**用户选定仿真桥接路线并明确"全部在 MuJoCo 中、不涉及真机"**；2026-11-20 已在 MuJoCo 桥内落地可选仿真 MID-360 射线雷达 + IMU（只读在线验证 `/livox/lidar`≈10 Hz、`/livox/imu`≈88 Hz、`map→mid360_link` TF；`fastlio2` 有 PointCloud2 仿真入口），详见审计第 27 节。

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

- ( ) 核心批次覆盖随机槽位、观察顺序、遮挡、空位、错类、歧义、无效深度和不可达抓取；另以受限 L2 批次检查端到端集成，不把 L2 与 L1 混作核心方法效果证据。
- ( ) 对站位选择与分层全身求解分别完成同条件基线、消融、成功率与计算/执行代价比较；失败场景和适用边界一并报告。
- ( ) 统计发现率、定位误差、抓取率、放置率、端到端成功率、耗时和安全失败率。
- ( ) 失败计入分母，按感知/定位/抓取/导航/放置归因。
- ( ) 分开报告 L1/L2、纯视觉/SLAM 估计与 Oracle 对照，不合并统计；L2 结果用于系统能力说明，不代替核心算法对照。

## 4. 每阶段的停止条件

出现真值泄漏、无效深度/TF 仍运动、抓取/放置失败未停稳、删除失败记录、许可证不清或范围扩张，立即停止扩展并回到当前阶段修复。阶段完成必须同时具备：代码测试、可复现日志、失败样本、安全停机证据和 Git 提交。

## 5. 交付物

- ( ) 本计划及范围变更记录。
- ( ) 真值隔离审计、数据流图和身份契约。
- ( ) 可复现仿真启动/重置/停机/批测脚本。
- ( ) RGB-D 搜索与 `TargetObservation` 模块。
- ( ) AnyGrasp/替代后端适配器及 Oracle 对照说明。
- ( ) 视觉授权的 `search_and_pick → deliver` 证据和正式统计。
- ( ) 站位选择与分层全身求解的基线、消融、失败归因和核心结论；L2 未知地图演示作为单独的系统集成证据。

## 6. 当前状态

- (✓) 已冻结目标、边界、L1/L2 分层和停止条件。
- (✓) 已完成阶段 A 初审并确认两个阻塞：登记槽位先验、Oracle 抓取候选；并识别到可复用的 RGB-D/GraspNet 组件，但它尚未接入本任务执行链。
- (✓) 已新增观测契约和预检 Oracle 显式开关；相关离线契约/搜索/tool 回归 90 项通过。
- (✓) 阶段 B 通过（2026-11-20）：单视点 RGB-D 红色商品搜索在隔离域 83 只读在线审计中 7 用例全部符合预期，授权路径不读槽位坐标；跨帧 tracker、物体朝向与场景级随机槽位属阶段 C 及以后。
- (✓) 阶段 C 通过（2026-11-20）：多视点主动搜索。观察位仅由货架几何派生（`shelf_viewpoints.py`），`active_search.py` 为可取消、fail-closed 状态机（停稳/过期帧/无效深度/歧义/超时/取消/视点用尽均具名终止，并拒绝有横向盲区的视点计划）；离线回归 16 例、审计脚本 5 例、可复现离线审计 13 场景全部符合预期，全套离线 `426 passed`。细节见阶段 A 审计第 21 节。仍未覆盖：跨帧 tracker、物体朝向、场景级随机化、多货架/遮挡与在线多视点批次，属阶段 C2 及以后。
- (✓) 阶段 A 通过（2026-11-20）：计划阶段 A 条目全部勾选，门槛证据（在线输入无真值泄漏、纯视觉 profile 可启动、Oracle 不在进程/订阅图中）见审计第 18–19 节。该结论仅限数字仿真与规划层；感知前端、抓取后端、连续路径与执行属阶段 B–E，任何执行仍 fail-closed。
- ( ) 阶段 C1 核心方法实验尚待完成：站位选择与分层全身求解已有对应机制，但正式同条件基线、消融和重复批次尚未形成；下一优先任务是在 L1 冻结条件并开展这两组实验，不等待 L2/C2 完成。与主线方案中的 E1/E2 实验定义保持一致。
- ( ) 阶段 C2 进行中（2026-11-20，L2 次级系统集成）：C2 四条条目均已完成契约与离线/只读在线验证——`mapping_contract.py`（占用栅格/地图版本/定位质量）、`exploration_contract.py`（frontier/信息增益选点 + fail-closed 状态机）、`trial_statistics.py`（L1/L2 分栏、禁止合并、失败入分母归因）；合成栅格审计 7 场景、隔离域 83 只读建图选点（零发布/零运动）、L1/L2 合成批次审计均通过。**整阶段仍不判通过**：L2 尚未做带运动、多帧融合的调查批次，L2 结论未成立；定位采方案 (a) 真值定位、无估计器，建图仍为单帧快照。C2 不阻塞 L1 核心算法实验，但完成前不得宣称未知地图闭环。见审计第 22–25 节。
- ( ) 阶段 C2 仿真雷达桥接 + 仿真 LIO（含带运动验证）（2026-11-20）：MuJoCo 桥新增可选仿真 MID-360（`mujoco.mj_multiRay` 射线雷达，ROS-free `lidar_sim.py`）+ IMU，按 `/livox/lidar`+`/livox/imu` 重发布（修复自遮挡分组掩码与 `np` 未导入两缺陷，新增 14 例离线测试，全套离线 `474 passed`）。已在仓库外本地前缀装 Livox-SDK2 并编译 `livox_ros_driver2`+`fastlio2`（独立 overlay `install_lio`，**未改动任何上游文件**），新增仿真配置 `lio_sim_mujoco.yaml`，静止只读批次 `/lio_odom` 10.1 Hz、漂移 mm 级、`RESULT: PASS`。**并已完成带运动验证**：`scripts/run_openflex_lio_motion_sim.sh`+`scripts/audit_openflex_lio_motion.py` 在仿真导航 profile 下经速度仲裁器 NAV2 模式只动底盘（低速、短轨迹），两次运行 `/lio_odom` 对真值 RMSE 0.037/0.046 m、最大 0.049/0.061 m、偏航 ≤0.06 rad，均 PASS、无 `MUJOCO_SAFETY_STOP`、无残留进程。**仍未做**：长距离/大转向压力测试、PGO/`map→odom`（需 GTSAM）、把 `RegistrationQuality` 接入 `LocalizationQuality`、逐点时间去畸变。故 C2 整阶段继续不判通过。见审计第 27–29 节。
- ( ) 阶段 C2 `icp_registration` 编译与真实质量发布（2026-11-20）：在临时隔离前缀成功编译，5 组 GTest 全过；`/localization_quality` 已发布 `DiagnosticArray`，隔离域 83 静态只读仿真读到 `accepted=true`、`inlier_ratio=1.000`、`inlier_rmse≈0.00013m`、`p50≈0.000005m`、`p90≈0.000006m`。仿真节点需 `use_sim_time:=true`。ROS-free 适配器已将拒绝或未健康样本转为 `converged=false`，新增 3 例测试（该文件共 10 例通过）。**尚未完成**：C2 建图节点未订阅质量话题，地图快照仍用真值 TF；当前同帧静态配准不是动态 SLAM 验证。详见审计第 31–32 节。
- ( ) L2 未知地图次级演示仍未完成；优先级低于 L1 站位选择/全身求解核心实验。下一步让 C2 建图链订阅 LIO 估计与 ICP 质量并由估计位姿构图，随后做多帧、带运动在线批次；并继续长距离/转向压力测试、PGO/`map→odom` 与逐点去畸变。当前检查点不改变前述未完成状态。
- ( ) C2 估计 TF 隔离与审计门控已补齐（2026-11-21）：MuJoCo 桥支持 `publish_ground_truth_tf:=false`，只读启动脚本提供显式 `OPENFLEX_PUBLISH_GROUND_TRUTH_TF=false` profile；包内 `lio_odom_tf_bridge`（脚本保留兼容入口）只转发 FAST-LIO `/fastlio2/lio_odom` 为 `odom→base_link`；`audit_openflex_exploration_live.py --estimator` 先经参数服务确认真值 TF 已关闭，再要求新鲜 LIO + ICP 质量并按深度帧时间查询 TF，质量异常 fail-closed。受影响仿真包已成功构建，相机及质量测试 15/15 通过；这仍只是防止真值泄漏的前置门禁，不等于定位已驱动建图。仍需在该 profile 中验证 ICP `map→odom`、LIO `odom→base_link`、MID-360 静态外参与质量话题共同驱动多帧地图，之后才能做带运动 L2 批次。
- ( ) 阶段 C2 估计 TF 实测通过（静止单帧、只读，2026-11-21；见审计第 35 节）：在 `publish_ground_truth_tf:=false` profile 中首次让估计链驱动建图并跑通审计。组合：`ros2 launch openflex_thesis_mujoco mujoco_core.launch.py publish_ground_truth_tf:=false publish_lidar:=true enable_viewer:=false start_lio_odom_tf_bridge:=true` + `lio_node`（`-r __ns:=/fastlio2`，话题 `/fastlio2/lio_odom`、`/fastlio2/body_cloud`）+ `icp_registration`（`use_sim_time:=true`、`pointcloud_topic:=/fastlio2/body_cloud`、`wheel_odom_topic:=/swerve_drive_controller/odom`、临时 `pcd_path`）+ 一次 `/initialpose` 种子。实测 `map→odom`、`odom→base_link`、`map→head optical` 成立，`/localization_quality` 为 `accepted=true`、`localization_healthy=true`、`inlier_ratio=1.000`、`inlier_rmse≈1.3e-4 m`；`scripts/audit_openflex_exploration_live.py --estimator` 返回 `outcome=next_position`、`occupied_cells=272`、`goal_xy≈[0.599, 0]`、退出码 0。本轮修掉三个真缺陷 + 两个审计脚本缺陷：① 真值 `map→mid360_link` 泄漏（删广播 + 回归测试，`base_link→mid360_link` 恢复为 `[0.300, 0, 0.120]`）；② `ros2 launch` 的 `-r __node:=` 是全局重映射，致辅助节点 `ControlModeReceiver` 与桥同名 → 参数/`get_parameters` 服务二义 → 审计门 fail-closed（改 `use_global_arguments=False`，`ros2 param get ... publish_ground_truth_tf` 现返回 `False`）；③ ICP 轮式话题 `/odom` 与桥发布的 `/swerve_drive_controller/odom` 不一致 → recovery wheel gate 恒 `valid=0 pos_delta=inf` → 定位永不 HEALTHY（对齐话题后达到 `Localization HEALTHY`）；④ 审计脚本时效性竞态（固定深度时间戳 vs 持续推进的里程计）；⑤ 审计内部探索门限 `0.05m` 与估计预算 `0.25m` 不一致。**仍未完成（下次优先）**：地图为 `/tmp` 临时自扫描 PCD 且批次为静止单帧、无带运动多帧融合与长距离/转向压力批次、无 PGO/回环与逐点去畸变、"节点重名"缺回归测试且编排未固化为 `scripts/run_openflex_estimator_mapping_sim.sh`、既有 `scripts/run_openflex_lio_sim.sh`/`run_openflex_lio_motion_sim.sh` 用无命名空间 `/lio_odom` 与本次 `/fastlio2/lio_odom` 不一致待统一。全部改动仍处未提交工作区（清单见审计第 35.4 节）。C2 整阶段继续不判通过。

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
