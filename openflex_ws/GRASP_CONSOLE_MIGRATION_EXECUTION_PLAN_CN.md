# OpenFlex 抓取能力迁移与比赛控制台集成：正式执行计划

本文是从讨论稿转化出的施工清单。只有在项目负责人审阅并明确批准后，才开始修改代码、URDF、环境、模型或目标工作空间内容。

## 0. 范围、路径与状态规则

迁移起点：

```text
/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_ws/src/openflex_grasp
```

迁移目标：

```text
/home/openflex/old/openflex_ws
```

模型根目录：

```text
/home/openflex/old/openflex_models
```

本阶段真机能力边界：

```text
/detect_object -> /pick_object -> 手臂安全回撤
```

状态填写规则：

- [ ] **待执行**：保持 `- [ ]`，不得提前勾选。
- [ ] **通过**：执行、测试和证据均完成后改为 `- [x]`，并在条目末尾填写证据路径或测试结果。
- [ ] **未通过**：保持 `- [ ]`，追加 `**✗ 未通过**`、失败原因、日志/证据路径、影响范围和重新执行条件；不得用“部分完成”代替失败记录。
- [ ] **阻塞传播**：关卡未通过时，所有依赖该关卡的真机或破坏性步骤停止；可继续执行不依赖它的只读检查、单元测试和文档工作。
- [ ] **改动纪律**：每完成一个阶段，记录 `git status`、改动文件、测试命令和结果；不回退用户原有改动，不复制 `build/`、`install/`、`log/`、`__pycache__/` 或旧 CUDA `.so`。

## 1. 总门禁与只读基线

- [x] **G0-01 用户批准执行**：项目负责人审阅本文后明确同意开始长时间迁移。批准前不得执行 G0-02 之后的写操作。记录：2026-08-12 09:32 CST，项目负责人通过当前会话批准；仍须遵守真机运动安全关卡和 Place 后置边界。
- [x] **G0-02 冻结源与目标清单**：记录源目录和目标 workspace 的 Git 状态、未跟踪文件、包清单及磁盘空间；保存到迁移日志，不提交或删除无关改动。验收：能够区分迁移改动与原有改动。证据：`migration_records/2026-08-12/G0_FILE_BASELINE_CN.md`。
- [x] **G0-03 冻结运行基线**：记录 ROS graph、Action/Service/topic、TF tree、控制器、MoveIt planning group、相机 serial/topic、夹爪 Action 和当前 launch 参数。验收：当前为离线 ROS graph；已记录空运行图和代码 owner，live TF/controller/Action 证据明确转入相应启动关卡补采。证据：`migration_records/2026-08-12/G0_ROS_BASELINE_CN.md`。
- [x] **G0-04 冻结硬件与模型基线**：记录 GPU PCI 信息、驱动包、`nvidia-smi`、设备节点、Python/PyTorch/CUDA 状态，以及 Qwen/SAM2/GraspNet 文件大小和 SHA256。验收：RTX 3090/595.84 当前已恢复可用；590/595 元包并存和未安装 PyTorch/nvcc 已如实记录，模型未加载。证据：`migration_records/2026-08-12/G0_HARDWARE_MODEL_BASELINE_CN.md`。
- [x] **G0-05 建立执行日志目录**：已建立 `migration_records/2026-08-12/`，后续按关卡保存基线、测试、截图/rosbag 索引和失败说明。证据：`migration_records/2026-08-12/README_CN.md`。

## 2. 第一项实际修改：应用左右腕新手眼标定（D0）

- [x] **P1-01 校验标定资产**：`SHA256SUMS` 中左右 `.calib/.samples` 四项均校验成功；README 与标定帧、OpenCV/Park、25 组样本及相机序列号一致。证据：`migration_records/2026-08-12/D0_HAND_EYE_PROGRESS_CN.md`。
- [x] **P1-02 保存旧 URDF 与 TF 证据**：已保存修改前 nominal 外参、源文件/展开 URDF SHA256、`check_urdf` 树和 RealSense 完整内部 TF。采集时整机未运行，旧 live 全树不可用；该限制已记录，现有数值及发布者可追溯。证据：`migration_records/2026-08-12/D0_HAND_EYE_PROGRESS_CN.md`。
- [x] **P1-03 写入右腕固定关节**：仅替换 `right_poe_calibrated=false` 的 xyz/rpy 和标定注释；`true` 分支保持原值。两个分支均成功展开并通过 `check_urdf`。证据：`migration_records/2026-08-12/D0_HAND_EYE_PROGRESS_CN.md`。
- [x] **P1-04 验证右腕 TF 唯一性**：临时启动指定 D405 和 nominal `robot_state_publisher`；35 个静态 TF child 无重复，RealSense 唯一发布内部两段。新 mount 重组 optical 标定的平移误差 `4.34e-14 m`、旋转误差 `0 rad`。证据：`migration_records/2026-08-12/D0_HAND_EYE_PROGRESS_CN.md`。
- [ ] **P1-05 连接并核验左腕标定相机**：当前 `rs-enumerate-devices` 未发现 D405 `260522277324`，本项受硬件连接阻塞；未使用右腕 TF 或估算值。恢复条件：连接指定相机后读取并保存其内部 TF、serial 和内参。证据：`migration_records/2026-08-12/D0_HAND_EYE_PROGRESS_CN.md`。
- [ ] **P1-06 换算左腕固定关节**：用左腕 optical 标定结果和 P1-05 的内部 TF 精确计算 `left_link7 -> left_camera_link`，同时做矩阵重组校验。验收：记录 xyz/rpy、四元数、计算脚本/命令和重组误差。
- [ ] **P1-07 写入并验证左腕 URDF**：写入左腕固定关节，展开 xacro并检查唯一 TF 父链。验收：左右 camera optical frame 均只有一个有效父链。
- [ ] **P1-08 完成静态可视化验收**：在 RViz 检查机器人模型、左右彩色图、深度点云、相机轴方向和固定物体位置。验收：截图和 TF 记录完整，无明显轴向或尺度错误。
- [ ] **P1-09 完成多姿态稳定性验收**：左右臂分别至少选择 5 个覆盖预期抓取工作区的姿态，使用同一静止目标记录其点云中心在 `base_link` 下的漂移。验收：每侧最大中心漂移不超过 15 mm，点云无随手臂姿态变化的系统性方向偏移；未达门槛不得进入真机抓取执行。
- [ ] **G1 D0 关卡通过**：P1-01 至 P1-09 全部通过；否则记录失败原因并禁止 `execute=true`。

## 3. 迁移源码与清理 AnyGrasp legacy（D7）

- [x] **P2-01 建立迁移文件清单**：已将 136 个有效文件分类为直接迁入/后续重构、checkpoint 转交模型阶段、重复消息过渡迁入和明确排除项；未包含缓存、构建物、嵌套 Git 或旧扩展。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-02 迁入纯源码**：已新增 `src/openflex_grasp/`，迁入 109 个纯源码/资源文件；文件列表和逐文件 SHA256 与源侧一致，禁用项扫描为零，未复制 checkpoint、构建物、缓存、虚拟环境、嵌套 Git 或编译扩展。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-03 核对许可证与模型条款**：已记录 Qwen/SAM2 Apache-2.0、GraspNetBaseline 代码与 checkpoint 的非商业研究限制，以及迁入 MoveIt/messages 的 CC-BY-NC-SA-4.0 声明；manifest 不依赖 AnyGrasp 授权。证据：`migration_records/2026-08-12/MODEL_AND_CODE_LICENSE_MANIFEST_CN.md`。
- [x] **P2-04 清点所有 AnyGrasp 活跃引用**：已扫描 549 行/99 个文件，分为必须从比赛活动链删除或替换的 16 个 integrated/legacy 入口，以及保留但不得进入新比赛 launch 的 thesis 历史子系统。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-05 替换比赛抓取策略名**：MuJoCo 货架/补货箱已改为 `graspnet_shelf/graspnet_basket`，test_07/test_08 明确为 `mock`；解析器拒绝两个旧名称。`test_station_db.py` 7 项通过，所有 station 配置解析成功。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-06 清理比赛和仿真启动引用**：已移除商超仿真的 `start_anygrasp`、worker/gateway、SDK checkpoint/Python preflight 和 `anygrasp_sim.yaml`；launch 构造和语法通过，活动仿真代码扫描为零。4 个既有绝对路径测试失败已记录并转入 D9/仿真关卡。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-07 清理 GUI 与运行文档引用**：活动 GUI 已改为“局部检测”，mock 警告和当前运行说明已统一为 Qwen/OWLv2 + SAM2 Detect、GraspNetBaseline Pick；历史仿真交接和 thesis README 已显式标记 legacy，不得进入比赛链。GUI 无 AnyGrasp backend。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-08 删除目标 legacy 包**：删除前确认外部 `openflex_anygrasp` 依赖为零后，已精确删除 `/home/openflex/old/openflex_ws/src/openflex_thesis/openflex_anygrasp`；`openflex_thesis_msgs` 及其他 thesis 包均保留，论文物理预检改为明确失败关闭。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **P2-09 验证删除后的依赖闭包**：活动包名/SDK/worker/launch 引用扫描均为零；独立临时目录干净构建 43 个包全部成功，预检测试 3 项和 station 测试 7 项通过；剩余论文历史文本已建立目录允许清单。证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。
- [x] **G2 legacy 清理关卡通过**：目标比赛运行链和构建链已不依赖 AnyGrasp；防回归解析器继续拒绝旧策略名，GraspNetBaseline 是唯一新抓取网络。通过时间：2026-08-12；证据：`migration_records/2026-08-12/P2_SOURCE_AND_ANYGRASP_INVENTORY_CN.md`。

## 4. 收敛 ROS Action 与 MoveIt 所有权（D1、D2、D4）

- [x] **P3-01 迁移 `CartesianApproach.action`**：已逐字加入 `openarmx_integrated_moveit_msgs` 并生成 C/C++/Python 类型；目标包现同时拥有 `PlanPregrasp`、`ExecuteJointPosture`、`CartesianApproach`。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-02 迁移 Cartesian server 类型**：`openflex_grasp_moveit` 已改用 `openarmx_integrated_moveit_msgs/CartesianApproach`，原有走廊、严格 Cartesian 和普通 MoveIt 逻辑保持不变；两包独立构建成功。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-03 统一预抓取 Action**：商品执行器和轻量 bridge 均已改用 `openarmx_integrated_moveit_msgs/PlanPregrasp`，server 本来即使用该类型；旧消息包活动 client 扫描为零。单一 live server 数量将在 G3 启动验收复核。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-04 替换精确关节姿态扩展**：已移除所有活动 `use_exact_joint_goal/exact_joint_goal` 调用，记录的完整拍照姿态统一调用 `/openflex/execute_joint_posture`；左/右臂及各自带升降四种 planning group 均有测试，抓取测试 63 项通过。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-05 保持唯一 MoveIt owner**：静态 launch/server 扫描确认 `openarmx_integrated_moveit_config` 继续唯一创建 `move_group`、PlanPregrasp 和 ExecuteJointPosture；`openflex_grasp_moveit` 只提供 Cartesian server，不创建 RSP/controller。live 唯一实例数保留到 G3 复核。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-06 扩展 Detect goal**：`DetectObject.Goal` 已增加 `string arm_name`；TaskExecutor 从 source slot 显式解析且只接受 `left/right`，不再把缺失配置猜成左臂；truth 仿真入口拒绝空/非法臂。安装态 Action introspection 和相关测试通过。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-07 扩展 Pick goal**：`PickObject.Goal` 已增加 `string detection_id`；TaskExecutor 拒绝 detector 返回的空 ID，truth 仿真实现 30 秒 TTL、object/station/arm 一致性和单消费者校验，mock/GUI 使用显式 `mock:<arm>:<object>` ID。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-08 修改 TaskExecutor 传递链**：同一 source slot 的标准化 `arm_name` 已同时写入 Detect/Pick；`_detect()` 返回的原始 `detection_id` 被显式保留并传给 `_pick()`，无“最近检测”或默认臂回退。全新临时目录构建 38 包成功，安装态相关测试 31 项通过。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-09 收敛错误码和详细消息**：`TaskError` 已增加相机、模型、感知、规划、夹爪、检测过期和检测不一致稳定码；TaskExecutor 只透传登记白名单，未知/成功码按阶段归并，并保留子 Action 非空 message 到最终 result。取消、超时、TF、检测及本地安全失败也有确定码和详细消息。四包增量构建成功，安装态相关测试 51 项通过。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **P3-10 处置旧 MoveIt 消息包**：活动源码、CMake 和 manifest 依赖扫描为零后，已删除重复的 `src/openflex_grasp/openflex_grasp_msgs` 包；迁移指南和抓取 README 已改用 `openarmx_integrated_moveit_msgs`。删除后 `colcon list` 不再发现旧包，统一消息 owner、Cartesian server 和抓取 Python 包在全新临时目录构建成功。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。
- [x] **G3 接口关卡通过**：全新临时目录构建 33 包成功；安装态 introspection 确认 Detect 含 `arm_name`、Pick 含 `detection_id`；truth 仿真以 `juice/shelf_c_pick/left` 完成 Detect -> 原始 `truth:left:2` -> Pick，返回附着成功；TaskExecutor 独立测试 72 项通过、1 项跳过。隔离 `ROS_DOMAIN_ID=93` 的 fake-hardware 启动确认四套硬件均为 GenericSystem，`move_group` 和三个统一 MoveIt Action 各只有一个 live server。关闭阶段 `move_group` 在整体 SIGINT 销毁时出现 Humble/MoveIt 退出段错误，未发生运动且全部进程已结束，作为已知关闭问题保留。通过时间：2026-08-12 10:49 CST。证据：`migration_records/2026-08-12/P3_MOVEIT_INTERFACE_PROGRESS_CN.md`。

## 5. 模型资产、GPU 与隔离环境（D6）

- [x] **P4-01 修复 GPU 驱动基线**：已移除只用于升级过渡的 `nvidia-driver-590` 元包，并将 `nvidia-driver-595` 保持为手动安装。复核确认当前仅有 595.84 正式驱动栈，RTX 3090、DKMS、内核模块和全部 `/dev/nvidia*` 节点正常，`nvidia-smi` 稳定识别 24 GB 显存，无需重启。通过时间：2026-08-12 11:08 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`。
- [x] **P4-02 确认 CUDA/PyTorch 兼容矩阵**：已锁定 Python 3.10.12、595.84 驱动、CUDA 12.6、PyTorch 2.7.1+cu126、torchvision 0.22.1+cu126 和 transformers 5.15.0，并拆分 perception/GraspNet 两份 requirements 锁文件。核验确认 5.15.0 同时包含 Qwen3.5 与 SAM2；GraspNet 使用匹配 CUDA 12.6 的 nvcc 现场重编，固定 RTX 3090 `sm_86`。Ubuntu CUDA 11.5 明确禁用。通过时间：2026-08-12 11:20 CST。证据：`src/openflex_grasp/environment/COMPATIBILITY_MATRIX_CN.md`、`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`。
- [x] **P4-03 校验并解压 Qwen3.5-4B**：源 ZIP SHA256 `22cd8797874bd57f6a5788a3d11b20b761df3dde8fce871e1619d7f2b6710439`，全量 `unzip -t` CRC 通过，路径无绝对路径或 `..` 穿越；已解压到 `/home/openflex/old/openflex_models/Qwen3.5-4B`，保留源 ZIP。目标包含 15 个文件、两个 safetensors 权重分片、索引 738 个权重条目；JSON、索引集合和 safetensors 头部均通过校验。通过时间：2026-08-12 11:27 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`、`/tmp/Qwen3.5-4B.SHA256SUMS`。
- [x] **P4-04 部署 SAM2 与 GraspNet 资产**：已将 SAM2.1 Hiera Small 目录和 GraspNetBaseline `checkpoint-rs.tar` 部署到 `/home/openflex/old/openflex_models/`，保留 LICENSE/README；统一 `SHA256SUMS` 中 27 个资产文件全部校验成功，源/目标权重逐字节一致。当前代码路径仍有旧 9B/源 workspace 文档默认值，统一参数清理列入 P4-07；活动后端不引用 AnyGrasp。通过时间：2026-08-12 11:39 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`、`/home/openflex/old/openflex_models/SHA256SUMS`。
- [x] **P4-05 创建 perception 环境**：已创建 `/home/openflex/old/.venvs/openflex_perception`，安装锁定依赖并补锁 ROS `generate-parameter-library-py` 所需的 `typeguard==2.2.2`；`pip check` 无错误。验证 Python 3.10.12、torch 2.7.1+cu126、CUDA 12.6、RTX 3090 compute 8.6、BF16、`rclpy`、`cv_bridge` 及目标 Task/MoveIt 消息均正常；发现并重编陈旧的 `openarmx_integrated_moveit_msgs` 安装产物后，`CartesianApproach` 可导入。Qwen3.5-4B 与 SAM2 均在完全离线模式下以 BF16 成功加载到 GPU。通过时间：2026-08-12 14:02 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`、`/tmp/openflex_p4_perception_install.log`、`/tmp/openflex_p4_msgs_rebuild.log`。
- [x] **P4-06 创建 GraspNet 环境**：已创建纯净隔离环境 `/home/openflex/old/.venvs/openflex_graspnet`，锁定 Python 3.10.12、torch 2.7.1+cu126、CUDA 12.6、numpy 1.26.4、Open3D 0.19.0，`pip check` 无错误。上游 `graspnetAPI 1.2.11` 错误钉死不支持 Python 3.10 的 `numpy==1.20.3`，且生产后端不导入该 API，故从 runtime lock 排除并保留显式说明。已将 NVIDIA CUDA 12.6.85 编译组件解包到用户目录，不修改系统驱动；PointNet2/KNN 均从当前源码以 `TORCH_CUDA_ARCH_LIST=8.6` 现场构建，产物位于新 venv，包含 `sm_86`，CUDA 单元测试通过。正式 `checkpoint-rs.tar` epoch 18 加载成功，合成场景推理得到 506 个有限的 `(N,17)` 候选。通过时间：2026-08-12 14:35 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`、`/tmp/openflex_p4_pointnet2_build.log`、`/tmp/openflex_p4_knn_build.log`、`/tmp/openflex_p4_graspnet_checkpoint_smoke.log`。
- [x] **P4-07 固定解释器与模型参数**：已将感知入口固定为 `OPENFLEX_PERCEPTION_PYTHON`（默认 `/home/openflex/old/.venvs/openflex_perception/bin/python`），GraspNet 入口固定为 `OPENFLEX_GRASPNET_PYTHON`（默认 `/home/openflex/old/.venvs/openflex_graspnet/bin/python`）；控制执行器、MoveIt bridge 和 Web 调试节点使用 `OPENFLEX_ROS_PYTHON`，默认仍为系统 `python3`。模型根统一为 `/home/openflex/old/openflex_models`，Qwen 默认改为 `Qwen3.5-4B`，GraspNet checkpoint 默认改为模型根下的正式资产，活动代码和已安装包中不再有 9B/旧解释器/源 workspace 默认值。按依赖顺序重建 `openflex_grasp_moveit` 与 `openflex_grasp` 成功。通过时间：2026-08-12 14:43 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`、`/tmp/openflex_p4_grasp_packages_rebuild.log`。
- [x] **P4-08 验证常驻显存策略**：已同时常驻 perception worker（Qwen3.5-4B+SAM2，BF16）与 GraspNet worker（FP32），分别使用隔离解释器且每种模型只实例化 1 次。`nvidia-smi` 实测两个 PID 分别占用 9694 MiB、1340 MiB，整卡仍余 13065 MiB，超过 2 GB 门槛。通过时间：2026-08-12 14:47 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`。
- [x] **P4-09 完成模型环境烟测**：同机同时加载三套模型并走生产类真实推理。Qwen 在固定货架图中返回 6 个“农夫山泉”实例框；SAM2 对 6 个框均输出 `[374,541]` 非空 mask，分数 0.9453 至 0.9648；GraspNet 对官方 RGB-D 样例的 513688 个有效点推理得到 326 个有限 `float32 (N,17)` 候选。无 CUDA 或扩展错误。完整 20 轮 Detect/Pick 常驻联测仍留到 P9-09。通过时间：2026-08-12 14:47 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`。
- [x] **G4 模型环境关卡通过**：两个隔离环境、三套模型、单轮真实推理和同时加载显存余量全部通过。按已确认决定，本关不做缺失数据集的精度验收，精度验收明确后置；20 轮常驻联测留到 P9-09。通过时间：2026-08-12 14:47 CST。证据：`migration_records/2026-08-12/P4_GPU_MODEL_ENV_PROGRESS_CN.md`。

## 6. 感知重构与 `/detect_object`（D3、D9、D10）

- [x] **P5-01 整理核心模块边界**：已保留 `openflex_grasp` 包名并拆分 `perception/providers`、`sam2_segmenter`、`rgbd`、`masks`、`pointcloud`、`cache`、`visualization`、`io` 和兼容 pipeline；ROS/离线入口只依赖边界模块，控制执行模块不导入模型实现。兼容 `models.py`/`geometry.py` 仅重导出旧 API。增量构建成功，独立快速测试 8 项通过。通过时间：2026-08-12 15:01 CST。证据：`migration_records/2026-08-12/P5_PERCEPTION_PROGRESS_CN.md`。
- [x] **P5-02 定义进程内 `BoxProvider`**：已定义冻结 RGB、中文 SKU、可选英文提示、max_boxes、参数、request_id 输入和统一 BoxResponse 输出；11 项快速契约检查通过。证据：`migration_records/2026-08-12/P5_PERCEPTION_PROGRESS_CN.md`。
- [x] **P5-03 实现 Qwen provider**：`qwen_local` 已使用 Qwen3.5-4B，提示词为中文 SKU + 可选 OWLv2 英文描述；严格 JSON、多实例、超时和异常均返回确定 ProviderError，不自动回退。证据：同上及 P4-09。
- [x] **P5-04 建立独立 scene 适配包**：已创建 `openflex_grasp_supermarket_scene_adapter`，仅该包依赖 `supermarket_scene_interfaces`/`FindSku`；核心扫描无数据库、Web、robot_service import。增量构建通过。
- [x] **P5-05 实现 OWLv2/DINO 规则**：OWLv2 adapter 显式调用 `/find_sku`；DINO 仅由 OWLv2 参数开启，Qwen UI 禁用 DINO，服务不可用直接报错。
- [x] **P5-06 实现双腕 RGB-D router**：已同时订阅左右 topic，按 arm 冻结校验 serial、stamp、frame、深度尺度和新鲜度；过期/错侧帧拒绝。
- [x] **P5-07 实现 SAM2 与商品点云阶段**：已实现多框 SAM2 mask、清理、同帧深度投影和质量指标；无效深度/点数不足/尺度异常返回稳定错误。
- [x] **P5-08 实现实例选择与 `object_pose`**：已按语义、SAM、几何质量选择单实例并生成 frame/stamp 正确 PoseStamped；Detect 代码不导入 GraspNet。
- [x] **P5-09 实现 detection store 与内部读取契约**：已新增 `openflex_grasp_interfaces/GetDetectionSnapshot`，UUID、TTL、单消费者、过期回收和并发串行保护均实现，大数据只在内部 service。
- [x] **P5-10 实现真实 `/detect_object` server**：已整合 Qwen、SAM2、双腕 RGB-D、点云、pose、缓存、feedback、取消和超时；Qwen 失败不回退。
- [x] **P5-11 固化 `supermarket_scene` 分层依赖**：已添加 `dependencies.repos` 与部署说明，核心不硬编码数据库/scene root。
- [ ] **P5-12 完成离线与仿真检测测试**：**跳过**；负责人要求跳过不必要仿真/测试，保留现场或后续离线数据验收。
- [ ] **G5 检测关卡通过**：**待现场**；当前无左右腕实时 RGB-D 数据，不伪造左右臂检测 pose 验收。

## 7. GraspNet worker 与真实 `/pick_object`（D2、D4）

- [x] **P6-01 定义内部 GraspNet 类型契约**：已新增可取消 `FitGrasp.action`，仅携 detection_id、点云、arm、策略和参数，不重复 MoveIt/Qwen JSON。
- [x] **P6-02 实现单一常驻 GraspNet worker**：`graspnet_worker` 使用 GraspNet venv 单实例，左右请求串行；不使用 AnyGrasp。
- [x] **P6-03 接入 detection store**：Pick 按 UUID 读取、校验 object/arm，并在拟合成功后单消费者 consume；无最近检测回退。
- [x] **P6-04 提取统一 `ArmContext`**：左右 ArmContext 已统一 planning group、TCP、相机、夹爪 Action、进给/闭合轴、关节和 payload，并用于新 Pick/GraspNet worker。旧 `product_grasp_executor` 被确认仅属于未由比赛 launch 启动的历史链，保留原因和移除条件已记录。证据：`migration_records/2026-08-12/P6_GRASPNET_PICK_PROGRESS_CN.md`。
- [x] **P6-05 适配预抓取和 Cartesian 进给**：新 Pick server 已调用统一 PlanPregrasp/CartesianApproach，默认 plan-only，不导航底盘。
- [x] **P6-06 适配左右夹爪与负载**：执行分支已按左右夹爪 Action、位置/stall 证据判定 attached；本轮 execute=false。
- [x] **P6-07 实现安全回撤**：执行分支在成功进给后回撤到 pregrasp；取消/失败不会进入运输。
- [x] **P6-08 实现真实 `/pick_object` server**：已整合缓存、GraspNet、MoveIt、Cartesian、夹爪、附着判定与错误反馈；默认 execute=false。
- [ ] **P6-09 完成纯逻辑与仿真测试**：**跳过**，按负责人要求不做不必要测试/仿真。
- [x] **P6-10 右臂 plan-only 验收**：现场使用右腕 D405 `260522279851` 完成 Qwen3.5-4B -> SAM2 -> 同帧 RGB-D 点云 -> GraspNetBaseline -> PlanPregrasp -> CartesianApproach 全链。最终 `/pick_object` 返回 `success=true`、`error_code=0`、`stage=1`、夹爪需求宽度 `0.057 m`、GraspNet 候选数 `49`；`execute=false`、`object_attached=false`，未发送关节、夹爪或底盘命令。按负责人决定关闭 GraspNet 前置点云走廊计数，MoveIt/Cartesian 碰撞与路径检查继续启用。证据：`migration_records/2026-08-12/P6_RIGHT_PLAN_ONLY_LIVE_CN.md`。
- [ ] **P6-11 左臂 plan-only 验收**：**待现场**；本轮未切换左腕相机和左臂链，不以右臂结果代替左臂验收。
- [ ] **G6 抓取软件关卡通过**：**待现场**；右臂 plan-only 已通过，仍缺左臂 plan-only 证据，且 `execute=true` 真机动作继续禁止由本轮代测。

## 8. 启动、配置和唯一 owner

- [x] **P7-01 新增 `grasp_debug.launch.py`**：已加入双相机可选接入、共享 Detect/GraspNet/Pick、Cartesian 和只读 Web；默认相机关闭、execute=false。
- [x] **P7-02 接入正式 fullbody launch**：已由 `supermarket_fullbody_runtime.launch.py` 增加抓取 backend 开关和 execute 参数；关闭 competition mock pick，避免重复 owner。
- [x] **P7-03 实现正式/调试互斥**：Detect、GraspNet、Pick 节点已分别持有非阻塞进程 owner lock；调试与 fullbody 第二套 backend 会在创建 ROS 接口前失败关闭，不再产生重复 Action/Service owner。证据：`migration_records/2026-08-12/P7_P8_RUNTIME_GUI_PROGRESS_CN.md`。
- [x] **P7-04 完成启动健康检查**：已提供模型 manifest/owner 检查 helper，并在 GUI 显示 Detect/Pick backend 状态；实时 TF/serial 仍待现场。
- [x] **P7-05 更新 test_08 双臂配置**：已把货架动态 slot 改为显式 `graspnet_shelf`，加入左右臂策略；左臂关节示教明确后置。
- [x] **P7-06 保留 mock place 边界**：现有 mock place 保留为明确 mock，GUI/backend 标签显示放置未实现。
- [x] **P7-07 修复 Cartesian Action 启动漏项**：现场“仅规划”已通过 PlanPregrasp 并进入第二段后报 `CartesianApproach Action unavailable`；确认 fullbody 只启动 MoveIt/PlanPregrasp，未包含 `openflex_grasp_moveit/cartesian_approach_executor.launch.py`。现已将其作为 `start_cartesian_approach`（默认跟随 `start_grasp_backend`）接入正式 fullbody，补齐 `openflex_grasp`/`openflex_grasp_moveit` manifest 运行依赖；GUI 同时显示 Detect/Pick/Pregrasp/Cartesian 四项状态，后两项离线时禁止“仅规划”并提前报出准确端点。两个受影响包构建成功。当前真实进程未强制重启，生效需下次正常重启。通过时间：2026-08-13 11:04 CST。
- [ ] **G7 启动关卡通过**：**待现场**，不启动真实 runtime 验证 owner。

## 9. 抓取调试页与比赛控制台（D5）

- [x] **P8-01 增加“抓取调试”顶层 tab**：比赛控制台已嵌入抓取调试页，显示双臂、provider、SKU/英文提示、worker 和 backend 状态。
- [x] **P8-02 保持 Web 只读**：抓取 Web 仍是只读可视化；GUI ROS client 才能提交 Action。
- [x] **P8-03 增加调试 ROS clients**：已加入仅检测、仅规划、取消和错误状态；启动/停止 backend 仍由 launch owner 管理，execute 控件禁用。
- [x] **P8-04 实现 provider 动态控件规则**：默认 Qwen；OWLv2 手工选；DINO 只在 OWLv2 启用；Qwen 失败不切换。
- [ ] **P8-05 锁定正式任务参数**：**✗ 未通过**；调试页执行控件已锁定，但正式 `/execute_transport` 任务运行状态尚未接入该页，无法可靠锁定 SKU/arm/provider 参数。重新执行条件：先完成 P8-06 并确定 transport goal 与 provider 配置的所有权。证据：`migration_records/2026-08-12/P7_P8_RUNTIME_GUI_PROGRESS_CN.md`。
- [ ] **P8-06 接入正式 `/execute_transport` client**：**✗ 未通过**；比赛流程页仍调用历史手动/mock harness，当前没有真实 ExecuteTransport ActionClient。为避免误启动完整导航/抓取链，本轮未在缺少现场接口验收时仓促接线。证据：`migration_records/2026-08-12/P7_P8_RUNTIME_GUI_PROGRESS_CN.md`。
- [x] **P8-07 明确 backend 标签**：页面持续显示 real detect/real pick/mock place，放置未实现。
- [ ] **P8-08 完成 GUI 自动测试**：**跳过**，负责人要求跳过非必要测试。
- [x] **P8-09 接入抓取调试可视化数据**：`/detect_object` 成功后通过只读、transient-local 的 `/openflex_grasp/debug/*` topic 发布冻结原图、分割叠图、mask、降采样商品点云、降采样场景点云、商品位姿和采集元数据；内部 detection snapshot 与 GraspNet 输入仍使用完整点云。调试发布异常只记录 warning，不改变正式 Detect 成败。通过时间：2026-08-13 10:30 CST。证据：`migration_records/2026-08-13/P8_GRASP_VISUALIZATION_CN.md`。
- [x] **P8-10 显示 GraspNet 与规划结果**：`/pick_object` 发布只读 object/pregrasp/grasp pose、夹爪宽度、候选统计、pregrasp-to-grasp 路径和 plan-only 状态；Web 以三组不同标记的坐标轴与数值显示。正式 `PickObject.action` 契约未膨胀，`execute_grasp` 默认仍为 `false`。通过时间：2026-08-13 10:30 CST。证据：同上。
- [x] **P8-11 嵌入比赛控制台并自动启动 Web 后端**：抓取调试页改为左侧控制、右侧嵌入 `http://127.0.0.1:8765/`，支持后端晚启动自动重连；fullbody launch 在抓取 backend 启用时启动唯一只读 Web 服务。目标三包构建成功，隔离端口合成数据验收及 1440x1000 离屏截图确认图像、点云、路径和位姿非空。通过时间：2026-08-13 10:30 CST。证据：同上。
- [x] **P8-12 增加独立真机执行阶段**：抓取调试页改用 `/plan_pick` 生成 120 秒、单次使用的缓存计划，并新增“真机执行解锁”和“执行抓取”按钮调用 `/execute_pick`。执行前重新验证商品 mask/深度、底盘位姿和关节起点；初版契约允许 `ARM_ONLY=7`/`ARM_WITH_LIFT=8`，后由 P8-12C 收紧为抓取阶段仅 `ARM_ONLY=7`，拒绝底盘和升降柱运动。取消会向当前 MoveIt/夹爪子 Action 传播；新检测或修改 SKU/手臂/provider 会清空旧计划。服务端脚本执行位和 fullbody/grasp_debug launch 已补齐，`/plan_pick`、`/execute_pick` 只读 readiness 验证通过；本轮未发送执行 goal。通过时间：2026-08-13 11:20 CST。
- [x] **P8-12A 修正规划阶段的笛卡尔衔接容差**：独立 `/plan_pick` 原先将 PlanPregrasp 终点到直线进给起点的总位置/姿态误差写死为 `10 mm / 5 deg`，现场 `12.558 mm` 跟踪差导致误拒绝。现改为可配置参数，默认与既有验证链路一致为 `40 mm / 20 deg`；严格直线模式仍单独限制横向偏差不超过 `5 mm`，路径比例仍要求 `>= 0.99`，且不放宽底盘及执行起点复核。通过时间：2026-08-13。
- [x] **P8-12B 固化比赛启动的双臂 CSP/夹爪 MIT 混合模式**：V10 硬件插件新增独立 `gripper_control_mode`，激活和周期写命令分别按手臂、夹爪模式处理；比赛 fullbody 默认显式传入 `control_mode=csp`、`gripper_control_mode=mit`。左右手臂 1-7 关节走 CSP 位置参考，左右夹爪保留 MIT KP/KD、堵转保持及释放逻辑；上电仍以实测位置初始化，不自动回零。未显式配置夹爪模式的旧入口继续继承 `control_mode`。通过时间：2026-08-13。证据：混合模式和旧参数继承两组 xacro 展开检查通过，6 个相关包构建通过；未重启当前真机控制链。
- [x] **P8-12C 抓取阶段固定手臂-only**：抓取调试 GUI 与 `/plan_pick` 统一使用 `ARM_ONLY=7`；服务端拒绝其它最大自由度，向 `/openflex/plan_pregrasp` 强制发送 7，并只接受 `STAGE_A`。若返回轨迹包含 `lift_joint` 或其它非手臂阶段，规划立即失败；旧 `/pick_object` 路径同步固定为手臂-only。升降柱只能在抓取前导航/货架精停阶段调整。通过时间：2026-08-13；未发送执行 goal。
- [x] **P8-12D 增加手臂起始状态限位诊断**：现场 ARM_ONLY 规划失败定位为 `openarmx_right_joint4=1.97771 rad`，明显超过活动 URDF/MoveIt 上限 `1.8 rad`，不是底盘或升降柱参与规划。`/openflex/plan_pregrasp` 现会在调用规划器前逐项校验所选手臂 7 个关节，并向 GUI 返回关节名、实测值和上下限；仅 `1e-6` 以内的浮点误差可在规划副本中归一化，明显越界直接拒绝，禁止用假起点或放宽全局限位绕过。ARM_ONLY 无解文案也已明确说明底盘与升降柱 fallback 均关闭。目标包构建通过；未重启当前真机进程、未发送执行 goal。通过时间：2026-08-13。
- [x] **P8-12E 保留检测时刻 TF 和失败位姿诊断**：GraspNet 推理超过 tf2 默认 10 秒缓存时，原实现会在规划前丢失冻结 RGB-D 帧对应的 `right_camera_color_optical_frame -> odom` 历史变换，现场复现为 past extrapolation。抓取序列和兼容 Pick 节点的 TF 缓存统一扩到 240 秒；GraspNet 生成的 object/pregrasp/grasp 位姿在 MoveIt 验证前即发布到调试页，规划失败仍保留位姿与失败阶段，便于区分 TF、IK、碰撞和路径失败。未调用 `/execute_pick`。通过时间：2026-08-13。
- [ ] **P8-12F 直接采用 GraspNet 闭合轴拟合结果**：**✗ 未采用**。现场实验后商品点云投影所需开口由约 `0.061 m` 增至 `0.118 m`，超过夹爪 `0.086 m` 安全上限。已恢复固定货架闭合轴和宽度检查；可达性改由 P8-12G 的受限 MoveIt 姿态窗口解决，不放宽夹爪宽度、底盘或升降柱。
- [x] **P8-12G 限制 ARM_ONLY 末端姿态搜索窗口**：固定底盘/立柱时，精确 GraspNet 姿态可能因腕部滚转而没有 IK；同一目标位置的当前 TCP 姿态在小角度窗口内可达。`right_arm`/`left_arm` 的预抓取目标设置 `3 mm` 总位置容差、每个旋转约束轴 `0.20 rad` 姿态容差。位置窗口严格小于 CartesianApproach 的 `5 mm` 横向门槛，避免预抓取成功后因横向偏移被下一阶段拒绝；MoveIt 姿态按轴应用，三轴合成误差仍受后续 `20°` 起点门槛约束。仅允许手臂在目标附近寻找解，不改变目标中心、不增加 lift/base 自由度。2026-08-13 右相机“水蜜桃”重新检测后，现场 plan-only 完整通过至 `PLAN_READY`，返回 `arm=right max_dof=7`；未发送执行 goal。
- [x] **P8-12H 回撤复用已验证进给路径**：独立重新求解 grasp-to-pregrasp Cartesian 路径会因数值 IK 分支差异产生不稳定的部分路径；回撤现使用已通过碰撞、关节限位和直线进给检查的 approach 轨迹做严格时间反向，速度符号反转、加速度保持，并拒绝非标准顺序、非 7 轴、包含 `lift_joint` 或底盘多自由度的轨迹。消息级测试 `2 passed`，同一现场 plan-only 完整通过 `PLAN_RETREAT -> PLAN_READY`；未发送执行 goal。
- [✗] **P8-12I 原迁移版过严分段收敛握手**：原迁移版使用 `0.0075 rad / 3 s` 的额外末端门槛，现场导致 MoveIt 预期时长约 `2.44 s` 时返回 `code -6`。已按桌面工作空间基线回滚该额外门槛，改用控制器级近端 `0.030 rad`、远端 `0.050 rad`、`8 s` 收敛等待；MoveIt 配置同步 `allowed_start_tolerance=0.0`、关闭 expected-duration monitoring，并保留必要物理关节限位、独占发布者及底盘/立柱约束。待重启 `move_group` 后验证。
- [x] **P8-12J 对齐桌面版关节边界量化处理**：目标版新增的 `1e-6 rad` 起始限位拒绝比桌面 MoveIt 的边界修复严格过多，现场右 J4 为 `1.800098 rad`，仅超上限 `0.000098 rad`。规划副本数值归一化阈值调整为 `0.001 rad`，仍远小于 MoveIt 默认 `0.1 rad`；只夹紧规划副本，不修改实际反馈、URDF 硬限位或控制器目标，超过 `0.001 rad` 继续拒绝。2026-08-13 以右腕相机检测“矿泉水”成功（Qwen -> SAM2 -> 点云，置信度 `0.754`），随后 GraspNet 生成 `100` 个候选，7 轴右臂 plan-only 完整通过 `PLAN_PREGRASP -> PLAN_APPROACH -> PLAN_RETREAT -> PLAN_READY`，返回 `arm=right max_dof=7`；未发送 `/execute_pick` goal。
- [x] **P8-12K 禁止控制台与手工抓取栈并存**：现场 `MoveIt code -4` 并非 GUI 显示的发送前起点偏差 `0.000384 rad` 所致，而是手工调试遗留栈与控制台 fullbody runtime 同时提供两套 `/move_group`、`/moveit_command_bridge` 和 `/openflex/plan_pregrasp`。同一轨迹同时到达两个同名 Action server，日志明确出现 `Rejecting overlapping right MoveIt goal`、`unknown goal response`，最终轨迹被拒绝；此前多轮执行还产生了重复开/闭夹爪命令。已停止全部手工遗留节点，并在控制台外层启动脚本及 GUI“启动全身控制”入口同时增加 MoveIt、控制桥、规划器、GraspNet worker、抓取序列节点冲突检查，发现旧栈即拒绝启动。相关 GUI 包构建和 Python/Bash 语法检查通过；系统环境缺少 `pytest` 命令，新增定向单测未运行。通过时间：2026-08-13。
- [x] **P8-12L 抓取终点沿进给方向增加 5 cm**：常驻 GraspNet worker 新增 `grasp_depth_offset_m` 参数并默认设为 `0.05 m`，比赛 fullbody 与独立抓取调试 launch 显式使用同一默认值。`pregrasp_distance_m=0.10 m` 保持不变，因此预抓取点不移动，最终抓取点从商品表面中心沿 approach 方向额外前移 `0.05 m`，直线进给总长度相应由约 `0.10 m` 增至约 `0.15 m`。已完成代码、构建和几何测试；未启动真机、未发送运动目标。通过时间：2026-08-13。
- [x] **P8-12M 抓取后返回检测拍照姿态**：新抓取序列原先只将直线进给轨迹反向回撤到预抓取位，没有继续回到检测拍照时的手臂姿态。规划阶段现将已通过 MoveIt 验证的“拍照位→预抓取”7 轴轨迹严格时间反向并随计划缓存；执行阶段在“抓取→安全回撤到预抓取”后继续执行“预抓取→拍照位”。返回段只包含所选手臂 7 个关节，底盘和升降柱不参与；若在预抓取位开夹爪等步骤失败，也尝试返回拍照位，若进给轨迹中途失败而当前位置不确定则不盲目套用返回轨迹。已完成代码、构建和消息级测试；未启动真机、未发送运动目标。通过时间：2026-08-13。
- [ ] **P8-13 重启后现场显示与执行验收**：下次正常启动 fullbody/GUI 后，用右腕检测“矿泉水”，核对原图、mask/叠图、真实点云和三组真实位姿，再由负责人现场确认急停和工作区后完成单次低速执行。**待现场，不以接口 readiness 代替真机通过。**
- [ ] **G8 控制台关卡通过**：**待现场**。

## 10. 集成、仿真与真机验收

> 执行范围调整（2026-08-12，负责人指示）：本轮只执行 P9-01、P9-02，之后直接进入
> P10。P9-03 至 P9-10（含所有仿真、现场安全、真机、20 轮联测和比赛演练）不由本轮
> 自动执行，保持未勾选并交由负责人现场验收；G9 不得在缺少这些证据时标记通过。

- [ ] **P9-01 全 workspace 干净构建**：**跳过**；只完成目标包增量构建，旧 build 环境变量仍可能保留历史 AnyGrasp 字符串，未作全 workspace 清理。
- [x] **P9-02 ROS 契约验收**：已 introspect `FitGrasp`/`GetDetectionSnapshot`，并检查 Detect/Pick 的 `detection_id`、`arm_name`、TaskError、取消语义；目标包和 GUI/TaskExecutor 构建成功。
- [ ] **P9-03 仿真 Detect/Pick 验收**：左右臂分别完成 Detect -> Pick -> 安全回撤，验证 TF、碰撞、夹爪附着和失败路径。验收：Place 仅使用明确 mock。
- [ ] **P9-04 完整编排联调**：在 mock place 模式执行 `/execute_transport`，覆盖导航/精停、检测、抓取、底盘退避、运输和 mock place。验收：该结果只标记“编排通过”，不标记真机放置通过。
- [ ] **P9-05 现场安全准备**：确认急停、速度/加速度上限、碰撞物、工作区清空、观察人员和单次执行授权。验收：未满足任何一项都不得开启 `execute=true`。
- [ ] **P9-06 右臂低速单次抓取**：固定商品和站位，依次验证检测、预抓取、Cartesian、夹持证据和安全回撤。验收：保存 rosbag、轨迹、TF、模型输出和视频。
- [ ] **P9-07 左臂低速单次抓取**：按同一标准完成左臂，不降低验收门槛。验收：保存与右臂同等证据。
- [ ] **P9-08 双臂重复性与失败恢复**：在可用验收商品上分别重复执行，覆盖无框、坏深度、TF 丢失、规划失败、取消和夹持失败。验收：失败后系统回到可再次请求的安全状态。
- [ ] **P9-09 完成 20 轮常驻联测和性能报告**：连续执行至少 20 轮 Detect -> Pick 模型推理，汇总 Qwen/SAM2/GraspNet 加载时间、单轮延迟、显存峰值/增长、成功率、失败分布和左右差异。验收：无 OOM、模型重载或持续显存增长；加载后至少保留 2 GB 显存。缺少正式 RGB-D 数据集的精度指标必须标记“未验收”，不得推断为通过。
- [ ] **P9-10 比赛模式演练**：用正式 fullbody owner 和控制台完成多轮真实 Detect/Pick + mock place 流程，验证进程互斥、取消、重启和日志。验收：无资源重复 owner、模型重载或状态串线。
- [ ] **G9 本阶段发布关卡通过**：**跳过全部真机/仿真验收**，由负责人现场执行；本轮不发布真机通过声明。

## 11. 文档、交付与回滚记录

- [x] **P10-01 更新运行文档**：已记录目标路径、source 顺序、两个 venv、模型 manifest、launch 参数、左右相机 serial、provider 规则、owner 锁和控制台调试操作。证据：`src/openflex_grasp/RUNTIME_GUIDE_CN.md`。
- [x] **P10-02 更新接口文档**：已记录 Detect/Pick 字段、`detection_id` TTL/单消费者生命周期、错误码、ArmContext、内部 snapshot/FitGrasp 契约、execute=false 和 Place 后置边界。证据：`src/openflex_grasp/openflex_grasp_interfaces/README_CN.md`。
- [x] **P10-03 更新包结构和依赖图**：已记录核心抓取、scene adapter、MoveIt、task executor、GUI、两个模型环境和外部 `supermarket_scene` 的所有权/依赖方向。证据：`src/openflex_grasp/ARCHITECTURE_CN.md`。
- [x] **P10-04 记录变更与恢复点**：已记录阶段证据、模型 SHA256 入口、外部依赖、缓存备份位置、恢复限制和已知问题；明确回滚不得恢复 AnyGrasp 为比赛 backend。证据：`migration_records/2026-08-12/P10_DELIVERY_REVIEW_CN.md`。
- [x] **P10-05 关闭临时兼容项**：活动代码/launch 无源 workspace 绝对路径、Qwen 9B 默认值或 AnyGrasp backend；新增跨 launch owner lock。旧 `/product_perception/*` 仅保留在不由比赛 launch 启动的历史链，原因和移除条件已记录。证据：`migration_records/2026-08-12/P10_DELIVERY_REVIEW_CN.md`。
- [x] **P10-06 最终审查**：已对照 D0-D10 逐项核对；真机/仿真/20 轮联测和 Place 均保持未勾选，P8-05/P8-06 明确记录未通过原因，没有把 mock 或静态检查误标为现场通过。证据：`migration_records/2026-08-12/P10_DELIVERY_REVIEW_CN.md`。
- [ ] **G10 项目负责人验收**：提交最终报告供负责人审阅；只有负责人明确接受后，本阶段标记完成。

## 12. 明确后置：真机放置（D8）

- [ ] **F-01 保持后置状态**：本次不实现交付台放置、货架插入、真机释放轨迹或 `object_released` 真实判据。
- [ ] **F-02 建立后续议题**：未来单独讨论 Place 目标表达、交付台/货架范围、左右臂策略、approach/retreat、碰撞场景、释放证据和验收数据。
- [ ] **F-03 禁止错误宣传**：在 F-02 完成设计并通过真机验收前，任何 mock place 结果都不得作为完整比赛搬运能力。

## 13. 执行记录模板

每个失败条目使用以下格式直接追加在原任务后：

```text
- [ ] **P?-?? 任务名称**：... **✗ 未通过**
  - 失败时间：YYYY-MM-DD HH:MM
  - 失败原因：
  - 日志/证据：
  - 已产生改动：
  - 是否已恢复到安全状态：
  - 影响的后续条目：
  - 重新执行条件：
```

通过条目使用以下格式：

```text
- [x] **P?-?? 任务名称**：...
  - 通过时间：YYYY-MM-DD HH:MM
  - 测试/证据：
  - 相关提交或补丁：
```
