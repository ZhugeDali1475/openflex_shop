# OpenFlex 硕士论文项目索引

更新日期：2026-11-04  
工作区：`/home/wgb/openflex_shop`  
论文主方法：CARE-Agent  
仿真主线：MuJoCo + ROS 2 Humble + MoveIt 2 + Nav2

## 1. 先看什么

第一次进入项目时，按以下顺序阅读：

1. [当前状态](IMPLEMENTATION_STATUS_2026-11-04.md)：已经完成什么、目前卡在哪里。
2. [论文任务计划](THESIS_TASK_PLAN_CARE_AGENT_CN.md)：论文题目、研究问题、算法、实验和写作安排。
3. [MuJoCo 实验手册](MUJOCO_THESIS_EXPERIMENT_RUNBOOK_CN.md)：从环境检查到正式统计实验的逐级操作步骤。
4. [脚本说明](scripts/README_CN.md)：项目内可用脚本及其用途。

如果你对整个课题还不熟悉，不要从这里直接跳去接大模型或写论文。先阅读第 8 节的
执行顺序，再按照第 9-16 节逐项完成。当前详细研究假设、指标和实验协议仍以
[论文任务计划](THESIS_TASK_PLAN_CARE_AGENT_CN.md)为准。

## 2. 当前结论

项目已经完成 MuJoCo 数字实验平台、MoveIt 基础执行、TC-WAM
候选评价、CARE-Agent 离线决策内核和 G0-G3 基线验收。

G3 已通过：新的左臂停姿
`[0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]` 已在 6 个真实右臂候选和
`-0.15/0/+0.15 m` 升降范围内通过机器人自身与仿真货架碰撞检查。

当前下一道阻塞是 G4：固定底盘单次抓取、夹爪闭合和独立附着验证。不能通过关闭
碰撞检查或放宽成功标准制造成功率。

## 3. 目录职责

| 路径 | 用途 | 是否论文主线 |
|---|---|---|
| `openflex_ws/src/openflex_thesis/` | 论文新增 ROS 包 | 是 |
| `openflex_ws/src/openflex_integrated/` | MoveIt、任务执行和系统集成 | 是 |
| `openflex_mujoco-1.0_basic/` | 整机 MuJoCo 模型与转换工具 | 是 |
| `scripts/` | 环境检查、MoveIt 验证、实验入口 | 是 |
| `openflex_calibration/` | 手眼与标定数据 | 是 |
| `openflex_models/` | GraspNet、SAM2、Qwen 权重 | 后续感知实验 |
| `.venvs/` | 感知环境 | 后续感知实验，不参与 ROS 构建 |
| `supermarket_scene/` | 已有商超可视化与业务系统 | 可作为场景素材，不作为论文算法核心 |
| `openflex_maps/` | 实测地图和点云 | Nav2/场景扩展 |
| `supermarket_scene-backups/` | 历史数据库备份 | 保留，不参与运行 |
| `openflex_drivers/` | 真机驱动与安装包 | 仿真实验不需要 |
| `openflex_vr_apk/` | VR 应用 | 与当前论文主线无关 |

## 4. 论文核心代码

| ROS 包 | 作用 |
|---|---|
| `openflex_care_agent` | 信念状态、技能契约、安全 Critic、故障恢复决策 |
| `openflex_tc_wam` | 对象记忆、抓取候选评价、联合 IK/碰撞/风险评分 |
| `openflex_thesis_control` | 底盘控制仲裁、局部对位、MoveIt 抓取与放置监督 |
| `openflex_thesis_mujoco` | MuJoCo 仿真桥、RGB-D、Oracle 感知和导航基线 |
| `openflex_thesis_experiments` | trial manifest、预检、日志和统计 |
| `openflex_thesis_msgs` | 论文 Action、Topic 和 Service 接口 |

## 5. 标准工作空间

只使用以下隔离构建空间：

```text
openflex_ws/build_v2
openflex_ws/install_v2
openflex_ws/log_v2
```

不要 source 旧的 `openflex_ws/install`，也不要使用 `/home/openflex/...`
或 `/home/openarmx/...` 的旧安装空间。

## 6. 当前硬件约束

| 设备 | 配置 |
|---|---|
| CPU | Intel i7-14700F，20 核 28 线程 |
| GPU | RTX 4060 Laptop，8 GiB |
| 内存 | 16 GiB |
| 系统 | Ubuntu 22.04.5 + ROS 2 Humble |

16 GiB 内存是完整系统的瓶颈。正式批量实验应关闭 RViz、浏览器中的大型页面和
其他常驻程序，并让 Qwen、SAM2、GraspNet、MuJoCo 尽量分时运行。

## 7. 数据与可再生成文件

必须保留：

- ROS/论文源码；
- MuJoCo 模型和网格；
- 模型权重；
- 标定文件；
- 地图；
- 实验 manifest、逐 trial 日志和统计结果；
- 代表性失败截图或 rosbag。

可以删除：

- `__pycache__`；
- `.pytest_cache`；
- colcon 历史日志；
- 项目级 npm cache；
- 已确认解压且不再引用的源码压缩包。

清理命令：

```bash
bash /home/wgb/openflex_shop/scripts/clean_generated_artifacts.sh
```

该脚本不会删除 `build_v2`、`install_v2`、模型、地图、数据库或实验数据。

## 8. 正确的建设顺序

### 8.1 是“先做 Agent 再测试”，还是先做基线

不是先写一个完整的 Agent，然后一次性放进 MuJoCo 验收。正确方式是先建立可信的
技能执行基线，再让 Agent 从最小闭环开始逐步进入仿真，每增加一个能力就测试一次。

原因是：如果导航、MoveIt、夹持验证和放置本身不稳定，Agent 无法区分“决策错误”
和“底层执行错误”，实验数据也没有意义。

正确顺序是：

```text
MuJoCo/ROS/MoveIt 环境可复现
  -> 固定状态机的单次抓放成功
  -> 重复试验和日志可复现
  -> Agent 读取真实状态并只做一次高层决策
  -> Agent 调用一个技能并验证后置条件
  -> 完整正常流程闭环
  -> 故障注入和规则恢复
  -> TC-WAM 能力约束
  -> 数据驱动恢复
  -> 对照、消融和正式统计实验
  -> 论文写作与复现包
```

### 8.2 当前不能跳过的阻塞

当前必须先完成：

1. 解决右臂抓取时升降柱与未使用左臂 `openarmx_left_link3` 的联合碰撞。
2. 通过固定底盘单次抓取，要求夹爪闭合、物体附着和运输姿态验证同时成立。
3. 完成固定放置、释放和放置验证。
4. 连续 10 次重复，如实保存成功和失败样本。

在这四步完成前，Agent 可以继续做离线单元测试，但不能把 MuJoCo Agent 闭环结果
作为论文主实验。不能通过关闭碰撞检查或放宽成功标准跨过这些门槛。

### 8.3 每次只做一个可验收切片

建议每次工作都采用同一闭环：

```text
写一个最小功能
  -> 加一个单元测试
  -> 在 MuJoCo 中跑一个确定性场景
  -> 保存日志和截图
  -> 更新状态文档
  -> 再进入下一功能
```

不要同时开发感知、LLM、恢复策略、导航和论文图片；否则出错时无法定位。

## 9. CARE-Agent 的具体搭建路线

### 9.1 已有实现

`openflex_care_agent` 已经是 ROS-independent 决策内核，不是空目录。当前包含：

- [BeliefState](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/belief.py)：
  任务、阶段、机器人状态、目标状态、能力状态和失败历史；
- [SkillContract](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/skills.py)：
  技能前置条件、后置条件、控制权、超时和静态技能图；
- [SafetyCritic](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/safety.py)：
  确定性安全前置条件检查；
- [FailureCode](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/failure.py)：
  结构化故障语义；
- [RecoveryPolicy](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/recovery.py)：
  按成功概率、信息增益、时间、距离和风险选择恢复技能；
- [CAREAgent](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/agent.py)：
  正常阶段决策和失败恢复入口。

2026-11-04 实际运行该包的单元测试，结果为 `9 passed`。因此下一步不是重写内核，
而是补 ROS 适配和闭环证据。

### 9.2 Agent 总体结构

```text
自然语言任务（可选）
        |
        v
TaskSpec / BeliefState
        |
        v
     CAREAgent
  决定下一步技能
        |
        v
SafetyCritic
  过滤控制权和状态前置条件
        |
        v
TC-WAM
  判断抓取、底盘、升降和碰撞是否可行
        |
        v
SkillExecutor
  Nav2 / MoveIt / 夹爪 / 验证服务
        |
        v
后置条件验证 + FailureCode
        |
        v
更新 BeliefState、记录 trace、继续或恢复
```

Agent 只负责“下一步做什么”。它不能直接发布底盘速度、关节角、夹爪命令，也不能
绕过 SafetyCritic。正常流程使用确定性阶段脚本，只在阶段切换、异常、目标变化或
执行失败时调用 Agent 决策。

### 9.3 尚缺的模块

按优先级补齐以下文件：

| 模块 | 职责 | 验收 |
|---|---|---|
| `observation_aggregator.py` | 汇总对象记忆、地图版本、里程计、关节状态、TF、传感器时间戳、抓取状态和控制权 | 从真实 Topic 生成合法 `BeliefState` |
| `capability_adapter.py` | 调用 `/thesis/compute_tc_wam`，转换能力、ready region 和恢复代价 | 结果绑定对象、抓姿、主动臂和升降解 |
| `skill_executor.py` | 将技能分派到 Nav2、MoveIt、对齐、放置和验证接口 | 一次只运行一个技能，支持取消和超时 |
| `failure_mapper.py` | 将 Action 返回码、超时和验证失败映射到 `FailureCode` | 每条失败都有结构化原因和证据 |
| `ros_agent_node.py` | 运行观察-决策-执行-验证循环 | MuJoCo 中完成一条规则 Agent 闭环 |
| `trace_logger.py` | 记录信念摘要、候选技能、安全违规、效用、选择结果和证据 | 每个 trial 可由 JSONL 复算 |
| `launch/care_agent.launch.py` | 启动 Agent 和配置 | 支持 FSM/CARE-R/CARE-D 模式切换 |
| `config/agent.yaml` | 阈值、超时、恢复预算和效用权重 | 配置进入实验 manifest |

### 9.4 Agent 循环的硬规则

1. 先更新状态，再决策；不得根据过期 Topic 决策。
2. 候选技能必须来自注册表，不能接受 LLM 临时创建的动作。
3. 候选技能必须通过技能图边、前置条件和 SafetyCritic。
4. `plan_and_grasp`、`place_object` 等不可逆动作必须有明确控制权。
5. 只有独立证据验证后置条件后，才能报告技能成功。
6. 超时、取消、Action 异常和非法状态统一 fail closed。
7. 每个失败至少记录故障码、原技能、尝试次数、时间、证据 ID 和恢复候选。
8. 同一时刻只允许一个高层任务、一个底盘控制源和一个操作控制源。

### 9.5 当前技能图中的待修点

现有 [skills.py](openflex_ws/src/openflex_thesis/openflex_care_agent/openflex_care_agent/skills.py)
已经声明技能图，但 `CAREAgent.decide()` 还没有强制校验“上一技能是否允许进入
下一技能”。应增加：

- 记录上一执行技能及结果；
- 决策时检查 `SkillRegistry.successors(previous_skill)`；
- 非法转移直接返回 `ABORT_HOLD`；
- 为非法边、缺失后置条件和控制权冲突增加单元测试。

### 9.6 CARE-R 与 CARE-D 的边界

先完成 CARE-R：

```text
结构化失败码
  -> 人工定义的恢复动作表
  -> 安全过滤
  -> 可解释效用排序
```

只有在足够 trial 数据、明确训练/校准/测试划分和概率校准后，才做 CARE-D：

```text
(Belief 摘要, 故障码, 候选动作)
  -> 恢复成功概率
  -> 恢复耗时/移动代价
  -> 校准下界
  -> 与规则效用比较
```

首选 LightGBM、逻辑回归或小型 MLP。第一版不采用端到端强化学习。若数据和训练
覆盖不足，论文主方法应降级并明确写成 CARE-R，而不是声称 CARE-D 已得到验证。

### 9.7 大模型只放在三个窄接口

```text
parse_goal(text) -> TaskSpec JSON
diagnose_failure(state_summary, history) -> FailureLabel JSON
rank_recovery_actions(state_summary, allowed_actions) -> action_id
```

所有输出必须经过 JSON Schema、允许动作列表和超时校验。LLM 不可用时必须自动回退
到规则策略。禁止 LLM 生成 ROS 命令、shell 命令、代码或新的安全阈值。

## 10. MuJoCo 测试环境怎样组成

你的测试环境应当是“OpenFlex 整机 + 商超任务场景”，但不需要一开始就复现完整
6 m x 8 m 商场。

### 10.1 三层场景

| 层级 | 包含内容 | 用途 |
|---|---|---|
| L0 机构层 | OpenFlex 机器人、升降柱、双臂、夹爪、TF 和控制器 | 验证碰撞、IK 和执行 |
| L1 单货架层 | 一个货架面、一个目标商品、一个交付桌 | 主实验和故障恢复 |
| L2 商超层 | 多货架、货位数据库、导航站点、通道和干扰商品 | 最终泛化和端到端展示 |

论文主实验应先在 L1 完成。L2 用于端到端和泛化，不应一开始成为工程负担。

### 10.2 当前实际资产

- `openflex_mujoco-1.0_basic/openflex_mujoco.xml`：当前论文脚本加载的 OpenFlex 整机模型。
- `supermarket_scene/mujoco/`：6x8 商超场景生成器、预生成 XML、YCB 和扫描物体资产。
- `supermarket_scene/shelf_inventory.db`：SKU、货架、货位和交付桌业务数据。
- `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/model_builder.py`：
  当前会构建简化实验货架、目标和交付桌。

当前 [run_thesis_offline.sh](scripts/run_thesis_offline.sh) 和
[run_thesis_moveit_mission.sh](scripts/run_thesis_moveit_mission.sh) 都通过
`OPENFLEX_MUJOCO_MODEL` 指向整机 MJCF，不会自动加载完整商超 XML。因此以后需要
明确完成一次“OpenFlex 整机与商超场景集成”，并把生成后的 MJCF、来源数据库版本、
启动命令和 SHA256 写入实验 manifest。

### 10.3 第一次 MuJoCo 验收

在工作区根目录按顺序执行：

```bash
cd /home/wgb/openflex_shop
bash scripts/check_care_environment.sh
bash scripts/verify_moveit_basic.sh
bash scripts/run_thesis_offline.sh
```

验收证据至少包括：

- MuJoCo 模型可加载；
- `/clock`、29 维 `/joint_states` 和 TF 正常；
- 三路 RGB-D 在需要感知时正常；
- 机器人关节和 MoveIt 状态一致；
- 目标、货架、交付桌和机器人没有初始穿模；
- 每次运行的配置、模型和控制模式都有记录。

### 10.4 从单场景到论文实验

不要让主实验直接采用随机全场任务。建议逐级增加因素：

1. 固定底盘、固定目标、无障碍；
2. 改变目标在货架中的高度和横向位置；
3. 增加相似商品、局部遮挡和深度噪声；
4. 增加底盘初始位置和朝向偏差；
5. 注入导航失败、对齐失败、IK/碰撞失败和抓取未附着；
6. 组合两个故障；
7. 在未参与训练的新货架和商品布局上测试泛化。

每个场景都应有独立 `scene_id`、参数快照、随机种子和成功判定规则。

## 11. 可参考的开源项目与论文

没有一个开源项目可以直接替换 CARE-Agent。你的方法应当组合已有能力，并研究
“能力评估 + 安全技能约束 + 故障恢复”这一组合问题。

| 项目 | 可借鉴内容 | 使用建议 |
|---|---|---|
| [PlanSys2](https://github.com/IntelligentRoboticsLabs/PlanSys2) | ROS 2 上的 PDDL 任务规划 | 作为分层任务规划基线或对照 |
| [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | 重试、Fallback、超时和技能组合 | 参考执行层；SafetyCritic 仍应独立实现 |
| [executive_smach](https://github.com/ros/executive_smach) | 确定性状态机 | 用于 FSM 基线 |
| [ROSA](https://github.com/nasa-jpl/rosa) | 大模型调用 ROS 工具、权限和工具抽象 | 参考 LLM-ROS 接口，不允许直接控制硬件 |
| [RAI](https://github.com/RobotecAI/rai) | ROS 2 Agent 任务编排 | 参考工程结构，不整体替换论文方法 |
| [PDDLStream](https://github.com/caelan/pddlstream) | 任务与运动联合规划 | 复杂任务规划扩展参考 |
| [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor) | 多阶段操作方法组合 | 复杂抓取序列的后续参考 |
| [VoxPoser](https://github.com/huangwl18/VoxPoser) | 语言约束、空间可供性和动作分解 | 能力感知与空间约束的扩展思路 |
| [LightGBM](https://github.com/microsoft/LightGBM) | 分类、回归和概率校准 | CARE-D 恢复模型的首选实现 |

相关代表性研究还包括：

- SayCan：用技能可供性约束语言模型的高层决策；
- Inner Monologue：使用环境反馈和语言反馈修正机器人计划；
- Code as Policies：让语言模型生成受约束的程序型策略；
- OpenVLA、Octo 等端到端视觉-语言-动作模型。

OpenVLA/Octo 只适合作为文献对比或未来工作，不是本课题第一阶段的主方法。
引用开源项目时，应同时引用对应论文、仓库版本和实际使用的组件，不能只写 GitHub URL。

## 12. 实验数据、图片和统计结果怎样产生

论文实验不是“多跑几次截图”。每一个结果都必须来自可追溯的 trial。

### 12.1 每次 trial 必须保存

```text
trial_id
code_commit / dirty_worktree_patch_hash
ROS、MuJoCo、模型和配置版本
scene_id、layout、目标商品、货架层和起始位姿
随机种子
故障注入计划
控制模式和 Agent 模式
逐步决策 trace
技能结果和 FailureCode
成功/失败及判定证据
总时间、恢复次数、碰撞和安全事件
关键 frame 路径或 rosbag 路径
```

### 12.2 日志格式

主日志建议使用 JSONL，一个决策或技能结果一行。最低字段包括：

```text
timestamp
trial_id
phase
belief_summary
previous_skill
candidate_skills
safety_violations
capability_summary
utility_scores
selected_skill
skill_result
failure_code
evidence_ids
elapsed_sec
```

不要把整个大型点云或图像塞进 JSONL；保存文件路径、相机 frame 和时间戳即可。

### 12.3 图片和视频

需要保存：

- 系统架构图；
- 技能图和状态机图；
- MuJoCo 初始场景；
- RGB、深度和分割结果；
- 抓取候选、IK/碰撞和 ready region；
- 成功抓取、失败恢复和放置验证关键帧；
- 所有用于论文结论的失败样本；
- 关键统计图。

建议每个 trial 只保存代表性帧；RGB-D 全量只在需要复算感知结果时保存，避免
16 GiB 内存和数据盘被视频占满。

### 12.4 统计方法

主指标至少包括：

- 任务成功率及 Wilson 95% 置信区间；
- 分阶段成功率；
- 恢复成功率；
- 平均/中位恢复时间和总任务时间；
- 无效技能调用率；
- 安全违规率和拦截次数；
- 碰撞、掉落、未附着和陈旧地图执行次数；
- 资源占用和单位 trial 计算时间。

配对二分类实验可使用 McNemar 检验，独立组可使用 Fisher 精确检验；连续时间指标
应说明分布并使用 bootstrap 或合适的非参数检验。报告效应量和置信区间，不能只报
一个 p 值，也不能从失败样本中挑出成功案例。

正式实验前必须冻结：

```text
训练集、校准集、测试集
场景划分
成功判定
超时阈值
重复次数
排除标准
统计脚本
```

## 13. 数学公式怎样设计

公式应直接对应你的算法模块，而不是为了“看起来数学化”堆砌。建议至少写清楚以下
五组公式。

### 13.1 信念状态更新

```text
B_{t+1} = Update(B_t, a_t, z_{t+1})
```

其中 `a_t` 是技能调用，`z_{t+1}` 是观测、Action 结果或验证证据。正文要逐项解释
任务、阶段、机器人、目标、能力、失败历史和剩余预算。

### 13.2 动作合法性过滤

```text
A_t^raw
  -> A_t^pre = {a | Pre(a, B_t)}
  -> A_t^valid = {a | Pre(a, B_t) AND Safe(a, B_t)}
```

若 `A_t^valid` 为空，只能重观测、请求人工或安全终止。

### 13.3 TC-WAM 能力估计

```text
p_ik(c) = 1/M * sum_m F_m(c, xi_m)
```

`c` 为对象、抓姿、底盘和升降组合，`xi_m` 为定位、碰撞或几何扰动。可用于下界的
方法包括经验成功率的 Wilson/Clopper-Pearson 下界，或经过校准的概率下界。

### 13.4 恢复动作效用

```text
J(a | B_t) =
    alpha * L(a)
  + beta  * I(a)
  - lambda_t * T(a)
  - lambda_d * D(a)
  - lambda_r * R(a)
  - lambda_h * H(a)
```

其中 `L` 是恢复成功下界，`I` 是信息增益，`T` 是耗时，`D` 是移动距离，`R` 是
风险，`H` 是历史失败惩罚。每个权重都要说明来源、固定方法，并通过敏感性或消融
实验验证。

### 13.5 统计指标

成功率使用二项比例和 Wilson 置信区间。恢复时间通常明显右偏，应报告中位数、
四分位数和 bootstrap 置信区间，避免只报均值。

正文至少要回答：

- 每个符号表示什么；
- 每个阈值为什么这样选；
- 公式如何对应代码；
- 哪个实验证明这个公式或模块有效；
- 去掉这一项后结果如何变化。

每个关键公式至少配一个单元测试、一个示意图和一个消融结果。没有实验对应的公式
可以放在方法定义中，但不能写成已验证的创新。

## 14. 参考文献怎样收集和管理

建议使用 Zotero 管理文献，配合 Better BibTeX 固定 cite key。PDF、笔记和 BibTeX
都要进入个人文献库，不要把来源不明的 PDF 直接放进论文仓库。

每篇核心文献至少记录：

```text
题目
年份
期刊/会议
研究问题
核心方法
机器人平台
数据集和指标
安全/恢复机制
局限性
与本文的关系
```

检索顺序建议：

1. 先找近几年机器人会议和期刊中的技能规划、具身 Agent、移动操作和恢复论文；
2. 再找这些论文引用的奠基工作；
3. 对每类方法补充一个强基线和一篇综述；
4. 最后核对开源实现的论文、版本和许可证。

优先使用 IEEE Xplore、ACM DL、SpringerLink、DBLP、Google Scholar 和
Semantic Scholar。能用同行评审论文时，不用博客替代；GitHub 只说明实现来源。

建议的文献分组：

- 分层任务规划和行为树；
- LLM/VLM 机器人任务规划与工具调用；
- 移动操作、抓取和任务与运动规划；
- 可供性、能力评估和风险约束；
- 故障检测、计划修复和机器人安全；
- MuJoCo 仿真、可复现实验和统计评价；
- 数据驱动恢复和概率校准。

每个研究问题至少准备 5-10 篇核心文献。不要按“凑够篇数”写作，要形成一张
“问题-方法-局限-本文改进关系”的文献矩阵。

## 15. 硕士论文推荐框架

以下框架适用于当前题目，可随学校模板调整：

### 摘要

用四段写清：

1. 货架取放中的目标不确定性、执行失败和安全约束问题；
2. 提出 CARE-Agent 的分层技能、信念状态和能力评估方法；
3. 提出结构化故障恢复与 CARE-R/CARE-D 方案；
4. 给出 MuJoCo 实验、主要指标、消融和结论。

关键词包括：移动操作、具身智能体、技能规划、能力评估、故障恢复、MuJoCo。

### 第 1 章 绪论

- 研究背景和货架取放任务需求；
- 研究问题与挑战；
- 国内外研究现状简述；
- 本文研究内容与贡献；
- 论文组织结构。

### 第 2 章 相关理论与技术基础

- 分层任务规划和技能图；
- LLM/VLM 机器人高层决策；
- 任务与运动规划、MoveIt 和 Nav2；
- 可供性、能力评估和机器人安全；
- 故障恢复与概率校准；
- MuJoCo、ROS 2 和评价方法。

### 第 3 章 OpenFlex 数字实验平台与任务建模

- OpenFlex 机器人、升降柱、双臂和相机；
- MuJoCo 整机模型和商超场景；
- ROS 2、MoveIt、Nav2 和 TC-WAM 接口；
- 货架取放任务状态、观测和控制边界；
- 数字仿真结论的适用边界。

### 第 4 章 能力感知分层智能体建模

- Executive Agent 与确定性技能层；
- BeliefState 定义和更新；
- SkillContract、技能图、前置/后置条件；
- SafetyCritic 和控制权约束；
- 动作合法性过滤；
- 与普通平面 Agent、多 Agent 对话方法的区别。

### 第 5 章 物理能力评估与抓取决策

- TC-WAM 输入和输出；
- IK、碰撞、升降和底盘联合可行率；
- 风险校准下界；
- ready region 绑定；
- 能力感知的技能参数选择；
- 计算复杂度、缓存和粗筛策略。

### 第 6 章 故障感知的任务恢复

- 结构化 FailureCode；
- 失败证据和状态更新；
- 恢复候选生成和安全过滤；
- CARE-R 规则效用；
- CARE-D 数据采集、训练和校准；
- 安全终止和人工接管条件。

### 第 7 章 实验设计与结果分析

- 平台、场景、故障注入和变量；
- 基线和消融设置；
- 指标、统计方法和可复现性；
- 单次基线、技能约束和 TC-WAM 对比；
- 单故障、复合故障和恢复算法对比；
- 泛化、运行时间和失败案例；
- 数字仿真结果的有效性边界。

### 第 8 章 总结与展望

- 研究结论；
- 主要创新；
- 尚未解决的问题；
- 真实机器人、接触动力学、多商品和开放环境展望。

### 参考文献与附录

附录可放：

- 技能契约表；
- ROS 接口和主题表；
- 实验配置；
- 补充统计结果；
- 场景和代码版本；
- 复现实验命令。

## 16. 从现在开始的执行清单

### 第 1 步：确认当前环境

```bash
cd /home/wgb/openflex_shop
bash scripts/check_care_environment.sh
bash scripts/verify_moveit_basic.sh
```

记录命令输出、失败项和修复结果。不要只记录“成功”。

### 第 2 步：完成固定底盘抓放基线

G3 左臂停放碰撞已经通过。现在继续完成抓取、附着、运输、放置和 10 次重复。
输出：

- 无碰撞左臂和升降柱配置；
- 单次完整执行日志；
- 失败码统计；
- 关键截图或短视频；
- 每次 trial 的 manifest。

### 第 3 步：接入 Agent 最小闭环

只实现一条：

```text
读取 BeliefState
  -> CAREAgent 选择一个技能
  -> SafetyCritic
  -> SkillExecutor
  -> 后置条件验证
  -> 写 trace
```

先用 mock 技能测试，再依次接真实 MuJoCo 技能。

### 第 4 步：完成 CARE-R

只接入结构化 FailureCode 和规则恢复表。跑通目标丢失、抓取失败和碰撞不可行三类
单故障。此时不接大模型。

### 第 5 步：接入 TC-WAM

把能力评估结果写入 BeliefState 和恢复代价。比较“无能力评估”和“有能力评估”的
差异，检查计算时间是否可接受。

### 第 6 步：建立正式实验流水线

冻结场景、随机种子、成功条件和统计脚本。生成 baseline、CARE-R、消融和故障实验
结果，所有结论都要能由日志重新计算。

### 第 7 步：最后再接 LLM 和 CARE-D

先实现规则回退和 schema 校验，再测试 Qwen。只有数据规模和覆盖范围足够时才训练
CARE-D；否则保留为补充实验，不把论文成败押在模型训练上。

### 第 8 步：边做实验边整理论文素材

每完成一个实验，立即保存：

- 一段方法说明；
- 一组公式；
- 一张原理图；
- 一张结果表；
- 一张统计图；
- 相关参考文献；
- 复现命令和版本。

不要等实验全部做完才开始写论文。

## 17. 课题完成的最低合格线

要形成一份可答辩的硕士论文，至少应具备：

1. 一个稳定可复现的 MuJoCo 货架抓放基线；
2. 一个真正接入 ROS 和 MuJoCo 的 CARE-Agent 闭环；
3. 可解释的技能契约和 SafetyCritic；
4. TC-WAM 能力评估的定量对比；
5. CARE-R 在单故障和复合故障上的实验；
6. 至少一组消融，证明关键模块不是无效工程包装；
7. 完整 trial 日志、统计脚本、关键图片和复现说明；
8. 围绕研究问题组织的正文、公式、实验表和参考文献。

完成这些后，再扩展真实相机、SAM2/GraspNet、更大商超场景或 CARE-D，而不是反过来
先堆模型。
