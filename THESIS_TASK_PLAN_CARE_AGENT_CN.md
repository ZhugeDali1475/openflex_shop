# 硕士学位论文任务计划与方法书

## 面向货架取放任务的能力感知分层具身智能体决策与恢复方法研究

版本：v2.0  
日期：2026-11-02  
论文类型：算法研究 + 数字孪生仿真验证  
主方法：CARE-Agent  
英文名称：Capability-Aware Recovery Embodied Agent  
主要平台：OpenFlex、MuJoCo、ROS 2 Humble、MoveIt、Nav2、Qwen/SAM2/GraspNet

> 项目入口和当前实施状态见：
> [PROJECT_INDEX_CN.md](PROJECT_INDEX_CN.md) 和
> [IMPLEMENTATION_STATUS_2026-11-03.md](IMPLEMENTATION_STATUS_2026-11-03.md)。
> 实际命令与逐级验收步骤见
> [MUJOCO_THESIS_EXPERIMENT_RUNBOOK_CN.md](MUJOCO_THESIS_EXPERIMENT_RUNBOOK_CN.md)。
>
> 本文件取代此前的 TC-WAM 单主线计划。旧计划中的硬件评估、MuJoCo 仿真、
> MoveIt 物理评价和风险受控可操作性内容继续有效，但论文核心调整为分层具身智能体
> 的任务决策和故障恢复。TC-WAM 作为 CARE-Agent 的物理能力评估器，不再单独承担
> 整篇论文的主创新。

---

## 0. 最终决定

### 0.1 论文主线

本文研究一个只负责高层决策的移动操作智能体：

```text
智能体决定下一步做什么
物理能力评估器判断这件事能不能做
确定性技能层执行具体动作
安全层决定动作是否允许执行
```

最终系统为：

```text
自然语言任务
  -> CARE-Agent 解析任务和建立世界状态
  -> 选择导航、观察、对位、抓取、运输、放置和验证技能
  -> TC-WAM 评估底盘、升降机械臂构型和抓姿的物理能力
  -> Nav2 / MoveIt 执行
  -> 验证技能检查后置条件
  -> 失败时进行故障归因和计划修复
```

### 0.2 不采用的路线

- 不做端到端 VLA；
- 不从零训练视觉语言模型；
- 不依赖未开放的 `openflex_vla`；
- 不把 ACT 作为论文成败条件；
- 不让多个大语言模型 Agent 自由对话并直接控制机器人；
- 不让 Agent 输出速度、关节角或电机指令；
- 不做完整三环节商超比赛作为主实验；
- 不把 ROS 节点改名为 Agent 后当作算法创新；
- 不把 Prompt、工程接口或 demo 单独算作创新；
- 不在仿真中声称真实抓取成功率。

### 0.3 最终论文题目

**《面向货架取放任务的能力感知分层具身智能体决策与恢复方法研究》**

备选题目：

1. 《面向移动操作的能力感知分层具身智能体任务规划与故障恢复研究》
2. 《不确定性下货架取放机器人分层具身智能体决策方法研究》
3. 《能力约束驱动的移动机械臂分层智能体任务规划与恢复方法研究》

---

## 1. 研究目标、问题与边界

## 1.1 研究目标

针对货架取放任务中的目标不确定、抓取不可达、底盘偏差、地图失效和抓放验证失败，
构建一个能够：

1. 维护任务、环境、机器人和失败历史状态；
2. 从带物理前置条件的技能中选择下一步动作；
3. 使用 TC-WAM 评估动作的物理可执行性；
4. 在失败后判断原因并选择重观测、重算、重对位或终止；
5. 保证所有底层动作经过确定性安全校验；
6. 在 MuJoCo 数字环境中完成定量验证；

的分层具身智能体。

## 1.2 核心研究问题

### RQ1：分层技能约束能否减少无效 Agent 动作？

问题：

> 将 ROS 操作能力建模为带前置条件、后置条件和失败语义的技能图，能否减少
> Agent 选择物理不可执行动作和违反控制权约束的动作？

假设 H1：

> 与平面 Agent 相比，技能图约束和硬前置条件可显著降低无效技能调用率、安全违规率
> 和安全拦截前的危险动作尝试次数。

### RQ2：物理能力评估能否提高 Agent 决策可靠性？

问题：

> 将底盘位姿、升降机械臂 IK、碰撞、关节余量和抓姿风险纳入 Agent 的动作评估，
> 能否比仅依赖视觉语义和规则模板获得更高的抓取成功率？

假设 H2：

> 能力感知 Agent 在货架边缘、高层/低层、相似目标和底盘偏移场景中具有更高的
> 任务成功率和更低的下游 MoveIt 失败率。

### RQ3：故障感知恢复能否改善长时序任务成功率？

问题：

> 根据结构化失败码、剩余时间、目标可见性和能力风险进行计划修复，能否比固定重试
> 状态机或平面 ReAct Agent 更快地恢复任务？

假设 H3：

> 故障感知恢复可提高单故障和复合故障场景的成功率，减少无效抓取、无效导航和
> 陈旧地图执行次数。

### RQ4：规则策略和数据驱动恢复策略如何取舍？

问题：

> 在有限仿真数据和计算资源下，带校准的轻量级恢复策略是否优于规则效用策略？

假设 H4：

> 规则效用版本在少数据条件下更稳定；数据驱动版本在训练覆盖充分的故障类型上
> 具有更高恢复成功率和更低时间代价，但在未见故障类型上可能退化。

## 1.3 研究范围

主实验只研究一条任务链：

```text
语言指定商品
  -> 到货架观察区
  -> 目标检测和实例选择
  -> 底盘/升降机械臂/抓姿联合能力评价
  -> 局部对位
  -> 抓取和附着验证
  -> 运输
  -> 放置和视觉验证
```

主实验只处理一个目标商品。两个商品的任务顺序只在最终集成实验中出现。

不在主实验中研究：

- 全场巡检；
- 补货；
- 错放归位；
- 多机器人协同；
- 动态人群；
- 开放世界复杂对话；
- 真机物理接触标定。

---

## 2. 电脑硬件与软件条件

## 2.1 硬件实测

| 项目 | 实测结果 | 论文含义 |
|---|---|---|
| 电脑 | Lenovo GeekPro-17IRR / 90X00002CP | 单机可完成仿真研究 |
| CPU | Intel i7-14700F，20 核 28 线程，最高 5.4 GHz | 足够支持 MuJoCo、MoveIt 和离线数据处理 |
| 内存 | 约 16 GiB，另有约 29.8 GiB swap | 最大瓶颈，完整栈建议串行运行 |
| GPU | NVIDIA GeForce RTX 4060，8 GiB，compute 8.9 | 可运行 SAM2、GraspNet 和量化 VLM |
| 驱动 | 580.159.03，支持最高 CUDA 13.0 | 主机环境可正常识别 GPU |
| 显示 | 2560×1440 | 批量实验时不打开图形界面 |
| 系统盘 | 74.5 GiB ext4，约剩 42 GiB | 仅放系统、构建和临时文件 |
| 数据盘 | 310 GiB ext4，`/home` 约剩 193 GiB | 足够保存论文数据 |
| OS | Ubuntu 22.04.5 LTS，kernel 6.8 | 与 ROS 2 Humble 匹配 |

## 2.2 软件实测

| 软件 | 当前状态 | 结论 |
|---|---|---|
| ROS 2 Humble | 已安装，约 369 个包 | 可用 |
| Nav2 | 已安装 | 可用于全局导航 |
| Gazebo 11 | 已安装 | 仅作排查备选 |
| MoveIt | 源码存在，但 install space 有旧路径残留 | 必须清理重建 |
| MuJoCo | LeRobot conda 环境 3.4.0 | 作为主仿真器 |
| PyTorch 感知环境 | 2.7.1+cu126，GPU 可调用 | 可运行 SAM2 和量化模型 |
| PyTorch LeRobot 环境 | 2.12.0+cu130，GPU 可调用 | 可用于 MuJoCo 和轻量策略 |
| GraspNet | torch CUDA 可用，PointNet2 可导入，KNN 缺失 | 必须修复 |
| Qwen3.5-4B | 本地约 8.8 GiB 权重 | bf16 无法完整装入 8 GiB 显存 |
| SAM2 small | 本地权重存在 | 可在 GPU 运行 |
| OpenFlex MuJoCo | 87 个网格中约 86 个是 Git LFS 指针 | 当前模型不可加载，必须修复 |
| ROS install | 旧机器路径残留 | 必须重新构建 |

## 2.3 资源策略

### GPU

主系统运行时只保留一个 GPU 大模型：

- 目标检测/语义解析：Qwen 4-bit 或 2B 级视觉语言模型；
- 分割：SAM2 small；
- 抓姿：GraspNet；
- 三者不同时常驻。

如果 Qwen bf16 无法装入 8 GiB：

1. 使用 bitsandbytes 4-bit；
2. 使用更小的视觉语言模型；
3. 用 GroundingDINO/OWLv2 承担高频目标定位；
4. Qwen仅负责低频任务解析和失败原因归类；
5. 所有模型都必须有离线缓存和超时回退。

### 内存

推荐把内存升级到 32 GiB，最好 64 GiB。若不升级：

- 同一时刻只开一个完整 ROS/MoveIt/MuJoCo 栈；
- 批量实验关闭 RViz、浏览器和大型 IDE；
- Qwen 推理和 MoveIt 执行分时进行；
- 用子进程隔离每次 trial，结束后强制释放资源。

### 存储

- 所有 trial 保存 JSONL/Parquet；
- 只对代表性和所有失败样本保存 RGB-D、视频和 rosbag；
- 建议至少预留 80 GiB 论文数据空间；
- `/tmp` 只用于派生 MJCF 和短期缓存。

---

## 3. 现有工程复用与调整

## 3.1 复用模块

| 模块 | 新论文用途 |
|---|---|
| `openarmx_task_executor` | 确定性状态机基线和底层任务技能 |
| `openarmx_supermarket_sim` | MuJoCo 物理、三路 RGB-D、MoveIt 和 Nav2 |
| `openflex_grasp` | Qwen、SAM2、GraspNet、实例点云和抓取工具 |
| `openflex_tc_wam` | 物理能力评估器和风险受控区域 |
| `openflex_thesis_control` | 控制权仲裁、BaseLock 和执行安全 |
| `openflex_thesis_experiments` | trial manifest、日志和统计基础 |
| `OpenFlex MuJoCo` | 整机模型和场景资产 |

## 3.2 现有系统与本文的区别

当前系统已经有：

- 裁判阶段门控；
- 单件运输状态机；
- 导航、精对位、抓取、放置和验证工具；
- 结构化错误码；
- MoveIt/底盘控制权约束；
- 对象记忆；
- 风险地图原型。

但当前系统：

- 正常流程是固定状态机；
- 抓取失败一般不自动重试；
- 失败锁存后需要人工处理；
- 工具选择主要依赖固定代码；
- 没有统一的世界信念状态；
- 没有失败原因驱动的计划修复；
- 没有把物理能力评估作为 Agent 动作合法性的一部分。

本文不是重写全部执行器，而是在其上层增加：

```text
世界状态
技能图
能力评估
故障归因
计划修复
安全校验
审计日志
```

---

## 4. CARE-Agent 总体设计

## 4.1 为什么是“分层智能体”而不是“多 Agent 聊天”

本文采用：

```text
一个 Executive Agent
  + 多个无自主决策权的技能角色
  + 一个确定性安全壳
```

原因：

- 机器人只有一个底盘控制权和一个操作机构控制权；
- 多个 Agent 并发决策会产生冲突；
- 自然语言辩论不能替代碰撞、停稳和控制权校验；
- 大模型调用延迟不适合底层闭环；
- 论文需要可复现、可审计、可解释。

技能角色可以使用“Agent”命名，但在系统上它们是受 Executive Agent 调用的工具模块。

## 4.2 逻辑角色

| 角色 | 核心职责 | 不允许做的事 |
|---|---|---|
| Executive Agent | 解析任务、维护阶段、选择技能、触发恢复 | 直接发布速度和关节命令 |
| Perception Agent | 目标定位、实例选择、重观测 | 直接调用 MoveIt |
| Navigation Agent | Nav2、靠站、路径重规划 | 绕过碰撞监控 |
| Alignment Agent | 精对位、ready region 进入、BaseLock | 在 MoveIt 执行中移动底盘 |
| Manipulation Agent | GraspNet、TC-WAM、MoveIt 抓取 | 越权控制另一侧手臂 |
| Verification Agent | 抓取、附着、运输和放置验证 | 在无证据时报告成功 |
| Recovery Agent | 失败归因和恢复动作候选生成 | 绕过安全校验器 |
| Safety Critic | 校验前置条件、控制权和风险 | 接受未注册的任意工具 |

## 4.3 世界信念状态

在时刻 \(t\)，Agent 使用结构化信念状态：

\[
B_t=
\langle
g,
p,
T,
r,
o,
u,
\mathcal G,
C,
M,
H,
\tau,
s
\rangle
\]

其中：

- \(g\)：语言任务或订单；
- \(p\)：当前任务阶段；
- \(T\)：剩余任务队列；
- \(r\)：机器人位姿、速度、关节和底盘锁状态；
- \(o\)：目标实例及其可见性、位置、类别和地图版本；
- \(u\)：目标、分割、抓姿和底盘不确定性；
- \(\mathcal G\)：当前抓姿候选集合；
- \(C\)：物理能力评估结果；
- \(M\)：语义、任务和失败记忆；
- \(H\)：历史动作及结果；
- \(\tau\)：剩余时间和重试预算；
- \(s\)：安全状态和传感器新鲜度。

状态更新：

\[
B_{t+1}
=
\operatorname{Update}(B_t,a_t,z_{t+1})
\]

\(a_t\) 为技能调用，\(z_{t+1}\) 为技能返回、传感器观测和验证证据。

## 4.4 技能定义

每个技能定义为：

\[
K=
\langle
\text{name},
\text{args},
\text{pre},
\text{post},
\text{failure},
\text{cost},
\text{owner},
\text{timeout}
\rangle
\]

并满足：

1. 调用前必须通过 `pre`；
2. 调用后必须验证 `post`；
3. 失败必须返回结构化 `failure`；
4. 技能只能拥有一个硬件控制域；
5. 超时、取消和异常必须 fail closed；
6. Agent 不得自行构造底层命令。

### 核心技能

| 技能 | 前置条件 | 后置条件 |
|---|---|---|
| `look_for_object` | 相机新鲜、地图版本有效 | 返回目标实例或明确未发现 |
| `navigate_to_station` | 定位有效、路径存在、底盘权可用 | 到达站点并停稳 |
| `fine_align` | 站点合法、传感器新鲜、底盘权已授予 | 进入容差并停稳 |
| `compute_capability` | 目标可见、抓姿候选有效 | 返回物理可行区域和风险 |
| `local_align_region` | ready region 绑定有效 | 进入同一绑定区域并 BaseLock |
| `plan_and_grasp` | 底盘停稳、MoveIt 权独占、目标版本一致 | 抓取验证通过且运输姿态安全 |
| `navigate_to_delivery` | 物体附着、底盘权交还 Nav2 | 到达交付站 |
| `place_object` | 底盘停稳、目标槽位有效 | 释放并且无异常 |
| `verify_place` | 放置证据新鲜 | 位置、稳定性和 SKU 验证 |
| `reobserve` | 机器人可安全观察 | 获得新的传感器证据 |
| `abort_hold` | 任意状态 | 进入零速度安全保持 |

## 4.5 技能图

技能图：

\[
G=(V,E)
\]

\(V\) 为技能，\(E\) 表示“执行前一技能后可能进入下一技能”。

边的合法性由当前信念状态决定，而不是固定写死所有可能路径。

示例：

```text
look_for_object
  ├─ success -> compute_capability
  ├─ ambiguous -> reobserve
  ├─ not_found -> navigate_alternative_view
  └─ stale_map -> refresh_scene

compute_capability
  ├─ feasible -> local_align_region
  ├─ no_feasible_action -> reobserve / reposition / abort
  └─ target_moved -> look_for_object

plan_and_grasp
  ├─ verified -> navigate_to_delivery
  ├─ grasp_failed -> reobserve/recompute
  ├─ collision -> alternate_grasp
  └─ repeated_failure -> abort
```

---

## 5. 算法设计

## 5.1 算法组件总览

CARE-Agent 包含五个算法组件：

1. 技能图与动作合法性过滤；
2. TC-WAM 物理能力评估；
3. 失败类型识别与状态更新；
4. 风险约束的恢复策略选择；
5. 安全壳与不可逆动作保护。

## 5.2 算法一：动作合法性过滤

给定状态 \(B_t\)，候选技能集合为：

\[
A_t^{raw}
\]

经过前置条件过滤：

\[
A_t^{pre}
=
\{a\in A_t^{raw}
\mid
\operatorname{Pre}(a,B_t)=\text{true}\}
\]

经过安全壳过滤：

\[
A_t^{valid}
=
\{a\in A_t^{pre}
\mid
\operatorname{Safe}(a,B_t)=\text{true}\}
\]

若 \(A_t^{valid}=\varnothing\)，Agent 只能选择：

- 重观测；
- 重新建立状态；
- 请求人工；
- 安全终止。

安全壳检查：

- 传感器新鲜度；
- TF 完整性；
- 地图版本；
- 底盘停稳；
- 控制权拥有者；
- 关节限位；
- 碰撞状态；
- 目标可见性；
- 抓取授权；
- 当前技能是否已在运行。

## 5.3 算法二：TC-WAM 物理能力评估

对候选：

\[
c=(o,g,b,h)
\]

在随机扰动 \(\xi\) 下计算联合可行率：

\[
p_{ik}(c)
=
\frac{1}{M}
\sum_{m=1}^{M}
F_{m}(c,\xi_m)
\]

其中：

- IK 成功；
- 碰撞通过；
- 关节余量；
- 预抓取到抓取的可执行性；
- 目标定位扰动；
- 底盘定位扰动。

TC-WAM 输出：

\[
C(c)=
\langle
p_{ik},
p_{lcb},
d_{move},
T_{plan},
R_{collision},
\mathcal R_{ready}
\rangle
\]

其中：

- \(p_{lcb}\)：风险校准后的成功下界；
- \(\mathcal R_{ready}\)：与实例、抓姿、活动臂和升降解严格绑定的连通可行区域。

Agent 只允许把 `p_lcb` 达到阈值的抓取和区域作为可执行技能参数。

## 5.4 算法三：故障识别

将执行结果标准化为：

\[
z=(status,error\_code,evidence,latency,state\_delta)
\]

故障类别至少包括：

| 编号 | 故障 | 典型来源 |
|---|---|---|
| F01 | 目标未发现 | Perception |
| F02 | 相似目标歧义 | Perception |
| F03 | 目标被遮挡 | Perception |
| F04 | 地图版本变化 | Memory |
| F05 | 定位或传感器陈旧 | Safety |
| F06 | 导航不可达 | Nav2 |
| F07 | 精对位失败 | Alignment |
| F08 | IK 或碰撞不可行 | TC-WAM/MoveIt |
| F09 | 抓取规划失败 | MoveIt |
| F10 | 夹爪闭合但未附着 | Verification |
| F11 | 运输姿态失败 | Manipulation |
| F12 | 放置失败或放置验证失败 | Place/Verification |
| F13 | 非法动作或控制权冲突 | Safety |

故障识别输出：

\[
\hat f=
\operatorname{Classify}(B_t,a_t,z_{t+1})
\]

第一版使用规则分类；第二版可使用 LightGBM 分类器。

## 5.5 算法四：恢复动作效用

对每个合法恢复动作 \(a\)，计算：

\[
J(a\mid B_t)
=
\alpha L(a)
+\beta I(a)
-\lambda_t T(a)
-\lambda_d D(a)
-\lambda_r R(a)
-\lambda_h H(a)
\]

其中：

- \(L(a)\)：恢复成功的校准下界；
- \(I(a)\)：预期信息增益；
- \(T(a)\)：预计耗时；
- \(D(a)\)：移动距离；
- \(R(a)\)：碰撞、掉落、控制权冲突等风险；
- \(H(a)\)：历史上同类故障的失败惩罚。

最终选择：

\[
a^*
=
\arg\max_{a\in A_t^{valid}}
J(a\mid B_t)
\]

### 信息增益

当目标丢失、遮挡或语义歧义时：

\[
I(a)
\approx
H(B_t^{o})
-
E[H(B_{t+1}^{o}\mid a)]
\]

仿真中可以使用：

- 目标检测置信度；
- 有效深度点比例；
- 相似实例间概率间隔；
- 回看到目标的概率；
- TC-WAM 可行候选增加量；

作为信息增益的近似。

## 5.6 算法五：规则版与数据驱动版

为了控制风险，CARE-Agent 分为两个可比较版本。

### CARE-R：规则效用版

使用专家规则和效用表：

```text
目标丢失：优先换视角重观测
相似目标：重新选实例
抓姿不可达：重算 TC-WAM
导航失败：改航点
抓取失败：回观察位后重算
地图失效：撤销旧状态
安全异常：立即 HOLD
```

优点：

- 不依赖训练数据；
- 可解释；
- 可以在第一阶段完成。

### CARE-D：数据驱动恢复版

从仿真中收集：

\[
(B_t,a_t,y_t,c_t)
\]

其中：

- \(y_t\)：技能是否恢复；
- \(c_t\)：恢复耗时、移动代价和安全代价。

训练：

1. 恢复成功的二分类模型；
2. 恢复代价的回归模型；
3. 按故障类型和货架高度分层的概率校准；
4. 使用校准下界进入效用函数。

首选模型：

- LightGBM；
- 逻辑回归；
- 小型 MLP。

不采用端到端强化学习作为第一版，因为：

- 仿真闭环成本高；
- 奖励设计复杂；
- 安全约束难以保证；
- 样本效率不一定优于监督学习。

可把上下文赌博机或离线强化学习放在展望中。

## 5.7 Agent 工作循环

```text
1. 读取结构化观测
2. 更新信念状态
3. 检测事件和失败
4. 生成候选技能
5. 执行前置条件过滤
6. 调用能力评估
7. 执行安全过滤
8. 计算动作效用
9. 选择技能
10. 执行技能
11. 验证后置条件
12. 记录结果和更新记忆
13. 重复，直到成功、终止或人工接管
```

正常流程使用阶段脚本，不调用大模型：

```text
解析目标
-> 到观察区
-> 感知目标
-> 能力评价
-> 对位
-> 抓取
-> 验证
-> 运输
-> 放置
-> 验证
```

Agent 在以下事件发生时进入：

- 阶段失败；
- 状态不一致；
- 目标消失；
- 地图版本改变；
- 规划不可行；
- 动作后置条件验证失败；
- 当前计划剩余时间不足。

## 5.8 大语言模型边界

大语言模型只允许三个接口：

1. `parse_goal(text) -> TaskSpec JSON`
2. `diagnose_failure(state_summary, history) -> FailureLabel JSON`
3. `rank_recovery_actions(state_summary, allowed_actions) -> action_id`

输出必须经过 JSON Schema 和允许动作列表校验。

禁止：

- 生成任意 ROS 命令；
- 生成 shell 命令；
- 修改安全阈值；
- 创建未注册技能；
- 直接执行自然语言中的代码；
- 在每一帧控制循环中调用。

如果 LLM 超时、输出非法或置信度不足，立即回退到规则策略。

## 5.9 记忆设计

第一版Memory使用结构化存储，不使用向量数据库。

### 任务记忆

- 当前目标；
- 已完成步骤；
- 剩余时间；
- 重试次数；
- 当前阶段。

### 语义记忆

- 物品类别；
- 货架/层高；
- 槽位；
- 地图版本；
- 目标曾在哪些视角出现。

### 情节记忆

- 上次抓取失败原因；
- 哪些站点曾失败；
- 哪些视角观察到目标；
- 哪些恢复动作有效。

### 安全记忆

- 禁止动作；
- 已锁存故障；
- 陈旧数据；
- 控制权冲突。

记忆只能用于提出候选动作，不能授权物理执行。

---

## 6. Agent 状态机与失败恢复

## 6.1 正常流程

```text
INIT
  -> PARSE_TASK
  -> NAVIGATE_OBSERVE
  -> ACQUIRE_TARGET
  -> EVALUATE_CAPABILITY
  -> ALIGN_TO_READY
  -> GRASP
  -> VERIFY_GRASP
  -> NAVIGATE_DELIVERY
  -> PLACE
  -> VERIFY_PLACE
  -> DONE
```

## 6.2 事件驱动恢复

```text
ACQUIRE_TARGET
  ├─ ambiguous -> REOBSERVE
  ├─ missing -> CHANGE_VIEW
  └─ map stale -> REBUILD_BELIEF

EVALUATE_CAPABILITY
  ├─ feasible -> ALIGN
  ├─ no feasible -> REOBSERVE / RECOMPUTE
  └─ controls invalid -> HOLD

ALIGN
  ├─ reached -> GRASP
  ├─ timeout -> REALIGN
  └─ stale -> REOBSERVE

GRASP
  ├─ attached -> VERIFY
  ├─ IK fail -> ALTERNATIVE_GRASP
  ├─ collision -> RECOMPUTE
  └─ repeated fail -> ABORT

VERIFY_GRASP
  ├─ safe -> TRANSPORT
  └─ failed -> RETREAT_AND_REOBSERVE

PLACE
  ├─ released -> VERIFY_PLACE
  └─ failed -> REALIGN_PLACE
```

## 6.3 安全终止条件

以下情况立即 `HOLD`：

- 定位失效；
- 里程计/关节/相机数据陈旧；
- 控制权冲突；
- 底盘未停稳；
- 地图版本不一致；
- 目标不可见且无执行授权；
- Agent 产生非法动作；
- 超过总时间预算；
- 安全壳内部错误；
- 连续多次恢复失败。

---

## 7. 软件工程实现

## 7.1 新建 `openflex_care_agent` 包

推荐结构：

```text
src/openflex_thesis/
└── openflex_care_agent/
    ├── openflex_care_agent/
    │   ├── state.py
    │   ├── skill_graph.py
    │   ├── skill_contracts.py
    │   ├── capability.py
    │   ├── failure.py
    │   ├── recovery.py
    │   ├── policy.py
    │   ├── memory.py
    │   ├── safety_guard.py
    │   ├── llm_adapter.py
    │   ├── observation_aggregator.py
    │   ├── ros_agent_node.py
    │   └── trial_logger.py
    ├── config/
    │   ├── agent.yaml
    │   ├── skills.yaml
    │   ├── recovery.yaml
    │   ├── safety.yaml
    │   └── scenarios.yaml
    ├── launch/
    │   └── care_agent.launch.py
    ├── test/
    └── package.xml
```

## 7.2 消息与动作

可以使用现有消息，并新增下列轻量消息：

```text
AgentBelief.msg
SkillInvocation.msg
SkillResult.msg
FailureEvent.msg
RecoveryDecision.msg
```

不一定要把所有 Agent 内部状态暴露为 ROS Topic。纯算法模块应能在无 ROS 环境下单元测试。

## 7.3 与现有 Action 连接

建议通过适配器调用：

| 能力 | 现有接口 |
|---|---|
| 导航 | `/navigate_to_pose` |
| 精靠 | `/dock_to_station`、`/shelf_fine_align` |
| 抓取 | `/pick_object` 或 `/thesis/execute_moveit_grasp` |
| 放置 | `/place_object` |
| 抓取验证 | `/thesis/grasp_status` |
| 对象记忆 | `/thesis/object_memory` |
| TC-WAM | `/thesis/compute_tc_wam` |
| 局部对位 | `/thesis/execute_local_alignment` |

同一时刻只允许一个上层任务、一个底盘控制源和一个操作机构控制源。

## 7.4 启动模式

### 模式 A：纯状态机基线

```bash
agent_mode:=fsm
llm_enabled:=false
recovery_enabled:=false
```

### 模式 B：规则恢复 Agent

```bash
agent_mode:=care_r
llm_enabled:=false
recovery_enabled:=true
```

### 模式 C：完整 CARE-Agent

```bash
agent_mode:=care_d
llm_enabled:=true
capability_enabled:=true
recovery_enabled:=true
```

---

## 8. 仿真系统与故障注入

## 8.1 主仿真器

主仿真器：**MuJoCo 3.4**。

原因：

- 现成 OpenFlex 模型；
- 双臂、升降、夹爪和底盘完整；
- Python 接口轻量；
- 支持 headless 渲染；
- 比 Isaac Sim 更适配 8 GiB GPU；
- 可与 ROS 2 和 MoveIt 集成。

不把 Gazebo 作为主实验平台，只作为接口排查备选。

## 8.2 推荐场景

初期不重建完整 250 商品商超，只使用：

- 一组货架；
- 3 个层高；
- 8～12 个商品类别；
- 一个交付台；
- 头部相机；
- 左右腕部相机；
- 可移动遮挡板；
- 可随机目标位姿。

## 8.3 故障注入服务

新增测试服务：

```text
/care_sim/inject_target_offset
/care_sim/inject_occluder
/care_sim/inject_similar_object
/care_sim/inject_grasp_slip
/care_sim/inject_grasp_verification_failure
/care_sim/inject_map_version_change
/care_sim/inject_navigation_block
/care_sim/inject_ik_failure
/care_sim/inject_place_offset
/care_sim/reset_scenario
```

注入器是实验工具，Agent 仍然只能通过正常传感器和状态接口判断故障，不能读取隐藏真值。

## 8.4 数字证据边界

所有数据必须标记：

- `simulation`
- `rendered_perception`
- `oracle_feasibility`
- `moveit_feasibility`
- `deterministic_attach`
- `contact_assisted_attach`

如果采用确定性附着代理，论文只能声明“数字执行成功”，不能声明真实抓取力学成功。

---

## 9. 数据集与场景划分

## 9.1 数据规模

### Pilot

- 50 个场景；
- 20 次完整任务；
- 每种主要故障 5 次；
- 目标：打通采集和日志，不用于最终结论。

### Training / Calibration

用于 CARE-D：

- 300～500 个恢复决策样本；
- 故障类型覆盖 F01～F12；
- 按场景和故障类型分层。

数据划分：

| 集合 | 比例 | 用途 |
|---|---:|---|
| 训练集 | 60% | 恢复结果预测 |
| 校准集 | 20% | 概率和阈值校准 |
| 测试集 | 20% | 最终评价 |

禁止按连续帧随机划分；测试场景必须按场景、物体和故障组合留出。

### Main evaluation

- 名义任务：每种方法 30 次；
- 单故障：每类关键故障、每种主要方法 20 次；
- 复合故障：每种方法 20 次；
- 消融：每个变体 20 次；
- 最终端到端：50 对配对 trial。

## 9.2 场景因素

- 物体类别；
- 货架低/中/高层；
- 左/中/右侧；
- 无/轻/重遮挡；
- 名义/横移/距离/朝向偏移；
- 目标静止/移动；
- 抓姿可达/不可达；
- 地图版本稳定/变化；
- 单故障/复合故障。

## 9.3 日志规范

每步至少记录：

```text
trial_id
scene_id
split
timestamp
agent_mode
phase
belief_digest
available_skills
filtered_skills
selected_skill
decision_source
reason
preconditions
capability_scores
llm_called
llm_latency_ms
tool_name
tool_args_hash
tool_result
error_code
postcondition_verified
base_owner
arm_owner
map_version
remaining_time
safety_blocked
task_success
failure_category
```

Agent 的每次决策都要可追溯，不能只保存最终“成功/失败”。

---

## 10. 完整实验方案

## 10.1 E0：平台可复现性实验

目的：

- 验证模型、ROS、MoveIt、RGB-D、TF 和仿真资产；
- 排除环境问题对算法结论的影响。

步骤：

1. 加载修复后的 MuJoCo 网格；
2. 验证正运动学；
3. 验证头部/腕部 RGB-D；
4. 验证相机外参和点云；
5. 验证左右臂 IK；
6. 验证升降联合规划；
7. 验证底盘 BaseLock；
8. 验证 MoveIt 抓取；
9. 验证抓取和放置日志。

通过标准：

- 模型加载无缺失网格；
- RGB-D 频率稳定；
- TF 唯一；
- 固定场景抓放成功；
- 故障注入服务可用。

## 10.2 E1：技能契约与安全壳单元实验

目的：

- 验证 Agent 不会调用非法技能；
- 验证前置条件和后置条件逻辑。

测试：

- 缺少目标时禁止抓取；
- 底盘未停稳时禁止 MoveIt；
- 地图版本不一致时撤销旧区域；
- ACT 激活时禁止 MoveIt；
- 传感器陈旧时返回 HOLD；
- 未知技能调用被拒绝；
- 非法参数被拒绝；
- 超时和取消传播正确。

指标：

- 单元测试覆盖率；
- 非法调用拦截率；
- 状态机死锁次数；
- 超时清理成功率。

通过标准：

- 所有安全单元测试通过；
- 不出现非零底盘与 MoveIt 并发；
- 不出现陈旧区域执行。

## 10.3 E2：基线闭环实验

比较：

- B0：固定状态机；
- B1：固定状态机 + 固定重试；
- B2：平面 ReAct Agent；
- B3：CARE-R（规则恢复）；
- B4：CARE-D（完整方法）。

场景：

- 名义货架抓放。

样本：

- 每个方法 30 次；
- 相同物体、起点和随机种子。

指标：

- 任务成功率；
- 阶段成功率；
- 完成时间；
- 工具调用数；
- 重试次数；
- Agent 调用延迟；
- 安全中止次数。

通过标准：

- CARE-R 与固定状态机在名义场景成功率接近；
- Agent 额外时间可测且可解释；
- 无安全违规。

## 10.4 E3：单故障恢复实验

重点故障：

1. 目标遮挡；
2. 相似商品；
3. 目标移动；
4. 抓姿不可达；
5. 抓取未附着；
6. 放置验证失败。

比较：

- 固定状态机；
- 平面 ReAct Agent；
- CARE-R；
- CARE-D。

样本：

- 每种故障 × 每种方法 20 次；
- 共 6 × 4 × 20 = 480 次，分批执行；
- 如果资源紧张，先完整跑 CARE-D 和两个关键基线。

指标：

- 恢复成功率；
- 无效动作次数；
- 安全中止次数；
- 恢复时间；
- 重观测次数；
- 禁用旧状态执行次数；
- 最终抓取成功率。

## 10.5 E4：复合故障实验

组合：

- 目标遮挡 + 底盘偏移；
- 相似商品 + 抓姿不可达；
- 目标移动 + 地图版本变化；
- 抓取未附着 + 重观测后目标不可见。

比较：

- 固定状态机；
- CARE-R；
- CARE-D。

样本：

- 每种组合 × 每种方法 20 次；
- 共 4 × 3 × 20 = 240 次。

指标：

- 最终成功率；
- 恢复动作序列；
- 最大重试深度；
- 超时率；
- 安全锁存率；
- 计划修复是否识别根因。

## 10.6 E5：消融实验

变体：

1. CARE-D 去掉能力评估；
2. 去掉技能前置/后置条件；
3. 去掉结构化记忆；
4. 去掉故障分类；
5. 去掉风险下界；
6. 只使用规则策略；
7. 去掉安全壳；

样本：

- 每个变体 20 次；
- 使用固定故障组合。

重点报告：

- 安全违规；
- 无效技能调用；
- 恢复成功率；
- 超时；
- 旧抓姿执行；
- 无意义重复重试。

## 10.7 E6：泛化实验

训练未见条件：

- 未见物体类别；
- 高低层组合；
- 侧边货架；
- 重度遮挡；
- 更大地图偏差；
- 更长恢复序列。

比较：

- CARE-R；
- CARE-D；
- 平面 Agent。

样本：

- 4 个泛化组 × 3 个方法 × 20 次；
- 共 240 次。

核心问题：

- 数据驱动恢复是否在训练分布外退化；
- 规则效用是否更稳健；
- 能力评估是否仍然是收益主要来源。

## 10.8 E7：运行时间与资源实验

记录：

- Agent决策耗时；
- LLM 调用耗时；
- 工具调用耗时；
- TC-WAM 候选计算时间；
- MoveIt 规划时间；
- GPU 峰值显存；
- 系统峰值内存；
- CPU 占用；
- trial 重启时间。

比较：

- 大模型关闭；
- Qwen 4-bit；
- Qwen 小模型替代；
- 规则 Agent；
- 完整 Agent。

目的：

- 证明系统在 8 GiB GPU / 16 GiB 内存上可运行；
- 找到模型调用频率和系统稳定性之间的平衡。

## 10.9 E8：最终端到端实验

固定最终方法：

```text
CARE-D + TC-WAM + Safety Critic
```

对照：

```text
FSM
FSM + rule retry
flat ReAct
CARE-D without capability
```

条件：

- 名义；
- 单故障；
- 复合故障；
- 未见物体；
- 高层/低层；
- 遮挡。

主方法至少 50 次，关键对照 30～50 次。使用配对 seed。

输出：

- 分阶段成功率；
- 总成功率；
- Wilson 置信区间；
- 平均完成时间；
- 恢复率；
- 安全违规率；
- 无效工具调用率；
- Agent决策轨迹案例。

---

## 11. 评价指标与统计方法

## 11.1 主指标

1. 端到端任务成功率；
2. 故障恢复成功率；
3. 安全违规次数；
4. 无效动作率；
5. 平均完成时间；
6. 平均重试次数；
7. 平均 Agent 调用延迟。

## 11.2 辅助指标

- 目标定位正确率；
- 相似目标区分率；
- 抓取规划成功率；
- 抓取验证成功率；
- 放置验证成功率；
- 陈旧状态执行次数；
- LLM 非法输出率；
- LLM 回退率。

## 11.3 统计

- 成功率：Wilson 95% 置信区间；
- 连续指标：均值、标准差、Bootstrap 95% 区间；
- 相同 seed：配对 Bootstrap；
- 成对成功率：McNemar；
- 连续非正态：Wilcoxon；
- 多组比较：Holm-Bonferroni；
- 报告效应量；
- 不只报告 p 值；
- 失败样本不得丢弃。

## 11.4 结果有效性

必须讨论：

- 仿真接触模型真实性；
- 确定性附着代理；
- Qwen 域偏移；
- GraspNet 在合成场景上的性能；
- 规则数据驱动模型的数据泄漏；
- 8 GiB 显存导致的模型替换；
- 单一机器人平台；
- 固定货架布局；
- 无真机结果。

---

## 12. 40 周实施计划

## 12.1 总体安排

| 阶段 | 周数 | 目标 |
|---|---:|---|
| P0 | 1-4 | 修复仿真和硬件环境 |
| P1 | 5-8 | 固定状态机基线 |
| P2 | 9-13 | Agent状态、技能图和安全壳 |
| P3 | 14-18 | TC-WAM能力评估 |
| P4 | 19-23 | 故障恢复和LLM接口 |
| P5 | 24-27 | 数据采集和CARE-D训练 |
| P6 | 28-32 | 主实验和消融 |
| P7 | 33-36 | 泛化、资源、补充实验 |
| P8 | 37-40 | 论文、答辩和复现包 |

## 12.2 W1-W4：环境复现

- 修复 Git LFS；
- 重建 MJCF；
- 重编 ROS install；
- 修复 GraspNet KNN；
- 验证 GPU、MuJoCo、RGB-D、MoveIt；
- 完成固定抓放。

交付：

- `baseline-env-v1` tag；
- 环境说明；
- 启动脚本；
- 第一份 rosbag。

Go/No-Go：

- 模型无法加载；
- MoveIt 不能执行；
- RGB-D 坐标不一致；
- 同时存在两个底盘发布者。

上述任一失败，不进入下一阶段。

## 12.3 W5-W8：固定状态机基线

- 使用现有 `ExecuteTransport`；
- 适配单目标货架场景；
- 实现 deterministic trial runner；
- 保存失败码和事件日志；
- 输出 baseline 成功率。

交付：

- 30 次名义 trial；
- 事件日志；
- 分阶段失败统计。

Go/No-Go：

- 名义成功率稳定；
- 故障注入可重复；
- trial 可自动重启。

## 12.4 W9-W13：Agent状态和技能图

- 定义 `BeliefState`；
- 定义 `SkillContract`；
- 定义技能图；
- 实现前置/后置条件；
- 实现 Safety Critic；
- 编写单元测试。

交付：

- `openflex_care_agent` 包；
- 技能配置；
- 状态机和单元测试；
- CARE-R 基础流程。

Go/No-Go：

- 没有非法动作通过；
- 无控制权冲突；
- 所有技能超时和取消正确清理。

## 12.5 W14-W18：TC-WAM能力接入

- 将候选实例、抓姿、底盘和升降解接入；
- 输出物理可行候选；
- 输出风险下界和 ready region；
- Agent 只使用可行能力；
- 对比 fixed/reach-only/TC-WAM。

交付：

- 能力评估器；
- 热力图和 ready region；
- 候选数据；
- 基线对比。

Go/No-Go：

- 能力评估不能泄露执行接口之外的真值；
- 高风险与低风险候选成功率有可区分趋势；
- 计算时间可接受。

## 12.6 W19-W23：恢复策略

- 实现失败码；
- 实现状态更新；
- 实现 CARE-R；
- 接入 Qwen 任务解析和失败归因；
- 实现规则回退；
- 引入黑盒状态测试。

交付：

- 故障状态机；
- 恢复动作表；
- 单故障 pilot；
- LLM 审计日志。

Go/No-Go：

- 大模型不可用时仍能安全运行；
- 非法 JSON 和未知动作全部被拒绝；
- 安全锁存可靠。

## 12.7 W24-W27：CARE-D数据与训练

- 采集恢复决策数据；
- 清洗并冻结数据 schema；
- 训练 LightGBM；
- 概率校准；
- 在 calibration set 上冻结阈值；
- 做离线策略比较。

交付：

- 训练集、校准集、测试集；
- 恢复模型；
- 校准曲线；
- CARE-D pilot。

Go/No-Go：

- 测试集无数据泄漏；
- 未校准模型和校准模型可区分；
- 未见故障类型退化被记录。

## 12.8 W28-W32：主实验

- E2 基线；
- E3 单故障；
- E4 复合故障；
- E5 消融；
- 生成论文主表。

交付：

- 所有主实验报告；
- 成功/失败分类；
- 安全事件表。

## 12.9 W33-W36：补充与复现

- E6 泛化；
- E7 资源；
- E8 端到端；
- 重跑关键实验；
- 修复数据和配置问题。

## 12.10 W37-W40：论文与答辩

- 完成论文；
- 生成所有图；
- 整理复现包；
- 完成答辩 PPT；
- 做一次完整重复实验。

---

## 13. 论文结构

### 第 1 章 绪论

- 商超移动操作背景；
- Agent 与具身智能；
- 技能图、任务规划和故障恢复；
- 研究问题与贡献。

### 第 2 章 平台与任务建模

- OpenFlex；
- MuJoCo；
- ROS 2/Nav2/MoveIt；
- Qwen/SAM2/GraspNet；
- 任务链和坐标系。

### 第 3 章 能力感知分层具身智能体建模

- 信念状态；
- 技能契约；
- 技能图；
- 安全壳；
- 控制权约束。

### 第 4 章 物理能力评估与动作选择

- TC-WAM；
- 多扰动采样；
- 风险下界；
- ready region；
- 能力约束动作选择。

### 第 5 章 故障感知的任务恢复

- 故障分类；
- 恢复动作；
- CARE-R；
- CARE-D；
- 计划修复算法。

### 第 6 章 仿真实验与结果

- 环境；
- 实验协议；
- 基线；
- 单故障；
- 复合故障；
- 消融；
- 泛化；
- 资源；
- 端到端。

### 第 7 章 总结与展望

- 贡献；
- 仿真边界；
- 真机迁移；
- 后续多任务和VLA拓展。

---

## 14. 论文创新点最终写法

### 创新点一

提出一种带物理前置条件、后置条件和失败语义的技能图建模方法，将移动机械臂的导航、
观察、对位、抓取、验证和恢复能力统一为受约束的可调用技能，使 Agent 只能在
安全且物理可执行的候选动作中决策。

### 创新点二

提出一种基于风险受控可操作性地图的能力感知动作选择方法，将底盘位姿、升降机械臂
联合 IK 解、抓取姿态、碰撞风险和成功概率下界纳入 Agent 决策，避免视觉语言层
产生物理不可执行计划。

### 创新点三

提出一种基于信念状态和结构化失败信息的计划修复方法，根据目标可见性、抓取可行性、
地图版本、剩余时间和历史恢复结果，在重观测、换实例、重定位、重规划和终止之间
自适应选择，提升长时序移动操作在复合故障下的恢复能力。

### 非创新点

- ROS 2；
- MuJoCo；
- MoveIt；
- Qwen；
- SAM2；
- GraspNet；
- 普通日志；
- Prompt；
- ACT；
- 普通状态机。

---

## 15. 交付物

### 代码

- `openflex_care_agent`；
- 技能图配置；
- 安全壳；
- TC-WAM 适配器；
- 故障注入器；
- trial runner；
- 统计和绘图工具。

### 数据

- 场景 manifest；
- 候选和能力数据；
- 恢复决策数据；
- Agent 决策轨迹；
- 结果 JSONL；
- 代表性 rosbag；
- 模型和校准器。

### 文档

- 环境搭建；
- 一键复现；
- 接口契约；
- 安全说明；
- 实验协议；
- 数据字典；
- 真机迁移清单。

### 论文图表

1. CARE-Agent总体架构；
2. 世界状态更新流；
3. 技能图；
4. 能力评估与 ready region；
5. 故障恢复状态机；
6. 单故障恢复对比；
7. 复合故障恢复序列；
8. 消融实验；
9. 时间和资源开销；
10. 分阶段成功率。

---

## 16. 风险与降级方案

| 风险 | 处理 |
|---|---|
| 16 GiB 内存不足 | 串行运行、headless、只开一个 ROS 栈；有条件升级到 32/64 GiB |
| Qwen 4B 显存不够 | 4-bit、2B 模型、GroundingDINO/OWLv2；Qwen只做离线解析 |
| MuJoCo资产缺失 | Git LFS或从本地STL/DAE重建 |
| GraspNet KNN失败 | 修复扩展或用PyTorch top-k替代 |
| CARE-D训练不足 | 主方法降为CARE-R，学习模块作为补充实验 |
| Agent调用延迟高 | 只在事件触发时调用；正常流程用FSM |
| 平面Agent不稳定 | 作为对照即可，不作为主方法 |
| TC-WAM计算慢 | 粗筛+补采样；缓存场景和候选 |
| 端到端trial太慢 | 分阶段实验为主，少量配对端到端 |
| 抓取附着过于理想 | 标注为执行代理，补充接触/滑移实验 |
| ACT训练失败 | 移除ACT，主系统不依赖 |

---

## 17. 立即执行的 14 天清单

### 第 1-2 天

- 修复 Git LFS；
- 拉取或重建 MuJoCo 网格；
- 验证 OpenFlex MJCF。

### 第 3-4 天

- 重建 ROS 工作空间；
- 验证 MoveIt；
- 验证 Nav2。

### 第 5-6 天

- 修复 GraspNet KNN；
- 运行一次离线抓姿；
- 记录 GPU 峰值。

### 第 7-8 天

- 验证 MuJoCo headless RGB-D；
- 验证相机和点云；
- 测量内存占用。

### 第 9-10 天

- 跑通固定底盘抓放；
- 实现 trial 日志；
- 冻结 `baseline-env-v1`。

### 第 11-12 天

- 建立 `openflex_care_agent` 包；
- 写 `state.py`、`skill_contracts.py`、`safety_guard.py`；
- 完成无 ROS 单元测试。

### 第 13-14 天

- 将现有 Action 注册到工具适配器；
- 跑通 `look_for_object -> navigate -> align -> grasp` 的规则智能体；
- 生成第一份 Agent 决策 trace。

---

## 18. 最终执行原则

1. 先修环境，再跑基线；
2. 先做确定性状态机，再做 Agent；
3. 先做规则恢复，再做学习恢复；
4. 先做单故障，再做复合故障；
5. 先做分阶段实验，再做端到端；
6. 正常流程不调用大模型；
7. 大模型只提出高层动作；
8. 物理执行必须通过硬约束；
9. TC-WAM只负责能力判断，不控制硬件；
10. Agent只负责决策，不直接控制硬件；
11. 所有失败都要保留证据；
12. 所有论文结论必须能由冻结数据和脚本复算。

**最终主线：CARE-Agent 负责“下一步做什么”，TC-WAM 负责“是否物理可行”，
Nav2/MoveIt 负责“如何执行”，安全壳负责“是否允许执行”。**
