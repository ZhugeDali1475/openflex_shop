# 硕士学位论文最终结构：AnyGrasp抓姿条件化TC-WAM主线

## 建议题目

**面向货架移动抓取的抓姿条件化全身可操作性地图与局部精细对位研究**

论文定位统一为：

> AnyGrasp抓姿条件化的TC-WAM联合底盘—升降柱/机械臂构型—抓姿选择，以及
> ACT驱动的局部底盘精细对位策略。

所有性能数值保留为 `【待真实实验填写】`。软件框架、synthetic smoke 或 mock
热力图不能写成物理实验结果。

# 摘要（重写方向）

轮式双臂机器人执行货架取物时，Nav2 可以到达目标邻域，但导航残余误差、目标感知
误差以及机械臂可达域的非均匀性，使固定操作位姿难以稳定支持抓取。AnyGrasp 能从
点云生成多种六自由度抓姿，却不能独立决定移动底盘应停在哪里，也不能保证某一抓姿
在机器人升降柱—机械臂联合约束下具有足够裕量。仅让学习策略端到端输出全身动作，
则会弱化物理可行性的显式约束和失败可解释性。

本文面向单目标货架抓取—持物运输—固定交付任务，提出 AnyGrasp 抓姿条件化的任务
约束全身可操作性地图 TC-WAM。机器人由 Nav2 到达观察/交接区后，第一次 AnyGrasp
生成 Top-K 粗抓姿。TC-WAM 在候选底盘位姿、升降柱—活动臂联合 IK 解层和抓姿之间
计算

\[
(b^*,h^*,g^*)=\arg\max_{b,h,g_k\in\mathcal F}S(b,h,g_k),
\]

并输出推荐联合计划、可解释热力图及与唯一 \((h^*,g^*)\) 严格绑定的可抓取就绪
区域。该区域只保留包含最优底盘位姿的连通高分分量，避免不同抓姿或升降解的高分格
被错误合并。

本文进一步采用 ACT 学习从 Nav2 交接区进入上述 ready 区域的局部底盘策略。ACT
仅输出全向底盘三维速度，不控制升降柱、机械臂或夹爪。进入绑定 ready 区域并持续
低速后，系统取消 ACT action chunk、锁定底盘，并调用第二次 AnyGrasp 利用近距离
点云更新抓姿。MoveIt 从实际关节状态使用 `left_arm_with_lift` 或
`right_arm_with_lift` 联合规划升降柱与活动臂，并在同一执行阶段控制夹爪、物体
附着和运输姿态。若近场抓姿在当前底盘全部不可达，系统局部重算 TC-WAM 并再次调用
ACT 对位。

实验分别验证：抓姿条件化 TC-WAM 分数与真实抓取成功率的关系；单一抓姿 ready 区域
对导航/PID/ACT 局部对位的作用；近场抓姿更新与局部重算；目标移位和短时遮挡恢复；
以及完整取送闭环。实验结果为【待真实实验填写】。

关键词：移动操作；AnyGrasp；全身可操作性地图；ACT；MoveIt；局部精细对位

# 第1章 绪论

## 1.1 研究背景与问题

围绕三个层次展开：

1. “到达货架邻域”不等于“处于某个抓姿的鲁棒可执行区域”；
2. 抓姿质量、底盘位姿和升降柱—机械臂联合构型相互耦合，不能分别贪心选择；
3. 局部学习控制应处理感知和底盘误差，但不应取代最终运动学、碰撞和轨迹执行约束。

由此采用“Nav2粗导航 + 物理地图决策 + ACT底盘对位 + MoveIt操作执行”的分层路线。

## 1.2 国内外研究现状

建议分为：

- 移动操作与操作底盘位姿选择；
- 机器人可达图、可操作度与全身规划；
- 点云抓姿生成与 AnyGrasp 类方法；
- ACT、模仿学习和局部移动控制；
- 动态目标状态维护与事件触发重规划。

不展开比赛业务、通用 VLA 或 π₀.₅。

## 1.3 问题定义

目标状态为

\[
o_t=(T_o^m,\Sigma_o,c_t,\tau_t,\nu_t),
\]

分别表示目标地图位姿、协方差、置信度、观测时刻和 map version。第一次 AnyGrasp
输出集合 \(\mathcal G_c=\{g_k^c\}_{k=1}^{K}\)。候选底盘为
\(b=(x,y,\theta)\)，联合操作解层 \(h\) 属于指定活动臂的 `with_lift` 规划组。

任务流程固定为：

\[
\text{Nav2}\rightarrow\text{AnyGrasp}_{coarse}\rightarrow\text{TC-WAM}
\rightarrow\text{ACT}_{base}\rightarrow\text{AnyGrasp}_{refined}
\rightarrow\text{MoveIt}_{lift+arm+gripper}\rightarrow\text{Nav2运输}.
\]

## 1.4 研究问题与假设

- RQ1：联合评价 \(S(b,h,g)\) 能否比固定操作位姿或 Reach-only 更好地区分真实抓取
  成功率？
- RQ2：与单点追踪相比，ACT 进入单一 \((h,g)\) 绑定 ready 区域能否提高对位与下游
  MoveIt 抓取成功率？
- RQ3：第二次 AnyGrasp 后采用“当前位姿验证优先、不可达再局部重算”能否提高恢复
  效率，并在目标移位/遮挡时保持安全？

对应假设：

- H1：Full TC-WAM 的高、中、低分区具有单调不同的经验抓取成功率；
- H2：抓姿绑定 ready 条件的 ACT 在未见初始偏移上优于 Nav2 单点、PID/MPC 和
  ACT 单点；
- H3：近场更新与版本化恢复降低无效 MoveIt 执行，遮挡期间盲抓次数为零。

## 1.5 创新点

1. 提出 AnyGrasp 抓姿条件化 TC-WAM，在底盘位姿、升降柱—活动臂联合 IK 解和抓姿
   间进行统一硬门控与软评分，输出 \((b^*,h^*,g^*)\)。
2. 提出与唯一 \((h^*,g^*)\) 严格绑定的 ready 区域，并用 ACT 学习仅靠底盘速度
   进入该鲁棒可执行集合，而非控制机械臂或追踪唯一几何点。
3. 构建粗/近两阶段 AnyGrasp、MoveIt 当前状态验证和不可达局部重算机制，实现分层
   控制权互斥和有限动态恢复。

ROS 接口、比赛任务执行器或单个 demo 不单独作为算法创新。

# 第2章 平台、任务边界与分层控制

## 2.1 机器人与MoveIt联合规划组

介绍全向底盘、升降柱、双臂、夹爪、相机、MID-360 和 ROS 2。主实验每次只启用一臂。
MoveIt 的 `left_arm_with_lift` 和 `right_arm_with_lift` 均包含 `lift_joint` 与对应
七轴机械臂，因此升降柱和活动臂可以联合规划和执行，而非先固定升降高度再规划手臂。

夹爪可以由独立 `GripperCommand` controller 驱动，但命令时序与所有权属于 MoveIt
抓取执行阶段。

## 2.2 模块分工

| 模块 | 计算职责 | 硬件职责 |
|---|---|---|
| Nav2 | 全局路径 | 底盘粗导航/运输 |
| AnyGrasp-1 | Top-K粗抓姿 | 无 |
| TC-WAM | 选择 `(b,h,g)` 与ready | 无 |
| ACT | ready条件下的局部策略 | 仅底盘速度 |
| AnyGrasp-2 | 近场抓姿更新 | 无 |
| MoveIt | 最终IK、碰撞和轨迹 | 升降柱、活动臂、夹爪 |

## 2.3 坐标系与TF

- `map`：Nav2、对象记忆与候选底盘位姿；
- `odom`：连续局部运动；
- `base_link`：ACT 速度和目标残差；
- 对象/相机坐标：AnyGrasp 输入输出；
- 左右真实 TCP：MoveIt IK 目标。

系统只能存在一个 `map->odom->base_link` 链和一个机器人关节 TF 发布者。导航运行时
禁止固定 `world->base_link`。

## 2.4 ACT冻结观测与动作

数值观测为 24 维：

\[
x_t=[p_t^{episode}(3),v_t^{measured}(3),z_t^{aff}(18)].
\]

动作固定为

\[
a_t=[v_x,v_y,\omega_z]_{base\_link}.
\]

ACT 不输出升降柱、机械臂或夹爪动作。历史 14-D/32-D 路径 checkpoint 和
29-D/11-D 手臂—夹爪—底盘 checkpoint 均与该 schema 不兼容。

## 2.5 两级控制权

- 底盘：`NAV2 / ACT / RECOVERY / BaseLock` 唯一所有者；
- 操作机构：`MOVEIT / ArmHold` 唯一所有者；
- `MOVEIT` 模式下底盘恒为零且由实测里程计确认停稳；
- ACT disabled 后必须清空 action chunk，方可进入 MoveIt。

# 第3章 AnyGrasp抓姿条件化TC-WAM

本章是论文核心。

## 3.1 第一次AnyGrasp与候选集合

Nav2 到观察区并停稳后，使用远距离点云生成 Top-K 粗抓姿。每个 \(g_k\) 至少包含
抓取 TCP、预抓取 TCP、approach、夹爪宽度、质量、可见性、观测序号和物理感知标志。
抓姿必须与对象 id、map version 和活动臂一致。

## 3.2 底盘与联合构型候选

在目标自由空间侧采样底盘位姿 \(b_i\)。对每个 `with_lift` 规划组设置多个 lift IK
seed，用于搜索不同升降柱—机械臂冗余解层 \(h_j\)。lift seed 不是最终独立命令，
真实 \(h\) 由完整联合 IK 解确定。

候选集合为

\[
\mathcal C=\{(b_i,h_j,g_k)\}.
\]

## 3.3 坐标变换与MoveIt只读评价

对假设底盘 \(b_i\)，把预抓取/抓取 TCP 和货架碰撞体变换到候选 `base_link`，使用
`left_arm_with_lift` 或 `right_arm_with_lift` 调用 IK 与 StateValidity。该阶段只读，
不发送轨迹。

## 3.4 硬门控

\[
F_{ijk}=F^{ground}_{i}\land F^{path}_{i}\land F^{IK}_{ijk}
\land F^{collision}_{ijk}.
\]

预抓取或抓取 IK 失败、碰撞、规划组/TCP 不一致均令分数为零，不能被抓姿质量等软分
补偿。

## 3.5 软评分

对硬可行候选计算可达性 \(R\)、最小归一化关节余量 \(J\)、扰动鲁棒率 \(U\)、
AnyGrasp 质量 \(Q\)、可见性 \(V\) 和移动得分 \(D\)：

\[
S_{ijk}=F_{ijk}(w_RR_{ijk}+w_JJ_{ijk}+w_UU_{ijk}
+w_QQ_k+w_VV_k+w_DD_i),\quad \sum w=1.
\]

扰动分布由目标定位误差和 Nav2 到位误差标定获得，权重在正式实验前冻结。

## 3.6 最优联合选择

\[
(b^*,h^*,g^*)=\arg\max_{(b,h,g)\in\mathcal C}S(b,h,g).
\]

输出的 `ManipulationPlan` 保存规划组、抓姿、实际联合 IK 解、lift position、预抓取和
抓取关节状态，供第二阶段验证与日志追溯。

## 3.7 抓姿绑定ready区域

二维热力图可在每个底盘格显示跨候选最大分，仅供可视化；控制 ready 集必须先固定
最优候选的活动臂、规划组、抓姿、升降解和版本：

\[
\mathcal R_{h^*,g^*}=\operatorname{CC}_{b^*}
\left\{b\mid F(b,h^*,g^*)=1,
S(b,h^*,g^*)\ge\max(\tau_{abs},\rho S^*)\right\}.
\]

`CC` 只保留包含 \(b^*\) 的连通分量。ready 消息必须携带 `grasp_id`、
`lift_position`、`planning_group`、`ready_region_id` 和 `map_version`，禁止混合不同
抓姿的高分格。

## 3.8 18维紧凑状态

顺序固定为：目标残差5、推荐底盘残差4、当前/最优得分与三种物理裕量5、抓姿质量1、
选定 lift position1、ready distance1、ready 标志1，共18维。ID 和版本作为
fail-closed 元数据，不作为连续归一化特征。

## 3.9 缓存与失效

缓存键至少包含对象/观测版本、活动臂、抓姿响应、规划组、机器人模型与货架场景版本。
目标移位、丢失后重观测或抓姿集合更新均使旧绑定 ready 区域失效。

# 第4章 ACT局部底盘对位与近场抓取更新

## 4.1 Nav2—ACT交接

Nav2 成功到观察区后先切 HOLD，实测速度连续低于阈值，再重置 ACT 回合原点。ACT 只在
本地交接区域运行，不在 Nav2 行驶中预伸机械臂。

## 4.2 ACT策略

\[
\hat a_{t:t+H}^{base}=\pi_{ACT}
(I_t,p_t^{episode},v_t^{measured},z_t^{aff}),
\quad a=[v_x,v_y,\omega_z].
\]

研究点是物理验证的抓姿绑定条件能否帮助策略进入非规则可抓区域，不宣称提出新的
Transformer 结构。

## 4.3 策略停止条件

只有当前底盘处于 `ReadyRegion.member_base_poses` 的容差范围、绑定完全一致、对象可见
且实测速度持续低于阈值，才判定对位完成。随后：disabled ACT、发布零速度、清空 chunk、
切换 BaseLock 并再次确认停稳。

## 4.4 第二次AnyGrasp

使用近距离点云生成 \(\mathcal G_r\)。固定当前实际底盘 \(b_{cur}\)，MoveIt 对新抓姿
逐一执行联合 IK 和碰撞验证：

\[
g_r^*=\arg\max_{g\in\mathcal G_r\cap\mathcal F(b_{cur})}Q(g).
\]

新抓姿无需与粗阶段 `g*` 完全相同。存在可达者时直接进入规划；全部不可达时才重算
局部 TC-WAM。

## 4.5 MoveIt独占操作执行

MoveIt 从当前 `/joint_states` 使用 `left/right_arm_with_lift` 规划预抓取与抓取联合
轨迹，控制夹爪，添加 AttachedCollisionObject，并进入升降柱—活动臂联合运输姿态。
执行期间底盘保持 `MOVEIT` BaseLock，ACT 无任何操作机构发布器。

## 4.6 局部重算与事件恢复

近场抓姿全部不可达时，执行器返回 `reposition_required`。状态机用新抓姿集合重算
TC-WAM，生成新的 `(b*,h*,g*)` 和 ready 绑定，再调用 ACT。重试次数预注册；超过上限
失败。目标移位、丢失或 map version 变化同样撤销旧抓姿、旧图和旧 chunk。

遮挡记忆只能指导重观测，不能授权最终夹爪闭合。

# 第5章 ROS 2工程实现

## 5.1 独立论文工程

介绍五个 `openflex_thesis_*` 包，与比赛消息和执行器零依赖，只复用硬件、Nav2、
AnyGrasp、MoveIt 和相机。

## 5.2 主要接口

- `GenerateGrasps`：COARSE/REFINED 两阶段 gateway；
- `EvaluateCandidates`：物理只读 `(b,h,g)` 评估；
- `ComputeTcWam`：联合计划、热力图和 ready 绑定；
- `ExecuteLocalAlignment`：ACT base-only 对位；
- `ExecuteMoveItGrasp`：当前位姿验证、联合执行或请求重对位；
- `RunPickDeliver`：完整状态机。

## 5.3 控制和安全

Nav2、ACT、Recovery 有独立速度输入，arbiter 产生唯一底盘输出并进入 collision monitor。
MOVEIT、PLACE 和 HOLD 模式恒为零。实测里程计陈旧、对象/版本/绑定失效时 fail closed。

## 5.4 物理与synthetic边界

mock AnyGrasp 强制 `physical_perception=false`，mock evaluator 强制
`physical_evaluation=false`。正式运行必须使用真实关节状态、标定 Planning Scene、
真实 TCP、物理点云和可执行控制器。

## 5.5 可复现性

保存 protocol、manifest、代码/配置版本、两次 AnyGrasp request 和 grasp IDs、联合计划、
ready region id、ACT episode、MoveIt 验证、重算次数、rosbag、视频和独立抓放证据。

# 第6章 实验设计与结果

## 6.1 前置标定

- 目标定位与 AnyGrasp 重复性；
- Nav2 观察区到位误差和停止时间；
- TCP、SRDF、Planning Scene 和联合规划组；
- 固定底盘固定物体的升降柱—活动臂联合抓取至少10次。

固定抓取建议达到至少8/10后再推进。

## 6.2 实验一：TC-WAM物理有效性

比较 Fixed、Reach-only、Full `(b,h,g)`。按预注册高/中/低分候选随机执行，报告成功率
和 Wilson 区间、Spearman 相关、IK/碰撞、关节余量、鲁棒率、抓姿质量与计算时间。

增加 ready 绑定审计：跨抓姿、lift stratum 或不连通孤岛的混入次数必须为零。

## 6.3 实验二：局部精细对位

| 方法 | 目标 | 绑定ready到达率 | 耗时 | 底盘距离 | 下游MoveIt抓取率 |
|---|---|---:|---:|---:|---:|
| Nav2-direct | 经验点 | 【待填】 | 【待填】 | 【待填】 | 【待填】 |
| PID/MPC | `b*` | 【待填】 | 【待填】 | 【待填】 | 【待填】 |
| ACT-point | `b*` | 【待填】 | 【待填】 | 【待填】 | 【待填】 |
| ACT-ready | 绑定ready集合 | 【待填】 | 【待填】 | 【待填】 | 【待填】 |

所有 ACT 组使用相同3维底盘动作、数据预算、主干、chunk、速度限制、随机种子和未见
起点。不得将抓取动作归因于 ACT。

## 6.4 实验三：TC-WAM条件消融

可比较 M0（图像+回合底盘）、M1（+目标残差）和 M2（+18维 TC-WAM），但三组都只
输出3维底盘速度。主指标为绑定 ready 到达率；抓取率是 MoveIt 下游验证。

## 6.5 实验四：近场更新与局部重算

预设当前底盘可达/不可达两组，报告近场 grasp id 变化、当前状态验证率、重对位触发率、
局部重算成功率、恢复耗时、重试次数和最终抓取成功率。

## 6.6 实验五：移位与遮挡

比较 Static 与 Event-memory。报告旧抓姿/ready 撤销、停止延迟、重观测、重算和最终
成功率；遮挡期间夹爪闭合次数必须为零。

## 6.7 端到端闭环

至少20次逐阶段报告：Nav2观察、粗抓姿、联合TC-WAM、ACT对位、近场抓姿、MoveIt抓取、
附着运输姿态、Nav2运输、固定放置和总成功率。

## 6.8 有效性威胁

讨论已知对象、点云质量、单平台、单活动臂、固定货架、AnyGrasp域偏移、少量真机样本、
固定放置和未验证跨机器人泛化。

# 第7章 总结与展望

总结顺序为：抓姿条件化联合地图、单抓姿 ready 集、ACT底盘对位、近场复核与重算、
系统闭环。只用真实数据回答 H1-H3。

展望可包括学习式评分、在线深度碰撞场、双臂选择、导航中保守预伸、放置可操作性和
通用对象识别，但不得写成本文已经完成。

# 附录A 必需图表

| 图表 | 内容 |
|---|---|
| 图1-1 | 分层技术路线与研究问题 |
| 图2-1 | Nav2/ACT/BaseLock/MoveIt控制权时序 |
| 图3-1 | `(b,h,g)` 候选和联合评分流程 |
| 图3-2 | AnyGrasp Top-K、lift-arm解层与底盘格 |
| 图3-3 | 分量热力图和最终得分 |
| 图3-4 | 单一 `(h*,g*)` ready 连通区域 |
| 图4-1 | 两阶段AnyGrasp与不可达局部重算 |
| 图6-1 | 分数—经验成功率校准 |
| 图6-2 | Nav2/PID/ACT-point/ACT-ready 对比 |
| 图6-3 | 近场更新和恢复时间轴 |
| 图6-4 | 端到端分阶段成功率 |

# 附录B 定稿检查

- [ ] 论文包无比赛消息/执行器依赖；
- [ ] `h` 明确定义为升降柱—活动臂联合 IK/规划解，不是 ACT 升降命令；
- [ ] ACT 为24维数值观测、3维底盘动作，且无操作机构发布器；
- [ ] ready 区域绑定单一 grasp/lift/group/version 并只含 `b*` 连通分量；
- [ ] MoveIt 使用真实 `left/right_arm_with_lift`、当前关节状态和标定场景；
- [ ] 两次 AnyGrasp 均保存物理感知标志、request id 和候选；
- [ ] MoveIt 不可达只触发有限次局部重算；
- [ ] MOVEIT 执行时底盘实测停稳且仲裁输出恒为零；
- [ ] 所有正式结果可追溯到 manifest、日志、rosbag 和视频；
- [ ] 摘要、创新点与结论没有超出真实证据。

