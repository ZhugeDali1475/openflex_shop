# AnyGrasp抓姿条件化TC-WAM工程与实验流程

## 1. 冻结的技术路线

论文只研究单目标、单活动臂的货架取送闭环。最终方案为：

```text
Nav2粗导航
-> 第一次AnyGrasp粗抓姿
-> TC-WAM联合选择(b*, h*, g*)和绑定ready区域
-> ACT仅控制底盘精对位
-> 底盘锁定
-> 第二次AnyGrasp近场更新
-> MoveIt联合执行升降柱+活动臂并控制夹爪
-> 不可达时局部重算和再次对位
-> 抓取验证、附着、运输姿态
-> Nav2运输和固定交付
```

模块职责固定如下：

| 模块 | 最终职责 | 硬件控制权 |
|---|---|---|
| Nav2 | 长距离到观察区、抓取后运输 | 仅底盘 |
| 第一次AnyGrasp | 远距离点云Top-K粗抓姿 | 无 |
| TC-WAM | 联合选择 `(b*,h*,g*)` 和绑定ready区域 | 无 |
| ACT | 交接区到ready区域的短距离精对位 | 仅底盘3维速度 |
| 第二次AnyGrasp | 近距离点云更新和复核抓姿 | 无 |
| MoveIt | 最终IK、碰撞、升降柱+活动臂轨迹、夹爪、附着和运输姿态 | 操作机构独占 |
| 仲裁器 | Nav2/ACT/恢复底盘源互斥；MoveIt期间BaseLock | 唯一底盘出口 |

这里的 \(h\) 是 `left_arm_with_lift` 或 `right_arm_with_lift` 的联合 IK 解层，
包含 `lift_joint` 和七轴活动臂。ACT 不控制 \(h\)。

不研究比赛清单、补货、错放归位、开放词汇、双臂同时抓取、导航中伸臂、放置
TC-WAM 或 π₀.₅。

## 2. 工程总体结构

```mermaid
flowchart LR
    Goal[RunPickDeliver] --> FSM[论文任务状态机]
    FSM -->|全局阶段| Nav2[Nav2]
    Nav2 --> NavCmd[/thesis/nav_cmd_vel]

    Camera[点云/目标观测] --> Memory[事件对象记忆]
    FSM -->|COARSE| AG1[AnyGrasp Gateway]
    Memory --> AG1
    AG1 --> G1[Top-K粗抓姿]
    G1 --> WAM[TC-WAM]
    Memory --> WAM
    WAM --> Eval[MoveIt只读联合IK/碰撞评估]
    Eval --> Plan[(b*,h*,g*) + 绑定ready]

    Plan --> Align[ACT对位监督器]
    Align --> ACT[ACT base-only policy]
    ACT --> ActCmd[/thesis/act_cmd_vel]

    NavCmd --> Arbiter[底盘速度仲裁/BaseLock]
    ActCmd --> Arbiter
    Arbiter --> Safe[/thesis/cmd_vel_selected -> collision monitor]
    Safe --> Base[全向底盘]

    FSM -->|REFINED且BaseLock| AG2[第二次AnyGrasp]
    AG2 --> Near[近场Top-K]
    Near --> MoveIt[MoveIt抓取执行器]
    Plan --> MoveIt
    MoveIt --> LiftArm[升降柱+活动臂联合轨迹]
    MoveIt --> Gripper[夹爪/附着/运输姿态]
    Verify[独立抓取证据] --> MoveIt
    MoveIt -->|reposition_required| WAM

    FSM --> Place[固定放置]
    FSM --> Logger[trial logger]
```

`openarmx_task_msgs` 和 `openarmx_task_executor` 不在依赖图中。

## 3. 真机任务流程

```mermaid
flowchart TD
    S[收到单目标任务] --> V{字段和活动臂有效?}
    V -- 否 --> F[FAIL并HOLD]
    V -- 是 --> N[Nav2到观察/交接区]
    N --> H1[撤销Nav2并输出零速度]
    H1 --> Stop1{实测速度持续低于阈值?}
    Stop1 -- 否/超时 --> F
    Stop1 -- 是 --> Reset[重置ACT局部回合坐标]
    Reset --> Obs[确认目标当前可见和版本新鲜]
    Obs --> AG1[第一次AnyGrasp: COARSE Top-K]
    AG1 --> WAM[联合评价S b,h,g]
    WAM --> Valid{存在物理有效(b*,h*,g*)?}
    Valid -- 否 --> RecoverObs[重观测或失败]
    RecoverObs --> Obs
    Valid -- 是 --> Bind[冻结绑定ready区域]
    Bind --> A[授予ACT底盘控制权]
    A --> Ready{进入绑定ready且持续低速?}
    Ready -- 否 --> Event{对象/版本/绑定失效?}
    Event -- 否 --> A
    Event -- 是 --> H1
    Ready -- 是 --> Cancel[禁用ACT并清action chunk]
    Cancel --> Lock[BaseLock并再次确认停稳]
    Lock --> AG2[第二次AnyGrasp: REFINED Top-K]
    AG2 --> M[进入MOVEIT模式]
    M --> Reach{当前实际底盘存在可达新抓姿?}
    Reach -- 是 --> Exec[联合执行lift+arm、夹爪、附着、运输姿态]
    Reach -- 否 --> Retry{尚有局部重算次数?}
    Retry -- 否 --> F
    Retry -- 是 --> LocalWam[使用新抓姿局部重算TC-WAM]
    LocalWam --> Bind
    Exec --> Verify{独立抓取和运输姿态证据?}
    Verify -- 否 --> Retry
    Verify -- 是 --> N2[Nav2运输到固定交付位]
    N2 --> Place[BaseLock后固定放置]
    Place --> Release{释放被独立验证?}
    Release -- 否 --> F
    Release -- 是 --> Done[SUCCESS并保存trial]
```

## 4. 控制权交接时序

```mermaid
sequenceDiagram
    participant FSM as 任务状态机
    participant MUX as 底盘仲裁器
    participant NAV as Nav2
    participant AG as AnyGrasp
    participant WAM as TC-WAM
    participant ACT as ACT
    participant MOVEIT as MoveIt
    participant ODOM as 实测里程计

    FSM->>MUX: NAV2
    FSM->>NAV: NavigateToPose(观察区)
    NAV->>MUX: nav_cmd_vel
    NAV-->>FSM: SUCCEEDED
    FSM->>MUX: HOLD
    ODOM-->>FSM: 连续停稳
    FSM->>ACT: reset_episode origin=current base
    FSM->>AG: GenerateGrasps(COARSE, Top-K)
    AG-->>FSM: physical coarse grasps
    FSM->>WAM: ComputeTcWam(target, grasps)
    WAM-->>FSM: plan(b*,h*,g*) + bound ready
    FSM->>MUX: ACT
    FSM->>ACT: enabled BASE_TWIST
    ACT->>MUX: act_cmd_vel
    ACT-->>FSM: bound ready + sustained low speed
    FSM->>ACT: disabled; clear chunk
    FSM->>MUX: HOLD/BaseLock
    ODOM-->>FSM: 连续停稳
    FSM->>AG: GenerateGrasps(REFINED, Top-K)
    AG-->>FSM: near-field grasps
    FSM->>MUX: MOVEIT BaseLock
    FSM->>MOVEIT: current joints + coarse plan + refined grasps
    alt 当前底盘可达
        MOVEIT-->>FSM: grasp verified + transport safe
        FSM->>MUX: NAV2
    else 全部不可达
        MOVEIT-->>FSM: reposition_required
        FSM->>WAM: local recompute with refined grasps
        WAM-->>FSM: new bound ready
        FSM->>MUX: ACT
    end
```

ACT action chunk 与 MoveIt 执行绝不重叠。`MOVEIT` 是底盘零速度所有权状态，不是
MoveIt 对底盘的命令权。

## 5. TC-WAM计算流程

```mermaid
flowchart TD
    T[目标位姿/协方差 + COARSE Top-K抓姿] --> B[采样候选底盘b]
    B --> H[为活动臂选择with_lift规划组及lift IK seeds]
    H --> Cross[构造(b,h,gk)候选]
    Cross --> IK[预抓取/抓取联合IK]
    IK --> Gate1{IK有效?}
    Gate1 -- 否 --> R1[分数0]
    Gate1 -- 是 --> Col[StateValidity + 货架Planning Scene]
    Col --> Gate2{无碰撞?}
    Gate2 -- 否 --> R2[分数0]
    Gate2 -- 是 --> Soft[关节余量/扰动鲁棒/抓姿质量/可见性/移动代价]
    Soft --> Score[计算S(b,h,gk)]
    Score --> Best[选择(b*,h*,g*)]
    Best --> Filter[固定active arm/group/h*/g*/map version]
    Filter --> Threshold[绝对+相对阈值]
    Threshold --> CC[仅保留包含b*的连通分量]
    CC --> Ready[绑定ready region]
    Ready --> Out[ManipulationPlan + 18维状态 + 热力图]
```

核心优化为

\[
(b^*,h^*,g^*)=\arg\max_{b,h,g_k\in\mathcal F}S(b,h,g_k).
\]

硬门控不能被软分补偿。ready 区域定义为

\[
\mathcal R_{h^*,g^*}=\operatorname{CC}_{b^*}
\{b\mid F(b,h^*,g^*)=1,
S(b,h^*,g^*)\ge\max(\tau_{abs},\rho S^*)\}.
\]

二维显示热力图可以在每个底盘格显示最大分，但控制用 ready 集不得跨抓姿或升降解
合并，这是可视化与控制语义的明确边界。

## 6. 第二次AnyGrasp与局部重算

```mermaid
flowchart TD
    P[ACT结束 + BaseLock] --> G[REFINED AnyGrasp]
    G --> Fix[固定当前实际底盘b_cur]
    Fix --> Check[对新Top-K逐一IK/碰撞检查]
    Check --> Any{存在可达抓姿?}
    Any -- 是 --> Pick[选择当前位姿最优抓姿]
    Pick --> Move[MoveIt联合规划和执行]
    Any -- 否 --> Limit{recovery count < max?}
    Limit -- 否 --> Fail[FAIL]
    Limit -- 是 --> Recompute[以新抓姿集合局部重算TC-WAM]
    Recompute --> NewReady[新(b*,h*,g*)和绑定ready]
    NewReady --> Align[ACT再次只移动底盘]
    Align --> P
```

新抓姿无需与第一次 `g*` 的 ID 相同。当前底盘存在任一物理可达新抓姿时，以当前
可达验证优先，避免因点云小变化造成无意义重对位。

## 7. 工程门控流程

```mermaid
flowchart TD
    E0[冻结24-D观测/3-D动作/绑定契约] --> G0{Schema与所有权检查?}
    G0 -- 否 --> E0
    G0 -- 是 --> E1[核对URDF/SRDF/TCP/TF/控制器]
    E1 --> G1{固定底盘联合抓取至少8/10?}
    G1 -- 否 --> C1[修正模型/规划组/控制器/夹爪]
    C1 --> E1
    G1 -- 是 --> E2[接入真实COARSE/REFINED AnyGrasp]
    E2 --> G2{坐标/物理标志/Top-K可复现?}
    G2 -- 否 --> E2
    G2 -- 是 --> E3[生成第一张物理(b,h,g) TC-WAM]
    E3 --> G3{绑定ready与分数物理有效?}
    G3 -- 否 --> C3[修正Scene/扰动/评分/连通条件]
    C3 --> E3
    G3 -- 是 --> E4[比例教师经arbiter进入ready]
    E4 --> G4{停稳/BaseLock/失效停止安全?}
    G4 -- 否 --> C4[修正速度链和门控]
    C4 --> E4
    G4 -- 是 --> E5[ACT pilot: record/train/replay/infer]
    E5 --> G5{24-D/3-D闭环通过?}
    G5 -- 否 --> E5
    G5 -- 是 --> E6[REFINED不可达局部重算]
    E6 --> G6{有限重试与执行权互斥?}
    G6 -- 否 --> E6
    G6 -- 是 --> E7[正式实验和端到端]
```

固定底盘联合抓取未通过前，不采集 ACT 正式数据；mock AnyGrasp 或 mock evaluator
通过时，不得进入物理结论。

## 8. 实验证据链

### 8.1 标定与前置实验

| 子实验 | 最少样本 | 输出 |
|---|---:|---|
| 目标与AnyGrasp稳定性 | 每位置300帧/至少30次推理 | 位姿方差、Top-K重复性、抓姿质量 |
| Nav2到观察区误差 | 30次 | dx、dy、dyaw、停止时间 |
| TCP/联合规划组一致性 | 每姿态10次 | lift+arm末端误差 |
| 固定底盘联合抓取 | 至少10次 | 规划、碰撞、夹持、附着、运输姿态 |

### 8.2 实验A：抓姿条件化TC-WAM物理有效性

比较 Fixed、Reach-only、Full `(b,h,g)` TC-WAM。预注册高、中、低分候选并随机化
真机顺序，报告抓取成功率、Wilson 区间、Spearman 相关、IK率、碰撞率、计算时间，
并检查 ready 区域没有跨 `(h,g)` 混合。

### 8.3 实验B：局部底盘对位

至少比较：

- Nav2 直接到经验点；
- PID/MPC 追踪 `b*`；
- ACT 追踪单一 `b*`；
- ACT 进入抓姿绑定 ready 区域。

ACT 条件组可继续使用 M0/M1/M2，但所有组动作均为相同的3维底盘速度。主指标是
绑定 ready 到达率、耗时、底盘距离、碰撞/接管和停止延迟；MoveIt 抓取成功率作为
下游结果单独报告，不能写成“ACT直接完成抓取”。

### 8.4 实验C：近场更新与重算

设置“新抓姿当前可达”和“全部不可达”两类预注册条件，测量 MoveIt 当前位姿验证率、
误触发重对位率、局部重算成功率、重试次数、恢复时间及最终抓取率。

### 8.5 实验D：移位与短时遮挡

比较静态缓存和事件记忆。目标移位、丢失或 map version 变化必须撤销旧抓姿、旧 ready
和 ACT chunk。遮挡记忆可指导重观测，但不能授权夹爪闭合。

### 8.6 实验E：端到端

至少20次，逐阶段报告：Nav2观察、粗AnyGrasp、TC-WAM、ACT对位、近场AnyGrasp、
MoveIt抓取、持物运输、固定放置和总成功率。总成功率不能掩盖阶段失败。

## 9. 单次正式实验数据

每个 trial 至少保存：代码和配置版本、对象/起点、粗与近场 request id、全部 grasp id、
选定 `(b*,h*,g*)`、planning group、ready region id、map version、ACT episode id、
MoveIt 可达判断、`reposition_required`、重算次数、抓取/附着/释放证据、rosbag 和视频。

## 10. 真机启动顺序

1. 机器人硬件、控制器和唯一 robot state publisher；
2. 唯一定位、里程计和 `map->odom->base_link`；
3. 点云、目标检测与真实 AnyGrasp 进程；
4. 可执行轨迹的 MoveIt、真实 `left/right_arm_with_lift` 和 Planning Scene；
5. Nav2，速度重映射到 `/thesis/nav_cmd_vel`；
6. TC-WAM、对象记忆和 AnyGrasp gateway；
7. 底盘仲裁、ACT alignment supervisor 和 ACT bridge；
8. MoveIt grasp executor、抓取验证、固定放置与释放验证；
9. trial logger、rosbag 和外部视频；
10. 发送 `/thesis/run_pick_deliver`。

发现第二个底盘直连速度源、第二个 `odom->base_link`、固定 `world->base_link`、ACT
机械臂发布器或 MoveIt 执行时非零底盘命令，必须立即停止实验。

