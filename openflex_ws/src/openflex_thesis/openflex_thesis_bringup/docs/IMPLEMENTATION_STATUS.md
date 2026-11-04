# 最终论文工程实现状态与真机边界

更新日期：2026-07-25

## 已实现的软件框架

- 论文独立消息和 Action：两阶段 AnyGrasp、三元组候选、联合计划、绑定 ready 区域、
  ACT 底盘对位、MoveIt 抓取、固定放置和完整取送；
- AnyGrasp 外部进程 gateway 与明确非物理的 mock server；
- 对 `(b, h, g_k)` 的候选生成和可解释评分，其中 `h` 属于
  `left/right_arm_with_lift` 联合 IK 解层；
- IK/碰撞硬门控、关节余量、鲁棒性、抓姿质量、可见性和移动代价；
- ready 区域按单一抓姿、升降解、规划组和 map version 过滤，并保留 `b*` 连通分量；
- 18维 `compact_state` 与 24-D observation / 3-D base-only ACT schema；
- Nav2/ACT/Recovery 速度互斥，以及 MOVEIT/PLACE/HOLD 零速度 BaseLock；
- ACT 回合原点重置、实测低速 ready 确认和失效后清 chunk 契约；
- 第二次 AnyGrasp 后当前位姿可达优先、不可达返回局部重对位请求；
- MoveIt 从真实关节状态联合规划升降柱与活动臂、夹爪、附着及运输姿态的执行接口；
- 商品移位、丢失、重观测和 map version 失效逻辑；
- 冻结实验协议、trial manifest、事件日志和统计工具；
- synthetic 完整任务烟雾测试边界。
- 通用外部/DREAM `PoseStamped -> ObjectObservation` 适配器，包含时间戳、TF、
  四元数、协方差和置信度的 fail-closed 检查；
- 正式物理证据静态预检与门控启动入口，预检失败时不启动论文节点；
- 论文 Python 包隔离不兼容的用户级 pytest 插件，离线任务完成后自动干净退出。

## 不能由软件框架替代的真机工作

### G0：模型、TF与控制器

- 对照 `openarmx_integrated_moveit_config` 核对 `left_arm_with_lift`、
  `right_arm_with_lift`、TCP、`lift_joint` 限位和关节顺序；
- 让 MoveIt trajectory controller 与真机控制器可执行映射一致；
- 保证唯一 `map->odom->base_link`、唯一 joint TF publisher，消除固定
  `world->base_link` 与 Nav2 冲突；
- 确认 MoveIt 规划起点来自新鲜 `/joint_states`。

### G1：固定底盘联合抓取

- 标定货架 Planning Scene、对象盒尺寸、touch links 和夹爪开闭参数；
- 固定底盘、固定对象，验证升降柱+活动臂联合预抓取、抓取、附着和运输姿态；
- 完成外部抓取验证器；建议至少达到 8/10 后再推进热力图实验。

### G2：真实两阶段AnyGrasp

- 实现 `/thesis/anygrasp/request` 到真实点云推理的适配；
- 分别验证远距离 COARSE Top-K 和近距离 REFINED Top-K；
- 标定坐标变换、姿态约定、质量分数、碰撞宽度和推理超时；
- 禁止把 mock `physical_perception=false` 结果用于实验。

### G3：物理TC-WAM

- 校准货架碰撞体并设置 `calibrated=true`；
- 使用真实 AnyGrasp 候选、真实 `/joint_states`、IK、StateValidity 和 Planning Scene；
- 验证 `(b,h,g)` 热力图以及 `(h*,g*)` 绑定 ready 区域；
- 对高/中/低候选执行真实抓取，建立分数—成功率证据。

### G4：base-only ACT

- 恢复/修改 LeRobot bridge，使用 FAST-LIO 回合位姿和 swerve 实测速度两个缓存；
- 冻结 24维数值观测、两路 RGB、3维底盘速度动作；
- 只发布 `/thesis/act_cmd_vel`，不存在 arm/lift/gripper action；
- 完成比例教师、20～30条 pilot、训练、回放和真机推理；
- 验证 binding/map version 失效时一个周期内归零并清 chunk。

### G5：重算闭环与运输

- 人为制造第二次 AnyGrasp 全不可达，验证
  `reposition_required -> 局部TC-WAM -> ACT -> REFINED AnyGrasp` 有限重试；
- 验证抓取成功后附着物进入联合运输姿态，再把底盘权交还 Nav2；
- 实现固定放置和独立释放验证。

## 最短真机验收顺序

```text
模型/TF/控制器
-> 固定底盘联合抓取
-> AnyGrasp粗/近坐标正确
-> 第一张(b,h,g)物理热力图
-> 单一(h,g) ready区域真机验证
-> 比例教师底盘对位
-> ACT pilot
-> 不可达局部重算
-> Nav2完整运输与固定交付
```

软件构建或 synthetic smoke 成功不等于论文实验完成。正式结果必须同时具有
`physical_perception=true`、`physical_evaluation=true`、冻结配置、manifest、rosbag、
外部视频和独立抓放证据。
