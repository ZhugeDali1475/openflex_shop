# OpenFleX Thesis ROS Workspace

> **当前毕业设计主线已更新**：失败记忆增强的代价敏感局部恢复决策。仓库根目录的
> `RECOVERY_RESEARCH_PLAN_CN.md`、`RECOVERY_THESIS_METHOD_CN.md` 和
> `RECOVERY_IMPLEMENTATION_REPORT_CN.md` 是当前研究入口。下文描述的是可复用的 ROS
> 工程底座；ACT 对位等能力保留在系统中，不再作为当前论文算法主线。

> **Legacy thesis subsystem.** 本目录保留论文接口、历史实验和离线 mock 以便追溯，
> 不进入 OpenFlex 商超比赛启动链。`openflex_anygrasp` ROS 包和 AnyGrasp SDK 已从
> 目标工作空间移除；物理预检会保持失败关闭。禁止把本目录中的 AnyGrasp 命名接口
> 解释为当前比赛 backend，当前比赛抓取只使用 GraspNetBaseline。

这是与比赛接口完全分离的论文工程。最终研究闭环固定为：

```text
Nav2粗导航到观察/交接区
  -> 第一次AnyGrasp生成Top-K粗抓姿
  -> TC-WAM联合选择(b*, h*, g*)及严格绑定的ready区域
  -> ACT仅控制底盘速度进入ready区域
  -> 禁用ACT并锁定底盘
  -> 第二次AnyGrasp用近距离点云更新抓姿
  -> MoveIt联合规划/执行升降柱+活动臂并控制夹爪
  -> 抓取验证、附着、运输姿态
  -> Nav2运输到固定交付位
```

其中，(b) 为候选底盘位姿，(g) 为 AnyGrasp 抓姿，(h) 为
`left_arm_with_lift` 或 `right_arm_with_lift` 中的升降柱—活动臂联合 IK 解层。
ACT 不控制升降柱、机械臂或夹爪。

```text
openflex_thesis/
├── openflex_thesis_msgs         两阶段抓姿、联合操作计划和论文专用Action
├── openflex_tc_wam              AnyGrasp网关、三元组评估、热力图和对象记忆
├── openflex_thesis_control      底盘仲裁、ACT对位、MoveIt抓取与任务状态机
├── openflex_thesis_experiments  冻结协议、trial日志和统计
└── openflex_thesis_bringup      启动、硬件门和论文/流程文档
```

依赖边界：

- 允许复用机器人描述、底层控制、相机、Nav2、AnyGrasp 和 MoveIt；
- 不依赖 `openarmx_task_msgs` 或 `openarmx_task_executor`；
- 不实现比赛的清单拣选、缺货补货和错放归位。

## 唯一硬件所有权

| 阶段 | 底盘 | 升降柱、活动臂、夹爪 |
|---|---|---|
| 长距离导航 | Nav2 | HOLD |
| 局部精对位 | ACT，仅 ([v_x,v_y,\omega_z]) | HOLD |
| 抓取执行 | BaseLock，实测停稳 | MoveIt抓取执行器独占 |

Nav2 和 ACT 只发布到独立输入，仲裁器唯一输出
`/thesis/cmd_vel_selected`，随后仍须经过 collision monitor。AnyGrasp 和 TC-WAM
只计算，不直接控制硬件。

## 构建与离线检查

```bash
cd /home/openarmx/openflex_all/openflex_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to openflex_thesis_bringup --symlink-install
source install/setup.bash
ros2 launch openflex_thesis_bringup offline_demo.launch.py
ros2 launch openflex_thesis_bringup offline_mission_smoke.launch.py
```

离线流程使用 synthetic AnyGrasp、evaluator、Nav2 和抓放证据，只验证接口和状态机；
`physical_perception=false` 或 `physical_evaluation=false` 的数据禁止进入论文结果。

正式物理证据必须从 fail-closed 入口启动：

```bash
ros2 run openflex_thesis_experiments thesis_preflight \
  --scope end_to_end --through-gate G7_end_to_end
ros2 launch openflex_thesis_bringup physical_evidence_system.launch.py \
  object_pose_topic:=/dream/active_object_pose object_id:=thesis_item_01
```

预检失败时正式系统不会启动。DREAM 或其他检测器需要发布带真实时间戳和合法
`frame_id` 的 `geometry_msgs/PoseStamped`；`object_observation_adapter` 仅在 TF 可转换到
`map`、观测不过期且四元数有效时发布 `/thesis/object_observation`。

## 真机接入边界

先独立启动已经验证的机器人 bringup、唯一 TF/里程计、相机、Nav2、AnyGrasp 与
可执行轨迹的 MoveIt，再启动论文层。正式运行前必须完成：

- `left_arm_with_lift`、`right_arm_with_lift`、TCP、关节限位和控制器映射核对；
- 货架 Planning Scene、目标几何和抓取验证标定；
- AnyGrasp 粗/近两阶段物理点云接口；
- ACT 24维数值观测、3维底盘速度动作的 record/train/replay/inference 闭环；
- `BaseLock -> MOVEIT -> NAV2` 的实测停稳与互斥验证。

第二次 AnyGrasp 后若存在当前底盘可达的新抓姿，MoveIt 直接选择并执行；若全部
不可达，执行器返回 `reposition_required`，状态机在有限次数内使用新抓姿局部重算
TC-WAM，再由 ACT 对位。

详细说明见：

- `openflex_thesis_bringup/docs/ENGINEERING_AND_EXPERIMENT_FLOW.md`
- `openflex_thesis_bringup/docs/INTERFACE_CONTRACTS.md`
- `openflex_thesis_bringup/docs/ACT_INTEGRATION.md`
- `openflex_thesis_bringup/docs/IMPLEMENTATION_STATUS.md`
