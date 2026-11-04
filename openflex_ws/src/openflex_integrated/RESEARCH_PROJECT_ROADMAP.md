# 比赛与论文同步工程路线

## 已建立的边界

| 层 | ROS 2 包 | 职责 |
|---|---|---|
| 任务接口 | `openarmx_task_msgs` | 比赛运输、检测、精停、抓取、放置 Action |
| 比赛编排 | `openarmx_task_executor` | Nav2 → 停车 → 感知 → MoveIt → 运输 → 放置 |
| 研究接口 | `openflex_wholebody_msgs` | P2、对象记忆、TC-WAM、ACT 条件、全身命令 |
| 研究原型 | `openflex_wholebody_research` | 候选、评分、热力图、记忆和实验配置 |
| 操作模型 | `openarmx_integrated_moveit_config` | 集成升降与双臂的 MoveIt 模型/控制器配置 |

## 决策

- 真机首版采用到位停车后操作；在 MoveIt 真机验证、命令仲裁和碰撞门控完成前，不开放导航中预伸。
- 比赛接口继续使用固定站点作可靠基线；TC-WAM 校准完成后，以 `b_handoff` 替代人工经验导航点，但抓取/放置 Action 不变。
- TC-WAM 完整图低频/事件触发更新，`preferred_base_delta`、`current_score` 和 `ready` 高频更新。
- ACT 是局部协同策略，不承担远距离导航，也不隐式替代 MoveIt/Nav2 的物理硬门控。

## 下一里程碑（真实 MoveIt 热力图）

1. 在假硬件中核对 `openarmx_left_hand_tcp`、`openarmx_right_hand_tcp`、臂基座与升降 TF；
2. 向 Planning Scene 加入实测桌面碰撞盒；
3. 实现 `EvaluateAffordance` 的 MoveIt 评估端：每候选、每臂、每模板计算预抓取/抓取 IK、碰撞和最小关节余量；
4. 替换 mock evaluator，保存候选 CSV 和 RViz 热力图；
5. 再接 Nav2 costmap/路径，最后才上真机执行抓取。
