# 比赛抓取包结构与所有权

```text
比赛控制台 openflex_navigation_gui
  ├─ 调试 client ───────────────> /detect_object, /pick_object
  └─ 正式任务（待 GUI 接线） ───> /execute_transport
                                      |
openarmx_task_executor <--------------+
  ├─ Detect/Pick 顺序、重试、导航/精停/回撤
  └─ Place 当前仅明确 mock

openflex_grasp
  ├─ detect_object_server ─> Qwen3.5-4B + SAM2 ─> detection store
  ├─ graspnet_worker <────── FitGrasp(point clouds)
  └─ pick_object_server ───> PlanPregrasp + CartesianApproach + gripper

openflex_grasp_supermarket_scene_adapter
  └─ /find_sku ─> 外部 supermarket_scene（OWLv2，可选 DINO）

openarmx_integrated_moveit_config
  └─ 唯一 move_group / PlanPregrasp / ExecuteJointPosture owner

openflex_grasp_moveit
  └─ 唯一 CartesianApproach server（不创建 move_group/controller/RSP）
```

## 依赖方向

- `openarmx_task_msgs`、`openarmx_integrated_moveit_msgs`、`openflex_grasp_interfaces` 是接口层。
- 核心 `openflex_grasp` 不依赖数据库、Web 商品服务或 `supermarket_scene_interfaces`。
- scene adapter 是可选正式依赖，只负责 OWLv2/DINO 框服务适配。
- GUI 不直接导入模型；模型 worker 不导入 GUI、数据库或任务编排。
- perception 与 GraspNet 使用两个 venv，但 ROS 控制节点继续使用系统 Python。

## 冲突处理

旧 `openflex_grasp_moveit` 的重复消息已迁到
`openarmx_integrated_moveit_msgs`，原 `openflex_grasp_msgs` 已删除。调试与 fullbody launch
使用相同 backend 名称，由进程 owner lock 互斥。旧 `/product_perception/*` 仅属于未被比赛
launch 引用的历史兼容链，不得和新 backend 混合启动。
