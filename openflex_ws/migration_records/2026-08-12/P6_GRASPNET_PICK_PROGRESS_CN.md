# P6 GraspNet 与 `/pick_object` 执行记录

## 范围

本阶段只完成软件链路和静态契约检查，不启动机械臂、夹爪、MoveIt runtime 或仿真。
正式抓取默认 `execute=false`，因此本记录不构成真机抓取通过声明。

## 已完成

- `FitGrasp.action` 位于 `src/openflex_grasp/openflex_grasp_interfaces/action/FitGrasp.action`。
  请求只携带 `detection_id`、`arm_name`、商品/场景点云、策略和超时；不复制 Qwen JSON，也不重复 MoveIt Action。
- `graspnet_worker` 使用 `/home/openflex/old/.venvs/openflex_graspnet` 对应的 GraspNetBaseline，模型进程只创建一次，左右臂请求串行。
- `pick_object_server` 通过 `GetDetectionSnapshot` 按 UUID 读取检测快照，校验 object/arm 后调用 FitGrasp；没有“最近检测”回退。
- `ArmContext` 已覆盖左右臂的 planning group、TCP、相机、夹爪 Action、关节和负载参数，并由新 Pick/GraspNet worker 使用。
- 预抓取、Cartesian 进给、安全回撤和左右夹爪 Action 已接入；当前 `execute=false` 时仅规划，不发送硬件命令。
- detection snapshot 在拟合成功后以 `consume=true` 消费，TTL/单消费者语义保持有效。

## 未完成项

- 旧 `product_grasp_executor.py` 仍是论文历史启动链，保留右臂历史状态机；它不由比赛 launch 启动。彻底删除/重构需要另行确认旧实验用户，不在本次比赛 backend 迁移中冒险修改。
- P6-09、P6-10、P6-11 依负责人指示跳过；缺少实时双腕 RGB-D 和现场 MoveIt 证据，不能标记 G6。

## 静态证据

```text
python -m compileall src/openflex_grasp/openflex_grasp/openflex_grasp
colcon build --packages-select openflex_grasp_interfaces openflex_grasp --symlink-install
ROS interface introspection: FitGrasp / GetDetectionSnapshot / PickObject
```

上述检查已通过；真机和仿真结果留待负责人现场验收。
