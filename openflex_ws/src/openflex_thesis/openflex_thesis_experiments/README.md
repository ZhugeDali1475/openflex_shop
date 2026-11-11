# openflex_thesis_experiments

当前保留通用日志、统计、安全预检与只规划诊断，服务于 RoboOS + RoboBrain2.5 分层调度。
方案入口见仓库根目录 `README.md`。旧学习恢复 benchmark、训练、矩阵和报告模块已移除，
原始代码、专属测试及实验数据见 `LEGACY_ARCHIVE_CN.md`；不继续旧版五方法或 25 次研究矩阵。

## 保留的命令

- `trial_logger`：任务、控制权和抓放事件记录。
- `generate_trial_matrix`、`summarize_trials`：通用试次展开与描述统计；当前新协议尚待冻结。
- `thesis_preflight`：既有物理证据预检，保持失败关闭，不是当前仿真方案必做流程。
- `audit_moveit_path`：保存的离散状态／失败日志审计，不把缺失检查当作安全。
- `probe_plan_path`：读取场景、请求路径规划并检查返回离散点，不执行轨迹。

`config/experiment_protocol.yaml`、`legacy_perception.yaml` 仍由既有物理预检引用，
只作安全门控和兼容配置保留，**不得直接冻结为当前大模型实验协议**。

## 构建

在已有依赖安装完成的工作区内：

```bash
source /opt/ros/humble/setup.bash
source openflex_ws/install_v2/setup.bash
cd openflex_ws
colcon --log-base log_v2 build --build-base build_v2 --install-base install_v2 \
  --symlink-install --packages-select openflex_thesis_experiments
source install_v2/setup.bash
```

## 只规划取证工具

运行前独立确认诊断协议、隔离域和节点身份，停止 mission／控制客户端。
`--confirm-simulation` 是操作者确认，不是软件认证；`ROS_LOCALHOST_ONLY=1` 也不能排除本机实机节点。
以下仅展示接口，不表示历史停车目标安全，也不应未经核验直接执行：

```bash
export ROS_LOCALHOST_ONLY=1
ros2 run openflex_thesis_experiments probe_plan_path \
  --target path/to/diagnostic_target.json --group right_arm_with_lift \
  --required-object-id mujoco_shelf_back \
  --required-object-id mujoco_shelf_board_target_level \
  --required-object-id mujoco_shelf_board_upper_1 \
  --required-object-id mujoco_shelf_board_upper_2 \
  --required-object-id mujoco_shelf_left_wall \
  --required-object-id mujoco_shelf_right_wall \
  --domain-id 175 --confirm-simulation --timeout-sec 60 \
  --output path/to/new_plan_probe.json
```

目标 JSON 包含 `lift_joint` 和所选侧 `openarmx_<side>_joint1` 至 `joint7` 的 8 个有限值。
已有输出文件拒绝覆盖；新诊断须使用新目录，不覆盖历史证据。

- 只连接 `/get_planning_scene`、`/plan_kinematic_path`、`/check_state_validity`，
  不调用执行 action、夹爪、场景写入或运动话题；其他进程仍可能运动，所以必须隔离。
- 输出保留请求／响应、单调时间、规划错误码、返回轨迹关节／时间点，以及每点完整状态和接触对。
  非活动臂、附着物、ACM、八叉树和场景回读均纳入诊断。
- 单次响应最多等待 10 秒，总时限默认 60 秒，最多审计 1000 个返回点；超限保留原响应并失败。
- 无轨迹、无响应、首尾不一致、缺失检查或场景变化均不补造成有效状态。
- 仅在规划成功、全部离散点有效、首尾／附着物一致且场景哈希未变等条件成立时报告
  `discrete_diagnostic_passed=true`。它不证明连续路径安全、场景原子一致或标定正确。
- 新规划无法还原历史 seed 6301 的内部第 61 点；规划器拒绝且未返回轨迹时，工具也拿不到该轨迹。

历史路径分析与负结果已归档，证据位置见 `LEGACY_ARCHIVE_CN.md`，未关闭问题见
`SAFETY_AND_TODO_CN.md`。本次清理不运行该探针、不重跑机器人任务。
