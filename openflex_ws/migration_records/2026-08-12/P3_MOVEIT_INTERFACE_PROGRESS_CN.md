# P3 MoveIt 接口收敛执行记录

## P3-01 CartesianApproach 迁入目标消息包

已将 `CartesianApproach.action` 逐字迁入：

```text
src/openflex_integrated/openarmx_integrated_moveit_msgs/action/CartesianApproach.action
```

并加入该包 `rosidl_generate_interfaces`。源/目标 Action 文件 `cmp` 一致，字段、常量、result 和 feedback 未改变。目标消息包当前生成：

```text
CartesianApproach
ExecuteJointPosture
PlanPregrasp
```

## P3-02 Cartesian server 切换类型所有权

`openflex_grasp_moveit` 的 CMake、package manifest、C++ include 和类型别名已从 `openflex_grasp_msgs` 切换为 `openarmx_integrated_moveit_msgs`。走廊、严格 Cartesian、普通 MoveIt、碰撞检查和执行代码未修改。

独立构建：

```text
colcon build --packages-select \
  openarmx_integrated_moveit_msgs openflex_grasp_moveit

Summary: 2 packages finished [27.1s]
```

构建目录：

```text
/tmp/openflex_p3_01_build
/tmp/openflex_p3_01_install
/tmp/openflex_p3_01_log
```

构建成功生成目标 Python/C/C++ Action 类型，并成功链接、安装 `cartesian_approach_executor`。

## P3-03 PlanPregrasp client 统一

已将以下活动 client 从 `openflex_grasp_msgs/PlanPregrasp` 切换到 `openarmx_integrated_moveit_msgs/PlanPregrasp`：

- `openflex_grasp.product_grasp_executor`
- `openflex_grasp.product_moveit_bridge`

`openflex_grasp/package.xml` 同步改为依赖目标消息包。除 legacy `openflex_grasp_msgs` 包和待更新迁移说明外，源码、manifest、CMake 和 launch 中的旧消息包活动引用为 0。

## P3-04 完整关节姿态改用 ExecuteJointPosture

商品执行器新增 `/openflex/execute_joint_posture` client。返回拍照位时仍先完成安全 Cartesian 回撤到 pregrasp，最后的完整记录姿态统一发送：

```text
left_arm
right_arm
left_arm_with_lift
right_arm_with_lift
```

旧 `PlanPregrasp.use_exact_joint_goal/exact_joint_goal` 不再被任何活动代码调用；相关字段只残留在待 P3-10 处置的 legacy 消息定义中。

验证：

```text
openflex_grasp 全部测试：63 passed
左右臂 x ARM_ONLY/ARM_WITH_LIFT 四种姿态规格：均覆盖
openflex_grasp 独立构建/安装：1 package finished
legacy exact-joint 活动扫描：0
旧消息包活动 client 扫描：0
```

构建目录：

```text
/tmp/openflex_p3_client_build
/tmp/openflex_p3_client_install
/tmp/openflex_p3_client_log
```

## P3-05 MoveIt 所有权核验

静态 launch 和 server 扫描结论：

- `openarmx_integrated_moveit_config` 唯一创建 `move_group`。
- 同一 package 唯一创建 `/openflex/plan_pregrasp` 和 `/openflex/execute_joint_posture` server。
- `openflex_grasp_moveit` 只创建 `cartesian_approach_executor`，不创建 `move_group`、`robot_state_publisher` 或 `controller_manager`。
- `openflex_grasp` 只作为上述 Action 的 client。

P3-05 的代码/launch 所有权通过；实际启动后的节点与 Action server 唯一实例数将在 G3 联调再次采集。

## P3-06 Detect goal 增加 arm_name

`openarmx_task_msgs/action/DetectObject.action` 的 Goal 已增加：

```text
string arm_name
```

TaskExecutor 只从本次 source station/slot 的 `arm_name` 解析一次标准化左右臂，并把同一个值传给 shelf-view、Detect 和 Pick。slot 缺失 `arm_name`、空字符串或非 `left/right` 会在开始导航前以 `INVALID_REQUEST` 失败；不再使用 `default_arm_name` 猜测缺失的比赛配置。

truth 仿真 Detect Action goal callback 同样拒绝空 object、空 station 和非法臂。安装态 introspection 已确认 Goal 包含 `arm_name`。

## P3-07 Pick goal 增加 detection_id

`openarmx_task_msgs/action/PickObject.action` 的 Goal 已增加：

```text
string detection_id
```

调用方行为：

- TaskExecutor 拒绝真实 Detect 成功但 `detection_id` 为空的结果。
- GUI mock 使用显式 `mock:<arm>:<object_id>`，mock server 拒绝空 ID、非法臂和跨臂 ID。
- truth 仿真使用 `truth:<arm>:<sequence>`，缓存记录 object、station、arm 和过期时间。
- truth Pick 在接受 goal 前检查 ID 存在、未过期、未消费，且 object/station/arm 全部一致；执行时标记为已消费。
- 仿真 detection TTL 默认 30 秒，reset 会清空全部缓存记录。

关闭 detection 或显式开启 nominal fallback 的旧调试模式会生成唯一 `nominal:<arm>:<uuid>`，不会伪装成真实 perception 快照；后续真实 GraspNet Pick server 将按 P6 detection store 契约拒绝 nominal ID。

## P3-08 TaskExecutor 显式传递链

TaskExecutor 当前链路为：

```text
source slot arm_name
  -> DetectObject.Goal.arm_name
  -> DetectObject.Result.detection_id
  -> PickObject.Goal.detection_id
  -> PickObject.Goal.arm_name
```

`_detect()` 返回 `(object_pose, detection_id, error_code)`；`_pick()` 必须显式接收 `object_pose`、`detection_id` 和同一个 `arm_name`。代码中不存在按“最近一次检测”查找，也不存在在 slot 配置缺失时使用默认臂继续运行。

验证结果：

```text
Action Python/C/C++ 类型生成：成功
安装态 ros2 interface show：Detect.arm_name / Pick.detection_id 均存在
TaskExecutor contract/station 测试：15 passed
truth detection store/sim world 测试：9 passed
GUI Action 状态测试：7 passed
安装态合并测试：31 passed
```

全新临时构建：

```text
colcon --log-base /tmp/openflex_p3_contract_log build \
  --packages-up-to openarmx_task_msgs openarmx_task_executor \
  openarmx_supermarket_sim openflex_navigation_gui \
  --build-base /tmp/openflex_p3_contract_build \
  --install-base /tmp/openflex_p3_contract_install

Summary: 38 packages finished [2min 33s]
```

上游构建仅出现既有 PCL、Vite chunk、header install 和 Scancontext return-path 警告；无本次 Action 契约构建失败。

## P3-09 稳定错误码和详细消息透传

`TaskError.msg` 新增：

```text
CAMERA_UNAVAILABLE=24
MODEL_UNAVAILABLE=25
PERCEPTION_FAILED=26
PLANNING_FAILED=27
GRIPPER_FAILED=28
DETECTION_EXPIRED=29
DETECTION_MISMATCH=30
```

TaskExecutor 新增集中 `error_contract.py`：

- 只允许 `TaskError` 已登记的非零码向高层透传。
- 子 Action 返回 0、未知值、无法转换值时，按 Detect/Pick/Place/Dock/安全回撤等当前阶段归并到稳定 fallback。
- 子 Action 的非空 `message` 原样保留；缺失时使用阶段默认说明。
- preflight、取消、超时、TF、底盘未稳定和配置错误等本地产生的失败也记录明确 message。
- `/execute_transport` 最终失败 result 使用最近一次实际失败 message，不再只返回数值模板。

验证：

```text
错误白名单/消息纯逻辑与既有 contract/station 测试：35 passed
四包增量构建：4 packages finished [16.3s]
安装态相关合并测试：51 passed
新增 TaskError Python 常量 24..30：全部可见
旧 int(payload.error_code) 无白名单透传扫描：0
```

构建日志：

```text
/tmp/openflex_p3_error_log
/tmp/openflex_p3_contract_build
/tmp/openflex_p3_contract_install
```

## P3-10 删除重复 MoveIt 消息包

删除前依赖审计：

```text
活动 Python/C++ client：0
CMake find_package/target：0
package.xml depend：0
launch 引用：0
```

已删除：

```text
src/openflex_grasp/openflex_grasp_msgs/
```

`PlanPregrasp`、`ExecuteJointPosture`、`CartesianApproach` 现在只由：

```text
src/openflex_integrated/openarmx_integrated_moveit_msgs
```

生成。`openflex_grasp_moveit` 和 `openflex_grasp` 的活动调用方均使用该统一类型。迁移指南与抓取 README 中会导致旧包恢复、重复构建或错误 introspection 的命令已同步改正；历史名称只保留一条“禁止恢复”的说明。

删除后验证：

```text
colcon list 中 openflex_grasp_msgs：0
重复 Action 源文件：0
统一消息 owner + Cartesian server + grasp Python 包构建：3 packages finished [22.4s]
```

构建目录：

```text
/tmp/openflex_p3_legacy_removed_build
/tmp/openflex_p3_legacy_removed_install
/tmp/openflex_p3_legacy_removed_log
```

## G3 接口关卡验收

通过时间：2026-08-12 10:49 CST。

### 全新构建

从当前源码使用独立 build/install/log 目录执行 `--packages-up-to` 构建：

```text
/tmp/openflex_g3_clean_build
/tmp/openflex_g3_clean_install
/tmp/openflex_g3_clean_log

Summary: 33 packages finished [1min 51s]
```

构建目标覆盖统一任务消息、TaskExecutor、truth 仿真、统一 MoveIt 消息、集成 MoveIt server 和 Cartesian server。五个上游包有既有 stderr 警告，但无构建失败。

### Action introspection 与 truth client/server

安装态 introspection 确认：

```text
DetectObject.Goal.arm_name: string
PickObject.Goal.detection_id: string
```

truth launch 的 live graph 中：

```text
/detect_object: 1 client / 1 server
/pick_object:   1 client / 1 server
```

实际调用顺序为：

```text
Dock shelf_c_pick
Detect object_id=juice station_id=shelf_c_pick arm_name=left
  -> detection_id=truth:left:2, found=true
Pick object_id=juice arm_name=left detection_id=truth:left:2
  -> success=true, object_attached=true
```

首次在未设置仿真 `current_station` 时发送 Pick 被正确拒绝为 station mismatch；设置当前站点并重新 Detect 后成功，证明 Pick 校验的是当前检测记录，而不是“最近一次检测”或手工猜测 ID。

### 测试

TaskExecutor 独立测试：

```text
72 passed, 1 skipped
```

GUI `test_competition_window.py` 独立复跑：

```text
22 passed
```

曾将 TaskExecutor、truth 仿真和 GUI 测试合并到同一 pytest 进程，结果为 `154 passed, 1 skipped, 5 failed`。其中四项依赖已不存在的旧 MJCF/scene 绝对路径，另一项为 GUI 测试间共享临时路径状态导致；包级隔离复跑已确认不属于本次 Action 接口回归。旧外部资产路径问题保留到后续仿真/全 workspace 清理，不据此宣称全 workspace 测试通过。

### fake-hardware live owner 核验

在隔离域 `ROS_DOMAIN_ID=93` 启动：

```text
integrated_robot_bringup_moveit.launch.py
  use_fake_hardware:=true
  use_rviz:=false
  start_execution_bridge:=false
  start_base_navigation_bridge:=false
```

`controller_manager.robot_description` 只包含四处 `fake_components/GenericSystem`，运行日志显示升降 `fake=true`，未加载真实 CAN hardware plugin，也未发送任何 Action goal 或轨迹。

live server 数量：

```text
/openflex/plan_pregrasp:          1 server /openflex_pick_task_executor
/openflex/execute_joint_posture: 1 server /openflex_joint_posture_executor
/openflex/cartesian_approach:    1 server /cartesian_approach_executor
/move_group:                     1 node
```

核验结束后两个 launch 均已停止。Cartesian server 与 controller manager 正常退出；`move_group` 在收到整体 SIGINT 后的对象销毁阶段触发 exit code `-11`，属于关闭阶段问题，运行期 server、消息类型和唯一 owner 验收此前均已完成。没有连接真机或执行运动，该退出问题需要在后续启动稳定性阶段继续跟踪。
