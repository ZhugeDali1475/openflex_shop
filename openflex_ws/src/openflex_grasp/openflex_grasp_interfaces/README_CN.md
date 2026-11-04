# OpenFlex 抓取接口契约

## 公共 Action

### `/detect_object` — `openarmx_task_msgs/action/DetectObject`

输入 `object_id`、站点/slot、`arm_name`、可选货架层/nominal pose、最低置信度和超时。
本实现使用 `arm_name` 冻结对应腕部同一时刻的 RGB、对齐深度和 CameraInfo，然后完成
Qwen 2D 框、SAM2 mask、实例点云和 `object_pose`。成功返回非空 `detection_id`。

### `/pick_object` — `openarmx_task_msgs/action/PickObject`

输入必须携带上一步原样返回的 `detection_id`、相同 `object_id` 和 `arm_name`。Pick 读取
内部快照、调用 GraspNet、生成 pregrasp/grasp，并调用统一 MoveIt Action。默认
`execute=false` 时只规划；成功不代表物体已附着，`object_attached` 仍为 false。

## `detection_id` 生命周期

ID 是 Detect 创建的 UUID，默认 TTL 30 秒。读取时同时核验 object、arm、frame 和时间；
不存在、过期或不匹配均失败。GraspNet 拟合和规划链成功后以 `consume=true` 单次消费；
不能复用、不能用“最近一帧”替代，也不能跨左右臂传递。

## 内部 Service/Action

`GetDetectionSnapshot.srv` 只在进程内/本机 ROS 图传输大数据：mask、商品点云、场景点云、
相机 serial、frame/stamp 和 pose。它不是外部业务 API。

`FitGrasp.action` 是单一常驻 GraspNet worker 的契约。它接收 detection ID、arm、策略和
两份点云，返回 grasp/pregrasp、宽度、分数和候选统计。它不接收 Qwen/SAM JSON，也不
拥有 MoveIt 或夹爪。

`QueryVlm.action` 是单一常驻 Qwen worker 的内部通用图文生成契约。它接收一张本机
ROS 图中的 RGB 图像和业务侧构造的 prompt，只返回模型原始文本。SKU 数据库查询、
购物票 JSON 解析和机器人动作都不属于该接口或模型进程。

## `ArmContext`

`openflex_grasp.arm_context` 是左右臂机器人适配边界，统一 planning group、TCP、相机
optical frame、夹爪 Action、approach/closing axis、关节列表和 payload。新 Pick/GraspNet
代码只能通过它选择左右臂；旧 `product_grasp_executor` 是隔离的历史链。

## 错误与取消

公共稳定码来自 `openarmx_task_msgs/msg/TaskError`：相机 24、模型 25、感知 26、规划 27、
夹爪 28、检测过期 29、检测不匹配 30。Action 接受取消；取消结果使用 16。未知内部异常
由上层按阶段归并，详细 `message` 应保留原始原因。

## 边界

Detect 不导入 GraspNet；FitGrasp 不调用 MoveIt；Pick 不做 2D 检测；任务编排属于
`/execute_transport`。本阶段没有真实 Place，实现或验收 mock place 都不能设置真实
`object_released=true`。
