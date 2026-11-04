# OpenFlex 比赛抓取运行手册

## 1. 固定路径与环境

目标 workspace：`/home/openflex/old/openflex_ws`，模型根：
`/home/openflex/old/openflex_models`。运行前按此顺序 source：

```bash
source /opt/ros/humble/setup.bash
source /home/openflex/old/openflex_ws/install/setup.bash
```

模型解释器：

```text
OPENFLEX_PERCEPTION_PYTHON=/home/openflex/old/.venvs/openflex_perception/bin/python
OPENFLEX_GRASPNET_PYTHON=/home/openflex/old/.venvs/openflex_graspnet/bin/python
OPENFLEX_MODEL_ROOT=/home/openflex/old/openflex_models
```

部署前执行 `sha256sum -c /home/openflex/old/openflex_models/SHA256SUMS`。Qwen/SAM2
使用 perception 环境和 BF16；GraspNet 使用独立环境和 FP32。

## 2. 相机与标定

右腕 D405 serial 为 `260322275347`，左腕 D405 serial 为 `260322276903`。
两台设备的左右安装位置必须与这两个序列号保持一致，不要互换。

serial 通过环境或 launch 参数提供：

```bash
export OPENFLEX_RIGHT_CAMERA_SERIAL=260322275347
export OPENFLEX_LEFT_CAMERA_SERIAL=260322276903
```

RealSense 是 `camera_link -> color_optical_frame` 的唯一发布者；标定后的腕部固定关节写在
机器人 URDF 的 nominal (`right_poe_calibrated=false`) 分支。上线前必须检查每个 optical
frame 只有一条父链。

## 3. 启动模式

调试模式（不启动相机、不执行运动）：

```bash
ros2 launch openflex_grasp grasp_debug.launch.py \
  start_cameras:=false execute:=false
```

正式比赛模式：

```bash
ros2 launch openarmx_task_executor supermarket_fullbody_runtime.launch.py \
  start_grasp_backend:=true execute_grasp:=false
```

`execute_grasp` 默认必须为 `false`。本迁移没有完成真机关卡，不应在无人现场确认时改为
`true`。调试和比赛 launch 不能同时拥有 backend；三个节点有进程锁，第二套启动会失败
关闭。锁目录默认 `/tmp`，可用 `OPENFLEX_BACKEND_LOCK_DIR` 修改。

正式 fullbody 在 `start_grasp_backend=true` 时还会启动
`/openflex/cartesian_approach`。抓取 plan-only 链需要四个 Action owner 同时在线：
`/detect_object`、`/pick_object`、`/openflex/plan_pregrasp`、
`/openflex/cartesian_approach`。控制台会显示 Detect/Pick/Pregrasp/Cartesian 四项状态；
任一规划端点离线时“仅规划”按钮禁用。

主要 launch 参数：

| 参数 | 默认 | 含义 |
|---|---:|---|
| `start_cameras` | `false` | 调试 launch 是否创建双腕 RealSense |
| `start_detect_server` | `true` | Qwen+SAM2 Detect owner |
| `start_graspnet_worker` | `true` | 单一常驻 GraspNet worker |
| `start_pick_server` | `true` | Pick Action owner |
| `execute`/`execute_grasp` | `false` | 是否允许硬件执行 |

## 4. Provider 规则

- 默认 `qwen_local`，输入是 SKU 中文原称呼；若有 OWLv2 英文提示，则作为附加描述。
- Qwen 失败直接报错，禁止自动切换 OWLv2。
- `owlv2_local` 由独立 scene adapter 调 `/find_sku`；DINO 只能复核 OWLv2。
- 云端 API 不在本阶段范围。

当前正式 `/detect_object` owner 固定使用 Qwen。控制台可显示 OWLv2/DINO 调试参数，但在
scene adapter 未接到统一 Detect provider 路由前不会伪装成 Qwen Detect 成功。

## 5. 控制台操作

比赛控制台的 fullbody 正式入口默认使用混合电机模式：左右机械臂 1-7 关节为 `CSP`，左右
夹爪电机为 `MIT`。对应 launch 参数为 `control_mode:=csp` 和
`gripper_control_mode:=mit`。硬件激活日志必须分别显示
`arm_control_mode=csp, gripper_control_mode=mit`；模式变更只在下次正常重启 fullbody 时生效，
禁止在控制链运行期间为加载配置而强制重启 controller_manager。

进入“抓取调试”页后：选择 `left/right`、填写 SKU，先点“仅检测”，成功得到
`detection_id` 后点“仅规划”。规划通过后会生成一个 120 秒有效、单次使用的 `plan_id`；
核对图像、点云、预抓取/抓取位姿后，勾选“真机执行解锁”，再点“执行抓取”并在确认框中
确认现场安全。取消按钮会取消当前 Detect、PlanPick 或 ExecutePick，并向执行中的 MoveIt/夹爪
子 Action 传播。页面的 backend 标签应为 `real detect + real pick + mock place`。右侧检查器显示
冻结原图、SAM2 mask、分割叠图、商品/场景点云和商品位姿；“仅规划”成功后再显示
GraspNet 预抓取/抓取位姿、连接路径、宽度和候选统计。位姿旁必须核对 frame ID 和时间戳。

`/execute_pick` 不重新规划：它只消费刚刚审查的缓存轨迹。执行前会重新分割复核商品，并
拒绝底盘自规划后位移超过 5 mm、偏航超过 0.5°或手臂/升降起点偏离阈值的计划。动作顺序
为预抓取、开夹爪、直线进给、闭合、回撤；预抓取、进给和回撤统一使用所选手臂与升降柱的
`*_arm_with_lift` 8 DOF 规划组，底盘仍禁止运动。计划
执行一次、超时、修改 SKU/手臂/provider 或重新检测后都必须重新规划。

规划器会把商品点云最低点转换到 `odom`，再加
`object_bottom_clearance_m`（默认 `0.01 m`）作为商品区域内的 TCP 最低高度。若拍照位低于
该高度，机械臂与立柱会先在原位置竖直抬高；随后预抓取、进给、回撤和返回轨迹均逐段采样
验证 TCP 高度。任一中间点低于安全线都会使规划失败，不会生成可执行 `plan_id`。
如果原拍照位低于安全线，抓取完成后停在抬高后的拍照侧安全位，不再自动下降回原低位。

`/plan_pick` 的 PlanPregrasp-to-Cartesian 衔接总位置/姿态容差由
`cartesian_start_position_tolerance_m` 和
`cartesian_start_orientation_tolerance_deg` 控制，默认 `0.04 m / 20 deg`。该容差用于吸收
MoveIt 规划终点的跟踪/数值差异；严格直线接近仍单独限制横向偏移不超过 `5 mm`，路径比例仍须
达到 `0.99`。这不会启用底盘运动，也不会绕过 `/execute_pick` 的底盘和关节起点复核。

检查器由 fullbody launch 在 `127.0.0.1:8765` 启动，GUI 会在后端晚启动时自动重连。
ROS 数据位于 `/openflex_grasp/debug/*`，只用于显示，不接受控制命令；显示点云经过降采样，
内部 detection snapshot 与 GraspNet 仍使用完整点云。更新这部分代码后必须正常重启 fullbody，
已运行的旧进程不会热加载新发布者或 Web 服务。

正式任务仍由 `/execute_transport` 所有；当前调试页没有发送 transport goal。Place 是后续
项目，任何 mock place 结果都不等于完整比赛搬运通过。

## 6. 非真机健康检查

```bash
ros2 interface show openarmx_task_msgs/action/DetectObject
ros2 interface show openarmx_task_msgs/action/PickObject
ros2 interface show openflex_grasp_interfaces/action/FitGrasp
ros2 interface show openflex_grasp_interfaces/srv/GetDetectionSnapshot
```

不启动硬件时只能验证包、接口、模型资产和 owner 规则；不得据此宣称左右臂抓取通过。
