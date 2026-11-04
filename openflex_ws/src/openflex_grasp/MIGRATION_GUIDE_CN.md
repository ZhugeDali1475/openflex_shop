# OpenFlex 商品抓取链路分析与迁移指南

本文面向把当前商品抓取能力集成到另一个 `openflex_ws` 的开发者。分析基于以下源码快照：

```text
openflex_all/
├── openflex_models/
└── openflex_ws/src/openflex_grasp/
    ├── openflex_grasp/          # 感知、抓取拟合、流程编排、Web 可视化
    ├── openflex_grasp_moveit/   # Cartesian/走廊 MoveIt Action Server
    └── graspnet-baseline/       # GraspNetBaseline 源码、CUDA 扩展和权重
```

这里的模型目录实际名为 `openflex_models`，不是 `openfx_models`。

## 1. 结论先行

完整链路是：

```text
右腕 RealSense 对齐 RGB-D
  -> Qwen3.5 根据商品名称输出一个或多个 2D 框
  -> SAM2 逐框分割
  -> 结合深度和内参生成商品点云
  -> 选择语义最匹配且有有效深度的实例
  -> GraspNetBaseline 对商品点云生成抓取候选
  -> 货架/篮筐几何规则形成最终 pregrasp 和 grasp
  -> TF 转到 odom
  -> MoveIt 到达 pregrasp
  -> 右臂 Cartesian 进给或升降柱垂直进给
  -> 夹爪闭合
  -> 抬升并返回拍照位
```

迁移不能只复制 `openflex_grasp` Python 包。最小可执行集合还包括：

- `openflex_grasp_moveit`，以及目标集成层的 `openarmx_integrated_moveit_msgs`。
- `graspnet-baseline` 源码、`checkpoint-rs.tar` 及本机重新编译的 CUDA 扩展。
- Qwen3.5、SAM2 模型目录。
- 目标机器人的 MoveIt/URDF/SRDF/控制器栈。
- `/openflex/plan_pregrasp` 的服务端实现，当前由 `openarmx_integrated_moveit_config` 提供。
- 右腕相机序列号和该机器人的手眼标定。

当前源码不调用 AnyGrasp SDK，也不需要 AnyGrasp 的机器授权文件。但是，这不等于整个方案可以无条件商用，详见第 13 节。

## 2. 组件职责和迁移边界

| 组件 | 职责 | 是否可直接迁移 | 目标端适配点 |
|---|---|---:|---|
| `openflex_grasp` | 文本定位、分割、点云、抓取拟合、执行状态机、Web 页面 | 大部分可以 | topic、frame、夹爪标定、模型路径 |
| `openarmx_integrated_moveit_msgs` | `PlanPregrasp`、`ExecuteJointPosture`、`CartesianApproach` Action 定义 | 使用目标集成层版本 | 接口两端版本必须一致，禁止再复制重复消息包 |
| `openflex_grasp_moveit` | 右/左臂严格 Cartesian、有限走廊和普通 MoveIt 规划 | OpenArmX 可直接用 | 非 OpenArmX 要改 planning group、tip link 和 MoveIt 配置 |
| `graspnet-baseline` | 生成抓取候选 | 源码和权重要一起迁移 | CUDA 扩展必须在目标机重编译 |
| `openarmx_integrated_moveit_config` | `/openflex/plan_pregrasp`，7/8/11 DOF 规划与执行 | 同构 OpenArmX 迁移 | 目标机器人不同则必须替换或实现兼容 Action Server |
| `openflex_hand_eye_calibration` | 从 YAML 发布腕部相机外参 | 包可迁移 | 每台机器必须重新标定，不能复用旧 YAML |
| `openflex_robot.env` | 保存左右相机序列号 | 格式可复用 | 每台机器填写自己的序列号 |

不要迁移源工作空间的 `build/`、`install/`、`log/`。这些文件包含绝对路径。本快照的 `install/` 仍引用 `/home/openflex/openflex_all/...`，在当前带 `桌面/openflex_all_gzk` 的路径下已经不能正确注册 ROS 包，目标工作空间必须重新 `colcon build`。

维护时优先从这些文件进入：

| 文件 | 作用 |
|---|---|
| `openflex_grasp/launch/grab_product.launch.py` | 完整链路总入口和产品级默认参数 |
| `openflex_grasp/launch/right_wrist_product_pointcloud.launch.py` | 相机、感知、拟合、执行和 Web 节点编排 |
| `openflex_grasp/openflex_grasp/models.py` | Qwen grounding 和 SAM2 分割 |
| `openflex_grasp/openflex_grasp/node.py` | RGB-D 同步、实例选择和点云生成 |
| `openflex_grasp/openflex_grasp/graspnet_backend.py` | GraspNet 加载、候选统计和加权拟合 |
| `openflex_grasp/openflex_grasp/grasp_geometry.py` | 货架/篮筐几何规则和最终位姿 |
| `openflex_grasp/openflex_grasp/grasp_node.py` | 抓取 ROS 节点和 TF 方向变换 |
| `openflex_grasp/openflex_grasp/product_grasp_executor.py` | 主执行状态机、夹爪、升降和恢复 |
| `openflex_grasp_moveit/src/cartesian_approach_executor.cpp` | 三种 MoveIt 路径模式及执行后校验 |
| `openarmx_integrated_moveit_msgs/action/*.action` | 上下游必须共同遵守的唯一 MoveIt Action ABI |

`product_moveit_bridge.py` 是只把 pregrasp 发送给 `PlanPregrasp` 的轻量可选桥。完整 `grab_product.launch.py` 将 `start_moveit_bridge` 设为 false，由 `product_grasp_executor.py` 统一负责规划、进给、夹爪和返回，迁移时不要同时启用两套 pregrasp 执行逻辑。

## 3. ROS 节点和数据链

主入口为：

```bash
grab_product.sh [--plan] [--shelf|--basket] PRODUCT_NAME
```

安装后等价入口：

```bash
ros2 run openflex_grasp grab_product --plan water bottle
```

`grab_product.launch.py` 会启动或包含以下节点：

| 节点 | 实现 | 输入 | 主要输出 |
|---|---|---|---|
| `/cam_right/cam_right` | `realsense2_camera` | RealSense | 对齐后的 RGB、深度、内参 |
| `/product_pointcloud` | `node.py` | RGB-D、目标字符串 | mask、overlay、商品/场景点云、感知 JSON |
| `/straight_grasp` | `grasp_node.py` | 商品点云、TF | 候选诊断、pregrasp、grasp、开口、抓取 JSON |
| `/cartesian_approach_executor` | C++ | MoveIt RobotState/PlanningScene | `/openflex/cartesian_approach` |
| `/right_product_grasp_executor` | `product_grasp_executor.py` | 抓取结果、TF、关节状态 | MoveIt/夹爪/升降动作和执行状态 JSON |
| `/right_grasp_web_visualizer` | `grasp_web_visualizer.py` | 图像、点云、结果 topic | 可选本地 Web 检查页 |

执行器启动后先记录当前 TCP、右臂七个关节和篮筐模式下的 `lift_joint`，把该状态定义为“拍照位”；等待感知节点就绪后才发布商品名称。它还会拒绝同名节点或共享 topic 上的重复流水线，避免两个抓取任务同时驱动机器人。

### 3.1 感知和实例选择

`ProductPointCloudNode` 使用近似时间同步接收：

- `/cam_right/color/image`
- `/cam_right/depth/image`
- `/cam_right/depth/camera_info`

默认同步误差上限 `0.12 s`。Qwen 输出归一化 2D 框和语义置信度；SAM2 对每个框产生 mask。深度按 `0.001` 转成米，默认保留 `0.07~1.5 m`，单个实例至少需要 100 个有效点。

最多处理 8 个实例。先剔除深度点不足的实例，再按以下优先级选择：

```text
Qwen 语义置信度 -> SAM 分割分数 -> 点云可抓取几何分数
```

也就是说，几何分数不会把“更好抓但语义不匹配”的物体选成目标。结果在 `/product_perception/result` 中以 JSON 给出，并包含所有实例的选择原因。

### 3.2 GraspNet 和最终位姿的关系

GraspNet 输入是分割后的商品点云，不是完整场景点云。默认随机采样 20,000 点、300 个 view，网络输出候选数组。候选会计算：

- 网络分数。
- 候选中心到分割点云稳健中心的距离。
- 候选 approach 与指定 approach 的夹角。
- 候选宽度和闭合轴。

启动入口把候选中心容差设为 `0.08 m`、approach 容差设为 `35 deg`。低于这些条件的候选仍会出现在诊断 JSON 中，但不参与约束统计。

关键点：GraspNet 不直接输出最终执行位姿。

- 抓取中心取商品点云 1%~99% 裁剪后的中位数，再沿 approach 增加 `grasp_depth_offset`。
- 实际夹爪所需开口取分割点云在闭合轴上的 2%~98% 跨度，再加 margin。
- GraspNet 的预测宽度当前只用于诊断，不直接控制夹爪。
- 完整场景点云虽然发布，但当前抓取拟合显式传入 `scene_points=None`，所以点云走廊碰撞检查是关闭的；真正的碰撞检查依赖 MoveIt PlanningScene。

货架模式会对 GraspNet 候选做加权拟合，用于网络分数和闭合轴诊断/拟合；篮筐模式的平面内闭合轴来自商品点云包围盒的窄边。迁移或替换模型时，必须保留这层几何后处理，除非同步修改下游约定和重新验证。

## 4. 两种抓取模式

| 项目 | `shelf` 货架模式 | `basket` 篮筐模式 |
|---|---|---|
| approach | `base_link +X` | `base_link -Z` |
| 闭合轴 | 默认 `base_link -Y`，取与当前 TCP 滚转最近的等价方向 | 商品点云平面包围盒窄边 |
| pregrasp | 商品表面中心沿 approach 反向 `0.08 m`（主入口） | 同样由 approach 反向生成，位于商品上方 |
| 拍照位到 pregrasp | 默认右臂 + 升降柱 8 DOF | 默认右臂 + 升降柱 8 DOF |
| 最后进给 | 右臂 + 升降柱严格 Cartesian，必要时可回退普通 MoveIt | 右臂 + 升降柱严格 Cartesian |
| 底盘 | 禁止 | 禁止 |
| 返回 | 抬高 TCP、回 pregrasp、恢复拍照关节位 | 升降柱回撤，再精确恢复右臂和升降拍照关节位 |

虽然 `PlanPregrasp.action` 定义了 11 DOF `WHOLE_BODY`，商品抓取执行器只接受 7 或 8 DOF，显式禁止 Stage C/底盘规划。

## 5. 执行状态机和安全语义

执行模式的主要顺序如下：

```text
记录拍照状态
  -> 执行前全开夹爪
  -> 等待唯一感知流水线
  -> 请求商品感知
  -> 校验 grasp_result 必须 valid、backend=graspnet、mode 匹配
  -> 将相机坐标 pose 转到 odom
  -> 再次全开夹爪
  -> 规划并执行 photo -> pregrasp
  -> 用实时 TF 检查实际 TCP 与 pregrasp 的位置误差
  -> 预检查并执行 pregrasp -> grasp
  -> 闭合夹爪
  -> 可选设置负载补偿
  -> 抬升/回撤
  -> 恢复拍照位
```

`--plan` 并不是全链路离线仿真。它会做感知并检查“拍照位到 pregrasp”可达性，但 `plan_only_complete` 中明确标记 `final_feed_deferred=true`；最后的 pregrasp 到 grasp 进给要在实际到达 pregrasp 后结合最新 RobotState 再检查。因此 `--plan` 成功是必要条件，不是执行必然成功的保证。

动作超时后，执行器会取消目标并把运动状态标为不确定。若错误发生时运动状态已知，执行器会尝试反向回拍照位；若是控制失败、终点容差失败或 Action 结果超时，则不会盲目自动恢复，需人工确认机器人实际位置。

默认夹爪总开口上限为 `0.086 m`，对应单指位置 `0.043 m`。当前入口闭合目标是物理零位 `0.0 m`；夹爪因抓到商品而 stalled 可被接受。迁移到不同夹爪时必须重新标定：

- `gripper_joint_max_position_m`
- `gripper_opening_scale`
- `gripper_close_position_m`
- `gripper_open_stall_tolerance_m`
- `gripper_max_effort`

## 6. ROS 接口契约

### 6.1 Topics

| 方向 | Topic | 类型 | 说明 |
|---|---|---|---|
| 输入 | `/product_perception/target` | `std_msgs/String` | 自然语言商品名 |
| 输入 | `/cam_right/color/image` | `sensor_msgs/Image` | BGR/RGB 图像 |
| 输入 | `/cam_right/depth/image` | `sensor_msgs/Image` | 对齐到彩色的深度 |
| 输入 | `/cam_right/depth/camera_info` | `sensor_msgs/CameraInfo` | 必须与对齐深度匹配 |
| 输入 | `/joint_states` | `sensor_msgs/JointState` | 七个右臂关节及篮筐模式的 `lift_joint` |
| 输出 | `/product_perception/mask` | `sensor_msgs/Image` | 选中实例二值 mask |
| 输出 | `/product_perception/overlay` | `sensor_msgs/Image` | 框、mask 和候选标注 |
| 输出 | `/product_perception/points` | `sensor_msgs/PointCloud2` | 商品 XYZRGB 点云 |
| 输出 | `/product_perception/scene_points` | `sensor_msgs/PointCloud2` | 完整场景 XYZRGB 点云，仅可视化/预留 |
| 输出 | `/product_perception/result` | `std_msgs/String` | 感知 JSON |
| 输出 | `/product_perception/grasp_candidates` | `std_msgs/String` | 候选接受/拒绝诊断 JSON |
| 输出 | `/product_perception/pregrasp_pose` | `geometry_msgs/PoseStamped` | 相机点云坐标系下预抓取位姿 |
| 输出 | `/product_perception/grasp_pose` | `geometry_msgs/PoseStamped` | 相机点云坐标系下抓取位姿 |
| 输出 | `/product_perception/gripper_width` | `std_msgs/Float32` | 总开口，单位米 |
| 输出 | `/product_perception/grasp_result` | `std_msgs/String` | 抓取拟合 JSON，执行器的有效性门控 |
| 输出 | `/product_perception/grasp_execution_result` | `std_msgs/String` | 阶段、是否接受、错误和路径指标 |
| 输出 | `/product_perception/grasp_path` | `nav_msgs/Path` | pregrasp 到 grasp 可视化 |
| 输出 | `/product_perception/grasp_markers` | `visualization_msgs/MarkerArray` | RViz 箭头和夹爪框 |

### 6.2 Actions 和服务

| 名称 | 类型 | 提供者 |
|---|---|---|
| `/openflex/plan_pregrasp` | `openarmx_integrated_moveit_msgs/action/PlanPregrasp` | `openarmx_integrated_moveit_config/openflex_pick_task_executor` |
| `/openflex/cartesian_approach` | `openarmx_integrated_moveit_msgs/action/CartesianApproach` | `openflex_grasp_moveit/cartesian_approach_executor` |
| `/right_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | 目标机器人的夹爪控制器 |
| `/lift_joint_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | 升降控制器，篮筐执行需要 |
| `/gravity_comp_node/set_parameters` | `rcl_interfaces/srv/SetParameters` | 可选负载补偿节点 |

`CartesianApproach` 支持三种路径：

- `STRICT_CARTESIAN`：整段 `computeCartesianPath`，要求路径比例达到阈值。
- `BOUNDED_CORRIDOR`：允许关节空间规划，但给 TCP 加有限走廊约束，并对轨迹重新采样验证横向偏差、路径长度、回退量和终点。
- `UNCONSTRAINED_MOVEIT`：普通 MoveIt pose/joint 目标规划，仍执行碰撞和关节限位检查，但没有自定义走廊约束。

当前主入口默认 `unconstrained_moveit=true`：长距离 photo 到 pregrasp 使用普通 MoveIt，最后抓取进给优先严格 Cartesian，失败时允许回退普通 MoveIt。对货架、篮沿等窄空间部署时，应根据风险决定是否关闭回退或启用走廊模式。

## 7. TF、URDF 和控制器契约

感知点云使用深度消息的 `frame_id`，当前通常为 `right_camera_color_optical_frame` 或对齐深度对应的 optical frame。完整 TF 必须连续：

```text
odom
  -> base_link
  -> ...
  -> openarmx_right_link7
  -> right_camera_color_optical_frame
  -> right_camera_depth_optical_frame   # 若深度消息使用此 frame
```

还必须存在：

```text
odom -> ... -> openarmx_right_hand_tcp
```

当前手眼 YAML 位于 `openflex_calibration/hand_eye.yaml`，其中右腕外参父 frame 是 `openarmx_right_link7`，子 frame 是 `right_camera_color_optical_frame`。这个数值只属于原机器人，不可复制到另一台真机。

MoveIt 侧当前写死或约定：

- planning group：`right_arm` 和 `left_arm`。
- TCP：`openarmx_right_hand_tcp`。
- 右臂关节：`openarmx_right_joint1` 到 `openarmx_right_joint7`。
- 升降关节：`lift_joint`。
- 规划坐标系：执行器默认 `odom`。

若目标仍是相同 OpenArmX 整机，只需保证同版描述包、MoveIt 配置和控制器。若目标是不同机器人，建议保持上层 topics/Actions 不变，在机器人适配层实现同等的 `PlanPregrasp`、`CartesianApproach`、夹爪 Action 和 TF；直接在 1900 行执行状态机里散改关节名会增加后续维护成本。

## 8. 模型和当前资产审计

代码默认查找：

```text
openflex_models/Qwen3.5-4B/
openflex_models/sam2.1-hiera-small/
openflex_models/graspnet-baseline/checkpoint-rs.tar
```

本快照实际情况：

| 资产 | 当前状态 | 结论 |
|---|---|---|
| `Qwen3.5-4B` | 已部署并完成离线加载 | perception worker 使用 BF16 |
| `sam2.1-hiera-small/model.safetensors` | 存在，184,305,280 bytes | 看起来完整，仍应做加载测试 |
| `paligemma-3b-pt-224` | 只有约 4.2 MB tokenizer/config，无权重 | 不可运行，也不能直接替换 Qwen |
| `checkpoint-rs.tar` | 存在，12,468,415 bytes | 通过代码最小尺寸检查，仍应做实际加载/推理测试 |

已记录校验值：

```text
60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868  checkpoint-rs.tar
0a4067b11ce1e23d5229203f11c718a823060d15a4b23fa2372a7d4b77cbbc60  sam2.1-hiera-small/model.safetensors
```

PaliGemma 不能通过改目录名直接替换 Qwen。`models.py` 使用多模态 chat template，传入 `enable_thinking=False`，并要求模型严格生成多实例 JSON 框。替换视觉语言模型时至少要改 processor 调用、prompt、输出解析和回归测试。

## 9. 环境和依赖

当前设计目标为 Ubuntu + ROS 2 Humble。ROS 侧主要依赖：

- MoveIt 2、`moveit_ros_planning_interface`。
- `realsense2_camera`。
- `rclpy`、`rclcpp`、`tf2_ros`、`message_filters`、`cv_bridge`。
- `control_msgs`、`trajectory_msgs`、`sensor_msgs`、`geometry_msgs`、`nav_msgs`、`visualization_msgs`。
- OpenArmX 的 integrated description、MoveIt config、控制器及手眼标定包。

模型 Python 环境拆为 `openflex_perception` 与 `openflex_graspnet`，避免 Qwen/SAM2 和
GraspNet 的依赖相互污染：

- PyTorch 和匹配的 CUDA runtime。
- `transformers`，版本必须同时支持 `AutoModelForImageTextToText`、`Sam2Model` 和 `Sam2Processor`。
- NumPy、SciPy、OpenCV、Pillow、Open3D、tqdm。
- ROS Python 包能够从两个解释器导入；分别使用 `OPENFLEX_PERCEPTION_PYTHON` 与
  `OPENFLEX_GRASPNET_PYTHON` 指定解释器。

控制、MoveIt bridge、任务编排和 Web 节点继续使用 ROS 系统 Python，不进入模型 venv。

PointNet2 和 KNN 含 CUDA/C++ 扩展，不要复制源机生成的 `.so`：

```bash
export CUDA_HOME=/home/openflex/old/.cuda/cuda-12.6-local/usr/local/cuda-12.6
export TORCH_CUDA_ARCH_LIST=8.6

cd "${OPENFLEX_GRASPNET_REPO}/pointnet2"
"${OPENFLEX_GRASPNET_PYTHON}" setup.py install

cd "${OPENFLEX_GRASPNET_REPO}/knn"
"${OPENFLEX_GRASPNET_PYTHON}" setup.py install
```

编译前确认 `nvcc --version`、`python -c 'import torch; print(torch.__version__, torch.version.cuda)'` 和显卡计算架构相容。

## 10. 推荐迁移步骤

### 10.1 复制源码，不复制构建产物

目标工作空间建议结构：

```text
<TARGET_ROOT>/
├── openflex_models/
├── openflex_calibration/hand_eye.yaml
├── openflex_robot.env
└── openflex_ws/
    └── src/
        ├── openflex_grasp/                 # 本目录整体
        ├── openflex_hand_eye_calibration/
        ├── openflex_moveit/
        └── 目标机器人的描述、控制器和 bringup 包
```

至少保留 `openflex_grasp` Python 包、`openflex_grasp_moveit` 和 GraspNetBaseline 源码，不要只复制 Python 包。MoveIt Action 统一由目标 workspace 的 `openarmx_integrated_moveit_msgs` 提供，禁止恢复或复制 `openflex_grasp_msgs`。

### 10.2 用环境变量消除绝对路径

在目标机的启动环境中设置：

```bash
export OPENFLEX_ROOT=/absolute/path/to/openflex_all
export OPENFLEX_WORKSPACE="${OPENFLEX_ROOT}/openflex_ws"
export OPENFLEX_GRASP_SUITE_ROOT="${OPENFLEX_WORKSPACE}/src/openflex_grasp"
export OPENFLEX_MODEL_ROOT="${OPENFLEX_ROOT}/openflex_models"

export OPENFLEX_GRASPNET_REPO="${OPENFLEX_GRASP_SUITE_ROOT}/graspnet-baseline"
export OPENFLEX_GRASPNET_CHECKPOINT="${OPENFLEX_MODEL_ROOT}/graspnet-baseline/checkpoint-rs.tar"
export OPENFLEX_QWEN_MODEL="${OPENFLEX_MODEL_ROOT}/Qwen3.5-4B"
export OPENFLEX_SAM_MODEL="${OPENFLEX_MODEL_ROOT}/sam2.1-hiera-small"

export OPENFLEX_ROBOT_ENV="${OPENFLEX_ROOT}/openflex_robot.env"
export OPENFLEX_PERCEPTION_PYTHON="${OPENFLEX_ROOT}/.venvs/openflex_perception/bin/python"
export OPENFLEX_GRASPNET_PYTHON="${OPENFLEX_ROOT}/.venvs/openflex_graspnet/bin/python"
```

机器人环境文件只保存机器专属配置：

```bash
export OPENFLEX_RIGHT_CAMERA_SERIAL=_replace_with_target_robot_serial
export OPENFLEX_LEFT_CAMERA_SERIAL=_replace_with_target_robot_serial
```

还可按机器调整：`OPENFLEX_GRASP_MODE`、`OPENFLEX_PREGRASP_DISTANCE_M`、`OPENFLEX_POSTGRASP_LIFT_M`、`OPENFLEX_TRAJECTORY_VELOCITY_SCALE`、`OPENFLEX_TRAJECTORY_ACCELERATION_SCALE`、`OPENFLEX_PAYLOAD_MASS_KG` 和 `OPENFLEX_PAYLOAD_COM_Z_M`。

### 10.3 放置并校验模型

```bash
test -d "${OPENFLEX_QWEN_MODEL}"
test -f "${OPENFLEX_SAM_MODEL}/model.safetensors"
test -f "${OPENFLEX_GRASPNET_CHECKPOINT}"
sha256sum "${OPENFLEX_GRASPNET_CHECKPOINT}" \
  "${OPENFLEX_SAM_MODEL}/model.safetensors"
```

Qwen 模型必须是完整本地 Hugging Face 模型目录，至少应包含权重分片/索引、config、processor 和 tokenizer 文件。由于代码使用 `local_files_only=True`，运行时不会自动联网补文件。

### 10.4 新机标定

1. 写入正确的 RealSense 序列号。
2. 对目标右臂和右腕相机重新做手眼标定。
3. 把结果写到 `${OPENFLEX_ROOT}/openflex_calibration/hand_eye.yaml`。
4. 启动标定发布节点后检查 TF 连续性和方向。

```bash
ros2 launch openflex_hand_eye_calibration apply_calibration.launch.py \
  calibration_file:="${OPENFLEX_ROOT}/openflex_calibration/hand_eye.yaml" \
  side:=right

ros2 run tf2_ros tf2_echo odom right_camera_color_optical_frame
ros2 run tf2_ros tf2_echo odom openarmx_right_hand_tcp
```

未完成新机手眼标定时，可以检查 RGB、深度和相机坐标点云，但禁止执行真机抓取。

### 10.5 重新构建工作空间

```bash
cd "${OPENFLEX_WORKSPACE}"
source /opt/ros/humble/setup.bash
rm -rf build/openflex_grasp build/openflex_grasp_moveit
rm -rf install/openflex_grasp install/openflex_grasp_moveit

colcon build --symlink-install --packages-up-to \
  openarmx_integrated_moveit_msgs \
  openflex_grasp_moveit \
  openarmx_integrated_moveit_config \
  openflex_hand_eye_calibration \
  openflex_grasp

source install/setup.bash
ros2 pkg prefix openflex_grasp
ros2 interface show openarmx_integrated_moveit_msgs/action/CartesianApproach
```

上面的删除只针对两个明确命名的旧构建目录；新工作空间首次构建时可直接省略。若迁移的是整套 OpenArmX，优先用 `--packages-up-to` 让 colcon 自动补齐依赖，不要手工猜所有底层包。

### 10.6 启动顺序

1. 启动硬件、控制器、`robot_state_publisher` 和 MoveIt。
2. 应用右腕手眼标定。
3. 确认无 VR、自由拖动或其他机械臂控制源。
4. 首次只规划。
5. 检查可视化和结果 JSON 后再执行。

```bash
ros2 launch openarmx_integrated_moveit_config \
  integrated_robot_bringup_moveit.launch.py \
  use_fake_hardware:=false use_rviz:=true \
  arm_control_mode:=csp gripper_control_mode:=mit

ros2 launch openflex_hand_eye_calibration apply_calibration.launch.py side:=right

"${OPENFLEX_WORKSPACE}/grab_product.sh" --plan --shelf water bottle
"${OPENFLEX_WORKSPACE}/grab_product.sh" --plan --basket water bottle
```

`grab_product` 会修改正在运行的 `/move_group` 的轨迹执行监控参数，并检查 `/moveit_command_bridge` 是否具有新版终点容差参数。因此目标端必须迁移相容版本的 integrated MoveIt 栈，或明确重写该脚本和执行契约。

## 11. 分阶段验证与验收

### 11.1 不上电执行的验证

```bash
python3 -m py_compile \
  openflex_grasp/openflex_grasp/*.py

cd "${OPENFLEX_GRASP_SUITE_ROOT}/openflex_grasp"
pytest -q test
```

然后分别在 perception/GraspNet 环境验证模型可加载、CUDA 扩展可 import，并用一帧录制
RGB-D 做离线推理。

### 11.2 ROS 图检查

```bash
ros2 action list | grep -E 'plan_pregrasp|cartesian_approach|gripper_cmd'
ros2 topic hz /cam_right/color/image
ros2 topic hz /cam_right/depth/image
ros2 topic echo --once /cam_right/depth/camera_info
ros2 topic echo /product_perception/grasp_execution_result
```

检查点：

- RGB 与对齐深度分辨率一致，内参属于该对齐图像。
- 商品点云贴合实际物体，没有整体偏移、镜像或尺度错误。
- RViz 中 pregrasp 位于物体外，grasp 位于合理夹持深度。
- 货架箭头沿机器人前向接近，篮筐箭头从上向下。
- MoveIt PlanningScene 中货架、桌面、篮筐和机器人自碰撞模型正确。
- `/product_perception/grasp_result` 的 `valid=true`、`backend=graspnet`、`grasp_mode` 正确。

### 11.3 真机验收顺序

1. 急停可用，低速度，工作区清空，先用大而规则的轻质物体。
2. `--plan` 连续运行多次，确认实例选择和姿态稳定。
3. 暂时增大 pregrasp 距离，执行到接近段并准备随时急停。
4. 验证夹爪全开、闭合零点和 stalled 行为。
5. 验证货架模式，不允许底盘和升降意外运动。
6. 升降完成回零后再验证篮筐模式。
7. 验证持物返回，最后才启用负载补偿。

建议把以下结果保存为每台机器的验收记录：模型 SHA256、软件提交版本、相机序列号、手眼标定文件、夹爪参数、成功/失败样本图、抓取 JSON 和 MoveIt 执行日志。

## 12. 常见故障

### 模型目录不存在或启动立即失败

检查 `OPENFLEX_QWEN_MODEL`、`OPENFLEX_SAM_MODEL`。当前目标机已部署完整 Qwen3.5-4B；
迁移到其他机器时必须复制完整权重、config、processor 和 tokenizer，代码不会在线下载。

### `pointnet2`、`knn` 或 CUDA symbol 导入失败

目标机重新编译两个扩展。确认编译时和运行时使用同一个 Python、PyTorch、CUDA，不要复制源机 `.so`。

### ROS 包找不到或 Python egg-link 指向旧目录

这是复用 `install/` 的典型现象。删除对应包的 `build/<pkg>` 和 `install/<pkg>` 后在目标工作空间重建，再重新 `source install/setup.bash`。

### 点云正确但抓取位姿整体偏移

优先检查手眼标定和 optical frame 名称，不要靠修改 `grasp_depth_offset` 补偿刚性外参误差。还要确认深度已对齐到彩色且 `CameraInfo` 与深度图对应。

### 选中了同类商品中的错误实例

查看 `/product_perception/result` 的 `instances`。当前策略优先语义置信度，不是优先最近或最好抓；如业务需要“左边第一个”“离机器人最近”等规则，应新增明确的实例选择策略，而不是调 GraspNet。

### `No solution within max_dof=7/8`

确认目标 pose 的 TF、pregrasp 距离、MoveIt 当前 RobotState、升降是否回零及 PlanningScene。货架和篮筐均允许手臂与升降柱 8 DOF，但不会用底盘兜底。

### `plan_only_complete` 后执行仍在最后进给失败

这是当前语义：只规划阶段把最终 feed 延后。最后 Cartesian feed 需要实际到达 pregrasp 后再算。检查 `cartesian_min_fraction`、障碍物、起点误差和是否允许普通 MoveIt 回退。

### 检测到重复流水线

彻底停止旧的 `grab_product` launch。共享 topic 设计不支持同一 ROS domain 内同时运行两套同名抓取流水线；多机器人部署要加 namespace 并把所有绝对 topic 参数化。

## 13. 许可证和授权

这套实现与 AnyGrasp SDK 授权的关系如下：

- 当前代码强制 `backend=graspnet`，没有加载 AnyGrasp SDK，也没有检查 AnyGrasp License 文件。
- 因此运行当前代码不需要 AnyGrasp 的机器授权或 Google 账号申请。
- 但是随包的 GraspNetBaseline 并非宽松开源许可。`graspnet-baseline/LICENSE` 写明仅限学术或非营利组织的非商业研究，个人许可不可转让、不可再授权，对复制、分发和衍生修改也有明确限制。
- `openflex_grasp_moveit` 和 `openarmx_integrated_moveit_msgs` 的 `package.xml` 标记为 `CC-BY-NC-SA-4.0`，包含非商业和相同方式共享条件。
- `openflex_grasp` 自身标记为 `Apache-2.0`，但不能覆盖上述子组件、模型权重及第三方依赖的许可。
- Qwen、SAM2 和各模型权重还要以部署资产附带的模型卡/许可证为准；当前部署的版本和
  SHA256 已记录在模型 manifest，但跨主体分发仍需重新核对条款。

所以，内部非商业研发迁机不涉及 AnyGrasp 激活，但仍应确认 GraspNet 和模型条款；商业产品、向客户交付、镜像分发或跨主体迁移前，必须由法务/版权负责人逐项确认或替换受限组件。代码中的“license-free GraspNetBaseline”提示只能理解为“不需要 AnyGrasp 运行时授权文件”，不能理解为“无许可证限制”。

## 14. 建议的后续工程化改造

若这个能力要长期集成到多个工作空间，建议按优先级处理：

1. 为 Qwen、SAM2、GraspNet 增加启动前 manifest 校验，记录模型版本、大小、SHA256 和许可证来源。
2. 把绝对 topic、右臂关节名、planning group、TCP 和控制器 Action 收敛到一份机器人适配 YAML。
3. 将“实例选择策略”参数化，支持语义优先、最近、画面区域和业务指定实例。
4. 明确 `--plan` 只验证到 pregrasp，或增加仿真 RobotState 下的完整 feed 预检。
5. 把当前未使用的场景点云接入可靠的场景碰撞体生成，或删去会造成误解的点云走廊参数。
6. 对不同夹爪建立独立标定配置，不把 `0.086/0.043/0.0 m` 固化为产品默认值。
7. 商业化前替换或取得 GraspNetBaseline、CC-BY-NC-SA 组件的适当授权。

完成这些改造后，上层商品抓取逻辑才能真正成为“模型、机器人和部署路径均可替换”的稳定模块。
