# openflex_grasp

`openflex_grasp` 是可迁移的 OpenFlex 商品抓取包，包含：

- Qwen3.5 商品文本定位与 SAM2 分割。
- RGB-D 商品点云生成。
- GraspNetBaseline 推理，以及货架正向抓取和篮筐俯抓位姿拟合，不回退到 PCA。
- 左/右臂与升降柱 8 DOF MoveIt 规划、Cartesian 进给、夹爪执行和返回拍照位；底盘不参与。
- 商品点云最低点加可配置余量形成 TCP 高度安全线，并对完整规划轨迹逐段验证。
- 分割图、点云和抓取候选可视化。
- 一条命令的抓取入口。

该包不保存手眼标定结果。运行时只要求 TF 中存在：

```text
odom -> ... -> openarmx_right_link7 -> right_camera_color_optical_frame
```

手眼结果由 `openflex_hand_eye_calibration` 单独保存和发布。机器人专属相机序列号由
`/home/openflex/old/openflex_robot.env` 提供。

## 外部模型

```text
/home/openflex/old/openflex_ws/src/openflex_grasp/graspnet-baseline
/home/openflex/old/openflex_models/graspnet-baseline/checkpoint-rs.tar
/home/openflex/old/openflex_models/Qwen3.5-4B
/home/openflex/old/openflex_models/sam2.1-hiera-small
```

GraspNetBaseline 源码和 `checkpoint-rs.tar` 权重随 `src/openflex_grasp` 一起迁移。它的 PointNet2 和 KNN CUDA 扩展必须针对目标机器人的 Python、PyTorch、
CUDA 和显卡架构在目标机本地编译。

```bash
export CUDA_HOME=/home/openflex/old/.cuda/cuda-12.6-local/usr/local/cuda-12.6
export TORCH_CUDA_ARCH_LIST=8.6
cd /home/openflex/old/openflex_ws/src/openflex_grasp/graspnet-baseline/pointnet2
/home/openflex/old/.venvs/openflex_graspnet/bin/python setup.py install
cd /home/openflex/old/openflex_ws/src/openflex_grasp/graspnet-baseline/knn
/home/openflex/old/.venvs/openflex_graspnet/bin/python setup.py install
```

## 构建

```bash
cd /home/openflex/old/openflex_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-select openarmx_integrated_moveit_msgs openflex_grasp_moveit openarmx_integrated_moveit_config openflex_grasp
```

## 启动顺序

1. 启动整机和 MoveIt：

```bash
source /opt/ros/humble/setup.bash && source /home/openflex/old/openflex_ws/install/setup.bash && ros2 launch openarmx_integrated_moveit_config integrated_robot_bringup_moveit.launch.py use_fake_hardware:=false use_rviz:=true arm_control_mode:=csp gripper_control_mode:=mit
```

2. 应用当前机器人的右手眼标定：

```bash
手眼标定已按迁移清单写入整机 URDF 固定关节，不再启动独立标定发布器。
```

3. 只规划，不执行：

```bash
/home/openflex/old/openflex_ws/grab_product.sh --plan water bottle
```

4. 确认 RViz 中点云、抓取位姿和右臂轨迹正确后执行：

```bash
/home/openflex/old/openflex_ws/grab_product.sh water bottle
```

从篮筐抓取时增加 `--basket`。该模式在 `base_link` 中沿 `-Z` 从商品上方向下接近，
只约束夹爪闭合轴与桌面平行，平面内角度取商品点云水平包围盒的窄边；默认的货架模式
仍沿 `+X` 正向接近：

```bash
# 先只规划篮筐俯抓
/home/openflex/old/openflex_ws/grab_product.sh --plan --basket water bottle

# 确认预抓取点位于商品上方、路径没有碰到篮沿后再执行
/home/openflex/old/openflex_ws/grab_product.sh --basket water bottle
```

也可以显式使用 `--shelf`，或在 ROS launch 中传入 `grasp_mode:=shelf|basket`。
篮筐模式的拍照位到预抓取位允许 MoveIt 联合规划右臂和立柱（最多 8 DOF），但预抓取到
抓取点仍保持纯右臂直线下探，且不会启用底盘规划。夹取后返回拍照位时同样允许立柱参与。

如果仅规划时报 `No solution within max_dof=8`，先确认旧的 `grab_product` launch 已经用
`Ctrl-C` 完全退出，再重新启动。执行器会拒绝多个同名感知/抓取流水线，并在首次
RobotState/TF 尚未就绪而返回 `NO_IK_SOLUTION` 时自动刷新后重试一次。更新 MoveIt
规划服务器代码后，需要重启整机 MoveIt launch 才能加载新的状态监视器预热逻辑。

安装后也可以直接使用 ROS 包内入口：

```bash
ros2 run openflex_grasp grab_product --plan water bottle
```

## 新机器人配置

每台机器人只需要提供两个工作区外文件，不修改 `openflex_grasp` 源码：

```text
/home/openflex/old/openflex_robot.env
手眼结果已固化在 openarmx_integrated_description 的左右相机固定关节中
```

`openflex_robot.env` 示例：

```bash
export OPENFLEX_LEFT_CAMERA_SERIAL=_replace_left_serial
export OPENFLEX_RIGHT_CAMERA_SERIAL=_replace_right_serial
```

手眼 YAML 格式见 `openflex_hand_eye_calibration/config/hand_eye.yaml.example`。未完成新机手眼
标定时只能检查相机图像和相机坐标系点云，禁止执行真机抓取。

## 主要接口

- `/product_perception/target`：商品名称。
- `/product_perception/points`：商品分割点云。
- `/product_perception/scene_points`：场景点云。
- `/product_perception/pregrasp_pose`：预抓取位姿。
- `/product_perception/grasp_pose`：抓取位姿。
- `/product_perception/grasp_result`：GraspNet 拟合结果。
- `/product_perception/grasp_execution_result`：执行阶段结果。
- `/right_gripper_controller/gripper_cmd`：右夹爪 action。
- `/openflex/cartesian_approach`：右臂 Cartesian action。

默认抓取入口固定使用 GraspNetBaseline、`shelf` 模式、右臂与升降柱 8 DOF、最大总夹爪开口
`0.086 m`，不移动底盘或另一侧手臂。
