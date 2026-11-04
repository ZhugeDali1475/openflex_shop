# OpenFleX Thesis MuJoCo

这是独立于比赛代码的论文数字实验工程，复用 `src/openflex_mujoco` 的整机 MJCF 和网格，服务于：

```text
仿真导航/真实Nav2 -> COARSE AnyGrasp -> TC-WAM(b,h,g,ready region)
                     -> ACT底盘精对位 -> REFINED AnyGrasp
                     -> MoveIt(lift_joint + active arm) -> 抓取验证 -> 固定放置
```

## 已实现边界

- 从上游 MJCF 运行时生成论文场景，不复制大型 mesh。
- 移除上游仅用于 viewer 展示的整机 `-90 deg` 偏转，使 MuJoCo 与 URDF/MoveIt 坐标一致。
- 加入 `base_x/base_y/base_yaw`、货架、商品、遮挡板、交付台、头部/腕部相机和预留 weld。
- 底盘采用带一阶速度响应的全向运动学模型，接收 `/thesis/cmd_vel_selected`。
- 发布 `/clock`、`/joint_states`、里程计、`map->odom->base_link` TF 和商品观测。
- 提供与 MoveIt 配置一致的升降柱、左右臂 `FollowJointTrajectory` Action 和左右夹爪 Action。
- 夹爪靠近商品并闭合时建立确定性 TCP 附着，持续发布独立 `GraspStatus`。
- 支持商品固定放置、名义重置、随机位置/姿态、遮挡条件和试验元数据。
- MuJoCo Oracle 作为现有 AnyGrasp gateway 的后端，结果始终为 `physical_perception=false`。
- 提供比例式局部对位和导航基线，节点名与日志均明确标记为“不是 ACT / 不是 Nav2”。

当前相机已写入 MJCF，但尚未发布渲染 RGB-D/PointCloud2。现阶段 Oracle 用于验证控制与 TC-WAM；论文的感知实验阶段必须再接入“渲染点云 -> AnyGrasp”，两类结果不可混记。

## 构建

```bash
cd /home/openarmx/openflex_all/openflex_ws
python3 -m pip install -r src/openflex_thesis/openflex_thesis_mujoco/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to openflex_thesis_mujoco
source install/setup.bash
```

只检查派生模型：

```bash
ros2 run openflex_thesis_mujoco build_thesis_scene
python3 -m mujoco.viewer /tmp/openflex_thesis_mujoco_scene.xml
```

## 运行层级

1. 只启动 MuJoCo、ROS 控制接口和 TF：

```bash
ros2 launch openflex_thesis_mujoco mujoco_core.launch.py enable_viewer:=true
```

2. 启动 TC-WAM 热力图闭环。默认使用 Oracle 抓姿、合成可达性评价和比例局部策略，只用于工程联调：

```bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py enable_viewer:=true
```

`auto_compute` 会在获得商品记忆后生成 `/thesis/tc_wam/score_grid`、候选 Marker、最优底盘位姿和绑定抓姿的 ready 区域。

3. 用 MoveIt 做真实 IK/碰撞评价：

```bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py \
  start_moveit:=true use_moveit_evaluator:=true enable_viewer:=true
```

4. 运行完整 pick-deliver 仿真任务：

```bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py \
  start_moveit:=true run_mission:=true enable_viewer:=true
```

先用默认 mock evaluator 打通执行，再切换 `use_moveit_evaluator:=true`。完整抓取成功还取决于当前 MoveIt 碰撞模型、IK 解算器和 Oracle TCP 姿态是否与真机标定一致。

## 接入 ACT 与 Nav2

替换比例局部策略：

```bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py start_scripted_policy:=false
```

ACT 进程订阅 `/thesis/act/policy_control`，只向 `/thesis/act_cmd_vel` 发布 `[vx, vy, wz]`。ACT 不得发布升降柱、机械臂或夹爪命令。

替换仿真导航基线：

```bash
ros2 launch openflex_thesis_mujoco thesis_mujoco.launch.py start_sim_navigation:=false
```

然后启动真正的 Nav2。当前桥尚未提供 LaserScan，因此真实 Nav2 闭环需要下一阶段加入 MuJoCo ray-cast 激光或外部仿真激光节点。不能把现有比例导航基线写成 Nav2 实验。

## 场景控制

```bash
ros2 service call /thesis_sim/reset_nominal std_srvs/srv/Trigger '{}'
ros2 service call /thesis_sim/randomize std_srvs/srv/Trigger '{}'
ros2 service call /thesis_sim/drop_object std_srvs/srv/Trigger '{}'
```

随机范围、遮挡概率、观测噪声和种子在 `config/mujoco.yaml`。每次重置的条件发布到 `/thesis_sim/trial_metadata`，实验日志必须保存该字段。

## 推荐实验顺序

```text
S0 接口冒烟
  Oracle AnyGrasp + mock evaluator + scripted policy
      |
S1 几何闭环
  Oracle AnyGrasp + MoveIt IK/collision + scripted policy
      |
S2 ACT名义场景
  固定商品，比较 ACT 与 scripted baseline
      |
S3 位置转移
  固定随机种子矩阵，重算 TC-WAM，比较成功率与对位代价
      |
S4 遮挡感知
  渲染点云 + AnyGrasp，单独统计检测/抓姿失败
      |
S5 真机迁移
  重新标定货架、TCP、商品几何和导航交接区
```

实验矩阵模板见 `config/experiment_matrix.yaml`。Oracle、mock、MoveIt 数字评价、真实 AnyGrasp 和真机结果必须用不同 `evidence_class` 分组报告。
