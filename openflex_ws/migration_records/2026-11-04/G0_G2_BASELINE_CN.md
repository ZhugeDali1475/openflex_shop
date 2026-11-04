# G0-G2 MuJoCo/MoveIt 基线验收记录

日期：2026-11-04 11:10 CST  
代码提交：`754ed09450137c8e1d573ef75d7c0e3460d5cad1`  
分支：`main...origin/main`  
执行者：Codex + 项目所有者现场确认

## 结论

G0、G1、G2 本轮通过。

当前已经具备继续开展固定底盘抓放基线工作的环境条件，但还没有完成：

- G3 抓取升降范围内的左臂停放姿态验证；
- G4 单次抓取和附着验证；
- G5-G8 运输、放置、端到端与重复实验。

因此本轮结论是“MuJoCo/MoveIt 基础环境可复现”，不是“抓放任务已完成”。

## G0：环境与核心单元测试

执行：

```bash
cd /home/wgb/openflex_shop
bash scripts/check_care_environment.sh
bash scripts/verify_moveit_basic.sh
```

结果：

```text
mujoco_model 71 29 23
openflex_care_agent: 9 passed
openflex_thesis_mujoco: 5 passed
care_environment_ok
```

MoveIt 基础检查结果：

- MoveIt 相关包数量：34；
- `openarmx_integrated_moveit_config` 已安装；
- `sim_moveit.launch.py` 存在；
- Pick-IK 插件已安装；
- `openarmx_integrated_description` 已安装；
- 输出 `MoveIt基础环境验证通过`。

## G1：MuJoCo 运行态

启动命令：

```bash
bash scripts/run_thesis_moveit_mission.sh run_mission:=false
```

自动任务保持关闭，只启动 MuJoCo、MoveIt、任务技能节点和 ROS 接口。

### 时钟

执行：

```bash
ros2 topic hz /clock
```

结果：约 `50.0 Hz`，最终窗口为：

```text
average rate: 50.029
min: 0.014s
max: 0.027s
std dev: 0.00091s
```

### 关节状态

执行：

```bash
ros2 topic echo --once /joint_states
```

结果：收到 29 个关节，包含：

- 8 个轮组关节；
- `lift_joint`；
- 左右臂各 7 个关节；
- 左右夹爪各 2 个关节；
- 头部 pitch/yaw 2 个关节。

### 底盘里程计

执行：

```bash
ros2 topic echo --once /swerve_drive_controller/odom
```

结果：

```text
frame_id=odom
child_frame_id=base_link
position=(0, 0, 0)
orientation_xyzw=(0, 0, 0, 1)
twist=zero
```

### TF

执行：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

启动初期存在一次 `map` 尚不可用的短暂查询失败，随后持续输出：

```text
Translation: [0.000, 0.000, 0.000]
Rotation xyzw: [0.000, 0.000, 0.000, 1.000]
```

结论：初始 TF 发布存在短暂竞争，但稳定后 `map -> base_link` 可查询，不影响本轮验收。

## G2：MoveIt IK、轨迹执行与整机状态

### IK

执行：

```bash
bash scripts/test_moveit_ik.sh
```

结果：

```text
state_validity valid=True contacts=0
compute_ik error=1 success=True
moveit_ik_smoke_test_ok
```

### 轨迹执行

执行：

```bash
bash scripts/test_moveit_move.sh
```

结果：

```text
start_joint -0.003061685185642742
move_group error=1 success=True planned_points=8
final_joint 0.05296097066968829
moveit_move_smoke_test_ok
```

结论：MoveIt 生成的 8 点轨迹实际驱动了 MuJoCo 关节。

### 整机状态与碰撞

执行：

```bash
python3 scripts/check_moveit_current_state.py --group right_arm_with_lift
```

结果：

```text
left_arm_positions=[
  0.057721,
  0.000915,
  -0.000001,
  1.736652,
  -0.000084,
  0.000021,
  0.012884
]
group=right_arm_with_lift valid=True contacts=0
```

结论：当前初始/测试状态无碰撞。已知左臂联合碰撞发生在抓取候选与升降组合下，
因此不能用当前状态的 `contacts=0` 代替 G3。

## 启动警告与边界

本轮启动存在以下警告，需要保留在后续记录中：

1. MoveIt 报告以下 link 有视觉几何但无碰撞几何：
   `d435_link`、`chest_link`、`head_base_link`、`head_pitch_link`、
   `head_yaw_link`、`mid360_link`。
2. 当前 MoveIt 配置未能加载 `DepthImageOctomapUpdater` 插件；
   本轮不依赖 Octomap，但感知避障实验前必须处理。
3. Oracle AnyGrasp 已明确标记为 `physical_perception=false`。
4. 仿真导航节点当前是比例导航基线，不是 Nav2 实测。
5. 本轮没有开启 RGB-D，后续感知实验必须单独验证三路相机、深度类型和时间戳。
6. 本轮所有结论均来自 MuJoCo，不能表述为真实机器人抓取成功率。
7. 验收完成后用 Ctrl-C 停止 launch 时，`move_group` 在节点析构阶段出现
   `exit code -11`（segmentation fault）。此时 IK、轨迹执行和状态检查均已完成，
   MuJoCo 桥及其他节点正常退出；该退出问题单独列为后续稳定性问题，不能用它
   否定本轮 G2 的运行期结果，也不能忽略。

## G3 下一道门

G3 仍需在真实抓取候选状态下完成左臂停姿和升降联合验证。

最低要求的升降测试点：

```text
-0.15 m
 0.00 m
+0.15 m
```

验收输出必须包含：

- 右臂抓取候选或至少一个真实 IK 状态；
- 对应 `grasp_id`；
- 对应 `lift_joint`；
- 左臂 `q1..q7`；
- `valid=True`；
- `contacts=0`；
- 至少三个升降高度的验证结果；
- 写入配置后的重启复验结果。

在 G3 通过前，不启动 CARE-Agent ROS 闭环，也不开始正式论文主实验。
