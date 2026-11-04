# G3 左臂停放姿态验收记录

日期：2026-11-04  
基线代码提交：`754ed09450137c8e1d573ef75d7c0e3460d5cad1`  
证据类型：MuJoCo 数字场景、Oracle AnyGrasp、MoveIt 碰撞查询

## 结论

G3 通过“真实 TC-WAM 候选约束下的碰撞筛选”验收。

选定左臂停姿：

```text
[0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
```

该姿态在 6 个有效的真实右臂候选和以下三个升降高度上均通过
`/check_state_validity`：

```text
-0.15 m
 0.00 m
+0.15 m
```

结果同时验证了机器人自身碰撞和仿真货架碰撞。机器可读结果：

[G3_LEFT_PARK_SEARCH.json](G3_LEFT_PARK_SEARCH.json)

## 输入候选

TC-WAM 共输出 135 个候选，其中 6 个满足：

- 主动臂为右臂；
- `evaluated=true`；
- `ik_valid=true`；
- `collision_free=true`；
- 包含完整的 `lift_joint` 和右臂七关节解。

| Candidate | Grasp | 候选升降 m | 分数 |
|---|---:|---:|---:|
| 6 | 100102 | -0.433062 | 0.790723 |
| 9 | 100101 | -0.450000 | 0.759704 |
| 0 | 100101 | -0.434509 | 0.754973 |
| 12 | 100100 | -0.430922 | 0.750704 |
| 15 | 100102 | -0.450000 | 0.750704 |
| 3 | 100100 | -0.432835 | 0.745973 |

候选来源：

```text
moveit_anygrasp_lift_arm_ik_validity_v3
physical_evaluation=false
```

因此该结果只能作为数字场景证据。

## 搜索方法

搜索固定来自候选的真实右臂关节解，将升降分别测试为：

```text
-0.15 m
 0.00 m
+0.15 m
```

左臂枚举：

```text
q1: [-1.5, -1.0, -0.7, -0.4, -0.2, 0.0, 0.2, 0.5]
q2: [-1.2, -0.8, -0.5, -0.3, -0.15, 0.0]
q3: [-0.8, -0.4, 0.0, 0.4, 0.8]
q4: [1.8, 1.2, 0.6, 0.0, -0.6, -1.2, -1.8]
q5=q6=q7=0
```

共搜索 1680 个左臂姿态。每个候选测试前，使用候选底盘位姿将
`sim_shelf_scene.yaml` 中的货架碰撞体转换到 `base_link`，然后调用 MoveIt
`/check_state_validity`。

结果：

```text
searched_left_poses=1680
feasible_left_poses=718
selected_left_joint_margin=0.942308
search_elapsed_sec=7.943
```

## 搜索器修正

原 `scripts/search_moveit_left_park.py` 使用任意参考右臂 IK，只测试一个升降
高度，无法作为 G3 验收。现已改为：

- 订阅 `/thesis/tc_wam/candidates` 的真实候选；
- 只使用已经通过 IK 与碰撞检查的右臂解；
- 使用 transient-local QoS 读取候选；
- 将货架碰撞体绑定到每个候选底盘位姿；
- 在三个升降高度上进行联合碰撞筛选；
- 输出可复算 JSON 和最优关节余量。

## 配置写入与重启复验

停姿已写入：

```text
openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml
openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py
```

重建 `openflex_thesis_mujoco` 后重启仿真，实际 `/joint_states`：

```text
[0.001519, -0.129203, 0.000268, -0.000187, 0.000297, -0.001735, 0.002317]
```

配置与实际 q2 相差约 `0.020797 rad`，属于初始刚体稳定后的小幅偏差。

重启后的整机状态检查：

```text
group=right_arm_with_lift valid=True contacts=0
```

## 证据边界

1. 本次使用的是 Oracle 抓取候选，`physical_evaluation=false`。
2. `-0.15/0/+0.15 m` 是固定真实右臂解后的升降碰撞扰动，不代表同一个 TCP
   抓取位姿在三个升降高度都有独立 IK 解。
3. 货架场景是 MuJoCo 标定的数字货架，不是实拍标定货架。
4. 本次只证明静态停姿在测试候选和升降范围内无碰撞，尚未证明执行轨迹、
   抓取附着、运输和放置安全。
5. 关闭仿真时 `move_group` 仍出现清理阶段段错误，与本次运行期验收分开处理。

## 下一道门

G3 已完成，下一道门为 G4：

```text
固定底盘
  -> 规划靠近目标
  -> 闭合夹爪
  -> 独立 GraspStatus 验证
  -> contact_detected / gripper_closed / object_retained 同时成立
```

G4 必须使用新的左臂停姿重新执行，不得引用旧的 `hands_up` 结果。
