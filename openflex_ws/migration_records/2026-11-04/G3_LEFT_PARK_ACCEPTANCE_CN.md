# G3 左臂停放姿态验收记录

日期：2026-11-04  
证据类型：MuJoCo 数字场景、Oracle AnyGrasp、MoveIt 碰撞查询

> **已作废（2026-11-05）**：本记录的碰撞场景有两处建模错误 —— `map -> base_link`
> 漏掉 0.17 m 底座高度、且没有把地面平面纳入碰撞集 —— 因此下面这个停姿
> `[0.2, -0.15, 0, 0, 0, 0, 0]` 在低升降处会把左臂压进地面，导致 G5 运输失败。
> 请使用 [G3 停姿复搜验收（2026-11-05）](../2026-11-05/G3_LEFT_PARK_ACCEPTANCE_CN.md)，
> 新停姿为 `[-0.7, -1.2, 0.4, 1.2, 0, 0, 0]`。本文件仅作为过程记录保留。

## 结论

G3 通过真实 TC-WAM 右臂 IK 候选约束下的碰撞筛选。

最终左臂停姿：

```text
[0.2, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
```

该姿态在 10 个 IK 可行的真实右臂候选上通过，测试升降集合同时包括：

```text
-0.15 m
 0.00 m
+0.15 m
```

以及每个候选自身的真实 lift 解，覆盖约：

```text
-0.450 m
-0.432 m
-0.429 m
-0.428 m
-0.426 m
```

机器可读结果：

[G3_LEFT_PARK_SEARCH.json](G3_LEFT_PARK_SEARCH.json)

## 为什么修正过两次

第一版只使用 collision-free 候选，并只测试三个固定升降高度，得到：

```text
[0.0, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
```

该结果在 G4 mission 中失败。诊断发现：

1. TC-WAM 候选自身升降约 `-0.43 m`，第一版没有加入测试；
2. 在该真实升降下，左臂会与前左轮碰撞；
3. 仅使用 `collision_free=true` 的旧运行候选会掩盖当前左臂姿态导致的碰撞。

修正后的搜索器：

- 接受所有 `evaluated=true` 且 `ik_valid=true` 的右臂候选；
- 在左臂搜索阶段重新判断碰撞；
- 自动加入每个候选自身的 lift 解；
- 同时保留 `-0.15/0/+0.15 m` 三个验收高度。

## 搜索规模

```text
TC-WAM 候选：225
右臂 IK 可行候选：10
左臂枚举姿态：1680
实际测试升降高度：9
全测试集可行姿态：560
最终最小归一化关节余量：0.936306
```

左臂 q1..q4 枚举：

```text
q1: [-1.5, -1.0, -0.7, -0.4, -0.2, 0.0, 0.2, 0.5]
q2: [-1.2, -0.8, -0.5, -0.3, -0.15, 0.0]
q3: [-0.8, -0.4, 0.0, 0.4, 0.8]
q4: [1.8, 1.2, 0.6, 0.0, -0.6, -1.2, -1.8]
q5=q6=q7=0
```

每个候选测试前，把 `sim_shelf_scene.yaml` 的货架碰撞体从 `map` 变换到候选
`base_link`，再调用 MoveIt `/check_state_validity`。

## 配置与运行复验

停姿已写入：

```text
openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml
openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py
```

重建并重启后，对 G4 mission 的实际运行验证：

```text
left_arm_positions=[
  0.216418,
  -0.158572,
  0.000857,
  -0.000899,
  0.000675,
  0.006410,
  0.013065
]
group=right_arm_with_lift valid=True contacts=0
```

该停姿成功让 G4 mission 通过 TC-WAM、底盘对位和 MoveIt 抓取阶段。

## 证据边界

1. Oracle 候选的 `physical_evaluation=false`，不是实拍视觉结果。
2. 候选自身 lift 高度和三个要求高度使用同一真实右臂关节解进行碰撞筛选，
   不代表所有 lift 都重新求解了相同 TCP 抓取位姿。
3. 货架碰撞体来自 MuJoCo 数字场景，不是实拍标定货架。
4. 左臂停姿只验证静态碰撞，不代表运输轨迹安全。
5. 关闭仿真时 `move_group` 仍在析构阶段出现段错误，与运行期验收分开处理。

## 与 G4 的关系

最终停姿已使 G4 抓取完成，独立状态为：

```text
contact_detected=true
gripper_closed=true
object_retained=true
transport_safe=true
verified=true
failure_code=0
```

随后运输姿态轨迹因附着物体与 `openarmx_right_link7` 被 MoveIt 判碰撞而失败，
该问题属于 G5，不影响 G4 的抓取和附着验收。
