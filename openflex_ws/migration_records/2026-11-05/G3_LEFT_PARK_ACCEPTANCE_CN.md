# G3 左臂停放姿态验收记录（2026-11-05 复搜）

日期：2026-11-05
证据类型：MuJoCo 数字场景、Oracle AnyGrasp、MoveIt 碰撞查询
结论：**通过**（在修正后的碰撞场景下重新搜索）

## 1. 为什么重搜

2026-11-04 的 G3 结果 `[0.2, -0.15, 0, 0, 0, 0, 0]` 有两个建模缺陷，
导致它通过了不真实的校验：

1. **底座高度漏项**：`map` 帧里的候选底盘位姿是地面平面位姿，而 `base_link`
   在其上方 0.17 m。旧搜索器（和旧版 `moveit_evaluator`）没有加回这 0.17 m，
   于是货架碰撞体在 `base_link` 里被放高了 0.17 m —— 恰好避开了右臂/左臂
   真实的碰撞位置。
2. **没有地面**：`sim_shelf_scene.yaml` 只有货架四件套，搜索器看不到左臂
   压到地面。

第 2 点直接造成了 G5 的阻塞：该停姿在 `lift_joint ≤ -0.36 m` 时把左手
指垫压进地面 40 mm 以上，地面反力（≈ 205 N + 219 N）经立柱把
`lift_joint` 顶高约 28 mm，右臂 TCP 因此每次都高出物体约 26 mm 而被夹爪门禁
拒绝。详见 [G5 根因记录](G5_LEFT_PARK_FLOOR_CONTACT_CN.md)。

## 2. 搜索器修正

`scripts/search_moveit_left_park.py`：

- 新增 `--base-mount-height-m`（默认 0.17），在把 `map` 障碍物变换到候选
  `base_link` 时补偿底座高度；
- 关节限位改用实机/URDF 真实值
  `q1[-3.3444, 0.9056] q2[-3.2708, 0.1292] q3[-1.57,1.57] q4[0,1.8]
   q5[-1.5,1.5] q6[-0.75,0.75] q7[-1.4,1.4]`
  （旧值比实机宽，会把踩在限位上的姿态算成“高余量”）；
- 默认把地面平面 `mujoco_floor`（`map` 系，顶面 z = 0）加入碰撞集，
  可用 `--no-floor` 关闭以复现旧行为；
- 升降验收集合改为 TC-WAM 的升降种子加三个验收高度
  `{-0.45, -0.15, 0.00, +0.15}`，并与每个候选自身的真实 lift 解取并集。

运行：

```bash
python3 scripts/search_moveit_left_park.py --output <json>
```

## 3. 搜索结果

```text
TC-WAM 候选：5（moveit_anygrasp_lift_arm_ik_validity_v3）
左臂枚举姿态：1680
实际测试升降高度：{-0.450, -0.150, 0.000, +0.150}
全测试集可行姿态：577
最终停姿：[-0.7, -1.2, 0.4, 1.2, 0.0, 0.0, 0.0]
最小归一化关节余量：0.6667
```

地面约束确实生效：判定日志里
`openarmx_left_left_finger <-> mujoco_floor` 出现 120 次、
`openarmx_left_right_finger <-> mujoco_floor` 出现 113 次，
即有一大批姿态被地面否决。

机器可读结果：[G3_LEFT_PARK_SEARCH.json](G3_LEFT_PARK_SEARCH.json)

## 4. 独立离线复核（ROS-free，约 5 秒）

```bash
OPENFLEX_MUJOCO_MODEL=/home/wgb/openflex_shop/openflex_mujoco-1.0_basic/openflex_mujoco.xml \
  python3 scripts/probe_left_park_contact.py
```

```text
park pose    : [-0.7, -1.2, 0.4, 1.2, 0.0, 0.0, 0.0]
[A] hold_left_arm_park = true -> settled lift error = +0.01 mm   (旧停姿 +27.88 mm)
```

左臂最低点离地面（地面平面 z = 0）：

| lift | 旧停姿 | 新停姿 |
|---|---|---|
| −0.650 | −0.2478 m | **+0.3092 m** |
| −0.450 | −0.0478 m | **+0.5092 m** |
| −0.150 | +0.2522 m | **+0.8092 m** |
| +0.300 | +0.7022 m | **+1.2592 m** |

新停姿在整个升降行程内都留出 ≥ 0.31 m 的地面间隙。

## 5. 配置落点

- `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml`
- `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py`
- `install_v2` 对应文件已同步（非 editable 安装）。

## 6. 证据边界

1. Oracle 候选的 `physical_evaluation=false`，不是实拍视觉结果。
2. 货架与地面碰撞体来自 MuJoCo 数字场景，不是实拍标定。
3. 左臂停姿只验证静态碰撞，不代表运输轨迹安全。
4. 顺带发现：在修正后的场景里，`lift_joint = -0.600 m` 时**右臂**会与
   `mujoco_shelf_board_target_level` 碰撞，与左臂停姿无关。即候选的升降
   解不能低于约 −0.55 m；该约束目前只体现在搜索器里，尚未进入
   `moveit_evaluator` 的候选评分。
