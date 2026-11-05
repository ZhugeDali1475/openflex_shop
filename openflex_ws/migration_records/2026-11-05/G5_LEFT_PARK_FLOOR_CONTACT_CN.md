# G5 阻塞根因：左臂停姿压入地面，把升降机构顶高 ~28 mm

日期：2026-11-05
阶段：P1 抓放基线 / 验收门 G5（运输姿态与附着保持）— 本文记录**定位当时的未通过状态**，修复与复验见 §0
本轮结论：G5 当时的阻塞已定位到物理层面，且可离线复现；不是 MoveIt 配置、不是 TF、不是夹爪门禁调参问题。

## 0. 修复状态（2026-11-05 已完成）

- 已按第 6 节方案 A 修复：左臂停姿改为 `[-0.7, -1.2, 0.4, 1.2, 0.0, 0.0, 0.0]`，
  搜索器补上 0.17 m 底座高度、真实关节限位与地面平面后重跑 G3。
  重跑记录见 [G3 停姿复搜](G3_LEFT_PARK_ACCEPTANCE_CN.md)。
- 端到端复验通过，G5 验收记录见
  [G5 运输姿态与附着保持验收](G5_TRANSPORT_ACCEPTANCE_CN.md)。
- 效果：`lift_joint` 稳态偏差 +27.88 mm → **+0.01 mm**；
  物体在 TCP 系 (−0.0014, 0.0006, 0.0028)，运输姿态执行成功。
- 第 5 节末尾的“候选选择为何变化”已在 G3 复搜记录中解释：
  旧搜索器/旧 evaluator 把货架放高了 0.17 m，候选评分因此不同。
- 第 7 节的问题 1 已完成；问题 2 现在转移到 G6（放置阶段失败，见 G5 验收记录第 5 节）。

## 1. 一句话结论

`hold_left_arm_park` 把左臂固定在 `[0.2, -0.15, 0, 0, 0, 0, 0]`，该姿态在
`lift_joint ≈ -0.445 m` 时让左手两片指垫落到地面平面以下（最深 −42.8 mm）。
桥每步把 `qpos` 写回该姿态，相当于用运动学约束硬顶地面，地面反力经立柱传到
**没有被约束的 `lift_joint`**，把升降机构顶高约 **28 mm**。右臂 TCP 因此比物体
高约 26 mm，MuJoCo 的夹爪门禁按设计拒绝（`|x|=0.0264 > 0.020`），抓取失败。

## 2. 复现（ROS-free，约 5 秒）

```bash
cd /home/wgb/openflex_shop
OPENFLEX_MUJOCO_MODEL=/home/wgb/openflex_shop/openflex_mujoco-1.0_basic/openflex_mujoco.xml \
  python3 scripts/probe_left_park_contact.py
```

实测输出（本轮，未改动）：

```text
[A] hold_left_arm_park = true (what the bridge does)
    settled lift = -0.417071  error = +27.88 mm
    floor contact openarmx_left_right_finger  dist=-0.01398 m  |F|= 219.31 N
    floor contact openarmx_left_left_finger   dist=-0.01488 m  |F|= 205.14 N
[B] control: left finger collision geoms disabled
    settled lift = -0.444946  error = +0.01 mm
[C] deepest left-arm mesh vertex vs lift command (floor plane is z = 0)
    lift=-0.650  deepest z=-0.24782 m
    lift=-0.445  deepest z=-0.04282 m
```

对照 A/B 是决定性的：同一个升降指令、同一个臂位姿、同一套重力前馈，
**只关掉左手指垫的碰撞几何，升降误差从 +27.88 mm 掉到 +0.01 mm**。

## 3. 运行时证据（隔离测试外的端到端运行）

运行：`timeout 300 bash scripts/run_thesis_moveit_mission.sh`，日志 `/tmp/g5n.raw.log`
（`install_v2` 与 `src` 已核对一致）。

任务确实走到了抓取阶段（因此不是导航/启动竞态）：

```text
phase=9 detail=validate refined grasps at current base, then jointly execute lift+arm
pregrasp result success=True: {'lift_joint': -0.418624, ...}
grasp  result success=True: {'lift_joint': -0.421610, ...}
Right gripper close action failed
MUJOCO_MISSION_FAILED code=53 message=failed to close gripper
```

注意 `pregrasp target lift_joint = -0.444954`，两次回报的实测升降都在 `-0.419 ~ -0.422`，
即**同向、同量级的 +23 ~ +26 mm 偏差**，与离线预测的 +27.9 mm 相符。

桥的接近日志（成对接触版）显示物体在 TCP 系中的位置：

```text
grasp proximity on right: object 0.026 m from the TCP
  (object in TCP frame (0.0264, -0.0007, -0.0002), gripper command 0.0000, attached=None)
  contacts=openarmx_left_left_finger--world;openarmx_left_right_finger--world;
           openarmx_right_left_finger--thesis_object;openarmx_right_right_finger--thesis_object;
           thesis_object--thesis_shelf;thesis_occluder--world
grasp candidate on right rejected: object laterally off the jaw axis (|x|=0.0264)
```

两条独立证据同时指向同一个数：接触清单里 `openarmx_left_*_finger--world` 是唯一
与地面接触的物体，而右臂 TCP 恰好多出 ~26 mm。

门禁判据（`bridge_node.py` `attach_rejection`）为
`z ∈ [-0.069, 0.016]`、`|y| ≤ 0.04275`、`|x| ≤ grasp_attach_lateral_tolerance_m = 0.020`。
实测只有 `|x|` 越界，`z`、`|y|` 都在范围内——即“深度和夹持宽度都对，只是整体高了一截”。

## 4. 已排除的假设

- **TF / 底座高度**：已修好且本轮实测正确。桥把 0.17 m 放在 `map -> odom`，`odom -> base_link` 的 z 为 0；
  运行时 `G5DBG pose_to_base frame=map->base_link in=(1.0000, -0.0000, 0.8439)
  T=(-0.3305, -0.4765, -0.1700)` ⇒ base_link 系物体 z = 0.6739。语义与 SRDF 的
  `base_planar_joint`（planar，无 z）一致。
- **`moveit_evaluator_node.py` 的 `base_mount_height_m=0.17` 与 TF 修复冲突**：不冲突。
  它只影响 evaluator 自己把 map 位姿换算进候选底盘 base_link，必须与 TF 采用同一 0.17，
  否则 IK 目标整体错 0.17。
- **夹爪门禁“太严”**：不是。`|x|=0.0264` 超过 0.020 是上述 26 mm 抬高的直接后果，
  放宽到 0.03 只是把缺陷藏进验收结论里。
- **导航/启动竞态**：本轮与 g5j/g5l/g5m 都推进到了抓取阶段，失败点在抓取几何。
- **重力前馈无效**：前馈本身有效。自由空间对照里，关掉 `gravity_compensation` 时
  `right_joint1` 静态误差 −80 mrad，开启后 < 0.5 mrad；升降残余的 8.38 mm 恰好等于
  `qfrc_constraint[lift] = 83.80 N ÷ kp 10000`，即地面接触反力，不是前馈不够。

## 5. 影响与边界

- 这个缺陷会让**任何把 `lift_joint` 用到 −0.25 以下的抓取候选**都抓不住；
  G3 的停姿搜索用货架碰撞体做了校验，但没有把地面平面纳入碰撞集，所以漏掉了它。
- G4 验收记录（park 姿态、右臂 IK 可行域）在**高升降**候选下依然成立，
  但在低升降候选项上不能外推。
- 本轮没有改动任何仓库源码；新增 `scripts/probe_left_park_contact.py` 作为可复现诊断。
- 未解决：g5j 选中的候选 `lift=-0.0727, right_joint1=-1.249`，g5n 选中的是
  `lift=-0.4450, right_joint1=0.021`，而两者物体位姿、底盘位姿完全相同。
  候选选择为何变化（评分对货架碰撞体 z 的敏感度，还是搜索的非确定性）尚未查清。

## 6. 修复选项

- **A（推荐，物理正确）**：把停姿从“垂在身侧”改成“收在身侧上方”，并让
  `scripts/search_moveit_left_park.py` 把地面平面加入碰撞集，在 `lift ∈ [-0.65, 0.30]`
  全程复查后重跑 G3。离线检查两个候选收臂姿态在最低升降处仍在 +0.219 / +0.284 m：

  ```text
  park=[0.0, -0.9,  0.6, -1.2, 0.0, 0.4, 0.0]  deepest z at lift=-0.65 = +0.21919 m
  park=[0.2, -1.2,  0.9, -1.4, 0.0, 0.5, 0.0]  deepest z at lift=-0.65 = +0.28405 m
  ```

- **B（配合 A）**：候选评分阶段把“左臂停姿 × 该候选 lift”的地面间隙作为硬约束，
  让不可行升降解直接落选，而不是等到运行时被物理拖偏。
- **C（不推荐）**：关掉停用臂指垫的碰撞几何。数值上能立刻拿到 +0.01 mm，
  但那等于人为删除一次真实碰撞，必须在论文里标注为模型简化，不能当成修复。

## 7. 修复后仍要回答的问题

1. 停姿改动需要重跑 G3 并更新 `G3_LEFT_PARK_ACCEPTANCE_CN.md` /
   `G3_LEFT_PARK_SEARCH.json` 与 `mujoco.yaml`。
2. 抓取成功后才会重新走到 G5 的真正验收项（运输姿态下仍保持附着）。
3. g5j 曾出现一次“物体被推进掌心、门禁通过”的假成功
   （`object in TCP frame (0.0045, 0.0012, 0.0018)`），修好抬高问题后要复查这类解是否还会出现。
4. `oracle_anygrasp_node.py:106` 的 `grasp.width = 0.055` 与实测开口不符，仍未被任何代码校验。
