# G5 运输姿态与附着保持验收记录

日期：2026-11-05
阶段：P1 固定抓放基线 / 验收门 G5
结论：**通过**（抓取 → 运输姿态 → 独立验证，附着在整个运输段保持）

## 1. 修复内容

单一改动：左臂停姿从“垂在身侧”改为“收在身侧上方”。

```text
旧: [ 0.2, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
新: [-0.7, -1.2,  0.4, 1.2, 0.0, 0.0, 0.0]
```

写入位置：

- `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/config/mujoco.yaml`
- `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py`（默认值）

同时修正了搜索器 `scripts/search_moveit_left_park.py` 的两处建模错误
（`map` → `base_link` 漏掉 0.17 m 底座高度；关节限位比实机宽），并把地面平面
纳入碰撞集。新停姿是重跑后的结果，见
[G3 停姿复搜记录](G3_LEFT_PARK_ACCEPTANCE_CN.md)。

## 2. 端到端运行证据

运行：

```bash
timeout 300 bash scripts/run_thesis_moveit_mission.sh
```

日志摘录（全文见 [G5_MISSION_TRANSCRIPT.txt](G5_MISSION_TRANSCRIPT.txt)）：

```text
phase=9  detail=validate refined grasps at current base, then jointly execute lift+arm
pregrasp result success=True: {'lift_joint': -0.436761, ...}
grasp    result success=True: {'lift_joint': -0.439823, ...}
simulated grasp attached thesis_item_01 to right TCP at distance 0.003 m
  (object in TCP frame (-0.0014, 0.0006, 0.0028))
transport target joints: {'lift_joint': -0.1, 'openarmx_right_joint1': 0.0, ...,
                          'openarmx_right_joint4': 1.8, ...}
transport result success=True: {'lift_joint': -0.105653,
                                'openarmx_right_joint1': 0.004265, ...,
                                'openarmx_right_joint7': 0.008729}
phase=10 detail=independent grasp verification passed
phase=11 detail=object attached; lift+arm in transport pose
phase=12 detail=Nav2 transports grasped object
```

关键数字：

| 指标 | 旧停姿 | 新停姿 |
|---|---|---|
| `lift_joint` 稳态偏差 | +27.88 mm | **+0.01 mm** |
| 物体在 TCP 系中的位置 | (0.0264, -0.0007, -0.0002)（被门禁拒绝） | **(-0.0014, 0.0006, 0.0028)（1.4 mm）** |
| 夹爪门禁 | `object laterally off the jaw axis (|x|=0.0264)` | 通过 |
| 夹指接触 | `openarmx_left_{left,right}_finger--world` 205 / 219 N | 无地面接触 |
| 运输姿态 | 未到达 | succeeded，关节偏差 ≤ 0.023 rad |

另外，旧症状在整个日志中出现 **0 次**：

```text
grep -c "thesis_grasped_\|start tree could not be initialized" /tmp/g5r.raw.log
0
```

## 3. 验收判据对照

| G5 判据 | 状态 | 证据 |
|---|---|---|
| 抓取后物体真实位于夹爪之间 | 通过 | 物体在 TCP 系 (−0.0014, 0.0006, 0.0028) |
| 夹爪闭合且门禁接受附着 | 通过 | `simulated grasp attached ... at distance 0.003 m` |
| 独立 GraspStatus 验证 | 通过 | `phase=10 independent grasp verification passed` |
| 运输姿态可规划且可执行 | 通过 | `transport result success=True` |
| 运输姿态下附着保持 | 通过 | `phase=11 object attached; lift+arm in transport pose` |
| 附着体不再与 `right_link7` 碰撞 | 通过 | 日志中 `openarmx_right_link7 <->` 无记录 |

## 4. 证据边界

1. 附着是 MuJoCo 的几何/焊接判定（`thesis_object_<arm>_weld`），不是真实摩擦抓取；
   论文中必须写成数字实验结论。
2. 用的是 Oracle AnyGrasp 候选，`physical_evaluation=false`，不代表实拍视觉。
3. 运输段只验证了静态运输姿态的可达与附着力门禁，没有单独量化运输途中的扰动。
4. 本轮只跑了 1 次；10 次重复属于 G8。

## 5. 下一步阻塞（属 G6，不属 G5）

mission 在 G5 之后立即进入放置阶段并失败：

```text
phase=13 detail=execute fixed delivery action
removed attached object for thesis_item_01 from the planning scene
phase=16 detail=no object is attached in MuJoCo
MUJOCO_MISSION_FAILED code=60 message=no object is attached in MuJoCo
mission_orchestrator: Ignoring unexpected goal response. There may be more than
  one action server for the action '/thesis/execute_fixed_place'
```

即 `moveit_grasp_executor` 在放置动作开始时就先从规划场景移除了附着体，
而 mission 在第 16 步仍要求 MuJoCo 里保持附着；同时
`/thesis/execute_fixed_place` 存在重复 action server。两者都属 G6 范畴，
需要单独定位。
