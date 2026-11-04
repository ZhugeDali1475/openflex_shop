# G4 固定底盘抓取验收记录

日期：2026-11-04  
平台：MuJoCo 3.4.0、ROS 2 Humble、MoveIt  
证据类型：Oracle 感知、数字确定附着，不是真实物理抓取

## 结论

G4 抓取和独立附着验证通过；G5 运输姿态未通过。

最终 mission 在抓取后得到：

```text
contact_detected=true
gripper_closed=true
object_retained=true
transport_safe=true
verified=true
failure_code=0
detail=MuJoCo deterministic attachment on right TCP
```

## 固定条件

```text
object_id: thesis_item_01
object_nominal_pose: [1.12, 0.0, 0.84, pi]
active_arm: right
perception: MuJoCo Oracle AnyGrasp
TC-WAM evaluator: MoveIt
selected grasp: 100102
left park: [0.2, -0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
base: 固定底盘
```

## 启动命令

```bash
cd /home/wgb/openflex_shop
bash scripts/run_thesis_moveit_mission.sh
```

mission 帧序列：

```text
phase=1  Nav2 to shelf observation/handoff zone
phase=2  confirm measured base stop
phase=3  wait for fresh target observation
phase=4  first AnyGrasp Top-K
phase=5  jointly score base, lift+arm IK and grasp
phase=6  ACT base alignment
phase=7  ready reached; cancel ACT and confirm base lock
phase=8  refined AnyGrasp
phase=9  validate refined grasps and jointly execute lift+arm
```

TC-WAM 选择 `grasp_id=100102`，mission 随后推进到 MoveIt 抓取。

## MoveIt 执行证据

预抓取轨迹：

```text
Controller 'lift_joint_controller' successfully finished
Controller 'right_joint_trajectory_controller' successfully finished
Completed trajectory execution with status SUCCEEDED
Solution was found and executed
```

抓取轨迹同样两次返回：

```text
Controller ... successfully finished
Completed trajectory execution with status SUCCEEDED
Solution was found and executed
```

MuJoCo bridge：

```text
simulated grasp attached thesis_item_01 to right TCP at distance 0.082 m
```

## 独立 GraspStatus

从 `/thesis/grasp_status` 实际读取：

```yaml
header:
  stamp:
    sec: 122
    nanosec: 581999999
  frame_id: map
object_id: thesis_item_01
map_version: 1
contact_detected: true
gripper_closed: true
object_retained: true
transport_safe: true
verified: true
failure_code: 0
detail: MuJoCo deterministic attachment on right TCP
```

抓取后整机状态：

```text
group=right_arm_with_lift valid=True contacts=0
```

## G5 失败与 G4 的边界

抓取成功后，执行器继续尝试运输姿态，但失败：

```text
Found a contact between 'openarmx_right_link7' and
'thesis_grasped_thesis_item_01'
Start state appears to be in collision with respect to group right_arm_with_lift
Motion planning start tree could not be initialized
```

随后 mission 返回：

```text
error_code=53
detail=transport-pose execution failed
```

这属于 G5，而不是 G4：

- G4 的夹爪闭合、接触、保持和独立验证均已成立；
- 运输姿态轨迹没有成功；
- 不能把 mission 最终失败误写成 G4 失败；
- 也不能把当前 G4 成功误写成完整抓放成功。

## 证据限制

1. 附着是 MuJoCo 的确定性数字代理，不是真实摩擦和接触力学。
2. AnyGrasp 来自 Oracle，`physical_perception=false`。
3. `transport_safe=true` 是附着验证字段，不代表运输姿态轨迹已执行成功。
4. 本轮为单次通过，尚未完成 G8 的 10 次重复。

## 下一项

G5 需要修复附着物体与 `openarmx_right_link7` 的自碰撞：

```text
attached object frame
-> right-hand TCP
-> touch links / allowed collision matrix
-> transport arm and lift target
-> independent retained verification
```

修复后必须重新完成一次 G4 抓取，再验证运输姿态和物体持续附着。
