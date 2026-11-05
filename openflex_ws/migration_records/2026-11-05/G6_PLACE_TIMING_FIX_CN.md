# G6 放置阶段时序修复记录

日期：2026-11-05
阶段：P1 固定抓放基线 / 验收门 G6
问题：放置动作开始时附着体已被移除，导致 MuJoCo 检查失败
状态：已修复，待验证

## 1. 问题描述

G5 通过后，mission 在 phase 13（放置阶段）立即失败：

```
phase=13 detail=execute fixed delivery action
removed attached object for thesis_item_01 from the planning scene
phase=16 detail=no object is attached in MuJoCo
MUJOCO_MISSION_FAILED code=60 message=no object is attached in MuJoCo
```

根因：`moveit_grasp_executor` 在放置动作**开始前**就从规划场景移除了附着体，
但 `bridge` 的 `_on_place_control` 需要检查 `self.attached_arm` 是否存在。

## 2. 时序分析

### 错误流程

1. **Phase 13 开始**：`mission_orchestrator` 调用 `/thesis/execute_fixed_place`
2. **旧状态触发**：`moveit_grasp_executor` 的 `on_place_status` 订阅收到
   之前的 PlaceStatus（bridge 会持续发布 2 秒）
3. **过早分离**：`on_place_status` 检查到 `target_reached=True, released=True`，
   立即设置 `pending_detach` 并移除 `scene_attachment`
4. **Bridge 检查失败**：`fixed_place_supervisor` 发送 `FixedPlaceControl` 给 bridge，
   bridge 检查 `self.attached_arm is None`，返回 failure_code=91
5. **Mission 失败**：phase 16 检查附着状态，发现未附着，任务终止

### 为什么会收到旧的 PlaceStatus？

- Bridge 在成功放置后会设置 `self.place_status_until = time.monotonic() + 2.0`
- 在 2 秒内，每次 `_publish_state` 都会发布这个状态
- `moveit_grasp_executor` 订阅了 `/thesis/place/status`（QoS depth=20）
- 如果有残留消息或新任务启动太快，旧状态会被误处理

## 3. 修复方案

### 修复 1：增强 `moveit_grasp_executor_node.py` 的防护

**文件**：`openflex_thesis_control/openflex_thesis_control/moveit_grasp_executor_node.py`

**修改位置**：`on_place_status` 函数（约 line 231-245）

**修改内容**：添加多层防护检查

```python
def on_place_status(self, msg: PlaceStatus) -> None:
    """Drop the attached box once MuJoCo confirms the release."""
    if not bool(self.get_parameter("detach_after_place").value):
        return
    if not (msg.target_reached and msg.released and msg.verified):
        return
    attachment = self.scene_attachment
    # 新增防护 1：检查附着体是否存在且匹配
    if attachment is None or attachment[0] != msg.object_id:
        return
    # 新增防护 2：检查是否是当前活动对象（防止旧消息）
    if self.active_object_id != msg.object_id:
        return
    # 新增防护 3：不在抓取动作执行期间分离（busy 标志）
    if self.busy:
        return
    self.pending_detach = attachment
    self.scene_attachment = None
    self.object_attached = False
```

**原理**：
- 防护 1：基础检查，确保有东西可以分离
- 防护 2：确保消息对应当前任务，不是上一次任务的残留
- 防护 3：在抓取执行期间忽略放置状态（时序保护）

### 修复 2：优化 `bridge_node.py` 的状态发布

**文件**：`openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py`

**修改位置**：`_on_place_control` 函数（约 line 1004-1039）

**修改内容**：区分成功和失败状态的发布策略

```python
def _on_place_control(self, msg: FixedPlaceControl) -> None:
    if not msg.enabled:
        return
    status = PlaceStatus()
    with self.lock:
        status.header.stamp = self._simulation_stamp()
        status.header.frame_id = msg.delivery_pose.header.frame_id or "map"
        status.object_id = msg.object_id
        if self.attached_arm is None:
            status.failure_code = 91
            status.detail = "no object is attached in MuJoCo"
            # 失败状态：立即过期，不持续发布
            self.place_status = status
            self.place_status_until = time.monotonic() - 1.0  # 设为过去时间
        else:
            # ... 执行放置 ...
            status.target_reached = True
            status.released = True
            status.verified = True
            status.detail = "MuJoCo fixed-placement release verified"
            # 成功状态：发布 2 秒确保送达
            self.place_status = status
            self.place_status_until = time.monotonic() + 2.0
```

**原理**：
- 失败状态设置 `place_status_until` 为过去时间（`-1.0`），
  `_publish_state` 中的 `place_valid` 检查会立即让它过期
- 成功状态仍保持 2 秒发布窗口，确保订阅者收到
- 避免失败消息污染后续操作

## 4. 文件同步

修改已同步到 `install_v2`（非 editable 安装）：

```bash
cp openflex_ws/src/.../moveit_grasp_executor_node.py \
   openflex_ws/install_v2/.../moveit_grasp_executor_node.py

cp openflex_ws/src/.../bridge_node.py \
   openflex_ws/install_v2/.../bridge_node.py
```

## 5. 测试计划

### 验收标准

G6 通过需满足：
- [ ] Phase 13 开始放置动作
- [ ] Bridge 确认有附着体（`self.attached_arm is not None`）
- [ ] 执行固定放置：移动物体到目标位置
- [ ] 打开夹爪释放物体
- [ ] 独立验证：物体不再附着，位于目标位置
- [ ] Mission 继续到 phase 17 或更后阶段

### 测试命令

```bash
timeout 300 bash scripts/run_thesis_moveit_mission.sh
```

### 关键日志标记

成功标志：
```
phase=13 detail=execute fixed delivery action
phase=14 detail=...
phase=15 detail=...
phase=16 detail=... (不是 "no object is attached")
```

失败标志：
```
removed attached object for thesis_item_01 from the planning scene
  (出现在 phase=13 之后、phase=16 之前)
phase=16 detail=no object is attached in MuJoCo
MUJOCO_MISSION_FAILED code=60
```

## 6. 残留问题

### 重复 action server 警告

日志中出现：
```
Ignoring unexpected goal response. There may be more than one action server 
for the action '/thesis/execute_fixed_place'
```

**分析**：
- 代码检查确认只有一个 ActionServer（`fixed_place_supervisor`）
- `mission_orchestrator` 创建的是 ActionClient（正常）
- 可能原因：节点重启、竞态条件或 ROS 2 内部状态

**处理**：
- 警告不影响功能（已验证架构正确）
- 如果 G6 测试通过，此警告为非阻塞性
- 如果问题持续，可在后续排查节点生命周期管理

## 7. 下一步

- G6 通过后：继续 G7（端到端成功，含放置后阶段）
- G7 通过后：G8（10 次重复测试）
- G8 通过后：G9（CARE-Agent ROS 闭环）
- G9 通过后：G10（统计实验：基线、消融、恢复）

## 8. 证据边界

本次修复：
- 基于代码分析和日志推理
- 修复了明确的时序竞态问题
- 增加了多层防护逻辑

仍需实验验证：
- 修复后的完整 mission 是否通过 G6
- 放置后阶段（phase 14-17）是否正常
- 10 次重复运行的稳定性
