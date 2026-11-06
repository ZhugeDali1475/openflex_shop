# G9 CARE-Agent ROS 闭环初步实现

日期：2026-11-05
阶段：P1 固定抓放基线 / 验收门 G9
状态：**部分完成**（决策核心验证通过，ROS 节点框架已实现）

## 1. 实现内容

### 1.1 决策核心验证（完成）

创建并通过了 CARE-Agent 决策核心的烟雾测试：

**测试覆盖**：
- ✅ 9个标准阶段的决策（全部通过）
- ✅ 3个终端阶段处理（全部通过）
- ✅ 失败恢复机制（通过）
- ✅ 安全检查（前置条件验证，通过）

**测试结果**：
```
✅ navigate_observe    -> navigate_to_station
✅ acquire_target      -> look_for_object
✅ evaluate_capability -> compute_capability
✅ align_to_ready      -> local_align_region
✅ grasp               -> plan_and_grasp
✅ verify_grasp        -> verify_grasp
✅ navigate_delivery   -> navigate_to_delivery
✅ place               -> place_object
✅ verify_place        -> verify_place
```

**关键验证**：
- BeliefState 聚合工作正常
- CAREAgent.decide() 根据阶段选择正确技能
- SafetyCritic 阻止不安全操作（无可见物体时拒绝抓取）
- RecoveryPolicy 在失败时提供恢复选项（GRASP_FAILED → RE_OBSERVE）

### 1.2 ROS 节点框架（已实现）

创建了 `care_agent_node.py`，包含：

**核心组件**：
- ✅ CAREAgent 决策核心集成
- ✅ 观测聚合（BeliefState 从 ROS 消息构建）
  - ObjectMemory → ObjectBelief
  - JointState, Odometry, GraspStatus → RobotContext
- ✅ Action client 基础设施（复用 mission_orchestrator 的客户端）
- ✅ Action server 接口（/thesis/run_care_mission）
- ✅ 决策循环框架

**观测聚合实现**：
```python
def aggregate_belief(self, task: TaskSpec, phase: TaskPhase) -> BeliefState:
    # 从 ROS 消息聚合到统一的 BeliefState
    # - 检查消息时效性（timeout）
    # - 计算 base_stopped（速度阈值）
    # - 构建 ObjectBelief 和 RobotContext
    return BeliefState(...)
```

**决策循环框架**：
```python
async def execute_mission(self, goal_handle):
    while phase not in terminal_phases:
        belief = self.aggregate_belief(task, phase)
        decision = self.agent.decide(belief, failure)
        
        if decision.fallback:
            return abort
        
        success, next_phase, error_code = await self.execute_skill(
            decision.skill, goal, belief
        )
        
        phase = next_phase if success else handle_failure
```

## 2. 未完成部分

### 2.1 技能执行器（Stubs）

当前 `execute_skill()` 是存根实现（stub），每个技能只是：
- 打印日志
- 等待 0.5 秒
- 返回成功和下一阶段

**需要完成**：
- 调用实际的 ROS action clients
- 等待 action 完成
- 解析结果并映射到 SkillOutcome
- 处理超时和取消

### 2.2 失败映射

当前没有实现从 ROS action 错误码到 FailureCode 的映射。

**需要完成**：
- 创建 `failure_mapper` 模块
- 为每个 action 定义错误码映射规则
- 例如：
  - Nav2 path_not_found → FailureCode.NAVIGATION_FAILED
  - MoveIt IK failure → FailureCode.IK_UNREACHABLE
  - Grasp not attached → FailureCode.GRASP_NOT_ATTACHED

### 2.3 能力估计

RecoveryPolicy 需要 CapabilityEstimate 来排序恢复选项，但当前未实现。

**需要完成**：
- 实现 `capability_estimator` 模块
- 基于当前状态估计技能成功概率
- 用于恢复决策排序

### 2.4 状态发布

当前节点不发布任务状态或决策信息。

**需要完成**：
- 发布当前 phase 和 decision
- 发布 BeliefState 摘要（用于调试/可视化）
- 发布失败历史

## 3. G9 验收判据

**完整 G9 要求**：
1. CARE-Agent 能够从 ROS 观测聚合 BeliefState
2. 决策核心根据 BeliefState 选择技能
3. 技能执行器调用实际 ROS actions
4. 失败自动映射并触发恢复
5. 端到端完整 mission 在 CARE-Agent 控制下成功

**当前完成度**：
- ✅ 决策核心工作正常（#1, #2）
- ✅ 观测聚合框架已实现（#1）
- ⏳ 技能执行为 stub（#3 未完成）
- ⏳ 失败映射未实现（#4 未完成）
- ⏳ 端到端测试未做（#5 未完成）

**判定**：**G9 部分完成**（约 40%）

## 4. 下一步实施计划

### 短期（完成 G9）

**优先级 1：实现一个端到端技能**
- 选择一个简单技能（如 NAVIGATE_TO_STATION）
- 实现完整的 action 调用和结果解析
- 验证从决策 → 执行 → 反馈的闭环

**优先级 2：实现失败映射**
- 为关键 actions 定义错误码映射
- 测试失败恢复流程

**优先级 3：实现所有技能执行器**
- 复用 mission_orchestrator 的 action 调用模式
- 为每个 SkillName 实现对应的 ROS action 调用

**优先级 4：端到端测试**
- 在 MuJoCo 环境中运行 CARE-Agent 节点
- 验证完整 pick-and-place mission
- 对比 mission_orchestrator 的性能

### 长期（G10）

完成 G9 后进入统计实验阶段。

## 5. 技术设计

### 5.1 技能执行器模式

```python
async def execute_skill(self, skill: SkillName, goal, belief: BeliefState):
    if skill == SkillName.NAVIGATE_TO_STATION:
        # 构建 Nav2 goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.pose.position.x = 1.12  # shelf position
        # ... set other fields
        
        # 发送 goal
        future = self.nav_client.send_goal_async(nav_goal)
        goal_handle = await future
        
        if not goal_handle.accepted:
            return (False, belief.phase, ERROR_CODE_REJECTED)
        
        # 等待完成
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return (True, TaskPhase.ACQUIRE_TARGET, 0)
        else:
            failure_code = self.map_nav_failure(result)
            return (False, belief.phase, failure_code)
```

### 5.2 失败映射模式

```python
def map_nav_failure(self, result) -> int:
    """Map Nav2 error to FailureCode."""
    error_map = {
        # Nav2 specific error codes
        1: FailureCode.NAVIGATION_FAILED,
        2: FailureCode.TIMEOUT,
        # ... more mappings
    }
    return error_map.get(result.error_code, FailureCode.INTERNAL_ERROR)
```

## 6. 证据文件

- `openflex_care_agent/care_agent_node.py` - ROS 节点实现
- `openflex_care_agent/test/test_care_agent_smoke.py` - 决策核心测试
- 测试输出：9/9 阶段决策通过，4/4 测试套件通过

## 7. 与现有系统的对比

| 方面 | mission_orchestrator | care_agent_node |
|------|---------------------|-----------------|
| 决策逻辑 | 硬编码的阶段序列 | CARE-Agent 动态决策 |
| 失败处理 | 立即终止 mission | 恢复策略排序和重试 |
| 安全检查 | 隐式（action 内部） | 显式（SafetyCritic） |
| 状态表示 | 分散的 ROS 消息 | 统一的 BeliefState |
| 可扩展性 | 需修改编排代码 | 添加技能和合约 |
| 可解释性 | 低（黑盒） | 高（decision.reason） |

## 8. 论文使用边界

**当前可以写**：
- ✅ CARE-Agent 决策架构设计
- ✅ BeliefState 和 SkillContract 形式化
- ✅ SafetyCritic 和 RecoveryPolicy 算法
- ✅ 决策核心的单元测试结果

**当前不能写**：
- ❌ "CARE-Agent 已在 ROS 中完整闭环"（技能执行未完成）
- ❌ "端到端性能对比"（未测试）
- ❌ "失败恢复在真实场景中的表现"（未实现）

**准确表述**：
> "我们实现了 CARE-Agent 的决策核心，包括 BeliefState 聚合、SafetyCritic
> 前置条件检查和 RecoveryPolicy 失败恢复排序。决策核心通过了 9 个标准阶段
> 和 4 个边界条件的单元测试。ROS 集成框架已实现，包括观测聚合和决策循环，
> 但技能执行器和失败映射尚未完成。"

## 9. 限制与已知问题

1. **技能执行为 stub**：无法实际控制机器人
2. **失败映射缺失**：无法从 ROS 错误恢复
3. **能力估计未实现**：恢复排序使用默认效用
4. **状态发布缺失**：难以调试和可视化

## 10. 总结

G9 的核心贡献——CARE-Agent 决策架构——已实现并验证。ROS 集成框架已搭建，
但从"原型"到"可运行系统"还需要完成技能执行器。

**优势**：
- 决策逻辑清晰可测试
- 架构设计完整
- 与现有系统解耦良好

**下一步**：
- 实现一个完整技能的端到端执行
- 验证 CARE-Agent 在真实环境的决策质量
