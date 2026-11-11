# openflex_care_agent

> 当前主线为 RoboOS + RoboBrain2.5 分层任务调度。本包保留固定流程所需的状态、
> 技能契约和安全/恢复兼容逻辑，不再作为独立学习恢复研究入口。
> 项目入口见仓库根目录 `README.md`；旧训练和研究运行器已归档。

`openflex_care_agent` is the ROS-independent decision core for CARE-Agent:

```text
Capability-Aware Recovery Embodied Agent
```

The first implementation contains:

- structured task, robot and object belief state;
- typed skill contracts with physical preconditions;
- a deterministic safety critic;
- failure-to-recovery candidate routing;
- risk-aware recovery utility selection;
- nominal phase-to-skill planning;
- pytest coverage for safety and recovery behavior.

This package does not publish robot commands. ROS adapters live in
`openflex_thesis_control`; they are execution infrastructure, not the new LLM scheduler.
