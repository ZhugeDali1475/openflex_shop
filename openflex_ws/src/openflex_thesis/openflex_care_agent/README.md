# openflex_care_agent

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

This package does not publish robot commands. ROS action adapters and the executable
agent node are added only after the simulation baseline is reproducible.
