# OpenFlex 商超移动取送毕业设计

当前主线：**RoboOS + RoboBrain2.5 + 现有 OpenFlex 执行技能**，在已有 MuJoCo 场景中
完成自然语言驱动的单商品取送与执行反馈闭环。实机与模型微调后置，不重做机器人或底层算法。

## 从这里开始

- [任务计划与阶段验收](LLM_HIERARCHICAL_ROBOT_TASK_PLAN_CN.md)
- [当前安全问题与下一步](SAFETY_AND_TODO_CN.md)
- [本地 GGUF 模型部署](ROBOBRAIN25_GGUF_LOCAL_CN.md)
- [RoboOS 模型客户端适配](ROBOOS_GGUF_INTEGRATION_CN.md)
- [OpenFlex 只读 MCP／ROS 接入](OPENFLEX_ROBOOS_READONLY_CN.md)
- [隔离仿真运动与单商品闭环验收](OPENFLEX_ROBOOS_MOTION_CN.md)
- [脚本分类与使用边界](scripts/README_CN.md)
- [旧路线归档与恢复说明](LEGACY_ARCHIVE_CN.md)

## 系统边界

```text
自然语言 + 图像／状态
        ↓
RoboBrain2.5 ↔ RoboOS Master／OpenFlex Worker
        ↓ MCP
OpenFlex 技能适配层
        ↓ ROS 2 Action／Service
观察、导航、TC-WAM、局部对位、MoveIt 抓取、受检放置与落稳
        ↓
现有 MuJoCo 机器人、货架与交付台
```

2026-11-10 已完成一轮真实 Master／Worker／Redis／MCP／ROS／MuJoCo 的自然语言单商品取送，
包含实际相机图像、逐步模型调用、导航、抓取、持物导航、放置、无接触撤臂与独立最终校验。
证据为 `local_dependencies/openflex_roboos/runs/roboos_delivery_20261110_201218/result.json`；
它不是任意场景成功率、可靠视觉身份识别或实机证明。完整负面矩阵、长期资源与正式对照实验仍未完成。

## 保留的工程底座

- `openflex_ws/src/openflex_thesis/openflex_roboos_bridge/`：默认只读、显式启用受控运动的 MCP／ROS 适配。
- `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/`：场景、相机、状态与仿真执行接口。
- `openflex_ws/src/openflex_thesis/openflex_tc_wam/`：候选评估与 ready 区域绑定。
- `openflex_ws/src/openflex_thesis/openflex_thesis_control/`：控制权仲裁、对位、抓取、放置和固定流程。
- `openflex_ws/src/openflex_thesis/openflex_care_agent/`：现有执行流程依赖的契约、安全与恢复兼容逻辑。
- `openflex_ws/src/openflex_thesis/openflex_thesis_experiments/`：日志、统计、物理预检和只规划诊断。
- 机器人描述、MoveIt 配置、模型、驱动、地图和比赛代码保持原位；不属于本次旧研究清理范围。

## 启动原则

默认仍按只读接入文档使用 `run_openflex_readonly_bridge.sh` 和 `run_openflex_readonly_sim.sh`。
完整模型运动入口为 `scripts/run_openflex_roboos_delivery.py --execute-isolated-simulation`，
需先准备独立运行环境、Redis 与受控技能服务，具体见运动验收文档。已有服务不要重复启动，不接实机域。

`scripts/run_thesis_moveit_mission.sh` 保留为固定流程对照入口，会发送仿真运动目标；
它不是新版大模型入口，也不是默认启动方式。相关安全检查通过前不得用它开展正式实验，
不得与 RoboOS Worker 或其他自动编排器并行运行。

Oracle 观测、脚本导航、比例式对位和几何附着代理必须按实际来源报告，不能写成真实视觉、
Nav2、已训练 ACT 或实机证据。软件测试通过不代表轨迹安全或毕业设计实验完成。
