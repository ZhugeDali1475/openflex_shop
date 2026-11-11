# OpenFlex 执行底座与 RoboOS 接入

当前毕业设计是 **RoboOS + RoboBrain2.5 分层任务规划与技能调度**，总入口见仓库根目录
`README.md` 和 `LLM_HIERARCHICAL_ROBOT_TASK_PLAN_CN.md`。本目录不是待删除的整套旧工程：
新版仍需要这里的仿真、消息、候选评估、抓取、放置和安全控制。

## 包的职责

| 包 | 当前用途 |
|---|---|
| `openflex_roboos_bridge` | 当前 MCP／ROS 只读状态与观察工具；运动工具尚未完成 |
| `openflex_thesis_mujoco` | 场景、RGB-D、状态、控制器接口及仿真基线 |
| `openflex_thesis_msgs` | 现有技能 Action／Service／消息契约 |
| `openflex_tc_wam` | 抓姿候选、联合可达性与 ready 区域 |
| `openflex_thesis_control` | 控制权仲裁、局部对位、MoveIt 抓取、固定放置及固定流程对照 |
| `openflex_care_agent` | 执行流程依赖的技能契约、安全与恢复兼容代码 |
| `openflex_thesis_experiments` | 日志、统计、安全预检与只规划诊断 |
| `openflex_thesis_bringup` | 已有执行／评估服务及兼容 launch，不是新版 Worker 启动器 |

## 当前接入边界

模型推理、RoboOS 模型客户端和 `get_robot_state`／`observe_scene` 已有验证。
完整 Master／Worker、导航／抓取／放置 MCP 技能和自然语言取送尚未完成。
先按根目录 `OPENFLEX_ROBOOS_READONLY_CN.md` 使用只读入口，不启动旧 mission 编排器。

`mission_orchestrator` 和 `thesis_mujoco.launch.py` 保留为执行参考与固定流程对照。
只设置 `run_mission:=false` 不会移除旧编排器；新版完整系统仍须单独设计启动入口，
不得让两个调度器竞争。`run_thesis_moveit_mission.sh` 会发送仿真运动，不是只读入口。

## 命名与证据边界

- AnyGrasp 命名是兼容接口；当前 MuJoCo Oracle 不是物理感知，SDK 不因保留接口而自动存在。
- ACT 消息和对位监督器不代表已完成模型训练；仿真比例策略须按真实实现报告。
- 仿真脚本导航不能写成真实 Nav2 效果；几何附着代理不能替代实机夹持证据。
- `openflex_thesis_bringup/docs/INTERFACE_CONTRACTS.md` 保留为现有接口参考，不是新版验收清单。
- 物理预检与失败关闭门控保留；配置中旧实验／硬件要求不作为当前仿真方案的前置工作。

旧研究入口和材料已移出项目，清单见根目录 `LEGACY_ARCHIVE_CN.md`。
历史碰撞与停车路径问题见根目录 `SAFETY_AND_TODO_CN.md`；必须继续处理，不能随旧文档删除。
