# 当前项目脚本

毕业设计主线见根目录 `README.md` 和 `LLM_HIERARCHICAL_ROBOT_TASK_PLAN_CN.md`。
旧学习恢复、G6/G8 批量实验与重复入口已归档，见 `LEGACY_ARCHIVE_CN.md`。
以下分类区分只读验证、底座开发和会执行运动的固定流程，不把它们混作大模型闭环。

## 当前主线：模型与只读接入

| 脚本 | 用途 |
|---|---|
| `setup_robobrain25_gguf.py` | 安装隔离推理程序及兼容库，校验 GGUF 权重 |
| `run_robobrain25_gguf.sh` | 启动本机回环地址的图文模型服务 |
| `validate_robobrain25_gguf.py` | 文本、图像及单工具输出验证，不执行工具 |
| `setup_roboos_gguf_client.py` | 建立模型客户端环境并应用本地兼容补丁 |
| `run_roboos_gguf_validation.sh` | 客户端契约检查，不启动完整框架、不执行机器人技能 |
| `setup_openflex_readonly.py` | 创建独立 MCP 环境，不改 ROS 依赖 |
| `run_openflex_readonly_bridge.sh` | 启动状态／观察工具，默认 ROS domain 83，不发送运动 |
| `run_openflex_readonly_sim.sh` | 仅启动现有仿真 bridge 与 RGB，隔离旧编排与控制入口 |
| `validate_openflex_readonly.py` | 检查真实 MCP／ROS，可选 `--model` 读实时图像 |
| `audit_openflex_head_camera.py` | 相机几何／分割审计；Oracle 分割不等于视觉识别，`--live` 读取已有快照 |
| `run_openflex_sim_navigation.sh` | 显式启用的隔离域 83 导航；可额外启用受控右臂抓取，复用同一任务锁 |
| `validate_openflex_navigation.py` | 实际 MCP 导航到站、取消、超时、互斥与重复请求验收，会运动 |
| `run_openflex_manipulation_preflight.sh` | 只规划辅助栈，不启动旧任务或 MoveIt 自动执行器 |
| `check_openflex_manipulation_safety.py` | 真实场景应用／回读、地面及全机器人测得起点有效性检查 |
| `probe_openflex_grasp_planning.py` | Oracle 抓姿、真实 TC-WAM／IK，独立全机器人端点检查，不执行运动 |
| `probe_openflex_grasp_trajectory.py` | 只规划预抓／抓取／持物撤离，MoveIt 与 MuJoCo 双重路径检查 |
| `validate_openflex_grasp.py` | 实际 MCP 抓取及取消／超时验证，会运动，不是完整自然语言取送 |
| `probe_openflex_place_trajectory.py` | 持物放置与释放后撤臂的只规划检查；释放后商品作为障碍物，禁止手指接触 |
| `validate_openflex_delivery_skills.py` | 实际持物导航／放置的成功、取消及超时验收，会运动 |
| `setup_openflex_roboos_runtime.py` | 独立完整框架环境与私有 Redis，不安装系统服务，不启动运动 |
| `run_openflex_roboos_redis.sh` | 只启动本机 Redis 16383，不覆盖系统 6379，不持久化 |
| `run_openflex_roboos_delivery.py` | 真实 Master／Worker／Redis／MCP 闭环；显式运动开关，逐次模型决策，独立终态验证 |
| `run_openflex_live_viewer.sh` | 桌面 MuJoCo 只读实时镜像，不另起物理循环或控制任务 |
| `openflex_runtime.py` | 统一 status／start／run／stop；显式隔离仿真启用，先停稳再关服务，不自动重置商品 |
| `run_openflex_recording.sh` | EGL 录制同时间戳 ROS 实测状态，独立于桌面开关；逐帧日志和可解码／非黑帧验证 |
| `view_openflex_model_result.py` | 桌面显示一次真实视觉描述及指定任务模型选技／后端反馈；只读，视觉文字不等于身份验证 |
| `openflex_delivery_protocol.json` | 三条件、固定分母、无自动重试的小规模评测协议 |
| `evaluate_openflex_delivery.py` | 先冻结源码／权重／协议，再采集同义指令、一次过期观察、同技能固定对照及录像 |

部署、端口、服务复用和验收记录分别见 `ROBOBRAIN25_GGUF_LOCAL_CN.md`、
`ROBOOS_GGUF_INTEGRATION_CN.md`、`OPENFLEX_ROBOOS_READONLY_CN.md`。
隔离运动、抓取预检及尚未完成的系统验收见 `OPENFLEX_ROBOOS_MOTION_CN.md`。
已有服务运行时不要重复启动；这些接口通过不代表自然语言取送已完成。

`setup_robobrain25_gguf.py`、下载与原精度验证等辅助脚本保留供模型环境维护，
当前已验证路线仍是 GGUF，不因脚本存在便声称其他部署路线可用。

## 离线研究与商超资产审计

`audit_openflex_research_assets.py` 用SQLite只读事务、XML及Office文档解析核验库存、模型覆盖与中期来源；
加 `--render` 仅在独立进程执行 `mj_forward` 生成静态截图，不执行 `mj_step`、ROS或机器人控制。
输出必须为新目录，禁止输出到原商超资产或中期目录；不会把不匹配的中文SKU猜成YCB模型。
相关论文与算法研究定位见 `thesis/商超轮臂机器人_研究主线与算法实验方案_CN.md`。

```bash
MUJOCO_GL=egl python3 scripts/audit_openflex_research_assets.py --output local_dependencies/openflex_thesis/NEW_RESEARCH_AUDIT --render
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q scripts/tests/test_openflex_research_assets.py
```

## 执行底座开发与诊断

ROS 脚本默认复用 `/home/wgb/openflex_shop/openflex_ws/install_v2`。

| 脚本／工具 | 用途与边界 |
|---|---|
| `check_care_environment.sh` | 检查模型、ROS 包和底座测试，名称为历史兼容 |
| `verify_moveit_basic.sh` | 检查 MoveIt、Pick-IK、描述包和启动文件 |
| `rebuild_mujoco_assets.sh` | 从 ROS 网格重建 MuJoCo 资源 |
| `run_thesis_offline.sh` | 无 MoveIt 的底座开发入口；含旧辅助节点，不是新版任务入口 |
| `start_moveit_sim.sh` | 为已有隔离仿真启动 MoveIt，不与其他规划栈重复运行 |
| `test_moveit_ik.sh`、`diagnose_moveit_candidate.sh` | IK／候选碰撞诊断 |
| `test_moveit_move.sh` | 会向仿真发送运动目标，使用前确认隔离域与安全条件 |
| `probe_moveit_workspace.sh` | 候选工作空间采样，不替代整条任务轨迹验收 |
| `check_moveit_current_state.py`、`search_moveit_left_park.py` | 状态／左臂姿态诊断 |
| `probe_left_park_contact.py` | 离线接触复现，不提供安全停车保证 |
| `audit_moveit_path` | 审计保存的路径失败日志或离散状态，不执行动作 |
| `probe_plan_path` | 只请求规划／有效性服务，必须隔离仿真且不替代连续路径安全证明 |

只规划工具的构建、参数与限制见 experiments 包 README。当前碰撞与接入待办见
`SAFETY_AND_TODO_CN.md`，不要为演示关闭碰撞或绕过夹持验证。

## 固定流程与对照边界

`run_thesis_moveit_mission.sh` 复用旧状态机和已有执行模块，**会发送仿真运动目标**。
仅在相应安全检查完成后人工运行；不与 RoboOS Worker、旧自动客户端或实机节点混用。
需要可视化时由该脚本传入 `enable_viewer:=true` 并设置合适渲染环境，不再维护重复脚本。

它是旧执行流程参考，不是当前默认启动方式，成功也不能证明大模型自主技能调度。
新版冻结实验的 `--policy fixed` 在同一 MCP 调用与验证入口内采用固定技能顺序，
与模型组共用安全门控和完成判定；不是运行旧脚本后将两个不同底层系统混作公平对照。
具体协议、故障注入边界和原始批次见根目录 `OPENFLEX_EVALUATION_CN.md`。

## 回归与清理

- 当前UNKNOWN运行现场不可清理或重启，先见根目录安全说明S0。停稳等待修复仅离线验证，未部署到现场。
- `tests/test_openflex_stop_audit.py`：过期/运动重置停稳窗口、有界超时和不吞其他错误；不提供现场解锁功能。
- `analyze_openflex_runtime.py`：只读分解原任务日志，区别轨迹区间、运动外时间和缺失的推理计时。
- `tests/test_openflex_readonly_bridge.py`：状态时效、未知处理、图像和工具白名单。
- `tests/test_roboos_gguf_integration.py`：模型客户端纯接口回归，无需启动模型。
- `clean_generated_artifacts.sh`：只清理其声明的可再生缓存／日志，不删除模型、地图和实验数据。

旧研究脚本、训练依赖和运行数据不再位于当前入口中；需要追溯时按归档说明查阅，
不要把归档脚本直接复制回来运行，也不要删除仍被执行底座依赖的 `openflex_care_agent`。
