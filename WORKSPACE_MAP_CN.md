# OpenFlex 工作空间地图

更新：2026-11-14。本文用于帮助开发者先确认源码、场景资产和执行入口，避免从多个 MuJoCo 目录中凭名字猜运行环境。

## 当前比赛半区目标

本阶段目标固定为**两组对称货架、两个交付台、一个红色测试物体**。不是四组货架/四个交付台，也不是多商品任务。

已盘点场景清单：
`local_dependencies/openflex_thesis/scene_inventory_20261114_skill_plan_v2/scene_inventory.json`。

| 角色 | 路径 | 允许用途 | 注意 |
|---|---|---|---|
| 单红成功运行回归基线 | `local_dependencies/openflex_thesis/retail_fixture_single_red_runtime_20261113_v7/` | 参考已成功运行的模型、运行配置和回归几何 | 场景中有两台桌，但当前在线技能只注册一个 `delivery_zone`；不可据此声称双目的地支持 |
| 单红静态场景源 | `local_dependencies/openflex_thesis/retail_fixture_single_red_20261113_v3/` | 场景结构/几何参考，包含两组货架、两台桌、单红物体 | `execution_enabled=false`；不是直接运行授权 |
| 半区父场景 | `local_dependencies/openflex_thesis/retail_fixture_half_left_20261113_v5/` | 离线场景派生源 | 有 52 个商品，不得当作本阶段单红运行场景 |
| 其他 `retail_fixture*` 版本 | 见只读清单 | 静态对照、历史实验、动力学或搜索审计 | 除非配置、哈希及目的明确核实，否则不作为启动入口 |

运行时 v7 为兼容旧桥接接口对 body 做过重命名：`shelf_group_2 → thesis_shelf`、`counter_top_left → thesis_delivery_table`；另一组仍以 `shelf_group_1`、`counter_top_right` 等名字出现。使用 `manifest`/运行配置和几何校验映射，不靠文件名或单个 body 名推断站点身份。运行目录中的 `scene.xml`、`runtime.yaml`、`shelf_scene.yaml` 必须作为同一版本配套使用。

## 顶层目录

| 路径 | 主要内容 |
|---|---|
| `openflex_ws/src/openflex_thesis/` | 毕设 ROS 2 包：MCP/ROS bridge、MuJoCo bridge、控制仲裁、TC-WAM、消息、bringup 和实验包 |
| `openflex_ws/src/openflex_armx/`、`openflex_chassis/`、`openflex_grasp/`、`openflex_integrated/`、`openflex_head/` | 自研 OpenFlex 机器人描述、底盘/机械臂/抓取/头部与既有集成底座 |
| `openflex_mujoco-1.0_basic/`、`openflex_models/`、`openflex_maps/`、`openflex_drivers/` | 机器人模型、MuJoCo 基础资产、地图和驱动 |
| `supermarket_scene/` | 商超原始场景和资产；生成器输入，禁止用派生场景覆盖源资产 |
| `scripts/` | 模型/框架启动、场景构建、离线盘点、只规划探针、验证和运动执行入口；先读 `scripts/README_CN.md` |
| `local_dependencies/` | 忽略管理的 RoboOS/模型依赖、场景副本、实验日志和证据；不会自动随源码仓库发布 |
| `thesis/` 与根目录 `*.md` | 研究主线、计划、阶段证据、安全边界、复现和交接说明 |
| `build/`、`install/`、`log/`、`.venvs/` | 本地构建结果、ROS 安装、日志和 Python 隔离环境，不是权威源码 |

## 代码职责

| 组件 | 职责 |
|---|---|
| `openflex_roboos_bridge/` | MCP 工具 schema 和 ROS 适配；`skill_contracts.py` 目前是纯 Python 契约草案，不是运行调度器 |
| `openflex_thesis_mujoco/` | MuJoCo 模型、桥接、仿真观测；`retail_scene_builder.py` 是离线商超场景生成器 |
| `openflex_thesis_control/` | 控制权仲裁和执行控制 |
| `openflex_tc_wam/` | 站位/操作候选相关评估能力 |
| `scripts/run_openflex_roboos_delivery.py` | 单商品 RoboOS/MCP 取送入口；当前运行技能仍以单目标/单目的地为主 |
| `scripts/audit_openflex_scene_inventory.py` | 只读列出 retail fixtures 的拓扑和哈希，不连接 ROS、不推进 MuJoCo |

## 工具风险分级

- 只读/离线：`audit_openflex_scene_inventory.py`、配置和结果解析、纯 Python 契约单测。
- 静态构建/`mj_forward`：`build_openflex_retail_*.py`、运行 profile 准备；只写入新目录，不代表运动安全或执行成功。
- 只规划/探针：`probe_openflex_*`、MoveIt workspace/path audit；要求隔离 ROS 域，规划通过不代表连续路径或运动安全。
- 会运动：`validate_openflex_*` 中标注会运动的工具、`run_openflex_roboos_delivery.py --execute-isolated-simulation`、`test_moveit_move.py`、旧 mission launcher。必须核对域、节点、控制权和安全门；本说明不是运动授权。

## 场景管理规则

1. 启动前核对配置实际指向的 `scene.xml` 与 `runtime.yaml`，并记录绝对路径和 SHA-256。
2. 运行结果必须保存对应 scene/config 哈希、request ID 和终态回读。
3. 每次新开发场景写入全新版本目录；构建器拒绝覆盖已有目录。保留 v7 和历史运行证据。
4. 先给场景分角色并解除非目标目录的默认入口引用；不要删除历史资产来“清理”。
5. “场景有两个台面”与“在线 Skill 支持两个目的地”是不同验收项。
