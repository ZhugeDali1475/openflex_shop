# OpenFlex + RoboOS + RoboBrain2.5 毕业设计任务计划书

## 最新研究决议（2026-11-12，优先于下方历史路线建议）

按作者最新决定，底盘观测表征不再作为论文主线，不再要求ACT/π₀.₅训练或相关消融。
毕业研究围绕六项内容：自研系统、机器人算法、商超现状与难题、具体解决方法、对照实验、局限。
工作题目为《面向商超商品取送的轮臂机器人系统设计与任务条件化全身规划方法研究》，待导师及实验确认。
作者已明确两项方法为本人原创：任务条件化站位与操作可行性协同、分层自由度释放全身求解。
论文按作者提出的方法组织；文献对照界定创新范围，实验评估效果，不把作者归属与性能优越混为一谈。
已有条件绑定和分层求解等实现，与接近/撤离完整验证等后续增强分别记录。

- [x] 重新审阅自研贡献、现有算法和单商品证据，形成新研究主线与实验方案；这不代表新算法验证完成。
- [x] 保存代表企业官方页面与近似论文；已读2026商超反馈重规划原文的方法/实验节，不再把大小脑调用本身称作创新。
- [ ] 处理当前UNKNOWN操作的停止证据缺口；未确认停止前禁止新增运动、清锁或重置运行现场。
- [ ] 将已有商超资产按盒/瓶/罐三类仿真SKU显式绑定实例与货位，完成支撑、相机与物理验收。
- [ ] 同预算比较分层IK、去路由/去解析修正，分别记录构型有效与轨迹有效。
- [ ] 实现并验证条件保留、接近/撤离与预算内轨迹验证，完成强基线和消融。
- [ ] 安全条件满足后完成配对完整取送、故障与总时间评测，真实结果决定贡献表述。
- [ ] 据实测结果完成正式论文章节与局限分析；历史“论文初稿完成”不等于本项完成。
- [x] 按作者要求清理被替代的旧论文、衔接稿、答辩稿和重复导出；仅保留当前研究文稿，原始文献、源码与实验记录不删。

详见`thesis/商超轮臂机器人_研究主线与算法实验方案_CN.md`及
`thesis/商超研究_来源核验与实验待办_CN.md`。以下“不重做算法”等为首版接入约束；
本次授权的研究改进以复用现有平台为前提，不另建系统，不覆盖已有安全约束，不虚构实验。

## 历史接入计划与记录

更新日期：2026-11-11。
目标：使用 `openflex_shop` 已有的 OpenFlex 机器人、MuJoCo 环境和执行算法，以 RoboOS 组织任务、RoboBrain2.5 理解与决策、已有模块作为“小脑”，完成自然语言驱动的移动取送与反馈重规划。

当前进度补充：2026-11-09 已完成 GGUF 路线的本机 CUDA 加载、中文图文推理与单工具接口冒烟测试，详见 `ROBOBRAIN25_GGUF_LOCAL_CN.md`。第 2–3 节保留初次调研记录，其中“未下载／未验证”等描述是当时状态；当前验收以阶段 B 条目及本机实测记录为准，尚未完成 RoboOS／机器人联调。

2026-11-11 状态说明：前两段为历史进度，不代表当前仍未联调。实际完整 Master／Worker／Redis／MCP／ROS
取送已在 11 月 10 日及 11 月 11 日通过；新增负面矩阵、冻结评测与交付说明见
`OPENFLEX_EVALUATION_CN.md`、`OPENFLEX_THESIS_AND_HANDOFF_CN.md`，原冻结修正版三条件已完成，提速补充实验另批进行。

后续进度：已完成 RoboOS 模型客户端接口侧 9 项真实端点检查及 9 项回归测试，保存本地兼容补丁；详见 `ROBOOS_GGUF_INTEGRATION_CN.md`。测试中的登记表、站点和工具反馈是契约测试输入，未启动完整 Master／Worker、Redis 或机器人，不把客户端通过当成完整闭环通过。

任务前的 `[ ]` 表示未完成，`[x]` 表示已有可检查的完成证据。下载源码、语法检查、接口联调和机器人任务成功分别记录，不能互相替代。

2026-11-10 清理：旧恢复研究、G6/G8 批量入口及历史实验材料已归档到项目外，详情见
`LEGACY_ARCHIVE_CN.md`。当前工程入口为 `README.md`，未解决的碰撞与接入问题统一记录在
`SAFETY_AND_TODO_CN.md`；删除旧路线不代表相关安全问题已经解决。

## 1. 已确定的方案，不再重新选一套系统

```text
自然语言任务 + 当前环境图像／机器人状态
                 ↓
RoboBrain2.5：理解任务、选择技能、根据执行结果重新决策
                 ↕
RoboOS stand-alone：Master + 单个 OpenFlex Worker、任务与反馈管理
                 ↓ MCP
OpenFlex 技能适配层：将工具请求转换为现有 ROS 2 Action／Service
                 ↓
现有观察、导航、TC-WAM、局部对位、MoveIt 抓取、固定放置
                 ↓
现有 OpenFlex MuJoCo 机器人与货架／交付台环境
```

- RoboOS 的 Master 和 Worker 是框架角色，首版共用同一个 RoboBrain2.5 推理服务，只注册一台 OpenFlex，不做多机器人协作。
- 不重做机器人、场景、导航或抓取算法，不另建一套模拟技能系统，不把模型训练设为前置条件。
- 首个目标是仿真取送；实机接入后置。已有脚本导航、Oracle 抓姿和几何夹持代理如被复用，必须按其真实边界报告。
- 原有任务状态机保留作执行逻辑参考和对照，但不能让大模型只调用唯一“完整取送”工具后声称已实现自主技能调度。

## 2. 已拉取的源码与适配结论

源码放在项目已有忽略规则覆盖的 `local_dependencies/` 下，不混入 ROS `src`，不覆盖已有代码。该目录设置 `COLCON_IGNORE`，避免根目录构建时扫描外部仓库。

| 项目 | 本地位置 | 已固定的版本 | 适配结论 |
|---|---|---|---|
| RoboOS | `local_dependencies/RoboOS` | 官方 `stand-alone`，提交 `1c39b85ef5c9dcb9d5da235665c19af21824e545`，detached HEAD | 采用；有单机部署流程、MCP 本地／远程技能接口、可配置模型端点 |
| RoboBrain2.5 | `local_dependencies/RoboBrain2.5` | 官方 `main` 当前提交 `af98c932aac9ff715d70da177088d7bb95573ff7` | 采用该模型系列；本机优先验证官方 4B 的量化运行，不直接照抄 8B 示例 |

两个仓库均为浅克隆，没有下载模型权重，没有递归拉取附属仓库；本轮未安装或执行其程序。源码许可证均为 Apache-2.0。

### 为什么采用 stand-alone，而不是直接运行 main

官方 main README 将单机部署引向 stand-alone。该分支已经提供 `slaver/demo_robot_local/skill.py` 和 `slaver/demo_robot_remote/skill.py`，与“现有机器人作为技能执行端”的需求对应。

但是它不是零适配即插即用：示例技能只是返回成功文本，不控制 OpenFlex；本项目必须用真实 ROS 调用及验证结果替换这些示例函数。

### 本机适合的模型规格

本轮在沙箱外只读查询：RTX 4060，总显存 8188 MiB；一次采样可用显存约 7016 MiB。根分区现约有 20G、项目所在 `/home` 约有 205G 可用，之前磁盘满的状态已不适用于本轮。

从 Hugging Face 官方元数据核对：

| 权重 | 访问与许可证 | 官方 BF16 权重文件体积 | 本项目决策 |
|---|---|---|---|
| `BAAI/RoboBrain2.5-4B` | 本轮元数据为未设访问闸门、Apache-2.0 | 9,653,631,848 字节，约 8.99 GiB | 优先验证 4-bit 量化；不承诺一定容纳图像、KV cache 与仿真渲染 |
| `BAAI/RoboBrain2.5-8B-NV` | 本轮元数据为自动访问闸门、Apache-2.0 | 17,534,339,552 字节，约 16.33 GiB | 不作为本机默认；有更大显存设备时可部署同一系列远程服务 |

4B 的官方 BF16 权重本身已超过本机显存，不能用“4B”名称推断其能原精度直接运行。量化兼容性、速度、峰值显存与任务调度效果都尚未验证。若本机量化不满足要求，保留 RoboBrain2.5，通过远程推理或经过验证的卸载方案解决，不擅自改成另一个大脑。

元数据留存在 `local_dependencies/review_metadata/robobrain25_4b_metadata.json` 和 `local_dependencies/review_metadata/robobrain25_8b_nv_metadata.json`。对应模型 revision 分别为 `ec4b3bf5e969b48ad17df432b479c10402a0079b` 与 `7ae187cc62e45cfa1c5904903a8f017f11b2037b`；后续下载与部署需记录实际采用的 revision。

## 3. 实际核对出的必要适配，不另起项目

| 发现 | 本项目需要做什么 | 源码依据 |
|---|---|---|
| RoboOS 文档和默认配置面向 RoboBrain2.0-7B | 将 Master、Worker 指向同一 RoboBrain2.5 服务，并单独验证接口 | `local_dependencies/RoboOS/README.md:156`；`local_dependencies/RoboOS/master/agents/planner.py:27` |
| RoboBrain2.5 官方 `inference.py` 是图文推理类，不是 RoboOS 所需的聊天服务端 | 提供兼容 `/v1/chat/completions` 的部署或薄封装，不能只运行推理示例就认为已接入 | `local_dependencies/RoboBrain2.5/inference.py:6`；`local_dependencies/RoboOS/slaver/agents/models.py:343` |
| 4B 模型元数据包含原生工具模板；RoboOS 还有代码块 JSON 转换路径 | 对齐工具 Schema、模型模板、原生 tool_calls／文本解析；不直接复制 2.0 的 Hermes 模板 | `local_dependencies/RoboOS/slaver/run.py:210`；`local_dependencies/RoboOS/slaver/agents/models.py:209` |
| stand-alone 运行依赖额外 FlagScale 组件 | 按官方文档核对其固定提交 `3fc2037f90917227bd4aebabd9d7b330523f437c`，在独立环境安装；它尚未下载或验证 | `local_dependencies/RoboOS/README.md:105`；`local_dependencies/RoboOS/slaver/run.py:17` |
| 当前模型调用主要组织任务和已完成状态，不能只配置 camera 路径就自动传图 | 显式接入现有相机截图，并把新鲜图像、状态与失败反馈送入 RoboBrain2.5 | `local_dependencies/RoboOS/slaver/agents/models.py:311`；`local_dependencies/RoboOS/slaver/agents/slaver_agent.py:202` |
| RoboOS 根据模型预测更新场景；相同连续调用／无工具调用也可能终止循环 | 以 ROS 实际结果更新位置与持物状态，将失败反馈给下一次决策；任务完成必须另查物理证据 | `local_dependencies/RoboOS/slaver/agents/slaver_agent.py:178`；`local_dependencies/RoboOS/slaver/agents/slaver_agent.py:220` |
| 上游两个完整 requirements 有不同版本约束，RoboOS 还固定 NumPy 2.x | 框架、推理与 ROS 适配分离环境；不把上游依赖直接装进已有 ROS／MuJoCo 环境 | 两仓库 `requirements.txt` |

这些差异意味着“路线适合，但需要接口适配”，不意味着改用其他框架或重做底层算法。源码语法检查通过也不能证明依赖、推理与联调已经可用。

## 4. 小脑怎么接：直接复用现有能力

第一版用单商品 `thesis_item_01`、已有货架与固定交付台；用户称谓通过登记表映射到此 ID，不让模型编造商品或目标坐标。位置使用现有配置，经核对后登记为站点名称，不由模型输出自由关节／位姿命令。

| RoboOS 工具 | 复用的 OpenFlex 能力 | 必要适配与成功条件 |
|---|---|---|
| `get_robot_state` | 里程计、关节状态、`/thesis/grasp_status`、`/thesis/place/status` | 返回新鲜度、执行阶段、持物与安全状态；未知状态不推断为成功 |
| `observe_scene` | MuJoCo 相机、对象观测与对象记忆 | 返回图像引用、目标 ID、时间戳及来源；首版真值／Oracle 必须标记 |
| `navigate_to_target` | `/navigate_to_pose` 与已有底盘仲裁 | 使用登记站点，申请 NAV2 控制权，等待终态并确认交接停稳；当前仿真后端是脚本导航，不冒充真实 Nav2 效果 |
| `grasp_object` | `/thesis/generate_grasps`、`/thesis/compute_tc_wam`、`/thesis/execute_local_alignment`、`/thesis/execute_moveit_grasp` | 在一个抓取技能内部完成两阶段抓姿、评价、局部对位及 MoveIt；遵守 HOLD／ACT／MOVEIT 交接，确认 grasp_verified 和 transport_safe |
| `place_to_affordance` | `/thesis/execute_fixed_place` 与原固定放置监督器 | 使用已有交付位与 PLACE 控制权，等待 release_verified；不假定存在真实硬件 Place |
| `cancel_current_task` | ROS Action 取消和控制权保持逻辑 | 等待取消确认、更新持物状态；未确认停止前不接受新运动任务；软件取消不替代硬件急停 |

接入位置以 `openflex_ws/src/openflex_thesis/openflex_thesis_control/openflex_thesis_control/mission_orchestrator_node.py:54` 的已有 Action 客户端与模式交接为参考，但不复制整个硬编码任务顺序作为唯一大脑工具。

相机已有实现：`openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py:58` 与 `openflex_ws/src/openflex_thesis/openflex_thesis_mujoco/openflex_thesis_mujoco/bridge_node.py:1011`。后续启动配置显式启用 RGB-D 并核对 `/thesis_sim/head/color/image_raw` 等话题，不能沿用关闭图像的历史任务脚本。

新入口只替换上层任务调度，保留底层技能与安全模块。当前 launch 无条件启动旧 `mission_orchestrator`；因此只设置 `run_mission:=false` 还不足以构成新的调度架构。后续新建专用 launch，不启动旧编排器、自动任务客户端或竞争式自动调度，避免双重调度。

## 5. 可勾选任务清单

### 阶段 A：拉取与适配核对（已完成）

- [x] A1 拉取 RoboOS 与 RoboBrain2.5 官方源码到独立目录，记录来源与提交。
- [x] A2 比较 RoboOS main 和 stand-alone，选择 stand-alone 并固定工作副本。
- [x] A3 阅读实际模型客户端、MCP 技能示例、反馈路径及 OpenFlex 对应接口，列出必要适配。
- [x] A4 核对本机资源与官方 4B／8B 权重元数据；后续权重部署进展见 B1。
- [x] A5 静态解析两仓库已跟踪 Python 文件：RoboOS 17 个、RoboBrain2.5 2 个，无语法错误；未导入或执行上游程序。
- [x] A6 按“现有机器人和环境 + RoboOS + RoboBrain2.5 + 现有小脑”重写本计划。

本阶段仅验收源码与适配核对；模型运行和机器人接入的后续证据分别见阶段 B、C。

### 阶段 B：让 RoboBrain2.5 成为可调用的大脑

- [x] B1 通过 GGUF 路线完成独立 CUDA 加载与中文图文基础验证：Q4_K_M + F16 mmproj、llama.cpp b12110、上下文 4096、图像 1024 tokens；记录转换仓库 revision、文件哈希与延迟。整卡采样最高 5012 MiB（含桌面，非分配器精确峰值）；未污染 ROS 环境。原精度／bitsandbytes 路线未验证。
- [x] B2 提供 RoboOS 能访问的模型服务，验证文本、图像、聊天历史、工具 Schema、tool_calls 或受约束文本调用、usage 与终止响应的兼容性。
- [x] B3 用“导航／抓取／放置”的实际工具描述检查中文任务分解和参数输出；只检查推理输出，不发送运动。
- [x] B4 记录模型与仿真／GUI／MoveIt 并行的资源采样；本机方案已完成单轮，但高 swap 和长期稳定性仍需单独评测，不宣称完整启动或分配器精确峰值。

验收：同一个 RoboBrain2.5 服务能返回可解析的技能选择，并接收真实格式的失败反馈；记录运行证据，不以聊天回复代替工具调用证明。

B2 部分进展：已在 `127.0.0.1:4567` 验证兼容聊天 API 的文本、图像、usage、正常终止及单个原生 `get_robot_state` 工具返回；没有执行该工具，未验证 RoboOS 客户端、多轮工具反馈和真实失败闭环，因此仍未勾选 B2。基础证据为 `local_dependencies/robobrain25_gguf/runs/20261109_194435_132754/result.json`。

B2／B3 接口侧更新：现已通过真实 RoboOS Worker 模型类和 Master 规划方法验证图片、多轮工具回包、导航／抓取／放置参数及失败／未知状态反馈；最终记录 `local_dependencies/robobrain25_gguf/runs/roboos_20261109_202330_244249/result.json`。模型配置已指向 GGUF 服务，兼容修复保存为 `scripts/patches/roboos-standalone-gguf.patch`。完整进程、真实登记表、MCP／ROS 技能 Schema 和实际反馈仍待 C 阶段，因此 B2／B3 保留未勾选，不用测试输入代替真实接入证据。

2026-11-10 系统侧更新：已运行实际 GlobalAgent、ToolCallingAgent、私有 Redis 16383 与完整 MCP 18556。
`local_dependencies/openflex_roboos/runs/roboos_delivery_20261110_195123/` 保存实际中文任务、模型 Master JSON、
真实工具 Schema、相机图片、原始模型调用与 ROS 工具结果。B2／B3 因此更新为通过；这轮撤臂碰倒商品，
整体依然 failed，不将接口或部分技能通过算作 E1。B4 仍单独记录并行采样，不能用模型单跑显存替代。

### 阶段 C：将现有 OpenFlex 小脑暴露为 MCP 技能

- [x] C1 新建轻量 `openflex_roboos_bridge` 适配包，承接 MCP 请求和 ROS 调用；先实现状态／观察，不重写底层算法。
- [x] C2 接通已有 MuJoCo 图像、目标观测和状态，建立单商品、站点及交付位映射；区分相机信息与 Oracle 信息。
- [x] C3 接通 `navigate_to_target`，验证 Action 结果、超时、取消、资源独占和停稳交接。空载与实测附着物路径均有隔离仿真证据，不是 Nav2 或连续碰撞证明。
- [x] C4 接通 `grasp_object`，封装现有 TC-WAM／对位／MoveIt 子流程；只处理为接入所必需的阻塞问题，保留碰撞与夹持验证。
- [x] C5 接通 `place_to_affordance`，先执行经检查的机械臂放置轨迹，再在实际位姿开爪并确认落稳；释放后商品加入障碍物，直线退出后无接触收臂，最终独立验证通过，无瞬移。
- [x] C6 补齐取消、错误、结果未知、重复调用和状态更新时间；技能成功／超时／取消有分开记录，共享互斥及未知锁定有回归测试，失败不报成功；完整框架负面矩阵另见 D5。

验收：每个工具都实际调用现有 OpenFlex 仿真能力；通过直接 MCP 调用可检查结果。先做只读和只规划检查，再按安全条件逐个执行，不需要另建模拟机器人系统。

C2–C4 更新：已修正头部相机误挂在底盘深度相机壳内的问题，在保留机器人几何的条件下验证商品完整进入视野；模型读图能看到商品，但把长方盒误称圆柱，不能声称可靠视觉身份识别。登记表核实单商品、取货观察点、桌面和释放中心高度，拒绝原来位于桌体内部的交付导航点。空载及持物导航均有成功、取消、超时与停稳证据。
抓取 `grasp_20261110_strict_success`／`grasp_20261110_strict_timeout`／`grasp_20261110_strict_cancel` 已分别记录成功、超时和取消，
`roboos_delivery_20261110_195123_skill_03` 是实际模型选取的完整抓取成功。放置已无瞬移，
但完整放置仍须通过释放后无接触撤臂与独立最终检查；详细证据及失败见 `OPENFLEX_ROBOOS_MOTION_CN.md`。

### 阶段 D：RoboOS + RoboBrain2.5 + OpenFlex 联调

- [x] D1 在独立框架环境准备 RoboOS、Redis 和文档要求的 FlagScale 组件；只启一个 Master、一个 OpenFlex Worker，共用同一模型端点。
- [x] D2 将 Worker 的 MCP 地址指向 OpenFlex 适配层；核对工具格式与候选筛选，避免默认仅选 3 个工具而隐藏必要的观察／状态技能。
- [x] D3 配置本项目机器人、场景和任务提示；接入图像、真实结果与验证状态，不根据大模型想象更新持物／交付结果；真实运动失败直接安全终止，不宣称已验证模型自主恢复。
- [x] D4 新建专用启动入口，复用现有 MuJoCo、MoveIt 与安全模块，不并行启动旧任务编排；所有运动技能串行获取控制权。
- [x] D5 确定性负面矩阵覆盖实际上游 Worker 的无工具、重复调用、解析失败、服务异常与预算耗尽，以及入口白名单、取消审计与未知状态；133 项框架／契约、204 项 ROS／仿真回归通过。服务失联使用显式异常／MockTransport，不宣称整机真实断服演练；见评测说明。

验收：用户提交自然语言后，由 RoboBrain2.5 经 RoboOS 选择 OpenFlex 技能；调用结果进入下一次决策，日志可追溯模型输出、工具参数、ROS 结果和终态。

### 阶段 E：完成毕业设计的取送演示与实验

- [x] E1 在现有场景完成单商品取送，保留真实相机／任务日志与独立状态。原桌面黑屏录像仍无效；2026-11-11 新批次同义指令轮补齐有效 EGL 实测镜像录像，988 帧均可解码且非黑，无陈旧间隙。
- [x] E2 冻结的同义指令未直接给工具 ID、未逐字复用默认任务，在 `evaluation_small_v1_1_20261111/model_synonym` 实际完成，7 次 Worker 推理／7 次工具、约 494.8 s；仍受提示中既定技能顺序约束，不证明自由规划或广泛语言泛化。
- [x] E3 冻结故障轮明确拦截第一帧并反馈“过期／未执行”，真实模型重新选 observe_scene 后完成整轮；8 次推理／8 次工具、约 532.4 s，独立完成与录像均通过。不伪称真实传感器掉线，不扩大到运动失败自主恢复。
- [x] E4 同技能固定顺序对照通过：7 次工具、0 次模型推理、4 次运动、约 484.9 s；与模型组共用执行及完成验证入口，不重启旧后端。正常模型组约 494.8 s，各条件一次且措辞有混杂，不作优越性结论。
- [x] E5 修正版冻结批次计划／采集／独立完成均为 3，三段有效录像、初始与最终状态、全部运动记录及资源样本已归档。首批启动失败／人工中止／未采集单独保留，分母仍为 3；见 `OPENFLEX_EVALUATION_CN.md`。

验收：既能展示自然语言驱动的真实仿真执行，也能给出“模型做了什么决策、失败如何反馈、结果如何验证”的证据。抓取失败或路径被拒绝时如实记录，不用关闭碰撞来换成功。

### 阶段 F：论文与交付

- [x] F1 原工程接入阶段曾授权先拟题；该题目现被上方研究决议替代，不声称导师／学校审批。
- [x] F2 历史上曾完成工程初稿与12页答辩材料的导出；旧文稿现按作者要求删除。本项仅记录历史产出，不代表当前正式论文与算法实验完成。
- [x] F3 提供版本与冻结源码、无密钥 `.env.example`、启动说明、三段有效演示视频、337 项 JUnit 和两批原始材料；交付目录及逐文件校验见 `OPENFLEX_THESIS_AND_HANDOFF_CN.md`。权重／环境另行准备，实机与微调不设为当前必做。

## 6. 接下来直接做什么

用户要求的 C2 → 导航／抓取／放置 → 可追溯单商品闭环已完成一轮。
验收目录 `local_dependencies/openflex_roboos/runs/roboos_delivery_20261110_201218/` 的最终 result 与独立复核均通过，
真实 Master／Worker 记录 7 次模型推理、7 次工具调用、约 478 s 的工具闭环。B4 保留 66 次并行资源采样，
整卡显存采样最高 5666 MiB、系统 swap 最高约 9509 MiB，不能代替长期性能评测。
19:51 轮撤臂碰倒商品的失败记录保留，不能改写为自动恢复成功。
后续 D5 确定性负面矩阵、E2／E3 同义指令与可控观察拒绝、E4 同技能对照、E5 冻结三轮及 F3 工程交付，
已在 2026-11-11 补齐，详见上述证据；不是从 E1 单轮成功推定。当前任务结束、无待处理运动且实测 HOLD。
历史六章工程初稿已按作者要求删除；当前研究方案、两项方法归属及Word入口见`thesis/README_CN.md`。
后续在不改变主架构和安全约束下做耗时分解与独立冻结的分阶段提速验证；问题、方案和失败均记入
`thesis/项目优化与提速分析_CN.md`。学校模板、个人信息、正式题目审批和提交仍需人工完成；新实验不覆盖旧结果。

2026-11-11 16:43：三批补充提速共6轮完成0轮，最后原运动请求UNKNOWN，当前只读HOLD不等于终态已审计。禁止新任务、清锁和重置；S0审计优先于继续提速。工作区的停稳有界等待修复已离线回归，未在未知现场重启部署；原三成功轮仍为论文主证据，不用补充失败轮替代。
运动仍仅允许显式启用的隔离仿真，绝不下发实机。

当前完成边界：GGUF 模型部署、RoboOS 模型客户端接口验证及 C1 只读 MCP 桥接已有证据，
包括读取真实 MuJoCo 消息和实时 RGB。C2 商品视野／语义映射、C3 空载及持物导航、C4 抓取、C5 无接触放置、
C6 技能反馈和 E1 一轮自然语言取送已验收；完整 Master／Worker 实际参与逐步技能调度。
只读验证未执行运动，不等于旧固定流程、实机或新版完整任务已经通过安全验收。
