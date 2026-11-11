# OpenFlex 隔离仿真运动接入记录

更新日期：2026-11-10。只针对本机 MuJoCo 数字场景；已有一轮完整自然语言单商品取送成功证据，绝不作为实机校准或稳定成功率证明。

## 一轮完整闭环通过

最终验收目录：`local_dependencies/openflex_roboos/runs/roboos_delivery_20261110_201218/`。
`result.json` 的 `passed`、`success`、`completion_verified`、`final_measured_completion` 均为 true，
真实 Master 收到相同 Worker 成功结果，`physical_verification=false`。

- 中文指令：“请把货架上的 thesis_item_01 商品取下，送到 delivery_zone 交付桌面并放稳。”
- 实际执行顺序：读状态 → 读实时图片 → 取货点导航 → 完整抓取 → 持物导航 → 放置及无接触撤臂 → 再读状态。
- 共 7 次 Worker 模型推理、7 次实际工具调用；每个动作等待真实终态后再推理，不批量执行模型预生成计划。
- 取货导航、抓取、持物导航、放置均为 succeeded／success=true／stopped_confirmed=true；
  抓取还要求 grasp_verified、transport_safe，放置还要求 place_verified。
- 最终测得商品中心约 `[-0.848278,-0.799824,0.560920]` m，直立、桌面支撑接触、已脱离夹爪；
  底盘与关节停稳，无安全急停。稍后的 `independent_final_audit.json` 再次独立复核通过。
- 首次状态工具到最终状态工具约 478 s；抓取约 265 s、持物导航约 23 s、放置含撤臂约 181 s。
  这是保守速度的单次开发验收，不是实时性或统计性能结论。

主要证据：`master_model_response.json`、`master_plan.json`、`worker_received_subtask.json`、
`model_tool_contracts.json`、`model_trace.json`、`tool_trace.json`、`master_received_result.json`、
`execution_sources_sha256.json`、`independent_final_snapshot.json`、`independent_final_audit.json`。
初始真实相机图像保留在多模态模型历史中；`measured_final_render.png` 是从同时间戳的最终实测状态离屏渲染，
不是另一次物理执行或桌面截图。桌面录制因离开期间显示器关闭而全黑，
`gui_video_manifest.json` 已标记 `valid_visual_evidence=false`，不得作为有效运动视频。

本轮并行资源采样 66 次、约 328 s，整卡显存采样最高 5666 MiB（含桌面／其他图形进程），
系统可用内存最低约 1544 MiB，系统 swap 使用最高约 9509 MiB；
见 `runtime_resources.jsonl`／`runtime_resources_summary.json`。这不是分配器峰值、完整启动峰值或长期稳定性测试，
内存压力与 swap 使用较高，不能据一次通过宣称持续高吞吐可用。

这轮通过不覆盖任意商品／场景、可靠视觉身份识别、Nav2、实机力觉、自动故障恢复或正式对照实验。
19:51 轮撤臂失败及更早的拒绝、超时、取消记录全部保留，不合并成成功，也不从开发试验挑选计算成功率。

## 已验收边界

1. **C2 相机与语义映射**：原 `thesis_head_camera` 位于底盘深度相机壳内，中心射线约 0.745 mm 即命中自身。
   现挂到头部 `head_yaw_link`，发布与渲染快照同时间戳的光学 TF，视场 70°；未隐藏或缩小机器人几何。
   现场对照 `local_dependencies/openflex_roboos/runs/camera_20261110_171530_645063/result.json`：旧图 0 商品像素，
   新图 468 商品像素，完整包围盒进入画幅。分割是模拟器 Oracle，不是视觉检测。
   实时模型读图证据 `local_dependencies/openflex_roboos/runs/20261110_171526_863021/result.json`：
   能看到红色商品及货架，但误称盒子为圆柱，不能宣称可靠形状／身份识别。
2. **商品与站点**：`thesis_item_01`，盒尺寸 0.07×0.06×0.16 m；取货观察位 `[0,0,0]`。
   旧交付导航位 `[-0.85,-0.8,0]` 位于桌体内部，禁止使用；空载验证的桌东侧接近位为 `[0,-0.8,π]`。
   桌面中心 `[-0.85,-0.8,0.48]`，面宽 0.7×0.7 m；配置 0.56 m 是直立商品的中心高度，不是桌面高度。
   接近位不等于机械臂可放置位，运行时仍逐次检查路径；视觉身份绑定保持未验证。
3. **C3 空载导航**：`local_dependencies/openflex_roboos/runs/navigation_20261110_172718_816187/result.json`
   为通过记录，含到站实测与停稳、实际取消、实际超时、并发拒绝、重复请求不重执行、交付接近和返回取货点。
   复用现有 `NavigateToPose` 比例导航服务，碰撞检查来自适配层采样，**不是 Nav2，也不是连续碰撞证明或持物运输验收**。

## 抓取预检与执行联调

- 规划入口 `run_openflex_manipulation_preflight.sh` 只启动 MoveIt 规划、对象记忆、Oracle 抓姿和真实 IK／有效性评估。
  不启动旧自动编排或抓取执行器；`allow_trajectory_execution=false`。Oracle 始终标明非物理感知。
- `check_openflex_manipulation_safety.py` 从生成 MJCF 读取货架／桌体，加入地面，应用并回读真实规划场景，
  检查测得的完整机器人起点。仅四个车轮允许接触地面；其他链接、货架和桌体继续碰撞检查。
- `probe_openflex_grasp_planning.py --search-base` 复用 GenerateGrasps、TC-WAM、真实 MoveIt IK，
  独立复核建议底盘位上的全机器人端点。观察位不等于抓取位，原地候选不可达时必须规划接近。
- `probe_openflex_grasp_trajectory.py <候选报告>` 请求预抓、抓取、持物撤回轨迹；完整机器人每 5 mm 升降／0.03 rad 关节增量检查，
  并在真实 MuJoCo 几何上重放；附着物使用正确 TCP 相对位姿和原始盒尺寸，不把碰撞忽略当作通过。
- 初始软接触记录 `grasp_trajectory_20261110_174539_358108/result.json` 检出商品—层板约 0.08 mm 重叠，未执行机械臂。
  现为商品增加 1 mm 保守接触包络，几何和碰撞开关不变，重新建立模拟器支撑接触。
  `local_dependencies/openflex_roboos/runs/grasp_trajectory_20261110_175953_785576/result.json`：
  三段轨迹分别检查 915／27／704 个状态，全机器人、附着盒与 MuJoCo 接触检查通过；这仍是只规划证据。
- 实验性 `grasp_object` 路由与导航使用同一个 `OperationStore`，跨技能互斥；到实际接近位后重做精抓和路径检查，
  经底盘锁定才发送现有 lift／右臂／夹爪控制器。夹持后还要依据实际附着重新规划撤回并检查保留状态。
  运行结果写入 `navigation_operations.jsonl`，默认只读 MCP 不注册此工具。
  直接技能 `grasp_20261110_strict_success` 已记录完整抓取、保留验证、撤离和运输姿态成功；
  `roboos_delivery_20261110_195123_skill_03` 进一步记录了真实模型选取的完整抓取成功。
  抓取取消／超时检查与成功记录分开保留，不把一次成功解释为任意起点都可行。

撤离联调发现：`grasp_20261110_third` 已真实完成预抓、精抓及独立夹持，但从实际附着状态直接规划运输姿态时，
MoveIt 在返回路径中发现商品—层板碰撞并返回 `INVALID_MOTION_PLAN`，因此任务如实为 failed，底盘与关节停稳，
没有执行被拒绝的轨迹。原始报告为
`local_dependencies/openflex_roboos/runs/grasp_trajectory_20261110_181646_931483/result.json`。
随后保留失败记录，明确重置独立仿真试验，不声称原任务自动恢复成功。
撤离现拆分为 4 cm 抬升、保持目标方向向货架外撤出 18 cm、再进入运输姿态；
IK 只搜索三个明确的升降种子，仍要求真实全机器人有效性检查，撤离后的完整商品包围盒必须离开货架前缘至少 5 mm。
OMPL 针对右臂带升降组使用更细路径检查分辨率，原有货架碰撞和独立返回路径复核保留。
以上修改不能代替新的执行验收，结果按对应 request_id 的持久化终态记录。

## 放置真实性修正

旧 `_on_place_control` 会直接写物体自由关节坐标并立即返回 verified；已删除该路径。
新后端只在新鲜 PLACE 所有权、底盘／机械臂静止、目标 ID／手臂／坐标系匹配、物体实际已在桌面上方时开爪，
等待测得的爪张开后原位脱离，不写物体位姿；随后要求桌面真实接触、物体姿态／边界／速度和连续 0.5 s 落稳。
取消不伪造放置成功；物体已释放时也不声称取消能恢复夹持。单元测试含真实重力落稳的独立物体夹具，
**不是机械臂取送测试**。现已接通受显式开关保护的 `place_to_affordance`；
`skills_20261110_first_nav_success` 和 `roboos_delivery_20261110_195123_skill_04` 已记录持物导航成功，
前者还分别记录了真实持物导航取消／超时、放置技能取消／超时和停稳。

### 19:51 轮完整模型执行与撤臂失败

`local_dependencies/openflex_roboos/runs/roboos_delivery_20261110_195123/` 中保留实际 Master、Worker、
Redis 分派、真实相机图片与每次模型调用／工具反馈。模型已实际完成取货导航、完整抓取、持物交付导航，
并执行预放置、下降、实际开爪、桌面接触及连续 0.5 s 落稳；这轮最终仍为 **failed**：撤臂碰倒了商品。
独立最终状态检测到商品中心降至约 0.511 m、位置偏移，未把控制器成功或最初落稳当作任务完成。

离线复核 `released_payload_retreat_audit.json` 检出旧撤臂轨迹第 4 个点的手指—商品接触。
根因是持物时允许的手指接触白名单被错误沿用于释放后，且 MoveIt 世界里没有已释放商品。
现在释放后将原尺寸商品及测得位姿加入规划世界并回读，禁止全部机器人链接触碰已释放商品；
先保持腕部方向沿 TCP 负 Z 直线退出 10 cm，再收至运输姿态。每段仍检查完整机器人及 MuJoCo 几何。
18 cm 退出和“退出后再抬升”的 Cartesian 候选因关节边界不可达被拒绝，没有执行。
`strict_retreat_replan_horizontal.json` 是从保存起点得到的只规划通过记录，**不是执行成功证据**。
撤臂执行期间还持续检查商品支撑、姿态和速度，一旦失稳立即取消，不能等整段结束再发现。

### 控制交接与模型执行契约

- 仲裁服务返回接受不表示 MuJoCo 已收到控制权。现在要求诊断回报同一 mode／epoch、健康且新鲜；
  MOVEIT 还要求实际机械臂守卫通过，PLACE 要求实际释放守卫通过，确认后才发送控制命令。
- 模型可能仍批量输出多个调用。宿主只执行模型返回的第一个，丢弃其余，拿到实际反馈后重新推理；
  不执行预生成整条调用列表，也不替模型补选技能。原始输出和串行化策略均记入 `model_trace.json`。
- Worker 不根据无调用、重复文本或想象式场景更新判成功；只有新鲜独立最终状态可以结束成功。
  安全前置条件不满足时返回未执行拒绝；真实动作失败或未知时停止，不自动换 ID 重试。

## 运行入口与隔离

先查看已有服务，禁止重复启动同一场景、规划栈或调度器：

```bash
systemctl --user status openflex-roboos-readonly openflex-roboos-navigation openflex-manipulation-preflight
```

默认观察：`bash scripts/run_openflex_readonly_bridge.sh`，MCP 18551、快照 18552。
空载导航需显式启用：

```bash
OPENFLEX_ENABLE_SIM_NAV=1 bash scripts/run_openflex_sim_navigation.sh
.venvs/openflex-mcp/bin/python -I scripts/openflex_mcp_entry.py --port 18554 --enable-navigation
```

实验性受控抓取需在同一隔离启动入口额外设置 `OPENFLEX_ENABLE_SIM_MANIPULATION=1`；只接 lift／右臂／右夹爪，
左臂、旧放置和场景修改服务仍被隔离。规划辅助栈单独运行 `bash scripts/run_openflex_manipulation_preflight.sh`。
模拟器重启导致时钟回退时，必须协调重启辅助规划／记忆栈，不复用旧对象记忆或 TF。
工具网关只绑定 127.0.0.1:18553，工具结果 running 不表示成功；始终用原 request_id 查询，unknown 不自动重试或释放任务锁。
所有启动入口固定 ROS domain 83，不连接原 domain 0；域隔离不是安全认证，不得开放网络端口或接入实机。

抓取 MCP 单独使用 18555，仍需显式启用：

```bash
.venvs/openflex-mcp/bin/python -I scripts/openflex_mcp_entry.py --port 18555 --enable-navigation --enable-manipulation
.venvs/openflex-mcp/bin/python -I scripts/validate_openflex_grasp.py --timeout-sec 600
```

第二条命令会实际执行仿真运动，不是只读检查。`--cancel-during-motion` 在真实轨迹开始后请求取消，
`--expect-timeout --timeout-sec <秒>` 检查超时；测试前需要明确独立场景起点，不把同一个请求 ID 用作重试。
`--request-id` 只能观察／重复查询同一参数的已有请求，后端必须返回同一持久化结果而非再次执行。

放置另外要求网关环境 `OPENFLEX_ENABLE_SIM_PLACE=1`，完整技能 MCP 使用 18556：

```bash
.venvs/openflex-mcp/bin/python -I scripts/openflex_mcp_entry.py --port 18556 --enable-navigation --enable-manipulation --enable-placement
```

实际 RoboOS 调度环境通过 `scripts/setup_openflex_roboos_runtime.py` 准备，使用独立 Redis 16383。
已有对应服务时不要重复启动。完整运动入口为：

```bash
env -u ALL_PROXY -u HTTP_PROXY -u HTTPS_PROXY .venvs/roboos-runtime/bin/python -I scripts/run_openflex_roboos_delivery.py --execute-isolated-simulation
```

它要求实际物体从货架起点开始、单调度锁和新鲜观测；不会自行重置仿真、瞬移商品或恢复失败任务。
需要新试验时先审计原请求停稳，明确保存失败结果，然后协调重启仿真和规划辅助栈。
`scripts/run_openflex_live_viewer.sh` 只读订阅同时间戳的里程计与诊断，不运行第二个物理循环或发送命令。
桌面需正确的 DISPLAY／XAUTHORITY；窗口显示 LIVE 才表示新鲜，STALE 表示画面冻结，不能当作实时证据。

## 尚未完成

- 无接触撤臂与独立最终验证已在上述整轮通过；不同初始条件下的重复性与长期资源评测仍未完成。
- 未冻结同义指令、可控失败和固定流程对照的正式评测协议，不能从单次开发验收推断模型提升。
- 上游 Worker 的完成判定和想象式更新已在可复现补丁中修正，但这不是实机安全认证。

不得把模型客户端测试、规划服务通过、夹爪动作成功或释放后端单元测试中的任意一项替代上述整项验收。
