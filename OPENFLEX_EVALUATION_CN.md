# 单商品取送：冻结评测与负面验收

日期：2026-11-11。本材料区分单元／契约故障注入、真实模型执行、实际仿真运动与独立终态。
不把其中任意一种替代其他验收，也不将仿真结果当作实机安全证明。

## 评测问题与固定条件

研究问题：在相同 OpenFlex 技能、相同场景和相同完成判定下，受约束的大模型调度能否
完成单商品取送、处理一次不可用观察，并产生可以审计的执行证据？

固定协议为 `scripts/openflex_delivery_protocol.json`：一轮同义指令、一轮首次观察被标记为
过期的可控故障、一轮相同技能的固定顺序对照，各条件一次，共三次；不允许自动重试。
这是可行性小样本，不是统计显著性实验。正常模型组与固定组的指令措辞不同，不能作因果归因。

- 隔离 ROS domain 83；场景重启恢复既定初始条件，不使用运行中瞬移商品来完成任务。
- 每轮开始前保存前一场景、确认 HOLD 和实测停稳，再同时重启仿真与规划栈。
- 初始商品必须未持有、位于登记的货架起点 4 cm 内，关节／里程计／诊断／目标记忆新鲜。
- 一次任务 1200 s 上限，Worker 12 步上限，运动共享互斥锁，失败不换 ID 自动重试。
- 两种调度共用同一 `execute`、MCP、ROS、规划检查和 `DeliveryContract` 完成验证器。
- 固定组明确不运行 Master／模型 Worker，不冒充大模型闭环；也不调用旧版释放后端。

源文件 SHA-256、权重 SHA-256、RoboOS 固定提交、Git HEAD 及未提交状态在正式采集前保存到
`frozen_protocol.json`，另有 seal 防止意外修改协议。批次内每轮检查源码漂移；发生漂移便停止。
Git HEAD 不足以复现未提交代码，因此必须同时保存哈希并最终归档对应源码版本。

## 指令与可控失败

同义指令为：“麻烦将架子上的那件商品取下来，运到交付桌，松开夹爪放稳后再确认结果。”
它没有直接给出 `thesis_item_01`、`pickup_zone`、`delivery_zone` 等工具 ID，也不是默认任务原文。
但 Worker 提示明确提供了合法技能顺序，Master 任务字段受 schema 约束为原指令；因此成功只能
说明在这些约束下的指令适配和工具调度，不证明自由任务分解或任意自然语言泛化。

故障组由实验器拦截第一次真实、新鲜的相机响应，将该帧丢弃，向真实 Worker 返回明确注明
“实验注入、过期、未发送运动”的拒绝反馈。不会伪造传感器故障，不修改 ROS 数据或安全门控。
模型可以重新选 `observe_scene` 获得新帧，或在预算内安全结束；不能使用被丢弃的图像继续运动。
实际选择与后续结果必须由 `model_trace.json`、`tool_trace.json` 判断，不能由注入代码推定。

## 负面验收 D5

确定性测试调用本机已打补丁的上游 `ToolCallingAgent`，不是另造一个测试 Worker。

| 情形 | 期望与证据范围 |
|---|---|
| 无工具／文本宣称完成 | 不产生成功；实际 Worker 测试 |
| 重复只读 | 继续反馈但不当成完成；实际 Worker 测试 |
| 重复运动 | 第二次执行前拒绝，不另发运动；Worker＋契约测试 |
| 非法 JSON／未知工具／额外参数 | 解析或白名单失败，执行前停止；Worker／入口测试 |
| 过期／缺失相机 | 不进入有效观测状态，不发送后续运动；契约及真实模型故障组 |
| 模型或技能服务失联 | 传播失败、无隐式重试；MockTransport／异常注入，不声称真实停服务演练 |
| 取消时网关失联 | 保存 cleanup 错误与未知状态，不能报告已停稳；MockTransport 测试 |
| 预算耗尽 | 明确 `not_completed`，无成功证据；实际 Worker 测试 |
| 取消／超时／共享控制权 | 已有实际技能终态记录＋回归测试；不等于整框架任意断电恢复认证 |

测试文件：`scripts/tests/test_roboos_verified_worker.py`、`test_openflex_runner_safety.py`、
`test_openflex_delivery_contract.py`、`test_openflex_runtime.py` 及新增评测／对照测试。
这一负面矩阵覆盖既定代码路径；真实模型服务或 Redis 在运动中掉线的整机演练仍属后续扩展。

## 采集与复现

先确保已有环境、模型、MCP、ROS 工作区可用；不要为了重现而同时启动旧任务编排。

```bash
cd /home/wgb/openflex_shop
python3 scripts/openflex_runtime.py status
python3 scripts/openflex_runtime.py start --enable-simulation-motion --gui
.venvs/roboos-runtime/bin/python scripts/evaluate_openflex_delivery.py freeze \
  --output local_dependencies/openflex_roboos/runs/NEW_UNIQUE_BATCH
.venvs/roboos-runtime/bin/python scripts/evaluate_openflex_delivery.py run \
  --output local_dependencies/openflex_roboos/runs/NEW_UNIQUE_BATCH \
  --execute-isolated-simulation
```

`run` 会执行三次实际仿真任务并重建各轮初始场景，不是只读命令；目录存在便不覆盖采集。
如需中止，用 `Ctrl+C`，必要时另开终端执行 `python3 scripts/openflex_runtime.py stop`。
停止请求不等于停稳证明；必须检查 safe-stop 记录，未知时禁止继续重置场景。

每轮保存初始状态、真实工具／模型历史、运动记录、资源样本、独立最终状态和 EGL 录像。
录像来自同时间戳 ROS 状态镜像，无第二个物理循环；只编码新鲜状态，间隙记入 `frames.jsonl`。
`manifest.json` 包含可解码／非黑帧检查；视频不单独证明成功，成功还要求独立完成验证。

首个批次 `evaluation_small_v1_20261111` 因错误地要求“首次查询前已有 TC-WAM 结果”而未就绪，
第一轮启动前失败、第二轮人工中止、固定组未采集；计划分母仍为 3，不抹除这批记录。
修复为等待真实停稳及目标记忆，将 TC-WAM 可用性保留给实际技能的服务／安全检查；
修正版另冻为 `evaluation_small_v1_1_20261111`，不在首批原目录覆盖重跑。

修正版采集结果以批次 `summary.json` 及各轮 `trial_result.json` 为准，完整结果见下表。
对应执行源码已另存 `frozen_sources.tar.gz`，其独立 manifest 记录压缩包 SHA-256；
归档只含冻结列表中的源码／配置，不含模型、虚拟环境、构建产物或密钥。

## 2026-11-11 回归结果

在对应依赖环境中运行：框架／MCP／契约 133 项通过；加载 ROS Humble 和 `install_v2`、
使用 recovery 环境运行 ROS／MuJoCo／控制相关 204 项通过，共 337 项。
JUnit 证据位于 `local_dependencies/openflex_roboos/runs/d5_regression_20261111_final/`。
曾直接用不完整的框架环境收集全部 ROS 测试，因未 source 工作区／缺 MuJoCo 发生收集错误；
随后按环境分组执行通过，不能把前次配置错误解释为实际机器人任务失败。

## 修正版已采集结果

批次：`local_dependencies/openflex_roboos/runs/evaluation_small_v1_1_20261111/`。

| 条件 | 实际调度 | 独立完成 | Worker 推理／工具 | 运动请求 | 执行耗时 | 视频 |
|---|---|---|---|---|---|---|
| 同义指令 | 真实 RoboOS＋GGUF | 通过 | 7／7 | 4 | 494.8 s | 988 帧，全可解码／非黑 |
| 首次观察过期 | 真实 RoboOS＋GGUF | 通过 | 8／8 | 4 | 532.4 s | 1049 帧，全可解码／非黑 |
| 固定顺序对照 | 同一 MCP／ROS 技能，无模型 | 通过 | 0／7 | 4 | 484.9 s | 966 帧，全可解码／非黑 |

第二轮原始反馈序列为：状态 → 被拒绝的观察 → 重新观察 → 取货导航 → 抓取 →
交付导航 → 放置 → 独立状态。第三次模型推理实际选择了重新观察，不是实验器替模型补调用。
多出来的是只读观察，没有额外运动；实际运动请求仍是四次，失败不换 ID 重试。
修正版计划 3 轮、实际采集 3 轮、独立完成 3 轮，无未采集项；各条件仅一次，不能称为可靠性成功率。
三轮共 12 个运动请求全部终态 succeeded 且 stopped_confirmed=true，最终无 safety_stop。
固定组的 master_worker_redis=false、worker_inferences=0，不把固定调用写成模型决策。
模型正常组比固定组多约 9.9 s，故障组比固定组多约 47.5 s；措辞、模型开销与规划耗时均有混杂，
这些差值只描述本次观测，不能证明模型比固定调度更优或更差。原失败批次仍单列保留，不能只展示修正版。

### 并行资源采样

| 条件 | 采样数 | 最低可用 RAM（MiB） | 最高已用 swap（MiB） | 整卡显存最高采样（MiB） |
|---|---|---|---|---|
| 同义指令 | 244 | 942.2 | 14676.0 | 5971 |
| 首次观察过期 | 261 | 1224.0 | 13400.0 | 5930 |
| 固定顺序 | 239 | 1350.3 | 13523.8 | 5917 |

来自各轮 resources.jsonl，约每 2 s 采样，不是精确峰值；swap 包含之前运行遗留页面，
显存是整卡总占用，不能归因于单个模型。固定组未推理但模型服务仍驻留。
本机确实完成任务，但接近内存限制，高 swap 不适合承诺低延迟或长期稳定运行。

### 证据交付

三段视频均在批次的 `<条件>/video/measured_state.mp4`，对应帧清单和验证 manifest 同目录。
它们是新鲜 ROS 实测状态的 EGL 只读镜像，不是桌面录像；固定视角有局部货架遮挡，
不能声称夹爪全程无遮挡。录像器由批次发 SIGINT 正常收尾，manifest 的 interrupted=true
表示录制停止方式，不表示机器人任务失败；有效性以解码、非黑、时效检查与独立终态分别判定。

交付目录：`local_dependencies/openflex_roboos/handoff/openflex_handoff_20261111/`，另有同名 tar.gz。
包含两批原始记录、三段有效视频、冻结执行源码、337 项 JUnit、资源汇总和交付文档，
每文件校验见 SHA256SUMS。不包含权重、虚拟环境、构建产物或密钥，不是脱离 Git 基线的离线安装包。

## 必须报告的局限

Oracle 物体身份／位姿，不是可靠视觉检测；右臂固定工作、左臂在仿真步后固定停姿；
比例导航非 Nav2；夹持几何代理非力传感；碰撞为采样与运行时接触检查，非连续安全证明；
单商品／固定初始条件／一次每条件／高 swap；无实机、无模型训练、无任意场景泛化结论。
