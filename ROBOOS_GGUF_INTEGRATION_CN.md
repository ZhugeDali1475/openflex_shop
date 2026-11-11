# RoboOS ↔ RoboBrain2.5 GGUF 客户端联调记录

更新日期：2026-11-09。

2026-11-10 补充：本文以下为早期客户端验收记录，不代表最新系统状态。实际完整 Master／Worker／Redis／MCP／ROS／MuJoCo
已完成一轮自然语言单商品取送，见 `OPENFLEX_ROBOOS_MOTION_CN.md` 与
`local_dependencies/openflex_roboos/runs/roboos_delivery_20261110_201218/result.json`。
仍不代表可靠视觉识别、实机验证或正式统计评测；早期“尚未启动完整框架”的表述只适用于本页原始验收日期。

## 1. 这一步完成的范围

已将本地 RoboOS 的 Master／Worker 模型配置指向同一服务：

```text
模型名称：robobrain2.5-4b-gguf
API 地址：http://127.0.0.1:4567/v1/
```

通过 RoboOS 原有的 `OpenAIServerModel` 和 `GlobalTaskPlanner.forward` 调用真实运行中的 GGUF 服务，不替换模型、不使用假模型输出。已验证中文技能选择、图片输入、多轮工具结果回传、失败后的安全下一步，以及 Master 子任务 JSON 的结构。

**这不等于完整 RoboOS 系统已经启动。** Master 规划器使用明确标记的机器人登记／场景测试数据，未连接真实 Redis 登记表；Worker 只使用模型类，没有连接 MCP、运行工具执行循环或控制机器人。后续只读桥接现已实现，见 `OPENFLEX_ROBOOS_READONLY_CN.md`；完整 Worker 尚未接入，不能用默认返回成功的 demo 技能冒充机器人。

## 2. 已修复的实际兼容问题

| 问题 | 处理 |
|---|---|
| Master／Worker 默认使用 RoboBrain2.0 路径 | 两者改用 `robobrain2.5-4b-gguf` 和本机回环 API |
| Worker 默认关闭原生工具调用 | 开启 `support_tool_calls`，避免原生 `tool_calls` 被当成文本重新解析后丢失 |
| MCP 工具转换缺少 `type`，Schema 写成根级 `input_schema` | 修正为 `type: function` 与 `function.parameters`；原格式曾导致真实服务 HTTP 500，修复后通过 |
| Worker 模型类只能构造单轮文本 | 增加可选 `messages` 参数，保留标准图片块、assistant 工具调用和 tool 回包；旧调用方式保持可用 |
| 默认输出预算固定为 8192 | 改为可配置 `max_tokens`，默认 512，匹配本机短上下文验证；截断或异常终止抛错，不当成成功 |
| 默认只筛选最多 3 个工具 | 改为最多 6 个、阈值 0，减少必要状态／观察技能被过滤的风险；未实测完整匹配器流程 |
| Master 默认清空 Redis DB | 改为 `clear: false`，不把清空已有状态作为启动默认行为 |
| Worker 仍指向 demo 机器人 | 改名 `openflex`，预留独立 MCP 地址 `127.0.0.1:18551`；该地址现有只读适配服务，但完整 Worker 尚未启动 |
| 环境含不被 HTTPX 接受的 `socks://` 代理变量 | 本地验证入口只在子进程内移除代理变量，并使用隔离 Python 模式；不改系统代理或用户配置 |

上游源码固定为 `1c39b85ef5c9dcb9d5da235665c19af21824e545`。因为 `local_dependencies` 被项目 Git 忽略，必要修改另存为可复现补丁：

```text
scripts/patches/roboos-standalone-gguf.patch
```

补丁包含 `master/config.yaml`、`slaver/config.yaml`、`slaver/agents/models.py`、`slaver/run.py` 四个文件。安装入口先检查固定提交，再检查能否应用／是否已经应用；冲突时停止，不覆盖其他修改。

## 3. 独立环境与依赖

客户端环境为 `.venvs/roboos-client`，使用 Python 3.10。依赖固定在 `scripts/requirements-roboos-client.txt`，包含与 RoboOS 一致的 `openai==1.75.0`、`PyYAML==6.0.2`、`redis==5.2.1`，以及 HTTPX、JSON Schema 和测试依赖。

FlagScale 按 RoboOS 文档固定提交 `3fc2037f90917227bd4aebabd9d7b330523f437c` 下载源码归档，通过本地路径导入真实协作组件；未执行其 `setup.py`，没有安装 FlagScale 的训练／推理后端或拉取子模块。

源码归档 SHA256 为 `7614831d618032101ebcfdd65732120c952a09d5bb92faccc75d4e4abd2ac2ba`。

系统缺少 `ensurepip`，因此使用 `venv --without-pip` 加固定校验和的官方 pip zipapp 引导环境，不要求 sudo 或系统升级。`pip.pyz` 本轮校验和为 `be510f6d23d345eeabdd3eec2599a3ca2df3cbc98c4afbe2631561ca12e744d7`；上游文件变化时应人工复核，不可关闭校验。

## 4. 如何复测

确认 GGUF 模型服务正在运行，然后在项目根目录执行：

```bash
bash scripts/run_roboos_gguf_validation.sh
```

报告保存到 `local_dependencies/robobrain25_gguf/runs/roboos_时间戳/result.json`。

需要恢复独立客户端环境或应用补丁时：

```bash
python3 scripts/setup_roboos_gguf_client.py
```

安装入口只准备模型客户端依赖和补丁，不安装整套 RoboOS requirements、不启动 Redis／Master／Worker，也不调用机器人。

纯接口回归测试不需要模型服务或显卡：

```bash
.venvs/roboos-client/bin/python -I -m pytest -q \
  scripts/tests/test_roboos_gguf_integration.py
```

## 5. 实测结果

最终端点复测记录：`local_dependencies/robobrain25_gguf/runs/roboos_20261109_202330_244249/result.json`。

| 检查 | 验证结果 |
|---|---|
| Worker 原生工具调用 | 返回 `get_robot_state`，参数为有效 JSON |
| 多轮工具结果回传 | 接收 assistant 工具调用与 tool 回包后返回最终内容 |
| Worker 图像输入 | 通过 RoboOS 模型类识别项目历史超市截图，不是实时相机 |
| 导航技能选择 | 返回 `navigate_to_target`，`target_id=pickup_zone` |
| 抓取技能选择 | 返回 `grasp_object`，`object_id=thesis_item_01` |
| 放置技能选择 | 返回 `place_to_affordance`，`affordance_id=delivery_zone` |
| 抓取失败反馈 | 返回 `observe_scene`，没有直接交付或将失败算作成功 |
| 执行结果未知 | 返回 `get_robot_state`，没有假设超时动作已经完成 |
| Master 子任务 JSON | 返回可解析结构，机器人名、非空子任务和正整数顺序字段有效 |

9 项真实端点检查通过。另有 9 项回归测试通过，覆盖参数与调用 ID 保留、图片／工具历史透传、usage、输出预算、异常终止拒绝、实际 MCP Schema 转换，以及非法工具／非法或错误站点参数拒绝。

这些检查是有限的契约冒烟测试，不是任务成功率评测。`pickup_zone`、`delivery_zone` 是此轮测试登记项，尚未映射实际 ROS 位姿，不应直接用于运动；所有反馈和机器人登记信息均为明确标记的测试输入，未伪装成真实传感器数据。JSON 结构正确也不代表每条规划理由、顺序或安全判断都正确，实际调度仍需约束和验证。

本轮早期代理错误、旧工具格式 HTTP 500、Master 代码围栏解析失败等记录保留。早期脚本曾临时使用替身模块验证导入，当前版本已改为真实固定版 FlagScale 组件，最终记录以前述报告为准。状态 `passed` 不代表 Redis 或实际机器人闭环已经通过。

## 6. 接下来接什么

下一步为 C1／C2：建立轻量 OpenFlex MCP／ROS 适配层，**先实现真实只读状态和相机观察**，核对时间戳、状态新鲜度和商品／站点登记表。

后续才启动单 Master／单 Worker，验证真实状态反馈；导航、抓取、放置执行接入及与 MuJoCo 同时运行的 GPU 资源测试仍未完成。RoboOS 默认执行循环存在“无工具调用即完成”、基于模型预测更新环境等边界，完整联调前需要以物理证据和真实结果替代，不能直接把默认循环当成安全的机器人执行器。
