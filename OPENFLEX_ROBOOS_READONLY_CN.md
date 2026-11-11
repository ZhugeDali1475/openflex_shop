# OpenFlex ↔ RoboBrain 只读 MCP／ROS 接入

更新日期：2026-11-10。本文说明默认只读接口；它仍只开放状态和观察。显式启用的隔离运动入口、相机修正及新验收见 `OPENFLEX_ROBOOS_MOTION_CN.md`。以下 2026-11-09 的验收表和停止服务状态为历史记录，不代表当前运行状态。

## 1. 实际实现

```text
RoboBrain GGUF API (127.0.0.1:4567)
        ↓ 原生工具选择（独立联调脚本，尚非完整 RoboOS Worker）
MCP Streamable HTTP (127.0.0.1:18551/mcp)
        ↓ 仅 GET /state、/observation
ROS 只读快照 (127.0.0.1:18552)
        ↓ 订阅已有消息／图像
现有 OpenFlex MuJoCo bridge
```

新增 ROS 包为 `openflex_ws/src/openflex_thesis/openflex_roboos_bridge`。ROS 进程使用系统 Python 3.10、Humble 和 `install_v2` 的自定义消息；MCP 使用独立 `.venvs/openflex-mcp`，不向 ROS 环境安装 Pydantic、NumPy 或 MCP SDK。

仅注册两个无参数工具：

| 工具 | 实际数据 |
|---|---|
| `get_robot_state` | 里程计、关节、抓取／放置证据、目标观测／记忆、控制模式、任务状态、TC-WAM 状态 |
| `observe_scene` | `/thesis_sim/head/color/image_raw` 的 JPEG 图像块及独立标记来源的目标观测／记忆 |

相机通过 MCP `ImageContent` 返回真实 JPEG 字节，不返回本机文件路径。支持 RGB/BGR/RGBA/BGRA/mono8、行填充检查及最大 640×480 缩放。本阶段不提供深度、点云、坐标变换或视觉定位。

快照进程没有运动发布器、ROS 服务客户端或 Action 客户端，保留 ROS 自带日志和参数事件发布器。两个 HTTP 服务只绑定回环地址，并检查 Host／Origin。接口无身份认证，不得通过端口转发暴露到不可信网络。

## 2. 真实性与过期规则

- 各来源保留 Header、时间戳、坐标系、接收年龄、源时间年龄和源时间推进年龄。
- 超过 3 秒、源时间在未来、时钟不可用、格式错误或从未收到时，数据返回 `null`；旧图不送给模型。
- 持续重发同一时间戳也会按单调墙钟失效，防止仿真时钟暂停后仍显示新鲜。重启／时间跳变时可能暂时返回未知。
- 所有状态保留 `physical_verification: false`。MuJoCo 目标位姿是 Oracle 真值，不是 RGB 检测结果，不能当作物理感知或抓取成功证明。
- `holding` 仅在新鲜抓取状态的验证、接触、闭爪、保留标志均为真时报告对象 ID，并注明仿真证据；否则为 `null`，不将未知改写成“空手”。
- 即使上游任务阶段报告 `SUCCEEDED`，也不自动生成任务级 `success: true`。
- 来源不齐时返回 `partial`。独立仿真未启动任务编排、物体记忆和仲裁节点，这些项显示缺失是预期行为，不伪造 idle／HOLD。

## 3. 安装与运行

环境已安装，重建方式：

```bash
/usr/bin/python3 scripts/setup_openflex_readonly.py
```

依赖全部固定在 `scripts/requirements-openflex-mcp.txt`。使用 MCP SDK `1.30.0`（v1 系列），不无版本限制地安装其他主版本。参考官方 v1 文档：<https://github.com/modelcontextprotocol/python-sdk/tree/v1.x>。安装复用前阶段已下载并校验的 pip zipapp，不改系统包。

两个终端分别运行：

```bash
# 终端一：只读接口；无仿真时也能启动，返回 unavailable
bash scripts/run_openflex_readonly_bridge.sh

# 终端二：仅启动现有 MuJoCo bridge，开启 RGB，不启动旧任务系统
bash scripts/run_openflex_readonly_sim.sh
```

两个入口默认使用专用 `ROS_DOMAIN_ID=83`，不会默认连接原来的 domain 0；如显式覆盖，两个进程必须一致。domain 隔离和重映射不是 ROS 安全认证机制。

仿真入口使用 EGL，场景写入忽略目录 `local_dependencies/openflex_roboos/readonly_scene.xml`。不启动旧 mission orchestrator、Nav2、MoveIt 执行器、脚本策略或自动任务客户端。MuJoCo 的命令话题、动作和修改场景服务重映射到独立禁用命名空间，但未删除其功能，不要调用这些接口。仿真仍有正常物理步进、重力和初始姿态保持，并非冻结所有关节。

脚本直接加载源码、使用已有 `install_v2` 消息，无需重建整个工作区。新包保留 ament_python 元数据及 `readonly_snapshot` 入口，MCP 依赖不由 colcon 安装。已单独通过 colcon 构建，构建／安装产物位于 `local_dependencies/openflex_roboos/colcon_*`，未覆盖原工作区构建产物。

## 4. 验证与实测证据

```bash
.venvs/openflex-mcp/bin/python -I -m pytest -q scripts/tests/test_openflex_readonly_bridge.py
.venvs/openflex-mcp/bin/python -I scripts/validate_openflex_readonly.py --expect-missing
.venvs/openflex-mcp/bin/python -I scripts/validate_openflex_readonly.py --expect-live --model
```

`--expect-missing` 在仿真停止、旧数据过期后运行；`--expect-live` 要求真实里程计、关节、抓取状态、目标观测和 RGB 新鲜。`--model` 额外调用 GGUF：先让模型从真实工具 Schema 选择 `observe_scene`，执行真实 MCP 工具，再把对应实时图像送给模型。给模型的反馈刻意不含 Oracle 目标坐标，避免把真值当视觉识别结果。验证脚本还检查未注册的导航工具被拒绝，不连接或调用导航 Action。

首次实测记录：

| 检查 | 结果／记录 |
|---|---|
| 单元测试 | 20 项通过：未知／过期、冻结时钟、无假成功、图像转换、工具白名单等 |
| 无仿真 | `local_dependencies/openflex_roboos/runs/20261109_204112_682839/result.json`，不可用、无图片、持物未知 |
| 真实 MuJoCo + MCP + GGUF | `local_dependencies/openflex_roboos/runs/20261109_204122_386562/result.json`，通过 |
| 相机 | 同目录 `camera.jpg`、模型输入 `model_camera.jpg`；320×240 JPEG，5476 字节 |
| 模型 | 原生 `observe_scene({})`，读图后正常返回描述 |
| 停止仿真 | `local_dependencies/openflex_roboos/runs/20261109_204220_775331/result.json`，旧状态失效、无旧图 |
| 原有模型客户端回归 | 9 项通过 |
| 锁存话题 QoS 修正后复测 | `local_dependencies/openflex_roboos/runs/20261109_204712_177740/result.json`，真实 MCP／ROS／GGUF 再次通过 |
| 最终停止仿真复测 | `local_dependencies/openflex_roboos/runs/20261109_204814_463680/result.json`，过期状态和图像不再返回 |
| 本机 HTTP 防护 | 非允许 Host：快照 403、MCP 421；快照跨站 Origin 403，未知路径 404 |

**历史视野限制（2026-11-09）：头部相机当时只看到灰色遮挡面、边缘及局部地板，未看到商品。** 2026-11-10 已修正误挂载位置并另存新旧对照证据，见运动接入文档。Oracle 目标数据仍不能代替视觉识别，模型读图也不等于抓取定位。

GGUF 与小尺寸 MuJoCo RGB 曾同时运行，单次 GPU 采样 5635 MiB；这不是完整导航／MoveIt 并行的显存峰值，B4 仍待验证。

## 5. 当前服务与后续边界

本次验证使用临时用户服务：

```bash
systemctl --user status openflex-roboos-readonly.service
systemctl --user stop openflex-roboos-readonly.service
systemctl --user stop openflex-roboos-readonly-sim.service
```

验证结束已停止仿真，保留只读 MCP 和先前 GGUF 服务。因此当前查询返回未知／不可用，不继续显示旧图。临时服务不设置开机自启。重启仿真可用终端入口，不要重复占用已有 MCP 端口。

C1 只读基础桥接完成。C2 和空载 C3 的新增证据见运动接入文档；默认两个只读工具的权限不变。完整 Redis／Master／Worker、图片反馈及自动完成判定尚未系统验收，不能宣布已能自主购物。
