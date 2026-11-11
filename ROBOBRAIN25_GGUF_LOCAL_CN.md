# RoboBrain2.5-4B GGUF 本机推理实测与启动

更新日期：2026-11-09。适用当前 Ubuntu 22.04 x86_64、RTX 4060 8GB 工作站。

后续接口进度：RoboOS 模型客户端与 GGUF 的 9 项真实端点检查和 9 项回归测试已通过，见 `ROBOOS_GGUF_INTEGRATION_CN.md`；完整框架及真实机器人技能尚未接入。本页以下保留模型初次部署的实测记录。

## 1. 本轮已经完成什么

- 已验证用户下载的 Q4_K_M 主模型、F16 视觉组件与 Hugging Face LFS SHA256 完全一致。
- 已将权重复制到项目忽略目录，保留下载目录中的原文件，不再依赖下载目录软链接。
- 已安装项目内独立的 llama.cpp CUDA 推理程序，真实使用 RTX 4060 加载并提供聊天服务。
- 已通过中文文本、官方样例图像、项目超市截图、单个原生 `tool_calls` 返回检查。
- 没有安装依赖到 ROS／MuJoCo 环境，没有升级系统库，没有调用 ROS、MCP 或任何运动技能。

这是 B1 的 GGUF 路线基础验收及 B2 的接口冒烟检查，不是完整 RoboOS 接入或机器人任务成功。

## 2. 权重与推理程序

| 项目 | 固定内容 |
|---|---|
| GGUF 仓库 | `mradermacher/RoboBrain2.5-4B-GGUF`，第三方转换权重 |
| GGUF 仓库 revision | `4967565e14c91e2d14a762f1ec18b4f1a1703786` |
| 主模型 | `RoboBrain2.5-4B.Q4_K_M.gguf`，2,716,070,016 字节 |
| 主模型 SHA256 | `9d83cfe5f74148ec334caa5a0f02ebce9d18fab54f0f5c28436096e15d046108` |
| 视觉组件 | `RoboBrain2.5-4B.mmproj-f16.gguf`，836,180,288 字节 |
| 视觉组件 SHA256 | `7ef45bb8e9db278c6a0b2a1b6131e8fcc967e734ea424d2f617ee92b47218bdf` |
| 推理程序 | 官方 `ggml-org/llama.cpp`，`b12110`，commit 前缀 `cd959bae9` |
| GPU 运行库 | 官方发行包配套 CUDA 12.8；使用现有 NVIDIA 580.159.03 驱动 |
| 默认设置 | 4096 上下文、1 个推理槽、全层 GPU 卸载、Flash Attention、图像 1024 tokens、关闭 reasoning |

GGUF 的仓库 revision 是转换文件的版本，不能当成官方原精度权重的来源提交证明；本轮未重新验证原精度／bitsandbytes 路线。

文件布局：

```text
local_dependencies/
├── COLCON_IGNORE
├── models/RoboBrain2.5-4B-GGUF/
│   ├── RoboBrain2.5-4B.Q4_K_M.gguf
│   └── RoboBrain2.5-4B.mmproj-f16.gguf
└── robobrain25_gguf/
    ├── compat/            Ubuntu 官方 libc6、libstdc++6 解包，未安装到系统
    ├── downloads/         固定发行包和远端元数据
    ├── runtime/           llama.cpp 与 CUDA 运行库
    ├── runs/              每次验证的完整回答、耗时和 GPU 采样
    ├── deployment.json    本机版本与验证边界
    └── server-smoke.log   首轮服务加载及推理日志
```

预编译程序要求 GLIBC 2.38 及较新的 libstdc++，系统原有库不满足。采用 Ubuntu 官方 `libc6 2.39-0ubuntu8.9`、`libstdc++6 14.2.0-4ubuntu2~24.04.1` 的项目内副本，并通过私有加载器仅为该进程加载。没有执行 `sudo`、系统包安装、修改 `/usr/lib` 或全局 `LD_LIBRARY_PATH`。

启动脚本切换到二进制目录，使私有加载器下的 GPU／CPU 后端发现正常；显式要求 `CUDA0`，不把静默 CPU 回退当成 GPU 成功。

## 3. 启动和停止

在项目根目录的终端运行，保持终端打开：

```bash
cd /home/wgb/openflex_shop
bash scripts/run_robobrain25_gguf.sh
```

终端看到 `listening on http://127.0.0.1:4567` 后，可在本机浏览器打开 `http://127.0.0.1:4567`，聊天或上传图片。模型别名是 `robobrain2.5-4b-gguf`。

- 默认只监听回环地址，CORS 限制为本地主机；不是远程开放服务，也没有配置正式访问鉴权。
- 前台启动时按 `Ctrl+C` 停止。启动脚本使用文件锁，防止本套运行库重复加载或在运行时被安装程序覆盖。
- 本轮已通过临时用户级服务保持后台运行，不配置开机自启。停止后才能再次前台启动。

```bash
systemctl --user status openflex-robobrain25-gguf.service
systemctl --user stop openflex-robobrain25-gguf.service
```

后台日志为 `local_dependencies/robobrain25_gguf/server.log`。若希望之后重新以临时用户服务启动：

```bash
cd /home/wgb/openflex_shop
systemd-run --user --unit=openflex-robobrain25-gguf --collect \
  --service-type=exec --property="WorkingDirectory=$PWD" \
  /usr/bin/bash "$PWD/scripts/run_robobrain25_gguf.sh"
```

上述简化重启命令的日志可用 `journalctl --user -u openflex-robobrain25-gguf.service -f` 查看。不要同时启动前台和后台两个实例。模型服务占用约 4GB 显存，暂不使用时建议关闭，给现有仿真留出资源。

健康检查及模型列表：

```bash
curl --noproxy '*' http://127.0.0.1:4567/health
curl --noproxy '*' http://127.0.0.1:4567/v1/models
```

文本请求示例：

```bash
curl --noproxy '*' http://127.0.0.1:4567/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{"model":"robobrain2.5-4b-gguf","messages":[{"role":"user","content":"请用中文介绍机器人抓取失败后应该如何处理。"}],"max_tokens":256,"temperature":0,"stream":false}'
```

## 4. 重跑图文与工具接口检查

先启动服务，再在另一个终端执行：

```bash
cd /home/wgb/openflex_shop
python3 scripts/validate_robobrain25_gguf.py --tool-probe

python3 scripts/validate_robobrain25_gguf.py \
  --image supermarket_scene/mujoco/screenshots/v5_overview.png \
  --tool-probe
```

验证脚本仅使用 Python 标准库，不需要 PyTorch、Transformers 或安装 OpenAI SDK。它发送图片的 base64 内容，不依赖推理服务访问图片公网地址。

`--tool-probe` 只提供 `get_robot_state` 的 Schema，检查返回的工具名称及 JSON 参数；**不执行工具**。它不是已经接通机器人状态查询的证明。

### 本机实测

| 测试 | 结果 | 请求耗时 |
|---|---|---|
| 中文文本 | 正常输出，正常终止 | 约 0.91–0.98 秒 |
| 官方房屋样例图 | 返回房屋布局及主要物体描述 | 约 1.67 秒 |
| 项目超市截图 | 返回 3D 超市、货架、商品、格纹地面和绿色桌面等描述 | 约 1.41–1.76 秒 |
| 单工具选择 | 原生 `tool_calls`，`get_robot_state`，参数字符串 `"{}"` | 约 0.30–0.35 秒 |

服务进程在上述检查后的显存快照约 **4142 MiB**；包括最终复测，`nvidia-smi` 0.5 秒采样得到的整卡最高占用 **5012 MiB**，包含桌面及其他程序。该数值不是推理程序分配器的精确瞬时峰值，也不能直接当成与 MuJoCo 并行运行的预算保证。耗时是单次短回答，包含缓存影响，不是稳定吞吐基准。

证据位于：

- `local_dependencies/robobrain25_gguf/runs/20261109_194354_110449/result.json`
- `local_dependencies/robobrain25_gguf/runs/20261109_194435_132754/result.json`
- `local_dependencies/robobrain25_gguf/runs/20261109_195011_240900/result.json`

首轮 `20261109_194252_881793` 因输出 256-token 上限截断而标为失败，保留记录；之后限制描述长度、提高输出预算至 512，并将图像 token 设置为 1024 后复测通过。`smoke_passed` 仅说明接口返回了未截断的最终内容／预期工具调用，回答的细节准确性仍需人工核对和任务数据评测。

## 5. 安装复现与下一步

本机已经安装好，不必重复执行安装。需要在相同平台恢复环境时，停止服务后运行：

```bash
python3 scripts/setup_robobrain25_gguf.py --models-from /home/wgb/下载
```

安装脚本固定官方发行包和 Ubuntu 库包的 SHA256，下载到项目目录、解包并校验模型副本，不安装系统包。已下载归档完整时复用，不再下载完整原精度模型。若固定版本上游归档下架，需要人工更新并重新验证，不能关闭校验。

接下来仍需要完成：

1. RoboOS Master／Worker 的模型端点与模型别名配置、客户端真实请求兼容性验证。
2. 多轮聊天、工具结果回传、失败反馈、终止行为及导航／抓取／放置参数检查。
3. 实时相机图像和真实机器人状态接入，不把历史截图当作实时观察。
4. MCP／ROS 技能适配与安全条件验证，再进行 MuJoCo 同时运行的资源测试。

不能因为模型已能聊天、看图或输出工具名，就把 B2–B4、C 阶段或机器人取送任务标为完成。
