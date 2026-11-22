# 换机备份与恢复指南（BACKUP / RESTORE）

本文件记录：哪些内容在 GitHub 上、哪些必须单独保存、以及换机后如何把可重新下载的部分还原。
生成时间：2026-11-22

---

## 一、已在 GitHub 上（不需要额外备份）

| 内容 | 位置 |
| --- | --- |
| 全部源码 / 脚本 / 文档 | `https://github.com/ZhugeDali1475/openflex_shop` (分支 `main`) |
| `openflex_maps/`（1526 个文件，159 MB） | 同上，已在仓库内 |
| `supermarket_scene`（submodule） | `https://github.com/ZhugeDali1475/supermarket_scene.git` (分支 `thesis-integration-20261104`) |
| `headeye/`、`openflex_calibration/`、`thesis/`、`supermarket_scene-backups/`、`openflex_mujoco-1.0_basic/` | 同上 |

恢复整个工程：

```bash
git clone --recurse-submodules https://github.com/ZhugeDali1475/openflex_shop.git
```

---

## 二、关键备份包（必须自己留一份）

| 项 | 值 |
| --- | --- |
| 文件 | `openflex_critical.tar.zst` |
| 压缩后大小 | 398 MB |
| 解压后大小 | 1.72 GB（5191 个文件） |
| SHA256 | `d9f89707463ed4c4d943359ea8c1017c356fde59e75a9258a07235dc6df5a943` |

包内内容（都是无法重新生成的）：

- `local_dependencies/openflex_roboos/` — 运行证据：`runs/`、`handoff/`、`navigation_operations.jsonl`、校准后的场景 XML
- `local_dependencies/openflex_thesis/` — 论文验证报告、文献整理、反例与优化数据
- `local_dependencies/review_metadata/` — 模型元数据（含 HF ModelScope revision / sha）
- `supermarket_scene/data/` — 自采货架图与 SKU 图（中文文件名）+ 数据库
- `supermarket_scene/shelf_inventory*.db*` — 被 submodule 忽略的数据库及历史备份
- `openflex_drivers/` — 离线 deb / whl 安装包
- `openflex_ws/local_dependencies/`（pick_ik）、`openflex_ws/.claude/`、`.vscode/`
- `openflex_mujoco-1.0_basic/openflex_integrated_robot.urdf`
- `openflex_ws/src/openflex_chassis/.../livox_ros_driver2/package.xml`

校验与解压（在仓库根目录执行，会按原相对路径还原）：

```bash
sha256sum -c SHA256SUMS.txt
cd ~/openflex_shop
zstd -d -c /path/to/openflex_critical.tar.zst | tar -xf -
```

重新打包：

```bash
cd ~/openflex_shop
tar -I 'zstd -3 -T0' -cf /path/to/openflex_critical.tar.zst \
  local_dependencies/openflex_roboos local_dependencies/openflex_thesis \
  local_dependencies/review_metadata supermarket_scene/data \
  supermarket_scene/shelf_inventory* openflex_drivers \
  openflex_ws/local_dependencies openflex_ws/.claude .vscode \
  openflex_mujoco-1.0_basic/openflex_integrated_robot.urdf \
  openflex_ws/src/openflex_chassis/hardware_sensor_layer/livox_ros_driver2/package.xml
```

> 另：`local_dependencies/` 整目录此前已单独压成 `local_dependencies.zip`（9.4 GB，12730 个文件），
> `openflex_drivers/` 压成 `openflex_drivers.zip`（33 MB）。两者内容完整，可一并保存。

---

## 三、可以重新下载的（约 22 GB，不必备份）

国内加速：`export HF_ENDPOINT=https://hf-mirror.com`

```bash
pip install -U "huggingface_hub[cli]"

# 推理模型
hf download Qwen/Qwen3.5-4B            --local-dir openflex_models/Qwen3.5-4B
hf download facebook/sam2.1-hiera-small --local-dir openflex_models/sam2.1-hiera-small
# graspnet-baseline 的 checkpoint-rs.tar 需从 graspnet/graspnet-baseline 官方渠道获取
#   -> openflex_models/graspnet-baseline/checkpoint-rs.tar

# 视觉模型（在 supermarket_scene 子模块内）
hf download facebook/dinov2-small                --local-dir supermarket_scene/vision/models/dinov2-small
hf download google/owlv2-large-patch14-ensemble  --local-dir supermarket_scene/vision/models/owlv2-large-patch14-ensemble

# RoboBrain2.5-4B（ModelScope 上也是 BAAI/RoboBrain2.5-4B）
hf download BAAI/RoboBrain2.5-4B --revision ec4b3bf5e969b48ad17df432b479c10402a0079b \
   --local-dir local_dependencies/models/RoboBrain2.5-4B
pip install modelscope
modelscope download --model BAAI/RoboBrain2.5-4B --local_dir local_dependencies/models/RoboBrain2.5-4B

# 量化版（GGUF）：mradermacher/RoboBrain2.5-4B-GGUF，revision 4967565e14c91e2d14a762f1ec18b4f1a1703786
#   RoboBrain2.5-4B.Q4_K_M.gguf      sha256 9d83cfe5f74148ec334caa5a0f02ebce9d18fab54f0f5c28436096e15d046108
#   RoboBrain2.5-4B.mmproj-f16.gguf  sha256 7ef45bb8e9db278c6a0b2a1b6131e8fcc967e734ea424d2f617ee92b47218bdf
#   -> local_dependencies/robobrain25_gguf/
```

其余可重下 / 可重克隆的内容：

| 目录 | 大小 | 来源 |
| --- | --- | --- |
| `local_dependencies/RoboBrain2.5/` | 27 M | `https://github.com/FlagOpen/RoboBrain2.5`（公开仓库） |
| `local_dependencies/RoboOS/` | 37 M | `https://github.com/FlagOpen/RoboOS`（公开仓库） |
| `local_dependencies/roboos_client/` | 54 M | FlagScale `3fc2037f90917227bd4aebabd9d7b330523f437c` + 打补丁 `scripts/patches/roboos-standalone-gguf.patch` |
| `local_dependencies/robobrain25_b1/` | 2.3 G | HF/ModelScope 缓存，见 `modelscope_files.json` / `download_modelscope.log` |
| `local_dependencies/robobrain25_gguf/llama.cpp-b12110/` | — | `ggml-org/llama.cpp` tag `b12110`（commit `cd959bae9`），CUDA 12.8 |
| `local_dependencies/roboos_runtime/redis` | 6.8 M | redis 运行时 |
| `.venvs/` | 13 G | `pip install -r` 重建 |
| `supermarket_scene/.venv/`、`web/node_modules/` | 5.8 G | `uv sync` / `npm install` |
| `build*/`、`install*/`、`log*/` | ~0.3 G | `colcon build` 重建 |

### 校验时注意

`openflex_models/SHA256SUMS` 里记录的是旧机器的绝对路径
（`/home/openflex/old/openflex_models/...`）。比对时要么换到本机原路径，要么用
`sha256sum -c` 前先修正前缀，否则会全部报 missing。

```bash
cd openflex_models && sed 's#/home/openflex/old/##' SHA256SUMS > /tmp/sums.local && sha256sum -c /tmp/sums.local
```
