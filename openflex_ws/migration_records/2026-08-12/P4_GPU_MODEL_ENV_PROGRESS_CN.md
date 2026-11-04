# P4 GPU 与模型环境执行记录

## P4-01 GPU 驱动基线

检查时间：2026-08-12 10:50-10:51 CST。

当前运行状态：

```text
GPU: NVIDIA GeForce RTX 3090
Driver Version: 595.84
CUDA compatibility reported by driver: 13.2
VRAM: 24576 MiB
loaded nvidia module: 595.84
DKMS: nvidia/595.84 for 6.8.0-136-generic installed
Secure Boot: disabled
/dev/nvidia0, nvidiactl, nvidia-uvm, nvidia-uvm-tools, nvidia-modeset: present
```

已安装的 NVIDIA 二进制库、DKMS、kernel source、utils 和 Xorg driver 均为 595.84。APT 对旧名称的解释为：

```text
nvidia-driver-590 590.48.01-0ubuntu0.22.04.4
  Transitional package for upgrades. This package can be safely removed.
  Depends: nvidia-driver-595
  Installed-Size: 37 KiB
```

`dpkg -L nvidia-driver-590` 只列出包文档，不含内核模块、库或工具。因此基线中所谓“590/595 混装”实际是遗留过渡元包与 595 正式包并存，不是两套运行驱动混装。

卸载模拟：

```text
apt-get -s remove nvidia-driver-590
  Remove: nvidia-driver-590 only
  Upgrade: 0
  Install: 0
```

为防止 595 在移除唯一手动元包后被 APT 视为可自动清理，计划按顺序执行：

```bash
sudo apt-mark manual nvidia-driver-595
sudo apt-get remove -y nvidia-driver-590
```

禁止执行 `apt autoremove`。该操作不需要卸载模块或重启。

实际执行在第一条 sudo 检查前被阻塞：

```text
sudo: 需要密码
```

因此没有发生任何系统包改动。P4-01 保持未勾选，P4-02 至 G4 按清单暂停；获得 sudo 授权后需执行上述精确命令，并再次保存 `dpkg-query`、`dkms status`、设备节点和 `nvidia-smi` 结果才能通过。

### P4-01 恢复执行并通过

通过时间：2026-08-12 11:08 CST。

用户完成过渡包清理后复核：

```text
nvidia-driver-590: 未安装
nvidia-driver-595: 595.84，install ok installed，manual
nvidia-dkms-595: 595.84，install ok installed
loaded nvidia module: 595.84
DKMS: nvidia/595.84 for 6.8.0-136-generic installed
nvidia-smi: RTX 3090, Driver 595.84, 24576 MiB
/dev/nvidia0, nvidiactl, nvidia-uvm, nvidia-uvm-tools, nvidia-modeset: 全部存在
```

`apt-get -s autoremove` 不会移除 595 驱动栈，只列出一项无关的旧 580 firmware。未执行 `autoremove`，也不需要重启。P4-01 正式通过，可以继续 P4-02。

## P4-02 CUDA/PyTorch 兼容矩阵

通过时间：2026-08-12 11:20 CST。

锁定组合：

```text
Python 3.10.12
NVIDIA driver 595.84
CUDA runtime/toolkit 12.6
torch 2.7.1+cu126
torchvision 0.22.1+cu126
transformers 5.15.0
RTX 3090 extension architecture: 8.6
```

核验事实：

- PyTorch 官方 `cu126` 索引同时提供 Python 3.10 的 `torch 2.7.1+cu126` 和 `torchvision 0.22.1+cu126`。
- Qwen3.5-4B 配置为 `model_type=qwen3_5`，模型 README 要求最新 Transformers。稳定版 4.57.6 未包含该模型，不能使用。
- 稳定版 Transformers 5.15.0 的上游目录同时包含 `qwen3_5` 与 `sam2`，Python 下限为 3.10，PyTorch 下限为 2.5。
- GraspNet 的活动推理路径不导入旧 dataset 中的 `torch._six`；PointNet2/KNN 使用 ATen/CUDA extension，必须针对所选 torch/CUDA 现场重编。
- Ubuntu 22.04 的 `nvidia-cuda-toolkit` 候选仅为 11.5.1，禁止与 cu126 PyTorch 混合构建。GraspNet 环境锁 `nvidia-cuda-nvcc-cu12==12.6.85`。

新增锁定文件：

```text
src/openflex_grasp/environment/requirements-perception.lock.txt
src/openflex_grasp/environment/requirements-graspnet.lock.txt
src/openflex_grasp/environment/COMPATIBILITY_MATRIX_CN.md
```

P4-02 仅确认版本与 ABI 方案；实际安装和模型加载仍由 P4-05/P4-06/P4-09 验收。

安装前解析修正：`tokenizers 0.23.0` 在 PyPI 只有候选版、没有正式 wheel；
Transformers 5.15.0 接受 `>=0.22.0, <=0.23.0`，因此锁定改为正式稳定版
`tokenizers 0.22.2`。首次 P4-05 pip 运行在依赖解析阶段停止，未安装模型包。

## P4-03 Qwen3.5-4B 资产校验与解压

通过时间：2026-08-12 11:27 CST。

源资产：

```text
/home/openflex/桌面/openflex_all_gzk/openflex_all/openflex_models/Qwen3.5-4B.zip
size: 7,398,012,817 bytes
sha256: 22cd8797874bd57f6a5788a3d11b20b761df3dde8fce871e1619d7f2b6710439
```

完整性与安全检查：

```text
unzip -t: No errors detected in compressed data
成员路径：全部以 Qwen3.5-4B/ 开头
绝对路径：0
.. 路径穿越：0
```

目标目录：

```text
/home/openflex/old/openflex_models/Qwen3.5-4B
```

目标校验：

```text
文件数：15
权重分片：model.safetensors-00001-of-00002.safetensors (5,329,398,688 bytes)
          model.safetensors-00002-of-00002.safetensors (3,990,429,408 bytes)
索引权重条目：738
索引引用分片集合：与实际两个分片完全一致
safetensors 头部：87 + 651 tensors，JSON 均可解析
config.model_type: qwen3_5
目标目录大小：8.8G
```

逐文件 SHA256 已保存到：

```text
/tmp/Qwen3.5-4B.SHA256SUMS
```

源 ZIP 未移动、未删除，目标目录此前不存在，因此没有覆盖未知用户文件。

## P4-04 SAM2 与 GraspNet 资产部署

通过时间：2026-08-12 11:39 CST。

部署位置：

```text
/home/openflex/old/openflex_models/sam2.1-hiera-small/
/home/openflex/old/openflex_models/graspnet-baseline/checkpoint-rs.tar
```

资产证据：

```text
SAM2 model.safetensors: 184,305,280 bytes
  sha256=0a4067b11ce1e23d5229203f11c718a823060d15a4b23fa2372a7d4b77cbbc60
GraspNet checkpoint-rs.tar: 12,468,415 bytes
  sha256=60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868
```

SAM2 配置、processor、video 配置、Hiera YAML、README 和许可证均已复制；GraspNet
保留 checkpoint、README 和许可证。源/目标模型权重使用 `cmp` 逐字节一致。

统一 manifest：

```text
/home/openflex/old/openflex_models/SHA256SUMS
27 个文件全部 sha256sum -c 成功
```

本阶段只复制模型资产；代码中旧 `Qwen3.5-9B`、源 workspace 文档路径由 P4-07 处理。
活动后端没有 AnyGrasp SDK、包或模型引用。

## P4-05 perception 隔离环境

通过时间：2026-08-12 14:02 CST。

环境与锁定结果：

```text
/home/openflex/old/.venvs/openflex_perception
Python 3.10.12
torch 2.7.1+cu126
torchvision 0.22.1+cu126
torch CUDA runtime 12.6
transformers 5.15.0
GPU NVIDIA GeForce RTX 3090, compute capability 8.6
BF16 supported: true
pip check: No broken requirements found
```

安装采用清华 PyPI 主源和 PyTorch cu126 官方额外源。官方 Triton wheel 直连较慢，
使用机器已有的 `127.0.0.1:7897` 代理通过可续传的 `curl` 下载到：

```text
/home/openflex/old/.cache/openflex_wheels/
triton-3.3.1-cp310-cp310-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl
```

NVIDIA wheel 经全局代理存在 TLS 握手超时，因此未把代理永久写入环境，也未改变模型
运行时网络配置。普通依赖使用国内源，PyTorch CUDA ABI 仍严格使用官方 cu126 wheel。

`pip check` 最初报告 ROS Humble 的 `generate-parameter-library-py 0.7.4` 缺少
`typeguard`。已在 perception lock 中补充与 Ubuntu 22.04 系统版本一致的
`typeguard==2.2.2`，安装后依赖检查通过。

ROS 导入验收覆盖：

```text
rclpy, cv_bridge
sensor_msgs/{Image,PointCloud2,CameraInfo}
geometry_msgs/PoseStamped
openarmx_task_msgs/{DetectObject,PickObject,CompetitionWorkItem,TaskError}
openarmx_integrated_moveit_msgs/{CartesianApproach,PlanPregrasp}
```

验收时发现当前 `install/openarmx_integrated_moveit_msgs` 仍是未包含
`CartesianApproach.action` 的陈旧生成物，而源码/CMake 已包含该 Action。只对消息包执行
`colcon build --packages-select openarmx_integrated_moveit_msgs --symlink-install` 后，三套
MoveIt Action 的 Python/C/C++ typesupport 均重新生成，目标消息导入成功。该操作不启动
MoveIt、不连接控制器、不产生真机运动。

离线模型加载使用：

```text
HF_HUB_OFFLINE=1
TRANSFORMERS_OFFLINE=1
local_files_only=True
dtype=torch.bfloat16
```

结果：

```text
SAM2_LOAD_OK: model_type=sam2_video, Sam2Processor, cuda:0, BF16
SAM2 allocated VRAM: 74.4 MiB

QWEN_LOAD_OK: model_type=qwen3_5,
              Qwen3_5ForConditionalGeneration/Qwen3VLProcessor,
              cuda:0, BF16
Qwen allocated VRAM: 8658.0 MiB; reserved 8668.0 MiB
```

SAM2 加载时 Transformers 提示 checkpoint 配置为 `sam2_video` 而入口类为
`Sam2Model`。357 个权重均成功加载，配置和处理器可用；后续 P4-09 仍需通过固定样例的
实际 mask 推理验证该兼容路径，P4-05 仅确认离线加载。

日志：

```text
/tmp/openflex_p4_perception_bootstrap.log
/tmp/openflex_p4_perception_install.log
/tmp/openflex_p4_msgs_rebuild.log
```

## P4-06 GraspNet 隔离环境

通过时间：2026-08-12 14:35 CST。

环境与依赖结果：

```text
/home/openflex/old/.venvs/openflex_graspnet
include-system-site-packages = false
Python 3.10.12
torch 2.7.1+cu126
torchvision 0.22.1+cu126
torch CUDA runtime 12.6
GPU NVIDIA GeForce RTX 3090, compute capability 8.6
numpy 1.26.4
scipy 1.15.3
open3d 0.19.0
pip check: No broken requirements found
```

安装使用清华 PyPI 主源和 PyTorch 官方 cu126 额外源，447.7 MB 的 Open3D wheel
下载完成后由 pip 缓存复用。本机代理仅用于必要的 NVIDIA 官方 CUDA 编译组件下载，
未永久写入任何运行环境。

`graspnetAPI 1.2.11` 的上游元数据硬编码 `numpy==1.20.3`，该版本没有 Python 3.10
wheel，也与本环境的 torch/scipy/Open3D 组合不兼容。生产入口
`openflex_grasp.graspnet_backend.GraspNetBackend` 直接使用仓库内的 `GraspNet` 和
`pred_decode`，不导入 `graspnetAPI`；后者只存在于上游独立 `demo.py/test.py`。因此将其
从 runtime lock 排除，并在锁文件中留下原因，避免通过 `--no-deps` 隐藏不完整依赖。
ROS Humble 的 `generate-parameter-library-py` 所需 `typeguard==2.2.2` 已显式补锁。

PyPI 的 `nvidia-cuda-nvcc-cu12` wheel 只有 `ptxas` 和 NVVM 组件，没有 `nvcc`
驱动程序。为满足“现场重编”要求，从 NVIDIA Ubuntu 22.04 官方仓库下载并校验 CUDA
12.6.85/12.6.77 的 nvcc、NVVM、CRT、CUDART headers/runtime、CCCL 和 driver headers，
只解包到用户目录：

```text
/home/openflex/old/.cuda/cuda-12.6-local/usr/local/cuda-12.6
nvcc release 12.6, V12.6.85
```

没有执行 apt 安装、没有改变 595.84 驱动，也没有复制任何旧扩展。编译固定：

```text
CUDA_HOME=/home/openflex/old/.cuda/cuda-12.6-local/usr/local/cuda-12.6
TORCH_CUDA_ARCH_LIST=8.6
MAX_JOBS=4
```

PointNet2 和 KNN 编译命令均产生：

```text
-gencode=arch=compute_86,code=sm_86
```

安装产物：

```text
/home/openflex/old/.venvs/openflex_graspnet/lib/python3.10/site-packages/
  pointnet2-0.0.0-py3.10-linux-x86_64.egg/pointnet2/_ext.cpython-310-x86_64-linux-gnu.so
  knn_pytorch-0.1-py3.10-linux-x86_64.egg/knn_pytorch/knn_pytorch.cpython-310-x86_64-linux-gnu.so
```

两个 ELF 均包含 CUDA fatbin 和 `sm_86` 标记。CUDA 单元测试覆盖 PointNet2 的
furthest-point sampling/gather 和 KNN 最近邻核，结果：

```text
fps_idx [[0, 2, 1]]
gather_shape (1, 3, 3), finite true
knn_indices [[[1, 3]]]
CUDA_EXTENSION_UNIT_TEST=PASS
```

正式 checkpoint 烟测：

```text
/home/openflex/old/openflex_models/graspnet-baseline/checkpoint-rs.tar
size 12,468,415 bytes
epoch 18
synthetic scene: 22,000 points, network sample: 20,000 points
pred_decode output: (506, 17), float32, all finite
score range: 0.0008991694 .. 0.8027821779
width range: 0.03965798 .. 0.10000000 m
GRASPNET_CHECKPOINT_INFERENCE=PASS
```

日志：

```text
/tmp/openflex_p4_graspnet_install.log
/tmp/openflex_p4_pointnet2_build.log
/tmp/openflex_p4_knn_build.log
/tmp/openflex_p4_graspnet_checkpoint_smoke.log
```

## P4-07 解释器与模型参数固定

通过时间：2026-08-12 14:43 CST。

模型进程与普通 ROS 进程已经按职责分开：

```text
product_pointcloud_node/offline
  -> OPENFLEX_PERCEPTION_PYTHON
  -> default /home/openflex/old/.venvs/openflex_perception/bin/python

straight_grasp_node/offline
  -> OPENFLEX_GRASPNET_PYTHON
  -> default /home/openflex/old/.venvs/openflex_graspnet/bin/python

product_grasp_executor/product_moveit_bridge/product_grasp_web
  -> OPENFLEX_ROS_PYTHON
  -> default python3 (ROS 系统 Python)
```

路径默认值统一为：

```text
OPENFLEX_ROOT=/home/openflex/old
OPENFLEX_WORKSPACE=/home/openflex/old/openflex_ws
OPENFLEX_MODEL_ROOT=/home/openflex/old/openflex_models
OPENFLEX_QWEN_MODEL=$OPENFLEX_MODEL_ROOT/Qwen3.5-4B
OPENFLEX_SAM_MODEL=$OPENFLEX_MODEL_ROOT/sam2.1-hiera-small
OPENFLEX_GRASPNET_REPO=$OPENFLEX_WORKSPACE/src/openflex_grasp/graspnet-baseline
OPENFLEX_GRASPNET_CHECKPOINT=$OPENFLEX_MODEL_ROOT/graspnet-baseline/checkpoint-rs.tar
```

上述六个目标路径均存在。活动源码和重新生成的 `install/openflex_grasp` 中不存在
`Qwen3.5-9B`、`OPENFLEX_PRODUCT_SEG_PYTHON` 或旧源 workspace 默认路径。历史迁移说明中
保留一处旧 `install/` 路径作为“不得复制陈旧安装产物”的问题证据，不属于运行默认值。

系统 `/usr/bin/python3` 已验证可导入 ROS Humble 的 `rclpy` 与 `cv_bridge`。模型 venv
只承载模型 worker，不用于任务控制、Web 或 MoveIt bridge。

首次只重建 `openflex_grasp` 时发现目标 `install/` 缺少 `openflex_grasp_moveit` 的
`package.sh`。按依赖顺序执行：

```text
colcon build --packages-select openflex_grasp_moveit openflex_grasp --symlink-install
Summary: 2 packages finished
```

安装后的脚本内容与源码一致，ROS 包可枚举全部目标 executable。

日志：

```text
/tmp/openflex_p4_openflex_grasp_rebuild.log       # 首次依赖缺失诊断
/tmp/openflex_p4_grasp_packages_rebuild.log       # 依赖顺序重建通过
```

## P4-08 常驻显存策略

通过时间：2026-08-12 14:47 CST。

在完全离线模式下同时启动两个独立生产 worker：

```text
PID 85247  /home/openflex/old/.venvs/openflex_perception/bin/python
           Qwen3.5-4B BF16：1 个实例
           SAM2.1 Hiera Small BF16：1 个实例

PID 85466  /home/openflex/old/.venvs/openflex_graspnet/bin/python
           GraspNetBaseline FP32：1 个实例
```

两个进程均在完成推理后保持驻留。`nvidia-smi` 同时采样结果：

```text
GPU: NVIDIA GeForce RTX 3090
total: 24576 MiB
used: 11063 MiB
free: 13065 MiB

PID 85247: 9694 MiB
PID 85466: 1340 MiB
```

剩余显存 13065 MiB，明显超过清单要求的 2 GB。进程表只有上述两个模型 worker，
perception worker 内部明确只构造一套 Qwen 和一套 SAM2，GraspNet worker 只构造一套
GraspNet，没有第三个模型进程或重复模型实例。验收完成后通过 SIGINT 干净终止两个临时
worker；本项验证的是既定常驻部署策略，不要求让验收进程永久留在后台。

## P4-09 三模型同时加载烟测

通过时间：2026-08-12 14:47 CST。

感知固定样例：

```text
image: /home/openflex/old/supermarket_scene/test_pic/局部图片.png
image shape: 374 x 541 RGB
SKU 中文原名: 农夫山泉
owlv2_prompt: 空，因此没有拼接英文提示词
production class: openflex_grasp.models.ProductSegmenter
```

Qwen `locate_all()` 返回 6 个独立实例，像素框：

```text
[24,213,67,340]
[56,213,97,340]
[95,213,136,340]
[134,213,175,340]
[173,213,214,340]
[207,213,248,340]
```

6 个框的语义置信度均为 0.99。SAM2 `segment_box()` 对每个框均返回
`[374,541]` 布尔 mask，mask 像素数分别为 3756、3708、3587、3743、3698、3678，
SAM 分数范围为 0.9453125 至 0.96484375。结果结构可解析且 mask 全部非空。

GraspNet 固定样例：

```text
data: src/openflex_grasp/graspnet-baseline/doc/example_data/
production class: openflex_grasp.graspnet_backend.GraspNetBackend
valid scene points: 513688
network sample: 20000
checkpoint epoch: 18
output: (326, 17), float32, all finite
```

GraspNet 测试直接从官方 `depth.png`、`workspace_mask.png` 和 `meta.mat` 生成米制点云，
随后调用生产后端 `predict()`。Qwen、SAM2 和 GraspNet 在同一 GPU、两个隔离进程中同时
保持已加载状态，三者均已完成一次真实推理；没有 CUDA OOM、PointNet2/KNN 扩展错误或
不可解析输出。GraspNet 上游 `torch.cross` 发出未来版本弃用警告，不影响本次结果。

本关按此前确认决定不做缺失数据集的精度验收。精度验收保持后置，完整 20 轮
Detect/Pick 常驻联测仍由 P9-09 执行。
