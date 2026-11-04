# OpenFlex 抓取模型环境兼容矩阵

锁定日期：2026-08-12。

## 共同基线

```text
OS: Ubuntu 22.04
Python: 3.10.12
GPU: NVIDIA GeForce RTX 3090, compute capability 8.6
NVIDIA driver: 595.84
CUDA driver compatibility reported by nvidia-smi: 13.2
PyTorch: 2.7.1+cu126
torchvision: 0.22.1+cu126
CUDA build toolkit for custom extensions: 12.6
```

驱动 595.84 可以运行 CUDA 12.6 用户态。Ubuntu 22.04 仓库中的
`nvidia-cuda-toolkit` 只有 11.5，不与所选 PyTorch CUDA 12.6 ABI 配套，禁止用于
PointNet2/KNN 构建。模型环境从 PyTorch 官方 `cu126` wheel 索引安装运行时；
GraspNet 环境另外安装 `nvidia-cuda-nvcc-cu12==12.6.85` 提供匹配编译器。

## Perception 环境

环境路径：

```text
/home/openflex/old/.venvs/openflex_perception
```

锁文件：`requirements-perception.lock.txt`。

关键版本：

```text
transformers 5.15.0
accelerate 1.14.0
huggingface-hub 1.27.0
tokenizers 0.22.2
safetensors 0.8.0
numpy 1.26.4
```

选择依据：Qwen ZIP 的 `config.json` 是 `model_type=qwen3_5`，导出时标注
`transformers_version=4.57.0.dev0`，其 README 明确要求最新 Transformers；稳定版
4.57.6 不包含 `qwen3_5`，不能使用。稳定版 5.15.0 已核验同时包含
`models/qwen3_5` 和 `models/sam2`，并要求 Python >= 3.10、PyTorch >= 2.5。
SAM2 资产导出时标注 4.56.0.dev0，5.15.0 继续提供其配置、模型、图像处理和
processor 实现。

运行约束：

```text
Qwen3.5-4B: BF16，不量化
SAM2.1 Hiera Small: BF16 autocast，不量化
本地模型加载: local_files_only=true
```

## GraspNet 环境

环境路径：

```text
/home/openflex/old/.venvs/openflex_graspnet
```

锁文件：`requirements-graspnet.lock.txt`。

GraspNetBaseline 推理只导入 `models/graspnet.py`、PointNet2 和 KNN，不导入训练
dataset 中遗留的 `torch._six`。PointNet2 与 KNN 必须在目标环境重新编译，不得复制
源机器 `.so`。构建前固定：

```bash
export TORCH_CUDA_ARCH_LIST=8.6
export CUDA_HOME="$(python -c 'from torch.utils.cpp_extension import CUDA_HOME; print(CUDA_HOME or "")')"
```

验收时必须确认 `CUDA_HOME` 非空、`nvcc --version` 为 12.6、
`torch.version.cuda` 为 12.6，并分别执行 PointNet2/KNN 的 CUDA 单元调用。

## ROS 隔离规则

两个模型环境只运行模型 worker。TaskExecutor、控制台、Web 和普通 ROS 节点继续使用
ROS Humble 系统 Python。模型 worker 通过显式解释器路径启动，并通过 source ROS
安装前缀获得 `rclpy`、`cv_bridge` 和生成的消息类型；不得把模型环境全局 source 到
控制台进程。

本矩阵只完成版本和 ABI 选型。实际安装、扩展编译、模型加载和显存验收分别属于
P4-05、P4-06、P4-08 和 P4-09。
