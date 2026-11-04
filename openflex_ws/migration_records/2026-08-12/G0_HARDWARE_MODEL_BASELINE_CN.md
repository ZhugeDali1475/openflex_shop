# G0 硬件与模型基线

采集时间：2026-08-12 09:33-09:36 CST。

## GPU

```text
GPU：NVIDIA GeForce RTX 3090，PCI 10de:2204
内核驱动：nvidia 595.84
nvidia-smi：成功
Driver Version：595.84
报告的 CUDA compatibility：13.2
显存：15 MiB / 24576 MiB
/dev/nvidia0、nvidiactl、nvidia-uvm、nvidia-modeset：存在
系统 Python：3.10.12
系统 Python torch：未安装
nvcc：未安装
```

包状态仍存在 `nvidia-driver-590` 与 `nvidia-driver-595` 两个元包，但实际 DKMS、库、工具和已加载模块均为 595.84。P4-01 必须先分析 590 元包依赖再决定是否清理，不能因为当前可用就忽略，也不能盲目卸载运行中的驱动。

## 当前 RealSense

`rs-enumerate-devices -s`：

```text
D435I 261722076016  firmware 5.15.1.55
D405  260522279851  firmware 5.15.1.55
D435I 261922077096  firmware 5.15.1.55
```

右腕标定 D405 `260522279851` 已连接；左腕标定 D405 `260522277324` 未连接，因此左腕 camera internal TF 换算当前被硬件条件阻塞。

## 模型资产

```text
Qwen3.5-4B.zip
  size: 7398012817
  sha256: 22cd8797874bd57f6a5788a3d11b20b761df3dde8fce871e1619d7f2b6710439

sam2.1-hiera-small/model.safetensors
  size: 184305280
  sha256: 0a4067b11ce1e23d5229203f11c718a823060d15a4b23fa2372a7d4b77cbbc60

graspnet-baseline/checkpoint-rs.tar
  size: 12468415
  sha256: 60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868
```

目标 `/home/openflex/old/openflex_models` 和 `/home/openflex/old/.venvs` 在基线时均不存在。模型尚未解压、加载或完成许可证/manifest 验收。
