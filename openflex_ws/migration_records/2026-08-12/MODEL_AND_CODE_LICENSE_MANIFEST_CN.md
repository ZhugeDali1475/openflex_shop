# 抓取迁移代码与模型许可证清单

采集时间：2026-08-12。本文记录随当前资产能直接核验的条款，不替代法律审查。

| 组件 | 版本/证据 | 声明许可证 | 本项目约束 |
| --- | --- | --- | --- |
| Qwen3.5-4B | ZIP SHA256 `22cd8797...10439`；ZIP 内 README 和 LICENSE | Apache-2.0 | 保留 LICENSE、模型来源和 SHA256 |
| SAM2.1 Hiera Small | `model.safetensors` SHA256 `0a4067b1...bbc60`；模型卡 front matter | Apache-2.0 | 当前资产目录没有独立 LICENSE 文件，部署时保留模型卡并补来源记录 |
| GraspNetBaseline | upstream `graspnet/graspnet-baseline` commit `280c215129f759ed8649cb4e89fc5dfee55f4f80` | Academic/non-profit organization noncommercial research use only | 仅用于当前比赛/非商业研发；禁止把它描述为宽松开源或默认可商用。商业使用、第三方分发或授权转移前必须联系权利人另行许可 |
| GraspNet `checkpoint-rs.tar` | SHA256 `60680087...a868`；上游 README 列为 RealSense pretrained weights | 与 GraspNet 项目相同的非商业使用说明 | 只在内部比赛环境部署，不进入 Git；保留来源与 SHA256 |
| `openflex_grasp` Python 包 | `package.xml` | Apache-2.0 | 自研部分按包声明管理；不能覆盖其调用的 GraspNet 上游限制 |
| `openflex_grasp_moveit` | `package.xml` | CC-BY-NC-SA-4.0 | 迁移/修改保持署名和非商业/相同方式共享限制，后续如需商用必须重新审查来源和授权 |
| `openflex_grasp_msgs` | `package.xml` | CC-BY-NC-SA-4.0 | 仅作 D1 过渡；重复 Action 迁入目标消息包时必须保留来源记录，不能因目标包许可证不同而静默改写权利状态 |

## Qwen 证据

```text
模型卡：license: apache-2.0
ZIP 内 LICENSE SHA256：50cbab8a892c5f2993b8c7351a99182507472def3b1374558308605d99b86b32
模型架构：Qwen3_5ForConditionalGeneration
配置 dtype：bfloat16
```

## SAM2 证据

```text
模型卡：license: apache-2.0
library_name: transformers
pipeline_tag: mask-generation
```

## GraspNet 关键限制原文摘要

上游 `LICENSE` 标题为：

```text
ACADEMIC OR NON-PROFIT ORGANIZATION NONCOMMERCIAL RESEARCH USE ONLY
```

上游 README 也说明 code/models 仅免费用于非商业使用，商业咨询需联系作者。本次删除 AnyGrasp 不代表 GraspNet 自动获得商业许可。

## 部署要求

- 模型目录保存 manifest、SHA256、来源 URL/commit、许可证副本或模型卡。
- 模型权重不放入 `openflex_ws/src` 和 Git。
- 不恢复 AnyGrasp SDK、授权文件或 checkpoint。
- 最终交付说明必须明确 GraspNet 的非商业限制。
