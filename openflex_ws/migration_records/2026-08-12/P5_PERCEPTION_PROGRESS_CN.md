# P5 感知重构执行记录

## 执行范围

负责人在 2026-08-12 要求加速执行，跳过非必要测试和仿真。本阶段采用“主链实现 +
快速契约/构建检查”，不运行机械臂、夹爪、底盘或耗时仿真。

## P5-01 核心模块边界

通过时间：2026-08-12 15:01 CST。

新增纯 Python 边界：

```text
openflex_grasp/perception/
  providers/          # 2D 框 provider
  sam2_segmenter.py   # SAM2 box -> mask
  rgbd.py             # 相机内参和冻结 RGB-D 帧
  masks.py            # mask 清理
  pointcloud.py       # RGB-D 投影、几何质量和实例选择
  cache.py            # TTL 基础缓存；P5-09 扩展 detection ownership
  visualization.py    # overlay
  io.py               # 离线文件输出
  pipeline.py         # 迁移期兼容组合层
```

`node.py` 和 `offline.py` 已直接依赖上述模块；任务控制与 MoveIt 执行代码不导入
Qwen、SAM2 或 Transformers。原 `models.py`、`geometry.py` 只保留兼容重导出，不再拥有
模型/几何实现。

快速验收：

```text
python compileall: PASS
colcon build --packages-select openflex_grasp --symlink-install: PASS
边界/几何/grounding 测试: 8 passed
RGB-D、点云、mask、cache 独立导入: PASS
```

测试期间发现冻结帧最初会把调用方原数组设为只读，已改为先复制再冻结；现在快照拥有
独立不可变数组，不影响调用方缓冲区。
