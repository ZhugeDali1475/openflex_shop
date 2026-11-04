# P8 抓取调试可视化实施记录

时间：2026-08-13 10:30 CST

## 实现结果

- Detect 成功后发布同一冻结 RGB-D 帧的原图、SAM2 mask、叠图、商品/场景点云、商品位姿、框、相机、帧和时间戳。
- Pick 在 GraspNet 拟合后发布 object/pregrasp/grasp pose、夹爪宽度、分数、候选统计及 pregrasp-to-grasp 路径；规划结束另发 plan-only 状态。
- `/openflex_grasp/debug/*` 全部为只读显示 topic，使用 reliable + transient-local；不新增执行服务或控制入口。
- Web 点云使用最多 8000 个商品点和 12000 个场景点；内部 snapshot 和 GraspNet 请求不降采样。
- 比赛控制台抓取页采用左侧调试控制、右侧嵌入现有 Web 检查器；Web 后端晚启动时自动重连。
- `supermarket_fullbody_runtime.launch.py` 在 `start_grasp_backend=true` 时启动唯一 `127.0.0.1:8765` Web 服务。

## 安全边界

- 未修改正式 Detect/Pick Action 契约。
- debug 发布失败只写 warning，不反向改变 Detect/Pick 结果。
- `execute_grasp` 默认仍为 `false`；本次未发送关节、夹爪或底盘命令。
- 抓取范围继续只允许 `ARM_ONLY=7` 或 `ARM_WITH_LIFT=8`，不移动底盘。

## 验证证据

- `python3 -m py_compile` 与 `node --check web/app.js` 通过。
- `colcon build --symlink-install --packages-select openflex_grasp openflex_navigation_gui openarmx_task_executor`：3 包成功。
- 在隔离端口 8876、隔离测试 topic 注入 2x2 RGB/overlay/mask、商品/场景点云、object/pregrasp/grasp pose 和两点路径。
- HTTP API 返回三张图均为 200，点云各 2 点，path version 为 1 且包含 2 个 pose，状态 JSON 含三种位姿和 `execute=false`。
- 系统 QtWebEngine 离屏加载成功，生成 1440x1000 截图 `/tmp/openflex_grasp_visualizer.png`；画布像素非空，图像、点云、三组坐标轴和路径均可见。

## 尚未验收

当前真实 fullbody 进程在本次构建之前已启动，因此不会热加载新 topic/Web 节点。本次没有为显示功能强制重启真实控制链。下一次正常重启后须检测“水蜜桃”，现场核对真实原图、mask/叠图、深度点云和实际位姿；该项保留为 P8-12，未标记真机通过。
