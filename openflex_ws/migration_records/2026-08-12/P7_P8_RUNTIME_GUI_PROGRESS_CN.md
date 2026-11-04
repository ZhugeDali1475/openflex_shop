# P7/P8 运行时与比赛控制台记录

## 启动所有权

`grasp_debug.launch.py` 和 `supermarket_fullbody_runtime.launch.py` 都可能提供同名
Detect、GraspNet、Pick backend。核心节点现在分别取得：

```text
/tmp/openflex_detect_object.lock
/tmp/openflex_graspnet_worker.lock
/tmp/openflex_pick_object.lock
```

锁是进程持有的非阻塞 advisory lock。第二套 launch 会在节点初始化阶段失败并打印
“backend owner ... already held”，不会创建重复 Action/Service owner。可用
`OPENFLEX_BACKEND_LOCK_DIR` 将锁目录改到受控运行目录。正常进程退出后锁由 close/内核释放，
锁文件本身可复用，不是运行状态数据库。

## 启动入口

- 调试入口：`ros2 launch openflex_grasp grasp_debug.launch.py start_cameras:=false execute:=false`
- 比赛入口：`ros2 launch openarmx_task_executor supermarket_fullbody_runtime.launch.py start_grasp_backend:=true execute_grasp:=false`
- 两者不能同时启动 backend；比赛 owner 应优先使用 fullbody 入口。
- 相机默认关闭，serial 通过 `OPENFLEX_RIGHT_CAMERA_SERIAL`、`OPENFLEX_LEFT_CAMERA_SERIAL` 或 launch 参数注入。

## GUI

比赛控制台新增“抓取调试”页，显示 Detect/Pick Action 状态、双臂、SKU 中文名、可选
OWLv2 英文提示、Qwen/OWLv2 provider 和 DINO 复核开关。Qwen 失败直接报错；DINO 仅
OWLv2 路线可用。当前按钮是“仅检测”和“仅规划”，执行勾选被锁定为关闭，Place 状态
明确显示为 mock/未实现。

正式 `/execute_transport` 仍由 `openarmx_task_executor` 所有；本页不伪造 transport
成功，也不把 `competition_mock_pick` 当成真实 Pick。

## 未完成项

- P7-03 已完成软件级 owner guard；仍需现场启动验证，故 G7 保持未通过。
- P8-05/P8-06 的正式任务参数锁和真实 transport client 尚未接入；调试页仅提供独立
  Detect/Pick client，待比赛任务字段与负责人确认后再接线。
- P8-08 GUI 自动测试依负责人要求跳过。
