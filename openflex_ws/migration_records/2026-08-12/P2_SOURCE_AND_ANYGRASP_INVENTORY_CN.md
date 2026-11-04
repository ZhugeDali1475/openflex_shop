# P2 源码迁移与 AnyGrasp 引用清单

采集时间：2026-08-12 09:44 CST。

## P2-01 迁移文件分类

源目录排除 `.git`、`__pycache__`、`.pytest_cache`、`*.egg-info`、`*.pyc`、`*.so` 和 checkpoint 后，共 136 个有效源码/配置/文档文件。

### 直接迁入后继续重构

```text
openflex_grasp/
  package.xml, setup.py, setup.cfg, resource/
  openflex_grasp/*.py
  scripts/*
  launch/*.launch.py
  config/*.yaml
  web/*
  test/test_*.py
openflex_grasp_moveit/
  CMakeLists.txt, package.xml, launch/, src/
openflex_grasp_msgs/
  CMakeLists.txt, package.xml, action/
graspnet-baseline/
  LICENSE, README.md, requirements.txt
  models/, utils/, dataset/, pointnet2/, knn/
  demo.py, test.py, train.py, command_*.sh
MIGRATION_GUIDE_CN.md
```

`openflex_grasp_msgs` 只是过渡迁入，随后按 D1 把 `CartesianApproach` 移入目标 `openarmx_integrated_moveit_msgs`，所有调用方迁移完成后处置重复消息包。

### 转交模型资产阶段，不放入目标源码

```text
graspnet-baseline/checkpoint-rs.tar
```

该 checkpoint 的基线 SHA256 已记录在 `G0_HARDWARE_MODEL_BASELINE_CN.md`，P4-04 将其部署到 `/home/openflex/old/openflex_models`。

### 明确排除

```text
build/, install/, log/
__pycache__/, .pytest_cache/, *.pyc
graspnet-baseline/.git/
*.egg-info/
旧编译 *.so
任何虚拟环境
```

源目录没有 Git 父仓库；迁移前排除缓存的内容摘要为：

```text
1cfde9be5e798f6bbddb7f7e86cd2706bf48d0d80b582cebd91b19b99fc6f7ff
```

## P2-04 AnyGrasp 初始分类

全 `src` 共匹配 549 行、99 个文件，其中 16 个文件属于 `openflex_integrated`，83 个属于 `openflex_thesis`。

### 必须从比赛活动链删除或替换

- `src/openflex_thesis/openflex_anygrasp/` 整包
- `openflex_thesis_bringup/package.xml` 对 `openflex_anygrasp` 的运行依赖
- `openflex_thesis_bringup/launch/anygrasp_right_wrist_camera.launch.py`
- `openarmx_supermarket_sim/launch/competition_physics_stack.launch.py` 的 AnyGrasp worker/gateway
- `openarmx_supermarket_sim/config/anygrasp_sim.yaml`
- `openarmx_supermarket_sim/preflight.py` 的 SDK/checkpoint/Python 检查
- 比赛 station 配置的 `anygrasp`、`mock_or_anygrasp` 策略
- GUI 和当前比赛运行文档中把 AnyGrasp 写成正式 backend 的说明

### 保留但不得进入新比赛 launch

- `openflex_tc_wam`、`openflex_thesis_control`、`openflex_thesis_experiments` 和 thesis 文档中的论文历史算法代码
- `openflex_thesis_msgs` 中仍被其他 thesis 包使用的消息
- thesis 独立 gateway/mock 名称和实验协议，除非后续单独决定废弃整个 thesis 子系统

这些保留项不得成为新 fullbody、Detect、Pick、仿真 preflight 或 GUI 的依赖。P2-09 将为纯历史文本建立允许清单，并要求所有活动依赖扫描为零。

许可证核对见 `MODEL_AND_CODE_LICENSE_MANIFEST_CN.md`。重要结论是 GraspNetBaseline 代码和 checkpoint 限学术/非营利组织的非商业研究使用；删除 AnyGrasp 不会消除这一限制。

## P2-02 纯源码迁入结果

首次 dry-run 发现 `graspnet-baseline/knn/build` 和 `pointnet2/build` 中仍有旧 `.o/.ninja` 产物，因此在实际复制前补充排除任意层级 `build/`、`*.o` 和 `.ninja_*`。最终复制规则同时排除：

```text
.git/  __pycache__/  .pytest_cache/  build/
*.pyc  *.so  *.o  .ninja_*  *.egg-info/
checkpoint-rs.tar
```

迁入结果：

```text
目标：/home/openflex/old/openflex_ws/src/openflex_grasp
文件数：109
大小：3.9M
源/目标文件列表 diff：0
源/目标逐文件 SHA256 diff：0
禁用项扫描：0 个命中
```

checkpoint 保留在源资产位置，等待 P4-04 部署到模型根目录。

## P2-05 抓取策略重命名

- MuJoCo `replenishment_box` 使用 `graspnet_basket`。
- MuJoCo 货架使用 `graspnet_shelf`。
- test_07/test_08 当前尚无真实 Detect/Pick backend，明确使用 `mock`。
- `StationDatabase` 明确拒绝 `anygrasp` 和 `mock_or_anygrasp`，错误信息提示三个新策略名；保留现有 `front/box_top/bottle_side` 等非 AnyGrasp 旧策略以避免扩大本项范围。

验证：

```text
test_station_db.py：7 passed
所有 stations*.yaml：解析通过
活动配置旧策略扫描：0
```

## P2-06 商超仿真活动入口清理

已从 `competition_physics_stack.launch.py` 删除：

```text
openflex_anygrasp package lookup
physical_anygrasp.launch.py include
openflex_tc_wam anygrasp_gateway node
start_anygrasp launch argument
AnyGrasp 延时启动 TimerAction
```

已从 preflight 删除 AnyGrasp ROS 包、SDK checkpoint 和 conda Python 检查，并删除 `config/anygrasp_sim.yaml`。深度桥注释改为中立的 RealSense-compatible `16UC1` 契约。

验证结果：

```text
Python py_compile：通过
launch generate_launch_description：通过，12 个实体且无 start_anygrasp
station + 不依赖外部模型的仿真测试：11 passed
活动仿真代码 AnyGrasp 扫描：0
```

同一测试命令另有 4 项失败，原因是既有测试硬编码缺失的 `/home/openflex/old/openflex_ws/src/openflex_mujoco/openflex_mujoco.xml` 和旧 `/home/openarmx/openflex_all/supermarket_scene/supermarket_scene_from_db.xml`。这些失败与 AnyGrasp 移除无关，已转入 P5-11/P9-03 的外部依赖和仿真可移植性修复，不能在最终仿真关卡忽略。

## P2-07 GUI 与活动运行说明清理

当前比赛 GUI、任务执行器 README、比赛框架和仿真 README 已统一为以下契约：

```text
/detect_object:
  默认 Qwen3.5-4B 2D 框，可由调试 GUI 手动选择 OWLv2
  DINO 只允许显式复核 OWLv2；Qwen 失败直接报错
  SAM2 生成选中实例 mask、点云和 object_pose

/pick_object:
  GraspNetBaseline 使用 detection_id 对应的同帧实例点云
  MoveIt/夹爪负责执行与附着验证

/competition/mock_pick_object:
  不启动真实检测、SAM2、GraspNetBaseline、MoveIt 抓取或夹爪
```

GUI 的活动模块名已从 `C / DREAM 检测` 改为 `C / 局部检测`，不再显示 AnyGrasp 为 backend。`openarmx_task_msgs` 和比赛配置也已改为两个模型 worker 分环境常驻，不再描述 DREAM/AnyGrasp 互斥占用 GPU。

`openarmx_supermarket_sim/CONTINUATION_HANDOFF_CN.md` 是 2026-07-29 历史快照，保留原文但在文件顶部增加 legacy 警告，明确不得按其中说明恢复 SDK、worker、checkpoint 或 launch。`openflex_thesis/README.md` 同样标注整套论文子系统不进入比赛启动链。

验证：

```text
当前 GUI/活动 README 中 AnyGrasp backend 命中：0
integrated 剩余 AnyGrasp 文件：3
  - 历史交接快照 1 个
  - station_db 对旧策略名的显式拒绝 1 个
  - 对应防回归测试 1 个
```

## P2-08 legacy 包删除

删除前再次扫描所有 `package.xml`、`CMakeLists.txt`、`setup.py`、launch、Python 和 YAML；除待删包自身外，`openflex_anygrasp` 包名引用为 0。

已删除的精确目录：

```text
/home/openflex/old/openflex_ws/src/openflex_thesis/openflex_anygrasp
```

验收结果：

```text
目录不存在：通过
openflex_thesis_msgs 保留：通过
其他 openflex_thesis 包保留：通过
```

同时删除了 `openflex_thesis_bringup` 中仅服务该包的右腕相机 wrapper/config，并移除 package 直接依赖。论文实验预检不再查找已删除包，改为读取本包安装的 `legacy_perception.yaml`，默认 `physical_candidates_available: false`，因此历史物理证据入口明确失败关闭。

## P2-09 删除后依赖闭包

活动依赖扫描范围：所有 `package.xml`、`CMakeLists.txt`、`setup.py`、launch、Python 和 YAML。

```text
openflex_anygrasp：0
anygrasp_sdk_v3：0
physical_anygrasp：0
start_anygrasp：0
```

干净构建使用独立临时目录，未读取工作空间旧 `build/install/log`：

```text
/tmp/openflex_p2_clean.4Lkchc/{build,install,log}
colcon --log-base ... build --base-paths src --build-base ... --install-base ... \
  --packages-up-to openflex_thesis_bringup openflex_thesis_experiments \
  openflex_navigation_gui openarmx_task_executor openarmx_supermarket_sim

Summary: 43 packages finished [3min 11s]
```

目标五个包均存在于临时 install。7 个包有 stderr，但内容是既有编译/前端体积警告，没有 AnyGrasp 缺包、导入或链接错误。

测试与运行时验证：

```text
preflight.py py_compile：通过
test_preflight.py：3 passed
test_station_db.py：7 passed
安装态 thesis_preflight --scope perception --skip-hardware-gates：
  按预期返回 1，并报告 physical_perception_candidates=False
```

剩余论文 AnyGrasp 文本允许清单按目录约束为：

- `src/openflex_thesis/openflex_tc_wam/**`
- `src/openflex_thesis/openflex_thesis_control/**`
- `src/openflex_thesis/openflex_thesis_experiments/**`
- `src/openflex_thesis/openflex_thesis_msgs/**`
- `src/openflex_thesis/openflex_thesis_mujoco/**`
- `src/openflex_thesis/openflex_thesis_bringup/docs/**`
- `src/openflex_thesis/openflex_thesis_bringup/config/hardware_gates.yaml`
- `src/openflex_thesis/openflex_thesis_bringup/launch/{thesis_system,offline_mission_smoke}.launch.py`
- `src/openflex_thesis/README.md`

这些文件保留论文历史接口、实验协议和离线 mock；它们不得被新比赛 fullbody、Detect、Pick、仿真 preflight 或 GUI include。任何允许清单之外新增的 AnyGrasp 活动引用都视为回归。
