# test_9 比赛站点精调与退避实施方案

更新时间：2026-08-14  
当前状态：核心软件链、重定位参考点流程和起步足迹预检已构建并通过单元测试；实时 `/scan`、真机运动和机械臂放置仍未现场验收。

## 1. 目标

在“比赛导航”选点界面中，为每个站点显式配置：

- 是否在 Nav2 粗导航完成后执行精调；
- 精调模式、目标净距、距离/角度容差和超时；
- 是否在取货后退避；
- 是否在放置后退避；
- 退避距离与速度。

货架和具有可观测前沿的交付台共用激光前沿精调算法，但允许各站点使用不同目标净距。站点策略随对应地图的 `competition_mission.yaml` 保存，避免 test_9 点位与 test_08 任务配置错配。

## 2. test_9 点云结论

- [x] 找到 test_9 三维点云：`/home/openflex/openflex_shop/openflex_maps/3d/supermarket_test_09/map.pcd`。
- [x] 找到 test_9 二维地图：`/home/openflex/openflex_shop/openflex_maps/2d/supermarket_test_09/`。
- [x] 对点云进行分高度俯视检查。
- [x] 确认疑似右侧交付台朝通道前沿约为 `y=-0.34 m`、`x=1.77~2.87 m`，连续长度约 `1.09 m`。
- [x] 确认疑似左侧补货台朝通道前沿约为 `y=-0.59 m`、`x=-3.00~-1.81 m`，连续长度约 `1.18 m`。
- [x] 确认两个台子在低高度点云中均有明显前沿回波，静态点云条件满足复用双侧前沿精调的基本要求。
- [ ] 在机器人实际粗导航位姿记录 3~5 秒实时 `/scan`，确认左右两侧回波在实时扫描裁剪后仍持续存在。
- [ ] 现场确认上述两个矩形目标分别确为交付台和补货台，而不是临近其它障碍物。

## 3. 站点数据模型

- [x] 在 `BenchmarkWaypoint` 中加入精调策略字段：
  - `fine_align_enabled`
  - `fine_align_mode`
  - `target_clearance_m`
  - `clearance_tolerance_m`
  - `fine_align_yaw_tolerance_rad`
  - `fine_align_timeout_sec`
- [x] 在 `BenchmarkWaypoint` 中加入退避策略字段：
  - `retreat_after_pick`
  - `retreat_after_place`
  - `retreat_distance_m`
  - `retreat_speed_mps`
- [x] 加入旧配置兼容属性：旧货架点默认需要精调和取货后退避；旧交付/补货点默认放置后退避。
- [x] 为所有新增数值增加统一合法性校验，包括正数、最大安全距离、速度上限及支持的模式。
- [ ] 明确并实现未来可能需要的 `marker` 模式；本阶段只启用激光前沿模式。

## 4. competition_mission.yaml 格式

- [x] 将新保存格式升级为 `schema_version: 2`。
- [x] 在站点下保存 `action.fine_align` 和 `action.retreat`。
- [x] 保留对旧版 `schema_version: 1` 的读取兼容。
- [x] 将旧名称 `shelf_scan` 读取为通用名称 `front_surface_scan`。
- [x] 增加损坏配置、非法模式、非法距离/速度和缺字段测试。
- [ ] 确认旧地图首次保存为 v2 后没有行为变化。
- [x] 决定并实现受控合并：mission 覆盖地图位姿、精调和退避；station YAML 保留机械臂、层高、货位和放置标定。

目标格式示例：

```yaml
schema_version: 2
map_name: supermarket_test_09
stations:
  - id: DELIVERY_COUNTER
    label: 交付台
    role: delivery_table
    pose: {x: 2.32, y: 0.37, yaw: -1.57079632679}
    action:
      fine_align:
        enabled: true
        mode: front_surface_scan
        target_clearance_m: 0.20
        clearance_tolerance_m: 0.01
        yaw_tolerance_deg: 1.0
        timeout_sec: 20.0
      retreat:
        after_pick: false
        after_place: true
        distance_m: 0.15
        speed_mps: 0.04
```

## 5. 比赛导航选点界面

- [x] 增加“到点后执行激光前沿精调”选项。
- [x] 增加精调模式下拉框。
- [x] 增加目标净距、距离容差、角度容差和超时输入。
- [x] 增加“取货后”“放置后”退避选项。
- [x] 增加退避距离和退避速度输入。
- [x] 货架角色的新建默认值设为：5 cm 精调、取货后退避。
- [x] 交付台/补货台角色的新建默认值设为：20 cm 精调、放置后退避。
- [x] 站点表格追加“精调策略”和“退避策略”两列。
- [x] 编辑已有站点时回填新增策略字段。
- [x] 完整检查 Qt 控件初始化顺序和角色切换行为。
- [x] 补充 GUI 单元测试，覆盖新增、编辑、保存、重新加载和角色默认值。
- [x] 增加“精调观测测试”按钮与状态显示。
- [x] 增加未通过观测验证时禁止开启实际精调的现场安全提示或确认机制。

## 6. 通用激光前沿精调执行

- [x] 将 GUI 的精调触发条件从固定 `role=shelf_approach` 改为站点的显式 `requires_fine_alignment`。
- [x] GUI 下发精调目标时使用站点自己的目标净距、距离容差、角度容差和超时。
- [x] 精调结果评估改为使用站点自己的目标参数。
- [x] GUI 文案开始从“货架精停”泛化为“前沿精调”。
- [x] 检查并统一主要残留的“货架精停”专用文案。
- [x] 在数据模型和任务配置覆盖时拒绝不支持的精调模式。
- [ ] 将内部变量和状态名从 `shelf_alignment` 逐步泛化，同时保持 ROS 接口兼容。
- [ ] 修正当前精调服务实际完成容差可能放宽至 3°、而 GUI 按 1°验收的不一致。
- [ ] 为交付台验证 20 cm 目标净距下的 ROI、左右角点和机械臂放置可达性。

## 7. 只观测、不运动的精调预检

- [x] 已在 `ShelfFineAlign.action` 源文件中加入 `measure_only` 字段。
- [x] 已在精调服务源代码中加入 `measure_only` 分支：获得滤波后的双侧回波后返回结果，并持续发布零速度。
- [x] 已在 GUI ROS 客户端源代码中加入 `measure_only` 参数赋值。
- [x] 重新构建 `openarmx_task_msgs`，生成包含 `measure_only` 的 ROS 接口。
- [x] 重新构建所有依赖 `ShelfFineAlign` 的包。
- [x] 增加 GUI “精调观测测试”按钮，发送 `measure_only=true`。
- [x] 在 GUI 显示净距、角差、滤波帧数和通过/不通过原因。
- [x] 增加观测模式的源代码兼容处理和接口检查。
- [ ] 现场确认观测模式全过程没有非零 `/cmd_vel` 输出。

注意：`ShelfFineAlign.Goal.measure_only` 已完成重新构建；下一步需要现场确认观测模式全过程没有非零速度输出。

## 8. 退避任务链

- [x] 现有任务执行器已经支持货架取货后的 `retreat_after_pick`。
- [x] 现有任务执行器已经支持目的站放置后的 `retreat_after_place`。
- [x] 现有 `/retreat_from_station` 支持受限直线后退、里程计监控、横向漂移和偏航保护。
- [x] 让 GUI 手动抓取链使用站点保存的退避距离和速度，而不是只使用全局手动输入框。
- [ ] 在交付台真实放置完成后调用 `after_place` 退避。
- [ ] 明确导航测试、手动联调和完整任务执行器之间唯一的运动所有者，避免重复精调或重复退避。
- [ ] 退避完成且底盘稳定后才允许下发下一 Nav2 目标。
- [ ] 对货架和交付台分别执行至少 10 次退避重复性测试。

## 9. test_9 配置与启动入口

- [x] 创建 `supermarket_test_09_competition.yaml`。
- [ ] 现场确定 test_9 初始位姿，不能直接照搬 test_08。
- [ ] 在比赛导航页重新标定所有 test_9 货架粗导航点。
- [ ] 标定右侧交付台和左侧补货台粗导航点。
- [x] 实现 station YAML 标定模板与 test_9 mission 的启动时自动合并；独立 `stations_supermarket_test_09.yaml` 仍可后续按现场标定生成。
- [ ] 将机械臂层高姿态、抓取策略和放置姿态与 test_9 站点 ID 合并。
- [x] 修改桌面启动入口默认使用 test_9 profile，并将当前地图 mission 传给任务执行器；旧 test_08 station YAML 作为标定模板回退。
- [x] 启动预检增加 mission overlay 检查，确保 mission 站点可被任务执行器加载。
- [x] 消除运行时只读取 test_08 动作策略的问题；启动时由当前 mission 覆盖位姿、精调和退避。
- [x] 当前 test_9 mission 已保存任务起点 `START=(-0.118, 0.051, 1.5603 rad)`；该点用于返回起点和默认重定位参考，不要求机器人精确停在该坐标。

基于静态点云的初始候选值（仅供现场选点起始参考）：

- 右侧交付台前沿中心约 `(2.32, -0.34)`；20 cm 最终净距对应机器人中心 `y≈0.32`，粗导航可先选 `y≈0.37~0.42`，朝向约 `-90°`。
- 左侧补货台前沿中心约 `(-2.41, -0.59)`；20 cm 最终净距对应机器人中心 `y≈0.07`，粗导航可先选 `y≈0.12~0.17`，朝向约 `-90°`。
- [ ] 使用实时定位和地图安全检查确认上述候选位置。
- [ ] 确认粗导航点给精调保留 5~10 cm 前进空间，并且没有进入障碍足迹。

## 9.1 重定位参考点与导航起点安全策略

- [x] 明确区分“本次重定位参考点”和“任务返回起点”：
  - 地图临时选点只给 ICP 提供当前机器人附近的搜索初值；
  - `start_position` 是可选任务点，只用于返回起点和下次启动默认值；
  - Nav2 始终从定位后的实时 `map -> base_link` 位姿规划，不从参考点坐标规划。
- [x] “地图选择初始位姿”在导航未启动时也可使用；此时只填写本次参考位姿，不发布 `/initialpose`。
- [x] 导航运行后重新选择初始位姿时，先停止当前运动，再发布 `/initialpose`。
- [x] 移除 `pose_confirmed`、输入值必须等于保存起点、必须先保存任务起点等硬限制。
- [x] 移除 `/localization_ok=true` 必须持续固定 5 秒的硬等待；重定位服务判定成功后可立即进入实时安全预检。
- [x] 保留 `/localization_ok=true`、当前 `map -> base_link` 有效且不超过 0.5 秒未更新的硬条件。
- [x] 订阅 `/global_costmap/costmap_raw`，要求代价地图不超过 1 秒未更新。
- [x] 按实时 `map -> base_link` 将带 padding 的底盘足迹投影到全局代价地图。
- [x] 足迹覆盖 Nav2 cost `253`（inscribed）、`254`（lethal）或 `255`（unknown）时禁止下发目标，并显示具体原因。
- [x] 上述预检对首个目标、手动航点、返回起点和自动巡游后续航点统一生效。
- [ ] 真机验证：在参考点附近存在 10~30 cm 和数度偏差时，ICP 能正确收敛且 Nav2 从实时位姿规划。
- [ ] 真机验证：人为制造足迹 lethal/unknown 后，GUI 必须拒绝目标；障碍清除后才恢复放行。

## 10. 测试与验收

- [x] 运行 `test_competition_mission.py`，补齐 schema v1/v2 往返测试。
- [x] 运行 `test_competition_window.py`，补齐 GUI 策略测试。
- [x] 运行 `test_benchmark.py` 相关 profile 兼容路径。
- [x] 运行 `test_shelf_fine_align_core.py` 和 `test_shelf_fine_align_node.py`。
- [x] 构建 `openarmx_task_msgs`、`openarmx_task_executor`、`openflex_navigation_gui`。
- [x] 使用可写 scene DB 副本执行桌面启动器 `--check` 预检。
- [x] 新增全局 costmap 足迹单元测试，覆盖 free、inscribed/lethal 和 unknown。
- [x] 完成重定位参考点语义修正后的 GUI 全量测试：`83 passed`。
- [x] 重新构建 `swerve_navigation` 和 `openflex_navigation_gui`。
- [ ] 静态验证所有站点保存、重载和地图切换均正确。
- [ ] 观测模式验证货架左右回波。
- [ ] 观测模式验证交付台左右回波。
- [ ] 低速空载执行交付台精调，不执行机械臂动作。
- [ ] 验证交付台最终净距约 20 cm、角差满足最终统一后的容差。
- [ ] 空载验证交付台后退 15 cm。
- [ ] 完成“货架精调→模拟抓取→退避→交付台精调→模拟放置→退避”的整链测试。
- [ ] 真机带货测试前完成碰撞监控、定位门和急停检查。

## 11. 推荐恢复顺序

1. [x] 先修复/完成消息接口改动并构建。
2. [x] 完成 mission v2 和 GUI 单元测试。
3. [x] 完成精调观测按钮，确保无运动预检可用。
4. [x] 建立 test_9 profile 和 station 配置绑定。
5. [ ] 现场选取 test_9 货架、交付台和补货台粗导航点。
6. [ ] 先观测回波，再空载低速精调。
7. [ ] 验证取后/放后退避。
8. [ ] 最后接入完整抓取与放置任务。
