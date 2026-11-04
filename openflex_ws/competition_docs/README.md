# 比赛环境配置文档与工具集

本文件夹包含 `supermarket_test_030` 真实比赛环境的完整解决方案、分析报告和自动化工具。

> 当前第一阶段实现以 [`phase1_current_workflow.md`](phase1_current_workflow.md)
> 为准：每个货架面一个精调锚点，再沿货架横移到商品。下方“多接近点”文档和
> 生成器保留为历史分析，不再作为 test_9 联调和 test_030 迁移方案。

---

## 📁 文件清单

### 📄 核心文档

#### 1. `competition_solution_summary.md`
**完整的比赛方案总结**

包含内容：
- 起点围栏冲突问题的解决方案（双阶段缓冲策略）
- 站点距离标注规则（货架0.51m，交付台0.66m）
- 地图信息和转换参数
- 比赛流程示例
- 检查清单

适用场景：首次阅读，了解整体方案

---

#### 2. `multi_approach_points_analysis.md`
**多接近点策略深度分析**

包含内容：
- 现有系统架构分析
- `compatible_approach_waypoints()` 函数工作原理
- 货架坐标系变换机制
- 右手工作空间限制分析
- 覆盖范围计算方法

适用场景：理解多接近点方案的技术原理

---

#### 3. `multi_approach_implementation_report.md`
**多接近点方案实施报告**

包含内容：
- 方案验证结果（逻辑正确性、无需修改代码）
- 生成的16个货架接近点详细布局
- 覆盖效果分析（11-12cm重叠，无盲区）
- 系统完整工作流程示例
- 与单点方案性能对比
- 配置部署步骤
- 性能预期（成功率提升15-30%）

适用场景：实施多接近点方案的完整指南

---

#### 4. `supermarket_test_030_competition_guide.md`
**test_030环境快速入门指南**

包含内容：
- 环境背景介绍
- 两大核心问题及解决方案
- 操作步骤和命令
- 常见问题解答

适用场景：快速查阅操作步骤

---

### 🛠️ 自动化工具

#### 1. `calculate_approach_points.py`
**交互式接近点坐标计算工具**

功能：
- 手动输入货架/交付台位置
- 自动计算符合规则的接近点坐标
- 生成 `competition_mission.yaml` 配置文件

使用方法：
```bash
cd /home/openflex/openflex_shop/openflex_ws/competition_docs
python3 calculate_approach_points.py
```

适用场景：手动标注少量站点

---

#### 2. `generate_multi_approach_points.py`
**基于数据库的多接近点自动生成工具**

功能：
- 读取货架库存数据库
- 自动生成每个货架面的多个接近点
- 计算覆盖范围并验证完整性
- 生成 `competition_mission.yaml` 和 `shelf_face_stations` 映射配置

使用方法：
```bash
cd /home/openflex/openflex_shop/openflex_ws/competition_docs

# 使用默认参数（50cm覆盖范围，0.05m净距）
python3 generate_multi_approach_points.py

# 自定义参数
python3 generate_multi_approach_points.py \
  --coverage 60 \
  --clearance 0.10 \
  --edge-margin 15
```

参数说明：
- `--db`: 数据库路径（默认：`/home/openflex/openflex_shop/supermarket_scene/shelf_inventory.db`）
- `--coverage`: 每个接近点覆盖范围（cm），默认50
- `--clearance`: 精调目标净距（m），默认0.05
- `--edge-margin`: 边缘裕度（cm），默认10
- `--output-dir`: 输出目录，默认 `/home/openflex/old/openflex_maps/2d/supermarket_test_030`

适用场景：批量生成多个货架的接近点配置

---

#### 3. `competition_start_buffer_move.py`
**比赛启动缓冲移动ROS2节点**

功能：
- 启动时自动前进30cm（从START到PRE_START）
- 避免起点围栏碰撞监控问题
- 实时进度显示和里程计反馈

使用方法：
```bash
cd /home/openflex/openflex_shop/openflex_ws/competition_docs

# 在比赛启动后立即运行
python3 competition_start_buffer_move.py

# 等待提示 "✅ 缓冲移动完成！"
# 然后在GUI中初始化定位到 PRE_START (0.0031, 0.0213, 1.5687)
```

适用场景：比赛启动流程中的第一步操作

---

## 🚀 快速开始指南

### 场景1：首次了解方案
1. 阅读 `competition_solution_summary.md`
2. 了解整体架构和解决方案

### 场景2：实施多接近点策略
1. 阅读 `multi_approach_points_analysis.md` 理解原理
2. 阅读 `multi_approach_implementation_report.md` 了解实施细节
3. 运行 `generate_multi_approach_points.py` 生成配置
4. 按照报告中的步骤部署和测试

### 场景3：比赛当天启动
1. 参考 `supermarket_test_030_competition_guide.md`
2. 运行 `competition_start_buffer_move.py` 完成缓冲移动
3. 在PRE_START初始化定位
4. 开始比赛任务

### 场景4：手动标注站点
1. 使用 `calculate_approach_points.py`
2. 按提示输入站点信息
3. 生成配置文件

---

## 📊 核心配置文件位置

### 生成的配置文件
```
/home/openflex/old/openflex_maps/2d/supermarket_test_030/
└── competition_mission.yaml  # 18个站点配置（2固定+16货架）
```

### 需要更新的配置文件
```
/home/openflex/openflex_shop/openflex_ws/src/openflex_integrated/
└── openarmx_task_executor/config/competition.yaml
    └── inventory.shelf_face_stations  # 需要添加货架面映射
```

---

## 🔧 配置部署流程

### 步骤1：生成多接近点配置
```bash
cd /home/openflex/openflex_shop/openflex_ws/competition_docs
python3 generate_multi_approach_points.py
```

输出：
- `/home/openflex/old/openflex_maps/2d/supermarket_test_030/competition_mission.yaml`
- 终端显示的 `shelf_face_stations` 映射配置

### 步骤2：更新competition.yaml
复制终端输出的 `shelf_face_stations` 配置到：
```yaml
# src/openflex_integrated/openarmx_task_executor/config/competition.yaml
inventory:
  enabled: true
  database_path: /home/openflex/openflex_shop/supermarket_scene/shelf_inventory.db
  prefer_inventory: true
  shelf_face_stations:
    "1:0": ['SHELF_1_FACE_0_LEFT', 'SHELF_1_FACE_0_MID_L', 
            'SHELF_1_FACE_0_MID_R', 'SHELF_1_FACE_0_RIGHT']
    "1:1": ['SHELF_1_FACE_1_LEFT', 'SHELF_1_FACE_1_MID_L', 
            'SHELF_1_FACE_1_MID_R', 'SHELF_1_FACE_1_RIGHT']
    "2:0": ['SHELF_2_FACE_0_LEFT', 'SHELF_2_FACE_0_MID_L', 
            'SHELF_2_FACE_0_MID_R', 'SHELF_2_FACE_0_RIGHT']
    "2:1": ['SHELF_2_FACE_1_LEFT', 'SHELF_2_FACE_1_MID_L', 
            'SHELF_2_FACE_1_MID_R', 'SHELF_2_FACE_1_RIGHT']
```

### 步骤3：在GUI中验证
```bash
# 启动导航GUI
ros2 launch openflex_navigation_gui navigation_gui.launch.py

# 加载地图和站点配置
# 检查所有接近点位置是否合理
```

### 步骤4：测试
在临时环境测试：
- 导航到各个接近点
- 验证精调功能
- 测试接近点自动选择
- 运行完整任务流程

---

## 📈 技术亮点

### ✅ 零代码修改
所有方案完全通过配置文件实现，无需修改系统代码

### ✅ 智能选择
`compatible_approach_waypoints()` 自动计算距离，选择最优接近点

### ✅ 覆盖完整
4个接近点/面，11-12cm重叠，无盲区

### ✅ 灵活可调
覆盖范围、净距、边缘裕度都可通过参数调整

### ✅ 性能提升
预期抓取成功率提升15-30%

---

## 🎯 待完成任务

- [ ] 在临时环境测量右手实际工作空间（确认是否为50cm）
- [ ] 运行 `generate_multi_approach_points.py` 生成最终配置
- [ ] 更新 `competition.yaml` 中的 `shelf_face_stations` 映射
- [ ] 在GUI中验证所有接近点位置
- [ ] 测试缓冲移动脚本 `competition_start_buffer_move.py`
- [ ] 在临时环境完整测试多接近点策略
- [ ] 适配到真实比赛环境 test_030
- [ ] 添加交付台接近点配置（手动或扩展工具）

---

## 📞 问题排查

### Q: 接近点生成后导航失败？
A: 检查以下几点：
1. 接近点是否在障碍物内（查看2D地图）
2. 精调净距是否过小导致无法到达
3. 朝向是否正确（机器人应面向货架）

### Q: 接近点选择不正确？
A: 检查：
1. `shelf_face_stations` 映射是否正确
2. 数据库中的槽位坐标是否准确
3. 接近点的覆盖范围是否包含目标商品

### Q: 右手工作空间测量结果不是50cm？
A: 重新生成配置：
```bash
python3 generate_multi_approach_points.py --coverage <实测值>
```

### Q: 精调总是超时？
A: 可能原因：
1. 目标净距太小（0.05m可能过于严格）
2. 激光扫描质量不佳
3. 货架表面不规则

解决方案：
- 增大 `target_clearance_m` 到 0.08-0.10m
- 增大 `clearance_tolerance_m` 到 0.02m
- 调整 `timeout_sec` 到 30秒

---

## 📝 版本历史

- **2026-08-16**: 初始版本，包含完整的多接近点方案和工具集

---

**如有问题，请参考各文档中的详细说明或联系系统维护人员。**
