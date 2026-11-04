# supermarket_test_030 比赛环境完整方案总结

## 📋 核心问题与解决方案

### 问题1：起点围栏冲突 ✅ 已解决

#### 问题描述
- 真实比赛起点：`(0.0025, -0.2787, yaw=1.5687)` 
- 机器人后端超出起点约 **18.1cm**，直接触碰围栏
- 碰撞监控会立即检测到障碍物，拒绝导航

#### 解决方案：双阶段缓冲策略

**阶段1：启动时前进到缓冲区**
```
START (0.0025, -0.2787) 
   ↓ 直线前进 30cm (绕过碰撞监控)
PRE_START (0.0031, 0.0213)
   ↓ 初始化定位
开始正常导航
```

**阶段2：返回时使用过渡点**
```
任务完成
   ↓ 导航
PRE_START (0.0031, 0.0213)
   ↓ 直线后退或低速移动 30cm
START (0.0025, -0.2787)
```

#### 实现方式

**方式A：使用提供的ROS2节点**
```bash
# 在比赛启动后立即运行
cd /home/openflex/openflex_shop/openflex_ws
python3 competition_start_buffer_move.py

# 等待提示 "✅ 缓冲移动完成！"
# 然后在GUI中初始化定位到 PRE_START
```

**方式B：手动发送速度命令**
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 持续6秒后停止
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**方式C：集成到比赛控制台**
修改比赛启动流程，在导航前自动执行前进指令。

---

### 问题2：站点距离标注规则 ✅ 已明确

#### 距离要求

| 站点类型 | 距离目标前沿 | 精调目标净距 | 退避策略 |
|---------|------------|-------------|---------|
| 货架接近点 (shelf_approach) | **0.51m** | 0.05m | 抓取后退避 |
| 交付台接近点 (delivery_table) | **0.66m** | 0.20m | 放置后退避 |
| 通用航点 (waypoint) | N/A | N/A | 无退避 |

#### 计算公式

```python
# 已知：目标前沿中心点 (x_target, y_target)，朝向 θ（法线方向，朝外）

# 货架接近点
clearance = 0.05  # 精调净距
robot_front = 0.46  # 机器人前端到base_link
approach_distance = clearance + robot_front  # 0.51m

x_approach = x_target + approach_distance * cos(θ)
y_approach = y_target + approach_distance * sin(θ)
yaw_approach = θ + π  # 机器人面向目标

# 交付台接近点同理，clearance = 0.20，approach_distance = 0.66m
```

#### 使用计算工具

```bash
cd /home/openflex/openflex_shop/openflex_ws
python3 calculate_approach_points.py

# 按提示输入货架/交付台位置，自动计算接近点
# 生成配置文件：/home/openflex/old/openflex_maps/2d/supermarket_test_030/competition_mission.yaml
```

---

## 🗺️ 地图信息

### supermarket_test_030（真实比赛环境）

**文件位置：**
```
/home/openflex/old/openflex_maps/
├── 3d/supermarket_test_030/
│   ├── map.pcd          # 3D点云地图
│   ├── poses.txt        # 建图轨迹
│   └── sc_data/         # 扫描上下文数据
└── 2d/supermarket_test_030/
    ├── map.pgm          # 2D占据栅格图
    ├── map.yaml         # 地图元数据
    └── competition_mission.yaml  # 比赛站点配置（待创建）
```

**地图参数：**
- 分辨率：0.05m/格
- 原点：[-4.724700, -3.148418, 0.0]
- 尺寸：197 × 142 格

**转换参数（已完成）：**
```bash
# 3D → 2D 转换已完成
cd /home/openflex/openflex_shop/openflex_ws
source install/setup.bash

ros2 run map_3d_2d_converter map_3d_2d_converter \
  --pcd /home/openflex/old/openflex_maps/3d/supermarket_test_030/map.pcd \
  --poses /home/openflex/old/openflex_maps/3d/supermarket_test_030/poses.txt \
  --output /home/openflex/old/openflex_maps/2d/supermarket_test_030 \
  --resolution 0.050 \
  --z-min -0.350 --z-max 2.800 \
  --obstacle-z-min -0.180 --obstacle-z-max 1.800 \
  --obstacle-min-hits 2 --free-min-hits 3 \
  --sor-mean-k 16 --sor-stddev 1.50 \
  --raycast-max-range 4.50 --crop-to-poses-margin 2.50
```

---

## 📍 必需站点配置

### 固定站点（已确定）

```yaml
stations:
  - id: START
    label: "起点"
    role: start_position
    pose: {x: 0.002498, y: -0.278702, yaw: 1.5687}
    action:
      fine_align: {enabled: false}
      retreat: {after_pick: false, after_place: false}
    # ⚠️ 仅用于标记，不要直接导航到此点

  - id: PRE_START
    label: "起点缓冲区"
    role: waypoint
    pose: {x: 0.0031, y: 0.0213, yaw: 1.5687}
    action:
      fine_align: {enabled: false}
      retreat: {after_pick: false, after_place: false}
    # ✅ 实际定位初始化点和返回目标点
```

### 待标注站点（需要您提供坐标）

根据比赛任务，您需要标注：

1. **货架接近点** (SHELF_A, SHELF_B, SHELF_C, ...)
   - 角色：`shelf_approach`
   - 距离货架前沿：0.51m
   - 精调净距：0.05m
   - 退避：抓取后退避 15cm

2. **交付台接近点** (DELIVERY_B, DELIVERY_C, ...)
   - 角色：`delivery_table`
   - 距离交付台前沿：0.66m
   - 精调净距：0.20m
   - 退避：放置后退避 15cm

---

## 🔧 标注工具与流程

### 方法1：使用计算工具（推荐）

```bash
cd /home/openflex/openflex_shop/openflex_ws
python3 calculate_approach_points.py
```

**交互流程：**
1. 选择站点类型（1=货架，2=交付台）
2. 输入站点ID（如 SHELF_A）
3. 输入目标前沿中心点坐标
4. 输入目标朝向（支持度数或弧度）
5. 自动计算接近点坐标
6. 重复添加所有站点
7. 自动生成 `competition_mission.yaml`

### 方法2：使用GUI可视化标注

```bash
cd /home/openflex/openflex_shop/openflex_ws
source install/setup.bash

# 启动导航GUI
ros2 launch openflex_navigation_gui navigation_gui.launch.py

# 或使用比赛控制台
# desktop-file-install /home/openflex/桌面/OpenFlex商超比赛控制台.desktop
# 点击桌面图标启动
```

**GUI操作步骤：**
1. 加载地图：选择 `/home/openflex/old/openflex_maps` → `supermarket_test_030`
2. 切换到"站点编辑"标签页
3. 在地图上点击添加站点
4. 设置站点属性（ID、角色、精调、退避）
5. 保存配置

---

## 🚀 比赛流程示例

### 启动流程

```bash
# 1. 机器人放置在START点 (0.0025, -0.2787)
# 2. 启动系统
cd /home/openflex/openflex_shop/openflex_ws
source install/setup.bash

# 3. 启动机器人（不初始化定位）
ros2 launch openarmx_task_executor supermarket_full_system.launch.py \
  map_yaml:=/home/openflex/old/openflex_maps/2d/supermarket_test_030/map.yaml \
  pcd_path:=/home/openflex/old/openflex_maps/3d/supermarket_test_030/map.pcd \
  initial_pose:="[0.0031, 0.0213, 0.0, 0.0, 0.0, 1.5687]"  # PRE_START

# 4. 在另一个终端，执行前进到缓冲区
python3 competition_start_buffer_move.py

# 5. 等待提示"✅ 缓冲移动完成！"

# 6. 在GUI中初始化定位到 PRE_START (0.0031, 0.0213, 1.5687)

# 7. 开始比赛任务
```

### 任务流程示例

```yaml
phase_1_picking:
  - PRE_START → SHELF_A (导航+精调+抓取+退避)
  - SHELF_A → DELIVERY_B (导航+精调+放置+退避)

phase_2_restocking:
  - DELIVERY_B → SHELF_C (导航+精调+抓取+退避)
  - SHELF_C → DELIVERY_D (导航+精调+放置+退避)

return_home:
  - CURRENT → PRE_START (导航)
  - PRE_START → START (直线移动/手动推)
```

---

## 📝 检查清单

### 赛前准备

- [ ] ✅ 已完成3D→2D地图转换 (test_030, test_040)
- [ ] 使用计算工具或GUI标注所有站点
- [ ] 验证站点距离符合要求（货架0.51m，交付台0.66m）
- [ ] 生成 `competition_mission.yaml` 文件
- [ ] 在临时环境测试站点可达性

### 启动测试

- [ ] 测试从START前进到PRE_START（30cm）
- [ ] 在PRE_START初始化定位成功
- [ ] 从PRE_START导航到各站点无碰撞
- [ ] 精调功能在所有站点正常工作
- [ ] 退避功能测试通过

### 导航参数验证

- [ ] 最大轮速：0.60 m/s（是否需要调整）
- [ ] 轮加速度：0.50 m/s²（是否需要调整）
- [ ] 精调超时：20秒（是否足够）
- [ ] 精调净距容差：0.01m（是否合理）
- [ ] 精调角度容差：1°（是否合理）

### 返回流程测试

- [ ] 从任务终点导航到PRE_START
- [ ] 从PRE_START移动到START（检查倒退是否允许）
- [ ] 如不允许倒退，测试低速导航或手动推

---

## 🛠️ 提供的工具文件

1. **`supermarket_test_030_competition_guide.md`**
   - 完整的比赛流程指南
   - 问题分析和解决方案
   - 操作步骤和注意事项

2. **`calculate_approach_points.py`**
   - 交互式站点坐标计算工具
   - 自动生成 `competition_mission.yaml`
   - 验证距离和参数

3. **`competition_start_buffer_move.py`**
   - ROS2启动缓冲移动节点
   - 自动前进30cm到PRE_START
   - 实时进度显示和里程计反馈

---

## 🔍 下一步行动

### 立即需要做的：

1. **确定货架和交付台位置**
   - 在GUI中打开 `supermarket_test_030` 地图
   - 目视确定货架/交付台的大致位置
   - 记录坐标和朝向

2. **标注站点**
   ```bash
   python3 calculate_approach_points.py
   # 按提示输入所有站点信息
   ```

3. **验证配置**
   - 在GUI中加载生成的 `competition_mission.yaml`
   - 检查站点位置是否合理
   - 必要时手动微调

4. **测试启动流程**
   ```bash
   # 在临时环境测试前进到缓冲区功能
   python3 competition_start_buffer_move.py
   ```

### 需要您提供的信息：

为了完成站点标注，请提供：

1. **货架位置**（至少2个）
   - 货架前沿中心点坐标 (x, y)
   - 货架朝向（法线方向，朝外）

2. **交付台位置**（至少2个）
   - 交付台前沿中心点坐标 (x, y)
   - 交付台朝向（法线方向，朝外）

3. **任务流程**
   - 比赛要求抓取和放置的顺序
   - 是否有特定的路径约束

---

## 📞 讨论建议

我们可以继续深入讨论：

1. **导航参数优化**
   - 速度和加速度限制是否需要调整
   - 精调参数在真实环境的适用性

2. **传感器配置**
   - Livox点云覆盖范围是否足够
   - 是否需要额外的碰撞监控配置

3. **容错策略**
   - 精调失败时的回退方案
   - 导航失败时的重试策略
   - 定位漂移时的重新初始化

4. **时间优化**
   - 如何缩短任务完成时间
   - 路径规划优化空间

您想先讨论哪个方面？或者我们可以先完成站点标注，然后进行更深入的技术讨论。
