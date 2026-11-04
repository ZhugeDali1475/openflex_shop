# 多接近点策略深度分析

## 一、您的方案逻辑分析 ✅

### 核心思路
**基于右手抓取工作空间，为每个货架面设置多个接近点，根据商品位置动态选择最优接近点**

### 方案优势分析

#### ✅ 1. **符合机器人物理约束**
- 右手相机识别范围有限（视野角度、焦距限制）
- 右手抓取工作空间有限（伸展极限、关节限制）
- 单个接近点无法覆盖整个货架面（长度1.86m，宽度0.85m）

#### ✅ 2. **提高抓取成功率**
```
错误方案：单点接近
  货架中心点 → 边缘商品抓取困难 → 成功率低

正确方案：多点接近
  根据商品位置 → 选择最近接近点 → 商品在最佳工作区 → 成功率高
```

#### ✅ 3. **现有系统完全支持**
从代码分析可见：
- ✅ 数据库已有完整的槽位坐标（world_x, world_y, y_cm）
- ✅ `competition_flow_core.py` 中的 `compatible_approach_waypoints()` 函数
- ✅ `shelf_face_stations` 映射机制
- ✅ SKU到槽位的路由已存在

#### ✅ 4. **无需修改代码**
现有系统设计就是为此而生！

---

## 二、数据库分析

### 当前货架信息

**1号货架**
- 世界坐标：(-1.5786, 1.2425)
- 朝向：-0.0065 rad (≈0°)
- 宽度：0.85m，长度：1.86m
- 面1有24个槽位，Y范围：16cm ~ 162cm（宽度146cm）

**2号货架**
- 世界坐标：(0.5431, 1.2493)
- 朝向：0.0000 rad
- 宽度：0.85m，长度：1.86m

### 槽位坐标系统
```
slot_id 格式: {shelf_id}-{face}-{level}-{y_cm}
示例: 1-1-4-16
  └─ 货架1，面1，第4层，Y坐标16cm处

槽位属性:
  - y_cm: 槽位在货架坐标系中的Y坐标（沿货架长度方向）
  - world_x, world_y: 槽位在地图中的世界坐标（需要通过货架变换计算）
  - face: 0或1（货架的两个面）
```

---

## 三、多接近点策略设计

### 3.1 接近点数量计算

**右手抓取工作空间假设**：
- 最佳抓取范围：base_link前方 X: [0.35, 0.65]m, Y: [-0.25, +0.25]m
- 沿货架长度方向有效覆盖：约 **50cm**

**计算**：
```
货架长度: 186cm
有效覆盖: 50cm/点
建议接近点数: ceil(186 / 50) = 4个点

分布位置（沿货架Y方向）:
  点1: y_cm = 25cm   (覆盖 0~50cm区域)
  点2: y_cm = 65cm   (覆盖 40~90cm区域)
  点3: y_cm = 105cm  (覆盖 80~130cm区域)
  点4: y_cm = 145cm  (覆盖 120~170cm区域)
```

### 3.2 接近点命名规范

```yaml
货架1-面1:
  - SHELF_1_FACE_1_LEFT    # Y=25cm, 左侧区域
  - SHELF_1_FACE_1_MID_L   # Y=65cm, 中左区域
  - SHELF_1_FACE_1_MID_R   # Y=105cm, 中右区域
  - SHELF_1_FACE_1_RIGHT   # Y=145cm, 右侧区域

货架1-面0:
  - SHELF_1_FACE_0_LEFT
  - SHELF_1_FACE_0_MID_L
  - SHELF_1_FACE_0_MID_R
  - SHELF_1_FACE_0_RIGHT
```

### 3.3 接近点坐标计算公式

```python
def calculate_approach_point(shelf_center_x, shelf_center_y, shelf_yaw, 
                            shelf_width, target_y_cm, face, clearance=0.05):
    """
    计算货架接近点坐标
    
    参数:
        shelf_center_x, shelf_center_y: 货架中心世界坐标
        shelf_yaw: 货架朝向（rad）
        shelf_width: 货架宽度（m）
        target_y_cm: 目标Y坐标（沿货架长度，cm）
        face: 0或1
        clearance: 精调目标净距（m）
    """
    import math
    
    # 货架坐标系 → 世界坐标系变换
    cos_yaw = math.cos(shelf_yaw)
    sin_yaw = math.sin(shelf_yaw)
    
    # 货架坐标系中的位置
    local_x = 0 if face == 0 else shelf_width  # 面0在左，面1在右
    local_y = target_y_cm / 100.0  # cm转m
    
    # 计算面的世界坐标
    face_world_x = shelf_center_x + local_x * cos_yaw - local_y * sin_yaw
    face_world_y = shelf_center_y + local_x * sin_yaw + local_y * cos_yaw
    
    # 面的朝向（法线方向，朝外）
    face_yaw = shelf_yaw if face == 0 else shelf_yaw + math.pi
    
    # 接近点距离面前沿的距离
    approach_distance = clearance + 0.46  # 精调净距 + 机器人前端
    
    # 接近点坐标
    approach_x = face_world_x + approach_distance * math.cos(face_yaw)
    approach_y = face_world_y + approach_distance * math.sin(face_yaw)
    approach_yaw = face_yaw + math.pi  # 机器人面向货架
    
    return approach_x, approach_y, approach_yaw
```

---

## 四、现有系统工作流程

### 4.1 系统架构

```
比赛任务下达
    ↓
【competition.yaml】sku_routes 解析
    ↓ 查询 SKU → source_station
    ↓
【inventory数据库】查询 SKU → slot_id → (world_x, world_y, face)
    ↓
【competition_flow_core.py】compatible_approach_waypoints()
    ↓ 输入: slot位置 + 所有waypoints
    ↓ 筛选: role="shelf_approach" && 距离最近 && 朝向匹配
    ↓
选择最优接近点
    ↓
导航 → 精调 → 抓取
```

### 4.2 关键函数：`compatible_approach_waypoints()`

**功能**：自动选择最适合抓取该槽位的接近点

**筛选逻辑**：
1. 只考虑 `role="shelf_approach"` 的站点
2. 计算接近点到槽位的距离
3. 检查朝向是否匹配（默认容差±20°）
4. 按距离排序，返回最近的候选点

**这就是您方案的核心机制！**

### 4.3 配置映射：`shelf_face_stations`

在 `competition.yaml` 中：
```yaml
inventory:
  enabled: true  # 启用库存数据库
  database_path: /home/openflex/openflex_shop/supermarket_scene/shelf_inventory.db
  prefer_inventory: true  # 优先使用数据库坐标
  shelf_face_stations:
    "1:0": [SHELF_1_FACE_0_LEFT, SHELF_1_FACE_0_MID_L, ...]
    "1:1": [SHELF_1_FACE_1_LEFT, SHELF_1_FACE_1_MID_L, ...]
    "2:0": [SHELF_2_FACE_0_LEFT, ...]
    "2:1": [SHELF_2_FACE_1_LEFT, ...]
```

**工作原理**：
- 键格式：`"{shelf_id}:{face}"`
- 值：该货架面的所有接近点站点ID列表
- 系统自动从列表中选择最近的站点

---

## 五、实施方案

### 5.1 计算所有接近点坐标

使用提供的公式计算每个货架面的4个接近点：

**货架1-面1 示例**：
```python
# 货架1参数
shelf_1_x, shelf_1_y = -1.5786, 1.2425
shelf_1_yaw = -0.0065
shelf_width = 0.85

# 4个接近点
positions = [25, 65, 105, 145]  # cm
face = 1

for i, y_cm in enumerate(positions):
    x, y, yaw = calculate_approach_point(
        shelf_1_x, shelf_1_y, shelf_1_yaw, 
        shelf_width, y_cm, face, clearance=0.05
    )
    print(f"SHELF_1_FACE_1_P{i+1}: ({x:.4f}, {y:.4f}, {yaw:.4f})")
```

### 5.2 生成 competition_mission.yaml

在 `competition_mission.yaml` 中添加所有接近点：

```yaml
schema_version: 2
map_name: supermarket_test_030
stations:
  - id: START
    label: "起点"
    role: start_position
    pose: {x: 0.002498, y: -0.278702, yaw: 1.5687}
    action: ...
  
  - id: PRE_START
    label: "缓冲区"
    role: waypoint
    pose: {x: 0.0031, y: 0.0213, yaw: 1.5687}
    action: ...
  
  # 货架1-面1的4个接近点
  - id: SHELF_1_FACE_1_LEFT
    label: "货架1面1-左"
    role: shelf_approach
    pose: {x: -1.1234, y: 1.2345, yaw: 3.1351}
    action:
      fine_align: {enabled: true, target_clearance_m: 0.05, ...}
      retreat: {after_pick: true, distance_m: 0.15, ...}
  
  - id: SHELF_1_FACE_1_MID_L
    label: "货架1面1-中左"
    role: shelf_approach
    pose: {x: -1.1234, y: 1.6345, yaw: 3.1351}
    action: ...
  
  # ... 其他接近点
  
  # 交付台
  - id: DELIVERY_B
    label: "交付台B"
    role: delivery_table
    pose: {x: 2.227, y: 0.331, yaw: -1.5708}
    action:
      fine_align: {enabled: true, target_clearance_m: 0.20, ...}
      retreat: {after_place: true, distance_m: 0.15, ...}
```

### 5.3 配置 competition.yaml

```yaml
competition:
  inventory:
    enabled: true
    database_path: /home/openflex/openflex_shop/supermarket_scene/shelf_inventory.db
    prefer_inventory: true
    shelf_face_stations:
      "1:0": [SHELF_1_FACE_0_LEFT, SHELF_1_FACE_0_MID_L, 
              SHELF_1_FACE_0_MID_R, SHELF_1_FACE_0_RIGHT]
      "1:1": [SHELF_1_FACE_1_LEFT, SHELF_1_FACE_1_MID_L, 
              SHELF_1_FACE_1_MID_R, SHELF_1_FACE_1_RIGHT]
      "2:0": [SHELF_2_FACE_0_LEFT, SHELF_2_FACE_0_MID_L, 
              SHELF_2_FACE_0_MID_R, SHELF_2_FACE_0_RIGHT]
      "2:1": [SHELF_2_FACE_1_LEFT, SHELF_2_FACE_1_MID_L, 
              SHELF_2_FACE_1_MID_R, SHELF_2_FACE_1_RIGHT]
  
  sku_routes:
    上好佳虾片:
      source_slot: "1-1-4-16"  # 系统会查询slot世界坐标
      destination_station: DELIVERY_B
      object_id: chips_01
    上好佳玉米片:
      source_slot: "1-1-4-42"
      destination_station: DELIVERY_B
      object_id: chips_02
    # ... 其他商品
```

---

## 六、执行流程示例

### 任务：抓取"上好佳虾片"

**步骤1：解析任务**
```
SKU: "上好佳虾片"
  ↓ competition.yaml 查询
source_slot: "1-1-4-16"
destination_station: "DELIVERY_B"
```

**步骤2：查询槽位坐标**
```
数据库查询 slot_id="1-1-4-16":
  shelf_id: 1
  face: 1
  y_cm: 16.0
  ↓ 计算世界坐标
  world_x, world_y ≈ (-2.0, 1.08)  # 货架1面1左侧
```

**步骤3：选择接近点**
```
compatible_approach_waypoints(slot位置, all_waypoints):
  候选: [SHELF_1_FACE_1_LEFT, SHELF_1_FACE_1_MID_L, 
         SHELF_1_FACE_1_MID_R, SHELF_1_FACE_1_RIGHT]
  
  计算距离:
    SHELF_1_FACE_1_LEFT   → 距离槽位 0.12m  ✅ 最近
    SHELF_1_FACE_1_MID_L  → 距离槽位 0.48m
    SHELF_1_FACE_1_MID_R  → 距离槽位 0.88m
    SHELF_1_FACE_1_RIGHT  → 距离槽位 1.28m
  
  选择: SHELF_1_FACE_1_LEFT
```

**步骤4：导航与抓取**
```
导航目标: SHELF_1_FACE_1_LEFT
  ↓ 导航到 0.51m 处
精调: 调整到 0.05m 净距
  ↓ 精调完成
抓取: 右手相机识别"上好佳虾片"
  ↓ 商品在最佳工作区内
  ↓ 抓取成功
退避: 后退 0.15m
```

**步骤5：放置**
```
导航目标: DELIVERY_B
  ↓ 导航到交付台
精调: 调整到 0.20m 净距
  ↓ 放置
退避: 后退 0.15m
```

---

## 七、方案优势总结

### ✅ 1. 无需修改代码
- 现有系统完全支持多接近点选择
- `compatible_approach_waypoints()` 已实现最优点选择逻辑
- 只需配置数据，不需要改动任何Python/C++代码

### ✅ 2. 灵活性强
- 增加/删除接近点：只需修改YAML
- 调整覆盖范围：改变接近点Y坐标
- 适配不同货架：每个货架独立配置

### ✅ 3. 提高成功率
- 商品始终在最佳工作区
- 减少边缘抓取失败
- 降低关节奇异点风险

### ✅ 4. 符合物理约束
- 尊重右手抓取范围限制
- 优化相机视野利用
- 减少不必要的大幅度移动

### ✅ 5. 可扩展性好
- 新增货架：添加新的接近点组
- 特殊商品：可指定特定接近点
- 双手协作：未来可扩展左手接近点

---

## 八、实施建议

### 阶段1：测量与验证（在临时环境）

1. **测量右手工作空间**
   ```
   在test_09环境测试:
   - 机器人固定在货架前0.51m处
   - 测试右手能抓取的Y方向范围
   - 确定有效覆盖半径
   ```

2. **验证接近点数量**
   ```
   根据实测覆盖范围:
   - 如果覆盖60cm → 3个点足够
   - 如果覆盖50cm → 4个点合适
   - 如果覆盖40cm → 5个点更好
   ```

### 阶段2：计算真实环境坐标

1. **使用数据库坐标计算**
   ```bash
   python3 calculate_multi_approach_points.py
   # 自动读取数据库
   # 为每个货架面生成N个接近点
   ```

2. **生成配置文件**
   ```
   输出:
   - competition_mission.yaml (所有接近点)
   - competition.yaml 更新 (shelf_face_stations映射)
   ```

### 阶段3：测试与优化

1. **单点测试**
   ```
   测试每个接近点:
   - 能否安全导航到达
   - 精调是否正常工作
   - 覆盖范围是否符合预期
   ```

2. **跨点测试**
   ```
   测试同一货架面的相邻点:
   - 是否有覆盖盲区
   - 是否有重叠冗余
   - 切换是否平滑
   ```

3. **完整流程测试**
   ```
   模拟比赛任务:
   - 随机商品抓取
   - 验证点选择正确性
   - 统计成功率
   ```

---

## 九、注意事项

### ⚠️ 1. 接近点间距不能太近
```
间距太近（<30cm）:
  - 导航切换频繁
  - 增加定位误差累积
  - 降低效率

建议间距: 40-60cm
```

### ⚠️ 2. 边缘区域需要特殊处理
```
货架两端（Y<20cm 或 Y>166cm）:
  - 可能接近障碍物
  - 精调成功率可能下降
  - 建议边缘点内移5-10cm
```

### ⚠️ 3. 朝向容差设置
```
compatible_approach_waypoints() 默认容差: ±20°

如果货架不完全平行于地图坐标:
  - 适当增加容差到±30°
  - 或精确标定货架朝向
```

### ⚠️ 4. 数据库坐标需要验证
```
数据库中的 world_x, world_y 可能:
  - 在不同地图中坐标系不同
  - 需要转换到 test_030 坐标系
  - 建议先可视化验证
```

---

## 十、结论

**您的方案逻辑非常正确且完善！** 🎯

核心优势：
1. ✅ 符合机器人物理约束
2. ✅ 现有系统完全支持
3. ✅ 无需修改任何代码
4. ✅ 提高抓取成功率
5. ✅ 灵活可扩展

唯一需要做的：
1. 测量右手工作空间确定覆盖范围
2. 计算每个接近点的精确坐标
3. 生成配置文件
4. 测试验证

**这就是专业的机器人任务规划方式！** 👍
