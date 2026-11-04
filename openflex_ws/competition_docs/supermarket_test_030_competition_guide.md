# supermarket_test_030 比赛流程指南

## 一、起点围栏问题解决方案

### 问题分析
- **真实起点**: (0.0025, -0.2787, yaw=1.5687 rad ≈ 90°)
- **机器人尺寸**: 长0.92m × 宽0.90m
- **风险**: 机器人后端超出起点18.1cm，会检测到围栏导致无法导航

### 推荐解决方案：双阶段缓冲策略

#### 阶段1：启动时前进到缓冲区
```yaml
# 启动流程
1. 机器人放置在START点 (0.0025, -0.2787)
2. 启动系统但暂不初始化定位
3. 通过底盘控制直接前进30cm（绕过碰撞监控）
   - 使用 /cmd_vel 话题发送速度命令
   - 线速度: 0.05 m/s，持续6秒
   - 或使用里程计反馈精确控制
4. 到达PRE_START点后初始化定位
5. 开始正常导航流程
```

**PRE_START缓冲点坐标**: (0.0031, 0.0213, 1.5687)

#### 阶段2：返回时使用过渡点
```yaml
# 返回流程
1. 完成任务后导航到PRE_START
2. 从PRE_START直线移动到START（两种方式）：
   方式A: 直接发送底盘后退命令（如果允许reverse）
   方式B: 使用低速导航目标点逼近
3. 到达START后停止
```

### 实现代码示例

#### 方法1：使用速度控制前进
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

class StartBufferMove(Node):
    def __init__(self):
        super().__init__('start_buffer_move')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.start_y = -0.2787
        self.target_distance = 0.30
        self.current_y = None
        
    def odom_callback(self, msg):
        self.current_y = msg.pose.pose.position.y
        
    def move_forward(self):
        cmd = Twist()
        cmd.linear.x = 0.05  # 5cm/s
        
        self.get_logger().info("开始前进到缓冲区...")
        rate = self.create_rate(20)  # 20Hz
        
        while self.current_y is None or (self.current_y - self.start_y) < self.target_distance:
            self.cmd_pub.publish(cmd)
            rate.sleep()
            if self.current_y:
                self.get_logger().info(f"已前进: {(self.current_y - self.start_y)*100:.1f}cm")
        
        # 停止
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("到达缓冲区！")

def main():
    rclpy.init()
    node = StartBufferMove()
    node.move_forward()
    rclpy.shutdown()
```

#### 方法2：在比赛控制台中集成
需要在比赛流程开始前调用底盘移动服务。

## 二、站点标注规则

### 距离要求

| 站点类型 | 距离目标前沿 | 精调目标净距 | 说明 |
|---------|------------|-------------|------|
| 货架接近点 | 0.55-0.65m | 0.05m | 精调后距离货架5cm |
| 交付台接近点 | 0.70-0.80m | 0.20m | 精调后距离交付台20cm |
| 通用航点 | N/A | N/A | 无精调要求 |

### 计算公式

假设货架/交付台前沿法线方向为θ（朝外），中心点为(x_shelf, y_shelf)：

```
接近点坐标计算:
x_approach = x_shelf + (clearance + robot_front) * cos(θ)
y_approach = y_shelf + (clearance + robot_front) * sin(θ)
yaw_approach = θ + π (机器人面向货架)

其中:
- clearance: 精调目标净距(货架0.05m, 交付台0.20m)
- robot_front: 机器人前端到base_link距离 = 0.46m
- 实际导航目标距离 = clearance + 0.46m
```

### 示例计算

假设货架A在(2.0, 1.5)，朝向θ=180°（-X方向）：
```python
import math
x_shelf, y_shelf = 2.0, 1.5
theta = math.pi  # 180度
clearance = 0.05
robot_front = 0.46

# 接近点在货架正前方
approach_distance = clearance + robot_front  # 0.51m
x_approach = x_shelf + approach_distance * math.cos(theta)
y_approach = y_shelf + approach_distance * math.sin(theta)
yaw_approach = theta + math.pi  # 机器人朝向货架

# 结果: (1.49, 1.50, 0.0)
```

## 三、使用GUI标注站点的步骤

### 1. 启动导航GUI
```bash
cd /home/openflex/openflex_shop/openflex_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动GUI（不启动机器人硬件）
python3 -m openflex_navigation_gui.main_window
```

### 2. 加载地图
- 地图根目录: `/home/openflex/old/openflex_maps`
- 地图名称: `supermarket_test_030`
- 自动加载 3D: `supermarket_test_030/map.pcd`
- 自动加载 2D: `supermarket_test_030/map.yaml`

### 3. 标注站点
1. 在地图上找到货架位置（灰色障碍物区域）
2. 使用鼠标点击货架前方0.5-0.6m处
3. 调整朝向箭头指向货架
4. 设置站点属性：
   - ID: SHELF_A, SHELF_B...
   - 角色: shelf_approach
   - 精调: 启用，目标净距0.05m
   - 退避: 抓取后退避
5. 重复标注交付台站点：
   - ID: DELIVERY_B, DELIVERY_C...
   - 角色: delivery_table
   - 精调: 启用，目标净距0.20m
   - 退避: 放置后退避

### 4. 保存配置
保存到: `/home/openflex/old/openflex_maps/2d/supermarket_test_030/competition_mission.yaml`

## 四、比赛流程框架

```yaml
# 示例流程（根据实际任务调整）
competition_flow:
  phase_1:
    - START -> PRE_START (初始前进)
    - PRE_START -> SHELF_A (导航+精调+抓取+退避)
    - SHELF_A -> DELIVERY_B (导航+精调+放置+退避)
  
  phase_2:
    - DELIVERY_B -> SHELF_C
    - SHELF_C -> DELIVERY_D
  
  return_home:
    - CURRENT -> PRE_START (导航)
    - PRE_START -> START (直线移动)
```

## 五、注意事项

### 定位初始化
- ⚠️ 不要在START点初始化定位（后端超出地图）
- ✅ 在PRE_START点初始化定位
- ✅ 初始位姿建议设为PRE_START坐标

### 精调参数验证
在真实环境测试前，使用临时环境验证：
1. 精调超时是否足够（20秒）
2. 净距容差是否合理（0.01m）
3. 角度容差是否合理（1°）

### 倒退功能检查
确认导航配置中是否允许倒退：
```bash
# 检查launch参数
grep "allow_reverse" src/openflex_chassis/navigation_layer/swerve_navigation/launch/*.py
```

如果`allow_reverse=false`，返回时不能使用倒退，需要：
- 方案A: 使用极低速导航逼近START
- 方案B: 修改launch允许短距离倒退

## 六、故障排查

### 问题1：机器人在START拒绝导航
- 原因: 碰撞监控检测到后方围栏
- 解决: 确认已执行前进到PRE_START的步骤

### 问题2：精调失败超时
- 原因: 货架表面不平或传感器遮挡
- 解决: 调整接近点距离，增加精调超时时间

### 问题3：返回时无法到达START
- 原因: 导航拒绝靠近围栏的目标点
- 解决: 使用PRE_START作为最终点，手动推到START

## 七、测试检查清单

- [ ] 前进到PRE_START功能测试通过
- [ ] 在PRE_START初始化定位成功
- [ ] 所有站点距离货架/交付台符合要求
- [ ] 精调功能在各站点测试通过
- [ ] 退避功能测试通过
- [ ] 完整流程模拟运行无碰撞
- [ ] 返回PRE_START功能测试通过
- [ ] 从PRE_START移动到START测试通过

---

**文档版本**: 1.0  
**创建时间**: 2026-08-16  
**适用地图**: supermarket_test_030  
