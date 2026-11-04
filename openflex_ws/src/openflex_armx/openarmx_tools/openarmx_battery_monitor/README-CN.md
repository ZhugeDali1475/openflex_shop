# OpenArmX 京东电池监控（RViz 叠加层）

[English](./README.md) | 中文

---

本项目用于读取京东电池 BMS 的 RS485/Modbus 数据，并在 RViz 3D 渲染区右上角显示图形化电量。

当前产品化目标版本采用“固定并包 + 充电头场景”策略：
- 不做两包独立识别推断
- 直接显示 BMS 上报 SOC（寄存器 `#116`）
- 不显示规则标签（避免误导与闪动）

## 1. 硬件连接

### 1.1 线束与接口（2+6）
依据京东电池资料与现场验证，常用通信针脚如下：

- `pin3`：`485A`（常见橙色）
- `pin5`：`485B`（常见灰色）
- `pin4`：放电识别（常见黄色）
- `pin6`：充电识别（常见白色）

供电线通常是红黑两根（P+/P-）。

### 1.2 USB 转 485 连接
- 电池 `485A` -> 转换器 `A/+`
- 电池 `485B` -> 转换器 `B/-`
- 转换器 USB 插到主机，串口一般为 `/dev/ttyUSB0`

### 1.3 并包通信关键点
- 两块电池通过三合一线束并包后，BMS 对外通常只上报“当前策略下的一个 SOC 结果”。
- 放电识别线（黄色）在并包/放电通信场景下需要按规格要求短接到 `P+`，否则可能通讯异常或不上报。

## 2. 软件架构

## 2.1 数据链路
1. `serial_modbus_rtu.py`：底层 Modbus RTU 客户端
2. `jd_battery_master_dual_node.py`：读取寄存器并发布 `/battery_state`
3. `openarmx_battery_overlay_display`：RViz Display 插件，显示电池图标和百分比
4. RViz 配置 `auto_pack_overlay.rviz`：固定渲染区右上角显示

## 2.2 主要寄存器（协议1-RS485）
- `#114`：总电压，单位 `0.01V`
- `#115`：总电流，`S16`，单位 `0.01A`（充电为正，放电为负）
- `#116`：SOC，单位 `%`（本版本唯一显示来源）
- `#118`：BMS 系统状态（bit10 充电、bit11 放电、bit12 充电器接入）

## 2.3 当前 SOC 显示逻辑（产品版）
- 若 `#116` 有效（0~100）：`percentage = #116 / 100.0`
- 若通信失败达到阈值：发布离线状态，RViz 叠加层自动隐藏（`Hide When Offline=true`）
- 不再显示“单包放高/双包取低/待机”等标签，避免边缘工况误判导致闪动

## 3. 运行方式

## 3.1 编译
```bash
source /opt/ros/humble/setup.bash
cd ~/openflex_all/openflex_ws
colcon build --symlink-install --packages-select openarmx_battery_monitor
```

## 3.2 启动（推荐）
```bash
source /opt/ros/humble/setup.bash
source ~/openflex_all/openflex_ws/install/setup.bash
pkill -f jd_battery_master_dual_node || true
pkill -f "ros2 launch openarmx_battery_monitor auto_pack_overlay.launch.py" || true
ros2 launch openarmx_battery_monitor auto_pack_overlay.launch.py start_rviz:=true
```

## 3.3 仅看话题数据
```bash
source /opt/ros/humble/setup.bash
source ~/openflex_all/openflex_ws/install/setup.bash
ros2 topic echo /battery_state --once
```

## 4. 参数说明（auto_pack_overlay.launch.py）

核心参数：
- `serial_port`：默认 `/dev/ttyUSB0`
- `baudrate`：默认 `19200`
- `master_slave_id`：默认 `3`
- `poll_hz`：默认 `6.0`
- `timeout_sec`：默认 `0.25`
- `read_retries`：默认 `1`
- `offline_after_failures`：默认 `3`
- `hold_on_read_fail_sec`：默认 `0.6`

说明：
- 这组参数偏“稳定优先”，用于抑制现场偶发串口抖动。
- 若环境干净可进一步提高 `poll_hz`。

## 5. RViz 显示原理

插件类型：`openarmx_battery_overlay_display/BatteryOverlayDisplay`

关键行为：
- 叠加层挂在 RViz 渲染窗口，不受 Dock 面板宽度变化影响
- 百分比文字绘制在电池图标内部
- 颜色分级：
  - `>=60%` 绿色
  - `30%~59%` 黄色
  - `<30%` 红色
- `Use Message Location=false`（不显示业务标签）
- `Hide When Offline=true`（离线自动隐藏）

## 6. 常见问题排查

### 6.1 明明有电池却显示离线
1. 检查 `A/B` 是否接反
2. 检查串口号（`ls -l /dev/ttyUSB*`）
3. 检查波特率是否 `19200 8N1`
4. 检查放电识别（黄线）是否按要求短接 `P+`

### 6.2 界面闪动
1. 先确认只启动了一个节点（避免多进程同时发布 `/battery_state`）
2. 保持当前默认超时/重试参数，不要过度调低

## 7. 版本说明

本版本定位：
- 单显示槽
- SOC 直通显示（#116）
- 去标签推断
- 面向“固定并包 + 充电头使用”的量产可用逻辑
