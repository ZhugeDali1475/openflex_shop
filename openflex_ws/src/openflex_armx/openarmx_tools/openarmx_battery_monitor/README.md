# OpenArmX JD Battery Monitor (RViz Overlay)

English | [中文](./README-CN.md)

---

This package reads JD battery BMS data via RS485/Modbus and displays a graphical battery level overlay in the top-right corner of the RViz 3D viewport.

Current production version strategy ("fixed parallel pack + charging head scenario"):
- No dual-pack independent identification inference
- Directly displays BMS-reported SOC (register `#116`)
- No rule-based labels displayed (avoids misleading info and flickering)

## 1. Hardware Connection

### 1.1 Wiring and Interface (2+6)

Based on JD battery documentation and field validation, common communication pins:

- `pin3`: `485A` (typically orange)
- `pin5`: `485B` (typically gray)
- `pin4`: Discharge identification (typically yellow)
- `pin6`: Charge identification (typically white)

Power lines are typically red and black (P+/P-).

### 1.2 USB-to-485 Connection
- Battery `485A` -> Converter `A/+`
- Battery `485B` -> Converter `B/-`
- Converter USB plugs into the host, serial port is usually `/dev/ttyUSB0`

### 1.3 Parallel Pack Communication Key Points
- After two batteries are connected in parallel via a 3-in-1 harness, the BMS typically reports only "one SOC result under the current strategy."
- The discharge identification wire (yellow) must be shorted to `P+` per specification in parallel/discharge communication scenarios; otherwise communication may fail or data may not be reported.

## 2. Software Architecture

### 2.1 Data Pipeline
1. `serial_modbus_rtu.py`: Low-level Modbus RTU client
2. `jd_battery_master_dual_node.py`: Reads registers and publishes `/battery_state`
3. `openarmx_battery_overlay_display`: RViz Display plugin, shows battery icon and percentage
4. RViz config `auto_pack_overlay.rviz`: Fixed overlay in top-right corner

### 2.2 Key Registers (Protocol 1 - RS485)
- `#114`: Total voltage, unit `0.01V`
- `#115`: Total current, `S16`, unit `0.01A` (positive=charging, negative=discharging)
- `#116`: SOC, unit `%` (sole display source in this version)
- `#118`: BMS system status (bit10=charging, bit11=discharging, bit12=charger connected)

### 2.3 Current SOC Display Logic (Production Version)
- If `#116` is valid (0-100): `percentage = #116 / 100.0`
- If communication failures reach threshold: publishes offline status, RViz overlay auto-hides (`Hide When Offline=true`)
- No longer displays "single-pack high / dual-pack low / standby" labels to avoid edge-case misjudgment flickering

## 3. Running

### 3.1 Build
```bash
source /opt/ros/humble/setup.bash
cd ~/openflex_all/openflex_ws
colcon build --symlink-install --packages-select openarmx_battery_monitor
```

### 3.2 Launch (Recommended)
```bash
source /opt/ros/humble/setup.bash
source ~/openflex_all/openflex_ws/install/setup.bash
pkill -f jd_battery_master_dual_node || true
pkill -f "ros2 launch openarmx_battery_monitor auto_pack_overlay.launch.py" || true
ros2 launch openarmx_battery_monitor auto_pack_overlay.launch.py start_rviz:=true
```

### 3.3 View Topic Data Only
```bash
source /opt/ros/humble/setup.bash
source ~/openflex_all/openflex_ws/install/setup.bash
ros2 topic echo /battery_state --once
```

## 4. Parameters (auto_pack_overlay.launch.py)

Core parameters:
- `serial_port`: Default `/dev/ttyUSB0`
- `baudrate`: Default `19200`
- `master_slave_id`: Default `3`
- `poll_hz`: Default `6.0`
- `timeout_sec`: Default `0.25`
- `read_retries`: Default `1`
- `offline_after_failures`: Default `3`
- `hold_on_read_fail_sec`: Default `0.6`

Notes:
- These parameters are tuned for stability to suppress occasional serial jitter in the field.
- If the environment is clean, `poll_hz` can be increased further.

## 5. RViz Display Details

Plugin type: `openarmx_battery_overlay_display/BatteryOverlayDisplay`

Key behaviors:
- Overlay is attached to the RViz render viewport, unaffected by dock panel width changes
- Percentage text is drawn inside the battery icon
- Color levels:
  - `>=60%` green
  - `30%-59%` yellow
  - `<30%` red
- `Use Message Location=false` (no business labels displayed)
- `Hide When Offline=true` (auto-hide when offline)

## 6. Troubleshooting

### 6.1 Battery connected but shows offline
1. Check if `A/B` wires are reversed
2. Check serial port number (`ls -l /dev/ttyUSB*`)
3. Check baud rate is `19200 8N1`
4. Check that the discharge identification wire (yellow) is shorted to `P+` as required

### 6.2 Display flickering
1. Confirm only one node instance is running (avoid multiple processes publishing `/battery_state`)
2. Keep current default timeout/retry parameters; do not over-tune them

## 7. Version Notes

This version targets:
- Single display slot
- Direct SOC pass-through display (#116)
- No label inference
- Production-ready logic for "fixed parallel pack + charging head" use case
