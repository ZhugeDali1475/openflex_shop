#!/usr/bin/env python3

import math
import os
import glob
import subprocess
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from openarmx_battery_monitor.serial_modbus_rtu import SerialModbusRtuClient
from openarmx_battery_monitor.serial_modbus_rtu import to_signed_16


def _clamp_soc(raw: int) -> float:
    return max(0.0, min(100.0, float(raw)))


def _soc_valid(raw: int) -> bool:
    return raw != 0xFFFF and 0 <= raw <= 100


def _voltage_valid(voltage: float) -> bool:
    return 5.0 <= voltage <= 120.0


@dataclass
class MasterSnapshot:
    voltage: float
    current: float
    state_soc: float
    bms_status: int


class JDBatteryMasterDualNode(Node):
    """
    Product mode:
    - Fixed parallel pack usage.
    - Only show BMS reported SOC (#116) in RViz.
    - No rule label inference in UI.
    """

    def __init__(self) -> None:
        super().__init__("jd_battery_master_dual_node")

        self._serial_port = str(self.declare_parameter("serial_port", "/dev/ttyUSB0").value)
        raw_port_candidates = str(
            self.declare_parameter(
                "serial_port_candidates",
                "/dev/ttyACM0,/dev/ttyUSB0,/dev/ttyACM1,/dev/ttyUSB1,"
                "/dev/ttyACM2,/dev/ttyUSB2,/dev/ttyACM3,/dev/ttyUSB3",
            ).value
        )
        self._baudrate = int(self.declare_parameter("baudrate", 19200).value)
        self._master_slave_id = int(self.declare_parameter("master_slave_id", 3).value)
        self._poll_hz = float(self.declare_parameter("poll_hz", 6.0).value)
        self._timeout_sec = float(self.declare_parameter("timeout_sec", 0.25).value)
        self._flush_before_write = bool(
            self.declare_parameter("flush_before_write", False).value
        )
        self._read_retries = max(0, int(self.declare_parameter("read_retries", 1).value))
        self._offline_after_failures = max(
            1, int(self.declare_parameter("offline_after_failures", 10).value)
        )
        self._hold_on_read_fail_sec = max(
            0.0, float(self.declare_parameter("hold_on_read_fail_sec", 0.6).value)
        )
        self._current_neutral_deadband_a = max(
            0.0, float(self.declare_parameter("current_neutral_deadband_a", 0.1).value)
        )

        self._state_start_register = int(self.declare_parameter("state_start_register", 114).value)
        self._state_register_count = int(self.declare_parameter("state_register_count", 5).value)

        self._battery_frame_id = str(
            self.declare_parameter("battery_frame_id", "battery_link").value
        )
        self._battery_topic = str(
            self.declare_parameter("battery_topic", "/battery_state").value
        )
        # Keep overlay label empty by default.
        self._battery_location = str(self.declare_parameter("battery_location", "").value)
        self._auto_discover_serial_port = bool(
            self.declare_parameter("auto_discover_serial_port", True).value
        )
        self._port_rescan_interval_sec = max(
            0.5, float(self.declare_parameter("port_rescan_interval_sec", 2.0).value)
        )
        self._fix_serial_permission = bool(
            self.declare_parameter("fix_serial_permission", True).value
        )

        self._serial_port_candidates = self._build_port_candidates(
            primary=self._serial_port,
            csv_list=raw_port_candidates,
        )

        self._pub = self.create_publisher(BatteryState, self._battery_topic, 10)
        self._client = SerialModbusRtuClient(
            port=self._serial_port,
            baudrate=self._baudrate,
            timeout_sec=self._timeout_sec,
            flush_before_write=self._flush_before_write,
        )

        self._consecutive_failures = 0
        self._is_online = False
        self._last_error_log_time = 0.0
        self._last_good_snapshot: MasterSnapshot | None = None
        self._last_good_time = 0.0
        self._last_port_scan_time = 0.0
        self._last_permission_hint_time = 0.0

        period = 0.2 if self._poll_hz <= 0.0 else 1.0 / self._poll_hz
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"JD master node started: port={self._serial_port}, baud={self._baudrate}, "
            f"master_slave_id={self._master_slave_id}, "
            f"state_regs=[{self._state_start_register}.."
            f"{self._state_start_register + self._state_register_count - 1}], "
            f"battery_topic={self._battery_topic}, "
            f"offline_after_failures={self._offline_after_failures}, "
            f"hold_on_read_fail_sec={self._hold_on_read_fail_sec}, "
            f"auto_discover_serial_port={self._auto_discover_serial_port}, "
            f"fix_serial_permission={self._fix_serial_permission}, "
            f"port_candidates={self._serial_port_candidates}"
        )

    @staticmethod
    def _build_port_candidates(primary: str, csv_list: str) -> list[str]:
        ports: list[str] = []
        if primary:
            ports.append(primary)
        for item in csv_list.split(","):
            candidate = item.strip()
            if candidate and candidate not in ports:
                ports.append(candidate)
        for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
            for candidate in sorted(glob.glob(pattern)):
                if candidate not in ports:
                    ports.append(candidate)
        return ports

    def destroy_node(self) -> bool:
        self._client.close()
        return super().destroy_node()

    def _make_client(self, port: str) -> SerialModbusRtuClient:
        return SerialModbusRtuClient(
            port=port,
            baudrate=self._baudrate,
            timeout_sec=self._timeout_sec,
            flush_before_write=self._flush_before_write,
        )

    def _decode_snapshot(self, state_values: list[int]) -> MasterSnapshot:
        if len(state_values) < 5:
            raise RuntimeError(f"State register response too short: len={len(state_values)}")

        voltage = state_values[0] * 0.01
        current = to_signed_16(state_values[1]) * 0.01
        state_soc = _clamp_soc(state_values[2]) if _soc_valid(state_values[2]) else math.nan
        bms_status = state_values[4]

        return MasterSnapshot(
            voltage=voltage,
            current=current,
            state_soc=state_soc,
            bms_status=bms_status,
        )

    def _snapshot_looks_like_battery(self, snap: MasterSnapshot) -> bool:
        return _voltage_valid(snap.voltage) and math.isfinite(snap.state_soc)

    def _read_snapshot_with_client(self, client: SerialModbusRtuClient) -> MasterSnapshot:
        state_values = self._read_registers_with_client(
            client,
            self._master_slave_id,
            self._state_start_register,
            self._state_register_count,
        )
        snap = self._decode_snapshot(state_values)
        if not self._snapshot_looks_like_battery(snap):
            raise RuntimeError(
                f"Unexpected battery registers: voltage={snap.voltage:.2f}, soc={snap.state_soc}"
            )
        return snap

    def _log_permission_hint(self, port: str) -> None:
        now = time.monotonic()
        if now - self._last_permission_hint_time < 5.0:
            return
        self.get_logger().warn(
            f"No permission to open {port}."
        )
        self._last_permission_hint_time = now

    def _try_fix_serial_permission(self, port: str) -> bool:
        if not self._fix_serial_permission:
            return False
        self.get_logger().warn(
            f"Trying to grant temporary permission for {port}. "
            "If sudo asks for a password, enter it in this terminal."
        )
        try:
            result = subprocess.run(
                ["sudo", "chmod", "a+rw", port],
                check=False,
                timeout=30.0,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Permission fix timed out for {port}")
            return False
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Permission fix failed for {port}: {exc}")
            return False
        if result.returncode != 0:
            self.get_logger().warn(f"Permission fix failed for {port}: sudo exit={result.returncode}")
            return False
        self.get_logger().info(f"Temporary serial permission granted for {port}")
        return True

    def _refresh_serial_port_candidates(self) -> None:
        self._serial_port_candidates = self._build_port_candidates(
            primary=self._serial_port,
            csv_list=",".join(self._serial_port_candidates),
        )

    def _switch_to_available_port(self) -> None:
        if not self._auto_discover_serial_port:
            return
        now = time.monotonic()
        if now - self._last_port_scan_time < self._port_rescan_interval_sec:
            return
        self._last_port_scan_time = now
        self._refresh_serial_port_candidates()
        active_port = self._client.port
        for candidate in self._serial_port_candidates:
            if not os.path.exists(candidate):
                continue
            probe = self._make_client(candidate)
            try:
                snap = self._read_snapshot_with_client(probe)
            except PermissionError:
                self._log_permission_hint(candidate)
                probe.close()
                if not self._try_fix_serial_permission(candidate):
                    continue
                probe = self._make_client(candidate)
                try:
                    snap = self._read_snapshot_with_client(probe)
                except Exception:
                    continue
            except Exception:
                continue
            finally:
                probe.close()
            if candidate != active_port:
                self.get_logger().warn(
                    f"Battery serial port selected: {active_port} -> {candidate} "
                    f"(voltage={snap.voltage:.2f}V, soc={snap.state_soc:.0f}%)"
                )
                self._client.set_port(candidate)
                self._serial_port = candidate
            return

    def _read_registers_with_client(
        self,
        client: SerialModbusRtuClient,
        slave_id: int,
        start_addr: int,
        register_count: int,
    ) -> list[int]:
        last_exc: Exception | None = None
        attempts = 1 + self._read_retries
        for attempt in range(attempts):
            try:
                return client.read_holding_registers(
                    slave_id=slave_id,
                    start_addr=start_addr,
                    register_count=register_count,
                )
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                if attempt + 1 < attempts:
                    time.sleep(0.03)
                    continue
                break
        raise last_exc if last_exc is not None else RuntimeError("Unknown read error")

    def _read_registers(self, slave_id: int, start_addr: int, register_count: int) -> list[int]:
        return self._read_registers_with_client(
            self._client,
            slave_id,
            start_addr,
            register_count,
        )

    def _read_master_snapshot(self) -> MasterSnapshot:
        state_values = self._read_registers(
            self._master_slave_id,
            self._state_start_register,
            self._state_register_count,
        )
        snap = self._decode_snapshot(state_values)
        if not self._snapshot_looks_like_battery(snap):
            raise RuntimeError(
                f"Unexpected battery registers: voltage={snap.voltage:.2f}, soc={snap.state_soc}"
            )
        return snap

    @staticmethod
    def _status_bit(value: int, bit: int) -> bool:
        if value == 0xFFFF:
            return False
        return ((value >> bit) & 0x1) == 1

    def _power_status(self, snap: MasterSnapshot) -> int:
        charging = self._status_bit(snap.bms_status, 10)
        discharging = self._status_bit(snap.bms_status, 11)
        if charging and not discharging:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if discharging and not charging:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        if snap.current >= self._current_neutral_deadband_a:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        if snap.current <= -self._current_neutral_deadband_a:
            return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

    def _publish_online(self, snap: MasterSnapshot) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame_id
        msg.location = self._battery_location
        msg.serial_number = ""
        msg.voltage = snap.voltage
        msg.current = snap.current
        msg.percentage = snap.state_soc / 100.0
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.present = True
        msg.power_supply_status = self._power_status(snap)
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self._pub.publish(msg)

    def _publish_offline(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame_id
        msg.location = ""
        msg.serial_number = ""
        msg.voltage = math.nan
        msg.current = math.nan
        msg.percentage = math.nan
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.present = False
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self._pub.publish(msg)

    def _publish_from_snapshot(self, snap: MasterSnapshot) -> None:
        if math.isfinite(snap.state_soc):
            self._publish_online(snap)
            return
        self._publish_offline()

    def _on_timer(self) -> None:
        now = time.monotonic()
        snapshot: MasterSnapshot | None = None
        read_exc: Exception | None = None

        try:
            snapshot = self._read_master_snapshot()
        except Exception as exc:  # noqa: BLE001
            read_exc = exc

        if snapshot is not None:
            if not self._is_online:
                self.get_logger().info("Master battery link recovered.")
            self._is_online = True
            self._consecutive_failures = 0
            self._last_good_snapshot = snapshot
            self._last_good_time = now
            self._publish_from_snapshot(snapshot)
            return

        self._consecutive_failures += 1
        self._client.close()
        self._switch_to_available_port()

        if (
            self._last_good_snapshot is not None
            and (now - self._last_good_time) <= self._hold_on_read_fail_sec
        ):
            self._publish_from_snapshot(self._last_good_snapshot)
            return

        if self._consecutive_failures >= self._offline_after_failures:
            if self._is_online:
                self.get_logger().warn("Master battery link offline, waiting reconnect...")
            self._is_online = False
            self._publish_offline()

        if now - self._last_error_log_time >= 3.0:
            self.get_logger().warn(
                f"Battery read failed: consecutive_failures={self._consecutive_failures}, "
                f"error={read_exc if read_exc is not None else 'unknown'}"
            )
            self._last_error_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JDBatteryMasterDualNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
