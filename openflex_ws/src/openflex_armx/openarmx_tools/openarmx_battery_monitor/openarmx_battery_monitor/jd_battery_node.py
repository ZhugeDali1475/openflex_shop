#!/usr/bin/env python3

import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from openarmx_battery_monitor.serial_modbus_rtu import SerialModbusRtuClient
from openarmx_battery_monitor.serial_modbus_rtu import to_signed_16


class JDBatteryNode(Node):
    def __init__(self) -> None:
        super().__init__("jd_battery_node")

        self._serial_port = str(self.declare_parameter("serial_port", "/dev/ttyUSB0").value)
        raw_port_candidates = str(
            self.declare_parameter(
                "serial_port_candidates",
                "/dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2,/dev/ttyUSB3",
            ).value
        )
        self._baudrate = int(self.declare_parameter("baudrate", 19200).value)
        self._slave_id = int(self.declare_parameter("slave_id", 3).value)
        self._poll_hz = float(self.declare_parameter("poll_hz", 5.0).value)
        self._timeout_sec = float(self.declare_parameter("timeout_sec", 0.2).value)
        self._read_retries = max(0, int(self.declare_parameter("read_retries", 0).value))
        self._offline_after_failures = max(
            1, int(self.declare_parameter("offline_after_failures", 1).value)
        )
        self._start_register = int(self.declare_parameter("start_register", 114).value)
        self._register_count = int(self.declare_parameter("register_count", 5).value)
        self._battery_frame_id = str(self.declare_parameter("battery_frame_id", "battery_link").value)
        self._battery_state_topic = str(self.declare_parameter("battery_state_topic", "/battery_state").value)
        self._serial_port_candidates = self._build_port_candidates(
            primary=self._serial_port,
            csv_list=raw_port_candidates,
        )

        self._pub = self.create_publisher(BatteryState, self._battery_state_topic, 10)
        self._client = SerialModbusRtuClient(
            port=self._serial_port,
            baudrate=self._baudrate,
            timeout_sec=self._timeout_sec,
        )
        self._consecutive_failures = 0
        self._last_error_log_time = 0.0
        self._is_online = False

        period = 0.2 if self._poll_hz <= 0.0 else 1.0 / self._poll_hz
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"JD battery node started: port={self._serial_port}, baud={self._baudrate}, "
            f"slave_id={self._slave_id}, regs=[{self._start_register}.."
            f"{self._start_register + self._register_count - 1}], topic={self._battery_state_topic}, "
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
        return ports

    def _switch_to_available_port(self) -> None:
        active_port = self._client.port
        for candidate in self._serial_port_candidates:
            if not os.path.exists(candidate):
                continue
            if candidate != active_port:
                self.get_logger().warn(
                    f"Serial port switched: {active_port} -> {candidate}"
                )
                self._client.set_port(candidate)
                self._serial_port = candidate
            return

    def destroy_node(self) -> bool:
        self._client.close()
        return super().destroy_node()

    def _read_registers(self):
        last_exc: Exception | None = None
        attempts = 1 + self._read_retries
        for attempt in range(attempts):
            try:
                return self._client.read_holding_registers(
                    slave_id=self._slave_id,
                    start_addr=self._start_register,
                    register_count=self._register_count,
                )
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                if attempt + 1 < attempts:
                    time.sleep(0.03)
                    continue
                break
        raise last_exc if last_exc is not None else RuntimeError("Unknown read error")

    def _on_timer(self) -> None:
        try:
            values = self._read_registers()
        except Exception as exc:  # noqa: BLE001
            self._consecutive_failures += 1
            # Force a close so next cycle re-opens serial and can recover quickly after re-plug.
            self._client.close()
            self._switch_to_available_port()
            if self._consecutive_failures >= self._offline_after_failures:
                if self._is_online:
                    self.get_logger().warn("Battery link offline, waiting for reconnect...")
                self._is_online = False
                self._publish_offline()
            now = time.monotonic()
            if now - self._last_error_log_time >= 5.0:
                self.get_logger().warn(
                    f"Battery read failed: consecutive_failures={self._consecutive_failures}, "
                    f"error={exc}"
                )
                self._last_error_log_time = now
            return

        if len(values) < 5:
            self._consecutive_failures += 1
            self._client.close()
            if self._consecutive_failures >= self._offline_after_failures:
                if self._is_online:
                    self.get_logger().warn("Battery link offline, waiting for reconnect...")
                self._is_online = False
                self._publish_offline()
            return

        if not self._is_online:
            self.get_logger().info("Battery link recovered.")
        self._is_online = True
        self._consecutive_failures = 0
        voltage = values[0] * 0.01
        current = to_signed_16(values[1]) * 0.01
        soc = max(0.0, min(100.0, float(values[2])))
        self._publish_online(voltage=voltage, current=current, soc_percent=soc)

    def _publish_online(self, voltage: float, current: float, soc_percent: float) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame_id
        msg.voltage = voltage
        msg.current = current
        msg.charge = math.nan
        msg.capacity = math.nan
        msg.design_capacity = math.nan
        msg.percentage = soc_percent / 100.0
        msg.present = True
        if current > 0.2:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif current < -0.2:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self._pub.publish(msg)

    def _publish_offline(self) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame_id
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JDBatteryNode()
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
