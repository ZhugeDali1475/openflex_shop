#!/usr/bin/env python3

import math
import os
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from openarmx_battery_monitor.serial_modbus_rtu import SerialModbusRtuClient
from openarmx_battery_monitor.serial_modbus_rtu import to_signed_16


def _parse_csv(raw: str) -> list[str]:
    return [item.strip() for item in raw.split(",") if item.strip()]


def _parse_int_csv(raw: str) -> list[int]:
    values: list[int] = []
    for item in _parse_csv(raw):
        values.append(int(item))
    return values


@dataclass
class BatterySlot:
    battery_id: str
    topic: str
    slave_id: int | None
    publisher: object
    failures: int = 0
    successes: int = 0
    online: bool = False
    last_success_time: float = 0.0
    last_voltage: float = math.nan
    last_current: float = math.nan
    last_soc_percent: float = math.nan


class JDBatteryMultiNode(Node):
    def __init__(self) -> None:
        super().__init__("jd_battery_multi_node")

        self._serial_port = str(self.declare_parameter("serial_port", "/dev/ttyUSB0").value)
        raw_port_candidates = str(
            self.declare_parameter(
                "serial_port_candidates",
                "/dev/ttyUSB0,/dev/ttyUSB1,/dev/ttyUSB2,/dev/ttyUSB3",
            ).value
        )
        self._baudrate = int(self.declare_parameter("baudrate", 19200).value)
        self._poll_hz = float(self.declare_parameter("poll_hz", 5.0).value)
        self._timeout_sec = float(self.declare_parameter("timeout_sec", 0.2).value)
        self._flush_before_write = bool(
            self.declare_parameter("flush_before_write", False).value
        )
        self._read_retries = max(0, int(self.declare_parameter("read_retries", 0).value))
        self._offline_after_failures = max(
            1, int(self.declare_parameter("offline_after_failures", 3).value)
        )
        self._online_after_successes = max(
            1, int(self.declare_parameter("online_after_successes", 1).value)
        )
        self._stale_timeout_sec = max(
            0.05, float(self.declare_parameter("stale_timeout_sec", 0.8).value)
        )
        self._inter_request_gap_sec = max(
            0.0, float(self.declare_parameter("inter_request_gap_sec", 0.004).value)
        )
        self._reset_port_after_failures = max(
            1, int(self.declare_parameter("reset_port_after_failures", 4).value)
        )
        self._start_register = int(self.declare_parameter("start_register", 114).value)
        self._register_count = int(self.declare_parameter("register_count", 5).value)
        self._battery_frame_id = str(self.declare_parameter("battery_frame_id", "battery_link").value)

        raw_battery_ids = str(self.declare_parameter("battery_ids_csv", "battery_1,battery_2").value)
        raw_topics = str(
            self.declare_parameter(
                "battery_topics_csv",
                "/battery_1/battery_state,/battery_2/battery_state",
            ).value
        )
        raw_slave_ids = str(self.declare_parameter("slave_ids_csv", "").value)
        self._auto_discover = bool(self.declare_parameter("auto_discover_slave_ids", True).value)
        self._scan_min_id = max(1, int(self.declare_parameter("scan_min_id", 1).value))
        self._scan_max_id = min(247, int(self.declare_parameter("scan_max_id", 16).value))
        self._discover_interval_sec = max(
            0.5, float(self.declare_parameter("discover_interval_sec", 2.0).value)
        )

        self._serial_port_candidates = self._build_port_candidates(
            primary=self._serial_port, csv_list=raw_port_candidates
        )

        battery_ids = _parse_csv(raw_battery_ids)
        if not battery_ids:
            raise ValueError("battery_ids_csv must contain at least one battery id")
        topics = _parse_csv(raw_topics)
        if not topics:
            topics = [f"/{battery_id}/battery_state" for battery_id in battery_ids]
        if len(topics) != len(battery_ids):
            raise ValueError(
                f"battery_topics_csv count={len(topics)} does not match battery_ids_csv count={len(battery_ids)}"
            )
        configured_slave_ids = _parse_int_csv(raw_slave_ids)
        if configured_slave_ids and len(configured_slave_ids) != len(battery_ids):
            raise ValueError(
                f"slave_ids_csv count={len(configured_slave_ids)} does not match battery_ids_csv count={len(battery_ids)}"
            )

        self._client = SerialModbusRtuClient(
            port=self._serial_port,
            baudrate=self._baudrate,
            timeout_sec=self._timeout_sec,
            flush_before_write=self._flush_before_write,
        )
        self._last_discover_time = 0.0
        self._last_error_log_time = 0.0
        self._last_discover_warn_time = 0.0
        self._global_fail_streak = 0

        self._slots: list[BatterySlot] = []
        for idx, battery_id in enumerate(battery_ids):
            topic = topics[idx]
            slave_id = configured_slave_ids[idx] if configured_slave_ids else None
            pub = self.create_publisher(BatteryState, topic, 10)
            self._slots.append(
                BatterySlot(
                    battery_id=battery_id,
                    topic=topic,
                    slave_id=slave_id,
                    publisher=pub,
                )
            )

        if configured_slave_ids and len(set(configured_slave_ids)) != len(configured_slave_ids):
            self.get_logger().warn(
                "Configured slave_ids_csv has duplicate IDs. "
                "Two batteries on one RS485 bus must use unique slave IDs."
            )

        slot_count = max(1, len(self._slots))
        min_period = slot_count * (
            self._timeout_sec * (1 + self._read_retries) + self._inter_request_gap_sec
        )
        requested_period = 0.2 if self._poll_hz <= 0.0 else 1.0 / self._poll_hz
        period = max(requested_period, min_period)
        self.create_timer(period, self._on_timer)

        mapping = ", ".join(
            f"{slot.battery_id}(slave_id={slot.slave_id if slot.slave_id is not None else 'auto'})->{slot.topic}"
            for slot in self._slots
        )
        self.get_logger().info(
            f"JD multi-battery node started: port={self._serial_port}, baud={self._baudrate}, "
            f"regs=[{self._start_register}..{self._start_register + self._register_count - 1}], "
            f"auto_discover={self._auto_discover}, poll_period={period:.3f}s, "
            f"flush_before_write={self._flush_before_write}, "
            f"offline_after_failures={self._offline_after_failures}, "
            f"stale_timeout={self._stale_timeout_sec:.3f}s, mapping: {mapping}, "
            f"port_candidates={self._serial_port_candidates}"
        )

    @staticmethod
    def _build_port_candidates(primary: str, csv_list: str) -> list[str]:
        ports: list[str] = []
        if primary:
            ports.append(primary)
        for item in _parse_csv(csv_list):
            if item not in ports:
                ports.append(item)
        return ports

    def _switch_to_available_port(self) -> None:
        active_port = self._client.port
        for candidate in self._serial_port_candidates:
            if not os.path.exists(candidate):
                continue
            if candidate != active_port:
                self.get_logger().warn(f"Serial port switched: {active_port} -> {candidate}")
                self._client.set_port(candidate)
                self._serial_port = candidate
            return

    def destroy_node(self) -> bool:
        self._client.close()
        return super().destroy_node()

    def _read_registers(self, slave_id: int):
        last_exc: Exception | None = None
        attempts = 1 + self._read_retries
        for attempt in range(attempts):
            try:
                return self._client.read_holding_registers(
                    slave_id=slave_id,
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

    def _discover_slave_ids_if_needed(self, force: bool = False) -> None:
        if not self._auto_discover:
            return
        now = time.monotonic()
        if not force and now - self._last_discover_time < self._discover_interval_sec:
            return
        self._last_discover_time = now

        discovered: list[int] = []
        for slave_id in range(self._scan_min_id, self._scan_max_id + 1):
            try:
                values = self._read_registers(slave_id)
                if len(values) >= self._register_count:
                    discovered.append(slave_id)
            except Exception:
                continue
            if len(discovered) >= len(self._slots):
                break

        if not discovered:
            return

        for idx, slot in enumerate(self._slots):
            new_id = discovered[idx] if idx < len(discovered) else None
            if new_id is not None and slot.slave_id != new_id:
                self.get_logger().info(
                    f"Discovered slave mapping: {slot.battery_id} -> slave_id={new_id}"
                )
                slot.slave_id = new_id

        if len(discovered) < len(self._slots) and now - self._last_discover_warn_time >= 5.0:
            self.get_logger().warn(
                f"Only discovered {len(discovered)} unique slave ID(s) on bus. "
                f"Need {len(self._slots)} for multi-battery differentiation. "
                "If both batteries share one ID, they cannot be distinguished on the same RS485 bus."
            )
            self._last_discover_warn_time = now

    def _publish_online(
        self,
        slot: BatterySlot,
        voltage: float,
        current: float,
        soc_percent: float,
        cache_sample: bool = True,
    ) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame_id
        msg.location = slot.battery_id
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
        slot.publisher.publish(msg)
        if cache_sample:
            slot.last_voltage = voltage
            slot.last_current = current
            slot.last_soc_percent = soc_percent
            slot.last_success_time = time.monotonic()

    def _publish_offline(self, slot: BatterySlot) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._battery_frame_id
        msg.location = slot.battery_id
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
        slot.publisher.publish(msg)

    def _publish_last_good(self, slot: BatterySlot) -> None:
        if not math.isfinite(slot.last_soc_percent):
            return
        self._publish_online(
            slot=slot,
            voltage=slot.last_voltage,
            current=slot.last_current,
            soc_percent=slot.last_soc_percent,
            cache_sample=False,
        )

    def _on_timer(self) -> None:
        if any(slot.slave_id is None for slot in self._slots):
            self._discover_slave_ids_if_needed()

        for slot in self._slots:
            if slot.slave_id is None:
                slot.failures += 1
                slot.successes = 0
                if slot.failures >= self._offline_after_failures:
                    if slot.online and (time.monotonic() - slot.last_success_time) > self._stale_timeout_sec:
                        self.get_logger().warn(f"{slot.battery_id} offline: no slave_id mapped yet")
                    if (time.monotonic() - slot.last_success_time) > self._stale_timeout_sec:
                        slot.online = False
                        self._publish_offline(slot)
                continue

            try:
                values = self._read_registers(slot.slave_id)
            except Exception as exc:  # noqa: BLE001
                slot.failures += 1
                slot.successes = 0
                self._global_fail_streak += 1
                if self._global_fail_streak >= self._reset_port_after_failures:
                    self._client.close()
                    self._switch_to_available_port()
                    self._discover_slave_ids_if_needed(force=True)
                    self._global_fail_streak = 0

                now = time.monotonic()
                within_stale_window = (now - slot.last_success_time) <= self._stale_timeout_sec
                if within_stale_window:
                    self._publish_last_good(slot)
                elif slot.failures >= self._offline_after_failures:
                    if slot.online:
                        self.get_logger().warn(
                            f"{slot.battery_id} offline (slave_id={slot.slave_id}), waiting reconnect..."
                        )
                    slot.online = False
                    self._publish_offline(slot)
                if now - self._last_error_log_time >= 5.0:
                    self.get_logger().warn(
                        f"Battery read failed: id={slot.battery_id}, slave_id={slot.slave_id}, "
                        f"failures={slot.failures}, error={exc}"
                    )
                    self._last_error_log_time = now
                continue

            if len(values) < 5:
                slot.failures += 1
                slot.successes = 0
                self._global_fail_streak += 1
                if self._global_fail_streak >= self._reset_port_after_failures:
                    self._client.close()
                    self._switch_to_available_port()
                    self._global_fail_streak = 0

                if slot.failures >= self._offline_after_failures and (
                    time.monotonic() - slot.last_success_time
                ) > self._stale_timeout_sec:
                    if slot.online:
                        self.get_logger().warn(
                            f"{slot.battery_id} offline: short response from slave_id={slot.slave_id}"
                        )
                    slot.online = False
                    self._publish_offline(slot)
                continue

            self._global_fail_streak = 0
            slot.failures = 0
            slot.successes += 1
            voltage = values[0] * 0.01
            current = to_signed_16(values[1]) * 0.01
            soc = max(0.0, min(100.0, float(values[2])))
            if not slot.online and slot.successes >= self._online_after_successes:
                self.get_logger().info(f"{slot.battery_id} recovered on slave_id={slot.slave_id}")
                slot.online = True
            self._publish_online(slot=slot, voltage=voltage, current=current, soc_percent=soc)

            if self._inter_request_gap_sec > 0.0:
                time.sleep(self._inter_request_gap_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JDBatteryMultiNode()
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
