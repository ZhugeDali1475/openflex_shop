#!/usr/bin/env python3

import os
import select
import termios
import time
from typing import List
from typing import Optional


def modbus_crc16(payload: bytes) -> int:
    crc = 0xFFFF
    for item in payload:
        crc ^= item
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def to_signed_16(raw: int) -> int:
    return raw - 0x10000 if raw & 0x8000 else raw


class ModbusError(RuntimeError):
    pass


class ModbusTimeout(ModbusError):
    pass


class SerialModbusRtuClient:
    _BAUD_MAP = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_sec: float,
        flush_before_write: bool = True,
    ) -> None:
        if baudrate not in self._BAUD_MAP:
            raise ValueError(f"Unsupported baudrate: {baudrate}")
        self._port = port
        self._baudrate = baudrate
        self._timeout_sec = timeout_sec
        self._flush_before_write = bool(flush_before_write)
        # Allow a grace period for trailing bytes of a frame, which helps on noisy/loaded buses.
        # On some USB-RS485 adapters the last CRC byte may arrive noticeably later.
        self._inter_byte_timeout_sec = max(0.05, min(0.35, timeout_sec * 1.5))
        self._fd = None

    @property
    def port(self) -> str:
        return self._port

    def set_port(self, port: str) -> None:
        if port == self._port:
            return
        self.close()
        self._port = port

    def open(self) -> None:
        if self._fd is not None:
            return

        fd = os.open(self._port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(fd)

        attrs[0] = 0
        attrs[1] = 0
        attrs[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        attrs[4] = self._BAUD_MAP[self._baudrate]
        attrs[5] = self._BAUD_MAP[self._baudrate]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.set_blocking(fd, False)
        self._fd = fd

    def close(self) -> None:
        if self._fd is None:
            return
        os.close(self._fd)
        self._fd = None

    def _ensure_open(self) -> None:
        if self._fd is None:
            self.open()

    def _read_available(self, max_bytes: int, timeout_sec: float) -> bytes:
        self._ensure_open()
        if timeout_sec <= 0.0:
            return b""
        readable, _, _ = select.select([self._fd], [], [], timeout_sec)
        if not readable:
            return b""
        try:
            return os.read(self._fd, max_bytes)
        except BlockingIOError:
            return b""

    def _write(self, payload: bytes) -> None:
        self._ensure_open()
        if self._flush_before_write:
            termios.tcflush(self._fd, termios.TCIFLUSH)
        view = memoryview(payload)
        deadline = time.monotonic() + self._timeout_sec
        while view:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ModbusTimeout("Timeout while writing Modbus request")
            _, writable, _ = select.select([], [self._fd], [], remaining)
            if not writable:
                continue
            try:
                written = os.write(self._fd, view)
            except BlockingIOError:
                continue
            if written <= 0:
                raise ModbusError("Serial write returned 0 bytes")
            view = view[written:]
        termios.tcdrain(self._fd)

    @staticmethod
    def _is_valid_crc(frame: bytes) -> bool:
        if len(frame) < 4:
            return False
        expected_crc = modbus_crc16(frame[:-2])
        recv_crc = frame[-2] | (frame[-1] << 8)
        return expected_crc == recv_crc

    def _extract_response_frame(
        self,
        rx_buffer: bytearray,
        slave_id: int,
        expected_byte_count: int,
    ) -> Optional[bytes]:
        while True:
            if len(rx_buffer) < 5:
                return None

            try:
                start = rx_buffer.index(slave_id)
            except ValueError:
                rx_buffer.clear()
                return None

            if start > 0:
                del rx_buffer[:start]
                if len(rx_buffer) < 5:
                    return None

            function_code = rx_buffer[1]
            if function_code == 0x83:
                frame_len = 5
                if len(rx_buffer) < frame_len:
                    return None
                frame = bytes(rx_buffer[:frame_len])
                if self._is_valid_crc(frame):
                    del rx_buffer[:frame_len]
                    return frame
                del rx_buffer[0]
                continue

            if function_code != 0x03:
                del rx_buffer[0]
                continue

            byte_count = rx_buffer[2]
            if byte_count != expected_byte_count:
                del rx_buffer[0]
                continue

            frame_len = 3 + byte_count + 2
            if len(rx_buffer) < frame_len:
                return None

            frame = bytes(rx_buffer[:frame_len])
            if self._is_valid_crc(frame):
                del rx_buffer[:frame_len]
                return frame

            del rx_buffer[0]

    def _read_response_frame(self, slave_id: int, expected_byte_count: int) -> bytes:
        now = time.monotonic()
        deadline = now + self._timeout_sec
        hard_deadline = now + (self._timeout_sec * 3.0)
        rx_buffer = bytearray()

        while True:
            frame = self._extract_response_frame(rx_buffer, slave_id, expected_byte_count)
            if frame is not None:
                return frame

            now = time.monotonic()
            if now >= hard_deadline:
                break
            remaining = deadline - now
            if remaining <= 0.0:
                if not rx_buffer:
                    break
                # Partial frame already received; extend waiting window for trailing bytes.
                deadline = min(hard_deadline, now + self._inter_byte_timeout_sec)
                remaining = deadline - now
                if remaining <= 0.0:
                    continue
            chunk = self._read_available(256, remaining)
            if chunk:
                rx_buffer.extend(chunk)
                # Once bytes start arriving, wait a bit for tail bytes of the same frame.
                deadline = min(
                    hard_deadline,
                    time.monotonic() + self._inter_byte_timeout_sec,
                )
                if len(rx_buffer) > 4096:
                    del rx_buffer[:-512]

        raise ModbusTimeout(
            f"Timeout waiting for Modbus response, slave_id={slave_id}, "
            f"buffered_bytes={len(rx_buffer)}, "
            f"buffered_hex={bytes(rx_buffer[:24]).hex(' ')}"
        )

    def _read_write_ack_frame(self, slave_id: int, start_addr: int, register_count: int) -> bytes:
        now = time.monotonic()
        deadline = now + self._timeout_sec
        hard_deadline = now + (self._timeout_sec * 3.0)
        rx_buffer = bytearray()

        expected_start_hi = (start_addr >> 8) & 0xFF
        expected_start_lo = start_addr & 0xFF
        expected_count_hi = (register_count >> 8) & 0xFF
        expected_count_lo = register_count & 0xFF

        while True:
            # Parse possible frames from sliding window.
            idx = 0
            while idx + 5 <= len(rx_buffer):
                if rx_buffer[idx] != slave_id:
                    idx += 1
                    continue
                fn = rx_buffer[idx + 1]
                if fn == 0x90:
                    if idx + 5 > len(rx_buffer):
                        break
                    frame = bytes(rx_buffer[idx : idx + 5])
                    if self._is_valid_crc(frame):
                        del rx_buffer[: idx + 5]
                        return frame
                    idx += 1
                    continue
                if fn != 0x10:
                    idx += 1
                    continue
                if idx + 8 > len(rx_buffer):
                    break
                frame = bytes(rx_buffer[idx : idx + 8])
                if not self._is_valid_crc(frame):
                    idx += 1
                    continue
                if (
                    frame[2] != expected_start_hi
                    or frame[3] != expected_start_lo
                    or frame[4] != expected_count_hi
                    or frame[5] != expected_count_lo
                ):
                    idx += 1
                    continue
                del rx_buffer[: idx + 8]
                return frame

            now = time.monotonic()
            if now >= hard_deadline:
                break
            remaining = deadline - now
            if remaining <= 0.0:
                if not rx_buffer:
                    break
                deadline = min(hard_deadline, now + self._inter_byte_timeout_sec)
                remaining = deadline - now
                if remaining <= 0.0:
                    continue
            chunk = self._read_available(256, remaining)
            if chunk:
                rx_buffer.extend(chunk)
                deadline = min(
                    hard_deadline,
                    time.monotonic() + self._inter_byte_timeout_sec,
                )
                if len(rx_buffer) > 4096:
                    del rx_buffer[:-512]

        raise ModbusTimeout(
            f"Timeout waiting for Modbus write ack, slave_id={slave_id}, "
            f"buffered_bytes={len(rx_buffer)}, "
            f"buffered_hex={bytes(rx_buffer[:24]).hex(' ')}"
        )

    def read_holding_registers(self, slave_id: int, start_addr: int, register_count: int) -> List[int]:
        if not 1 <= slave_id <= 247:
            raise ValueError(f"Invalid slave_id {slave_id}, must be [1, 247]")
        if not 1 <= register_count <= 125:
            raise ValueError(f"Invalid register_count {register_count}, must be [1, 125]")

        payload = bytes(
            [
                slave_id,
                0x03,
                (start_addr >> 8) & 0xFF,
                start_addr & 0xFF,
                (register_count >> 8) & 0xFF,
                register_count & 0xFF,
            ]
        )
        crc = modbus_crc16(payload)
        request = payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self._write(request)

        expected_byte_count = register_count * 2
        frame = self._read_response_frame(slave_id=slave_id, expected_byte_count=expected_byte_count)

        function_code = frame[1]
        if function_code == 0x83:
            exception_code = frame[2]
            raise ModbusError(f"Modbus exception response, code=0x{exception_code:02X}")
        if function_code != 0x03:
            raise ModbusError(f"Unexpected function code: 0x{function_code:02X}")

        byte_count = frame[2]
        if byte_count != expected_byte_count:
            raise ModbusError(f"Unexpected byte count: {byte_count}, expected {expected_byte_count}")

        raw_data = frame[3:-2]
        registers: List[int] = []
        for idx in range(0, len(raw_data), 2):
            registers.append((raw_data[idx] << 8) | raw_data[idx + 1])
        return registers

    def write_multiple_registers(self, slave_id: int, start_addr: int, values: List[int]) -> None:
        if not 1 <= slave_id <= 247:
            raise ValueError(f"Invalid slave_id {slave_id}, must be [1, 247]")
        register_count = len(values)
        if not 1 <= register_count <= 123:
            raise ValueError(f"Invalid register_count {register_count}, must be [1, 123]")
        for value in values:
            if not 0 <= value <= 0xFFFF:
                raise ValueError(f"Invalid register value {value}, must be [0, 65535]")

        byte_count = register_count * 2
        payload = bytearray(
            [
                slave_id,
                0x10,
                (start_addr >> 8) & 0xFF,
                start_addr & 0xFF,
                (register_count >> 8) & 0xFF,
                register_count & 0xFF,
                byte_count,
            ]
        )
        for value in values:
            payload.append((value >> 8) & 0xFF)
            payload.append(value & 0xFF)

        crc = modbus_crc16(bytes(payload))
        request = bytes(payload) + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        self._write(request)

        ack = self._read_write_ack_frame(
            slave_id=slave_id,
            start_addr=start_addr,
            register_count=register_count,
        )
        function_code = ack[1]
        if function_code == 0x90:
            exception_code = ack[2]
            raise ModbusError(f"Modbus write exception response, code=0x{exception_code:02X}")
        if function_code != 0x10:
            raise ModbusError(f"Unexpected write ack function code: 0x{function_code:02X}")
