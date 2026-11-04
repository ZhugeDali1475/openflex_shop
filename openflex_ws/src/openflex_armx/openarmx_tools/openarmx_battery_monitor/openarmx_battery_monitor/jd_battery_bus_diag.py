#!/usr/bin/env python3

import argparse
import math
from dataclasses import dataclass

from openarmx_battery_monitor.serial_modbus_rtu import SerialModbusRtuClient
from openarmx_battery_monitor.serial_modbus_rtu import to_signed_16


def _parse_slave_ids(spec: str) -> list[int]:
    ids: set[int] = set()
    for token in (item.strip() for item in spec.split(",") if item.strip()):
        if "-" in token:
            a, b = token.split("-", 1)
            start = int(a)
            end = int(b)
            if start <= end:
                seq = range(start, end + 1)
            else:
                seq = range(start, end - 1, -1)
            for sid in seq:
                if 1 <= sid <= 247:
                    ids.add(sid)
        else:
            sid = int(token)
            if 1 <= sid <= 247:
                ids.add(sid)
    return sorted(ids)


@dataclass
class DiagStat:
    slave_id: int
    success: int = 0
    fail: int = 0
    last_error: str = ""
    last_voltage: float = math.nan
    last_current: float = math.nan
    last_soc: float = math.nan

    @property
    def total(self) -> int:
        return self.success + self.fail

    @property
    def success_rate(self) -> float:
        if self.total == 0:
            return 0.0
        return self.success / self.total


def main() -> int:
    parser = argparse.ArgumentParser(
        description="JD battery RS485 bus diagnostic scanner."
    )
    parser.add_argument("--serial-port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--timeout-sec", type=float, default=0.25)
    parser.add_argument("--flush-before-write", action="store_true", default=False)
    parser.add_argument("--start-register", type=int, default=114)
    parser.add_argument("--register-count", type=int, default=5)
    parser.add_argument(
        "--slave-ids",
        default="1-16",
        help="Comma/range list, e.g. 3,4 or 1-16",
    )
    parser.add_argument("--passes", type=int, default=5)
    parser.add_argument("--read-retries", type=int, default=1)
    args = parser.parse_args()

    ids = _parse_slave_ids(args.slave_ids)
    if not ids:
        print("[diag] no valid slave ids parsed")
        return 2

    stats: dict[int, DiagStat] = {sid: DiagStat(slave_id=sid) for sid in ids}
    client = SerialModbusRtuClient(
        port=args.serial_port,
        baudrate=args.baudrate,
        timeout_sec=args.timeout_sec,
        flush_before_write=args.flush_before_write,
    )
    print(
        "[diag] start:",
        f"port={args.serial_port},",
        f"baud={args.baudrate},",
        f"timeout={args.timeout_sec:.3f}s,",
        f"regs=[{args.start_register}..{args.start_register + args.register_count - 1}],",
        f"ids={ids},",
        f"passes={args.passes},",
        f"retries={args.read_retries},",
        f"flush_before_write={args.flush_before_write}",
    )

    try:
        for _ in range(max(1, args.passes)):
            for sid in ids:
                stat = stats[sid]
                ok = False
                last_exc: Exception | None = None
                for _try in range(max(1, args.read_retries + 1)):
                    try:
                        values = client.read_holding_registers(
                            slave_id=sid,
                            start_addr=args.start_register,
                            register_count=args.register_count,
                        )
                        if len(values) >= 3:
                            stat.last_voltage = values[0] * 0.01
                            stat.last_current = to_signed_16(values[1]) * 0.01
                            stat.last_soc = float(values[2])
                        stat.success += 1
                        ok = True
                        break
                    except Exception as exc:  # noqa: BLE001
                        last_exc = exc
                        client.close()
                if not ok:
                    stat.fail += 1
                    stat.last_error = str(last_exc) if last_exc else "unknown"
    finally:
        client.close()

    print("[diag] summary:")
    any_success = False
    for sid in ids:
        st = stats[sid]
        if st.success > 0:
            any_success = True
            print(
                f"[diag] id={sid:>3}  ok={st.success:>2}/{st.total:<2} "
                f"({st.success_rate * 100:>5.1f}%)  "
                f"V={st.last_voltage:>6.2f}  I={st.last_current:>6.2f}  SOC={st.last_soc:>5.1f}"
            )
        else:
            print(
                f"[diag] id={sid:>3}  ok={st.success:>2}/{st.total:<2} "
                f"({st.success_rate * 100:>5.1f}%)  err={st.last_error}"
            )

    if not any_success:
        print("[diag] result: no responding slave found")
        print("[diag] checklist:")
        print("[diag] 1) 485A/B是否接反；电池A->转换器A(+), 电池B->转换器B(-)")
        print("[diag] 2) 两块电池需唯一地址；同地址无法区分")
        print("[diag] 3) 确认电池通信已唤醒，识别线/使能线状态正确")
        print("[diag] 4) 尝试只接1块电池单测，再并上第2块")
        return 2

    print("[diag] result: responding slave detected")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
