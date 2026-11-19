#!/usr/bin/env python3
"""Read-only high-rate sampler of the MuJoCo control acknowledgement.

Polls the isolated readonly snapshot (18552) and records the diagnostic
``control_ack`` (mode/epoch/inputs_healthy/fresh/arm_control_valid plus the
numeric ``mode_receipt_age_sec``/``actual_twist`` that a bare "revoked" hides)
alongside the object/attachment state.  It never sends motion or MCP commands;
it exists to locate why ``require_control_ack`` reports a revoked acknowledgement.

A record is written on every change of the acknowledgement signature *and* on
every crossing of the stationary-base gate, so a short base-motion excursion or a
receipt that ages past ``fresh`` is captured with the age and twist that caused it.
"""
import argparse
from datetime import datetime
import json
import math
import time
from urllib.request import ProxyHandler, Request, build_opener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=300.0)
    parser.add_argument('--rate-hz', type=float, default=10.0)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    opener = build_opener(ProxyHandler({}))
    period = 1.0 / args.rate_hz
    deadline = time.monotonic() + args.duration
    handle = open(args.output, 'w')
    records = 0
    previous = None
    try:
        while time.monotonic() < deadline:
            started = time.monotonic()
            try:
                with opener.open(Request('http://127.0.0.1:18552/state'), timeout=2) as response:
                    snapshot = json.load(response)
                diagnostic = snapshot['sources']['simulation_diagnostic']['data']
                ack = diagnostic.get('control_ack') or {}
                twist = list(ack.get('actual_twist') or [0.0, 0.0, 0.0])
                moving = math.hypot(twist[0], twist[1]) > 0.02 or abs(twist[2]) > 0.03
                record = {
                    't': round(started, 4), 'wall': datetime.now().astimezone().isoformat(),
                    'mode': ack.get('mode'), 'epoch': ack.get('epoch'),
                    'inputs_healthy': ack.get('inputs_healthy'), 'fresh': ack.get('fresh'),
                    'arm_control_valid': ack.get('arm_control_valid'),
                    'place_control_valid': ack.get('place_control_valid'),
                    'mode_receipt_age_sec': ack.get('mode_receipt_age_sec'),
                    'receipt_gap_sec': ack.get('receipt_gap_sec'),
                    'receipt_gap_max_sec': ack.get('receipt_gap_max_sec'),
                    'publisher_stamp_gap_sec': ack.get('publisher_stamp_gap_sec'),
                    'publisher_stamp_gap_max_sec': ack.get('publisher_stamp_gap_max_sec'),
                    'actual_twist': twist, 'base_out_of_gate': moving,
                    'attached_arm': diagnostic.get('attached_arm'),
                    'delivery_support_destination_id': diagnostic.get('delivery_support_destination_id'),
                    'safety_stop': diagnostic.get('safety_stop') is not None,
                }
            except Exception as error:
                record = {'t': round(started, 4), 'wall': datetime.now().astimezone().isoformat(),
                          'error': str(error)}
            signature = {k: record.get(k) for k in ('mode', 'epoch', 'inputs_healthy', 'fresh',
                                                    'arm_control_valid', 'attached_arm',
                                                    'safety_stop', 'base_out_of_gate',
                                                    'receipt_gap_max_sec')}
            if signature != previous:
                handle.write(json.dumps(record, ensure_ascii=False) + '\n')
                handle.flush()
                records += 1
                previous = signature
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        handle.close()
        print(f'wrote {records} transitions to {args.output}')


if __name__ == '__main__':
    main()
