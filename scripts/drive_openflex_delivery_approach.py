#!/usr/bin/env python3
"""Carry-navigate the attached red object to one delivery station, then stop.

This is the isolated-simulation "navigate_to_target" atom only. It never places,
releases or resets; a caller must re-probe the placement afterwards. Evidence is
written to a new runs directory.
"""
import argparse
import asyncio
from datetime import datetime
import json
from pathlib import Path
import time

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client


ROOT = Path(__file__).resolve().parents[1]


async def run(arguments, output):
    request_id = arguments.request_id or datetime.now().strftime('carry_nav_%Y%m%d_%H%M%S_%f')
    trace = []
    async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
        async with streamable_http_client(f'http://127.0.0.1:{arguments.port}/mcp', http_client=client) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                await session.initialize()

                async def call(tool, parameters):
                    response = await session.call_tool(tool, parameters)
                    if response.isError:
                        raise RuntimeError(str(response.content))
                    result = response.structuredContent or json.loads(next(block.text for block in response.content if block.type == 'text'))
                    trace.append({'time': datetime.now().astimezone().isoformat(), 'tool': tool,
                                  'arguments': dict(parameters), 'result': result})
                    (output / 'events.json').write_text(json.dumps(trace, ensure_ascii=False, indent=2))
                    return result

                started = await call('navigate_to_target', {'target_id': arguments.target,
                    'request_id': request_id, 'timeout_sec': arguments.timeout_sec})
                deadline = time.monotonic() + arguments.timeout_sec + 30
                result = started
                while time.monotonic() < deadline:
                    result = await call('get_navigation_result', {'request_id': request_id})
                    if result['status'] != 'running':
                        break
                    await asyncio.sleep(0.5)
                else:
                    raise RuntimeError('Observer timed out; robot status unknown, no retry')
                state = await call('get_robot_state', {})
                passed = (result['status'] == 'succeeded' and result['success']
                          and result.get('stopped_confirmed') and state.get('holding') == 'thesis_item_01')
        return {'passed': bool(passed), 'request_id': request_id, 'final_result': result,
                'holding_after': state.get('holding'), 'events': trace,
                'scope': 'isolated_MuJoCo_carry_navigation_only_no_place_NOT_PHYSICAL'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='delivery_zone')
    parser.add_argument('--timeout-sec', type=float, default=90.0)
    parser.add_argument('--request-id')
    parser.add_argument('--port', type=int, default=18556)
    arguments = parser.parse_args()
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('carry_nav_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    report = {'passed': False}
    try:
        report = asyncio.run(run(arguments, output))
        if not report['passed']:
            raise RuntimeError('Carry navigation did not reach a stopped, holding terminal state')
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(output / 'result.json')


if __name__ == '__main__':
    main()
