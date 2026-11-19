#!/usr/bin/env python3
"""Drive one real isolated-MuJoCo placement through the skill gateway.

Minimal, non-injecting driver: verify attached state, request
``place_to_affordance`` for the single registered object on the validated left
table, poll the persisted result, then read the independent final state.
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
    request_id = arguments.request_id or datetime.now().strftime('mcp_place_%Y%m%d_%H%M%S_%f')
    events = []
    async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
        async with streamable_http_client(f'http://127.0.0.1:{arguments.port}/mcp', http_client=client) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                await session.initialize()

                async def call(name, parameters):
                    response = await session.call_tool(name, parameters)
                    if response.isError:
                        raise RuntimeError(str(response.content))
                    result = response.structuredContent or json.loads(
                        next(block.text for block in response.content if block.type == 'text'))
                    events.append({'time': datetime.now().astimezone().isoformat(), 'tool': name,
                                   'arguments': parameters, 'result': result})
                    (output / 'events.json').write_text(json.dumps(events, ensure_ascii=False, indent=2))
                    return result

                before = await call('get_robot_state', {})
                if before.get('holding') != arguments.object_id:
                    raise RuntimeError(f'Placement requires a fresh retained object; holding={before.get("holding")}')

                parameters = {'object_id': arguments.object_id, 'affordance_id': arguments.affordance_id,
                              'request_id': request_id, 'timeout_sec': arguments.timeout_sec}
                started = await call('place_to_affordance', parameters)

                deadline = time.monotonic() + arguments.timeout_sec + 30
                result = started
                while result['status'] == 'running':
                    if time.monotonic() > deadline:
                        raise RuntimeError('Placement observer timed out; robot status unknown, no implicit stop or retry')
                    await asyncio.sleep(0.5)
                    result = await call('get_skill_result', {'request_id': request_id})

                repeated = await call('place_to_affordance', parameters)
                if repeated != result:
                    raise RuntimeError('Repeated request did not return identical persisted result')

                after = await call('get_robot_state', {})
                passed = bool(result['status'] == 'succeeded' and result['success']
                              and result['stopped_confirmed'] and result.get('place_verified')
                              and after.get('holding') is None)
                return {'passed': passed, 'request_id': request_id, 'before': before, 'after': after,
                        'final_result': result, 'events': events,
                        'scope': 'real_MCP_MuJoCo_left_table_placement_NOT_physical'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--request-id')
    parser.add_argument('--object-id', default='thesis_item_01')
    parser.add_argument('--affordance-id', default='sim_delivery_top_left',
                        choices=['delivery_zone', 'sim_delivery_top_left'])
    parser.add_argument('--timeout-sec', type=float, default=600.0)
    parser.add_argument('--port', type=int, default=18556,
                        help='isolated MCP delivery skills gateway port')
    arguments = parser.parse_args()
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / (arguments.request_id or 'mcp_place_pending')
    output.mkdir(parents=True, exist_ok=True)
    report = asyncio.run(run(arguments, output))
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / report['request_id']
    output.mkdir(parents=True, exist_ok=True)
    (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps({'request_id': report['request_id'], 'passed': report['passed'],
                      'status': report['final_result'].get('status'),
                      'place_verified': report['final_result'].get('place_verified'),
                      'detail': report['final_result'].get('detail'),
                      'holding_after': report['after'].get('holding'),
                      'result_path': str(output / 'result.json')}, ensure_ascii=False))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
