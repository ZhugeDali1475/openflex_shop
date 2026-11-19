#!/usr/bin/env python3
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
    request_id = arguments.request_id or datetime.now().strftime('mcp_grasp_%Y%m%d_%H%M%S_%f')
    events = []
    async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
        async with streamable_http_client(f'http://127.0.0.1:{arguments.port}/mcp', http_client=client) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                await session.initialize()

                async def call(name, parameters):
                    response = await session.call_tool(name, parameters)
                    if response.isError:
                        raise RuntimeError(str(response.content))
                    result = response.structuredContent or json.loads(next(block.text for block in response.content if block.type == 'text'))
                    events.append({'time': datetime.now().astimezone().isoformat(), 'tool': name,
                                   'arguments': parameters, 'result': result})
                    (output / 'events.json').write_text(json.dumps(events, ensure_ascii=False, indent=2))
                    return result

                parameters = {'object_id': 'thesis_item_01', 'request_id': request_id, 'timeout_sec': arguments.timeout_sec}
                started = await call('grasp_object', parameters)
                if started['status'] == 'running':
                    competing = await call('navigate_to_target', {'target_id': 'pickup_zone',
                        'request_id': request_id + '_busy', 'timeout_sec': 30})
                    if competing['status'] != 'rejected_busy':
                        raise RuntimeError('Cross-skill mutual exclusion failed')
                cancel_sent = False
                deadline = time.monotonic() + arguments.timeout_sec + 30
                while time.monotonic() < deadline:
                    result = await call('get_skill_result', {'request_id': request_id})
                    if result['status'] != 'running':
                        break
                    moving = any(event['kind'] == 'trajectory_start' for event in result.get('events', []))
                    if arguments.cancel_during_motion and moving and not cancel_sent:
                        await asyncio.sleep(0.5)
                        await call('cancel_current_task', {'request_id': request_id})
                        cancel_sent = True
                    await asyncio.sleep(0.5)
                else:
                    raise RuntimeError('Observer timed out; robot status unknown, no implicit stop or retry')
                repeated = await call('grasp_object', parameters)
                if repeated != result:
                    raise RuntimeError('Repeated request did not return identical persisted result')
                if arguments.cancel_during_motion:
                    passed = cancel_sent and result['status'] == 'canceled' and not result['success'] and result['stopped_confirmed']
                elif arguments.expect_timeout:
                    passed = result['status'] == 'timed_out' and not result['success'] and result['stopped_confirmed']
                else:
                    state = await call('get_robot_state', {})
                    passed = (result['status'] == 'succeeded' and result['success'] and result['stopped_confirmed']
                              and result.get('grasp_verified') and result.get('transport_safe')
                              and state.get('holding') == 'thesis_item_01')
                return {'passed': bool(passed), 'request_id': request_id, 'final_result': result,
                        'cancel_sent': cancel_sent, 'events': events,
                        'scope': 'real_MCP_MuJoCo_grasp_NOT_physical_NOT_RoboOS_delivery_loop'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--request-id')
    parser.add_argument('--timeout-sec', type=float, default=600.0)
    parser.add_argument('--port', type=int, default=18555,
                        help='isolated MCP skills gateway port (18555 grasp-only, 18556 delivery)')
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--cancel-during-motion', action='store_true')
    modes.add_argument('--expect-timeout', action='store_true')
    arguments = parser.parse_args()
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('mcp_grasp_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    report = {'passed': False}
    try:
        report = asyncio.run(run(arguments, output))
        if not report['passed']:
            raise RuntimeError('Grasp validation failed; inspect real results, do not infer successful transport')
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(output / 'result.json')


if __name__ == '__main__':
    main()
