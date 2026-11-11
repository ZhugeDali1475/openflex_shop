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
    records, trace = [], []
    prefix = arguments.request_prefix or datetime.now().strftime('delivery_skills_%Y%m%d_%H%M%S')
    async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
        async with streamable_http_client('http://127.0.0.1:18556/mcp', http_client=client) as (reader, writer, _):
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

                async def operation(tool, parameters, expected='succeeded', cancel_event=None):
                    started = await call(tool, parameters)
                    request_id = parameters['request_id']
                    if started['status'] == 'running':
                        competing = await call('grasp_object', {'object_id': 'thesis_item_01',
                            'request_id': request_id + '_busy', 'timeout_sec': 600.0})
                        if competing['status'] != 'rejected_busy':
                            raise RuntimeError('Shared cross-skill mutex failed')
                    deadline = time.monotonic() + parameters['timeout_sec'] + 30
                    canceled = False
                    while time.monotonic() < deadline:
                        result = await call('get_skill_result', {'request_id': request_id})
                        if result['status'] != 'running':
                            break
                        if cancel_event and not canceled and any(event['kind'] == cancel_event for event in result.get('events', [])):
                            await asyncio.sleep(0.75)
                            await call('cancel_current_task', {'request_id': request_id})
                            canceled = True
                        await asyncio.sleep(0.5)
                    else:
                        raise RuntimeError('Observer timed out: no automatic retry, reset or claim of stop')
                    records.append(result)
                    (output / 'terminal_results.json').write_text(json.dumps(records, ensure_ascii=False, indent=2))
                    if result['status'] != expected or not result.get('stopped_confirmed') or result['success'] != (expected == 'succeeded'):
                        raise RuntimeError(f'Unexpected terminal result for {request_id}: {result.get("status")}: {result.get("detail")}')
                    if cancel_event and not canceled:
                        raise RuntimeError('Requested real-motion cancellation was not exercised')
                    if await call(tool, parameters) != result:
                        raise RuntimeError('Same request ID changed or executed again')
                    return result

                if not arguments.skip_navigation:
                    nav = {'target_id': 'delivery_zone', 'timeout_sec': 1.0, 'request_id': prefix + '_nav_timeout'}
                    await operation('navigate_to_target', nav, 'timed_out')
                    nav.update(timeout_sec=90.0, request_id=prefix + '_nav_cancel')
                    await operation('navigate_to_target', nav, 'canceled', 'control_acquired')
                    nav.update(request_id=prefix + '_nav_success')
                    await operation('navigate_to_target', nav)
                place = {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone',
                         'timeout_sec': 1.0, 'request_id': prefix + '_place_timeout'}
                if not arguments.skip_faults:
                    await operation('place_to_affordance', place, 'timed_out')
                    place.update(timeout_sec=600.0, request_id=prefix + '_place_cancel')
                    await operation('place_to_affordance', place, 'canceled', 'trajectory_start')
                place.update(timeout_sec=600.0, request_id=prefix + '_place_success')
                result = await operation('place_to_affordance', place)
                state = await call('get_robot_state', {})
                if not result.get('place_verified') or state.get('holding') is not None:
                    raise RuntimeError('Real settled release and post-retreat placement not verified')
    return {'passed': True, 'request_prefix': prefix, 'terminal_results': records,
            'scope': 'actual_isolated_MuJoCo_skills_NOT_PHYSICAL_NOT_model_dispatch'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--request-prefix')
    parser.add_argument('--skip-navigation', action='store_true')
    parser.add_argument('--skip-faults', action='store_true')
    arguments = parser.parse_args()
    output = ROOT / 'local_dependencies/openflex_roboos/runs' / datetime.now().strftime('delivery_skills_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    report = {'passed': False}
    try:
        report = asyncio.run(run(arguments, output))
    except Exception as error:
        report['error'] = str(error)
        raise
    finally:
        (output / 'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print(output / 'result.json')


if __name__ == '__main__':
    main()
