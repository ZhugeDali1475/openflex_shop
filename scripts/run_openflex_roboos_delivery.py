#!/usr/bin/env python3
import argparse
import asyncio
from copy import deepcopy
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
from types import SimpleNamespace

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client
from mcp.types import CallToolResult, TextContent


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'local_dependencies/roboos_client'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco'))
from flag_scale.flagscale.agent.collaboration import Collaborator
from openflex_delivery_contract import DeliveryContract, MOTION_TOOLS, state_summary, terminal_summary
from openflex_delivery_experiment import reject_stale_frame, run_fixed
from openflex_runner_safety import cancel_and_audit, check_stop, validate_selection


MODEL = 'robobrain2.5-4b-gguf'
REDIS_PORT = 16383


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def serial_selection(answer):
    if answer.tool_calls:
        answer.tool_calls = answer.tool_calls[:1]
    return answer


def failure_detail(error):
    nested = getattr(error, 'exceptions', None)
    if nested:
        return '; '.join(failure_detail(item) for item in nested)
    return f'{type(error).__name__}: {error}'


def collaboration():
    return Collaborator(host='127.0.0.1', port=REDIS_PORT, db=0, clear=False)


def master(arguments):
    sys.path.insert(0, str(ROOT / 'local_dependencies/RoboOS/master'))
    import threading
    import yaml
    from agents.agent import GlobalAgent
    import agents.planner as planner_module

    output = arguments.output
    planner_module.MASTER_PLANNING_PLANNING = '''You are the RoboOS Master for one isolated simulation delivery.
Registered robots: {robot_name_list}. Their actual tools: {robot_tools_info}. Measured/static scene: {scene_info}.
Decompose the user's task into exactly ONE complete end-to-end delivery subtask for openflex, not individual skills.
Do not invent success, agents, objects or destinations. The Worker will choose skills using actual camera/tool feedback.
Output JSON with reasoning_explanation and subtask_list containing exactly one object:
{{"reasoning_explanation":"brief reason","subtask_list":[{{"robot_name":"openflex","subtask":"preserve the user's delivery objective","subtask_order":1}}]}}
User task: {task}'''

    class BoundedMaster(GlobalAgent):
        def _extract_json(self, text):
            try:
                return json.loads(text)
            except (ValueError, TypeError):
                return super()._extract_json(text)

        def __init__(self, config):
            self.finished = threading.Event()
            self.result = None
            super().__init__(config)

        def reasoning_and_subtasks_is_right(self, plan):
            return bool(super().reasoning_and_subtasks_is_right(plan) and len(plan['subtask_list']) == 1
                and plan['subtask_list'][0].get('robot_name') == 'openflex'
                and isinstance(plan['subtask_list'][0].get('subtask'), str)
                and plan['subtask_list'][0].get('subtask_order') == 1)

        def _handle_result(self, text):
            data = json.loads(text)
            if data.get('task_id') != arguments.task_id or data.get('robot_name') != 'openflex':
                return
            result = data.get('subtask_result')
            if not isinstance(result, dict):
                return
            self.result = data
            save(output / 'master_received_result.json', data)
            self.collaborator.update_agent_busy('openflex', False)
            self.finished.set()

        async def _dispath_subtasks_async(self, task, task_id, groups, refresh):
            try:
                tasks = [task for group in groups.values() for task in group]
                if len(tasks) != 1 or tasks[0]['robot_name'] != 'openflex':
                    raise RuntimeError('Only one bounded delivery subtask may be dispatched')
                self.collaborator.update_agent_busy('openflex', True)
                sent = self.collaborator.send('roboos_to_openflex', json.dumps({
                    'task_id': task_id, 'task': tasks[0]['subtask'], 'order': 'false'}))
                if not sent:
                    raise RuntimeError('No real Worker Redis subscription acknowledged dispatch')
                if not self.finished.wait(timeout=1200):
                    raise TimeoutError('Bounded master did not receive Worker terminal evidence')
            except Exception as error:
                self.result = {'task_id': task_id, 'success': False, 'error': str(error)}
                save(output / 'master_dispatch_error.json', self.result)
                self.finished.set()

    config = yaml.safe_load((ROOT / 'local_dependencies/RoboOS/master/config.yaml').read_text())
    config['collaborator'].update(host='127.0.0.1', port=REDIS_PORT, db=0, clear=False)
    config['profiling'] = False
    config['model']['model_retry_planning'] = 1
    config['logger']['master_logger_file'] = str(output / 'master.log')
    scene = output / 'scene.yaml'
    scene.write_text(yaml.safe_dump({'scene': [{'name': 'isolated_openflex', 'object_id': 'thesis_item_01',
        'pickup': 'pickup_zone', 'destination': 'delivery_zone', 'physical_verification': False,
        'identity': 'registered simulation object, not visually recognized identity'}]}))
    config['profile']['path'] = str(scene)
    configuration = output / 'master_config.yaml'
    configuration.write_text(yaml.safe_dump(config))
    instance = BoundedMaster(str(configuration))
    original_completion = instance.planner.global_model.chat.completions.create

    def bounded_completion(**kwargs):
        kwargs['max_tokens'] = 512
        kwargs['response_format'] = {'type': 'json_schema', 'json_schema': {
            'name': 'isolated_delivery_plan', 'strict': True, 'schema': {
                'type': 'object', 'additionalProperties': False,
                'properties': {'reasoning_explanation': {'type': 'string'}, 'subtask_list': {
                    'type': 'array', 'minItems': 1, 'maxItems': 1, 'items': {
                        'type': 'object', 'additionalProperties': False,
                        'properties': {'robot_name': {'type': 'string', 'enum': ['openflex']},
                            'subtask': {'type': 'string', 'enum': [arguments.task]},
                            'subtask_order': {'type': 'integer', 'enum': [1]}},
                        'required': ['robot_name', 'subtask', 'subtask_order']}}},
                'required': ['reasoning_explanation', 'subtask_list']}}}
        response = original_completion(**kwargs)
        save(output / 'master_model_response.json', {'request': kwargs, 'response': response.model_dump()})
        return response

    instance.planner.global_model = SimpleNamespace(chat=SimpleNamespace(
        completions=SimpleNamespace(create=bounded_completion)))
    instance._handle_register('openflex')
    time.sleep(0.6)
    plan = instance.publish_global_task(arguments.task, False, arguments.task_id)
    if not instance.reasoning_and_subtasks_is_right(plan):
        raise RuntimeError('Real Master model did not provide a permitted single-item delivery plan')
    save(output / 'master_plan.json', plan)
    if not instance.finished.wait(timeout=1210):
        raise TimeoutError('Master completion remains unknown')


async def worker(arguments):
    sys.path.insert(0, str(ROOT / 'local_dependencies/RoboOS/slaver'))
    from agents.models import OpenAIServerModel
    from agents.slaver_agent import ToolCallingAgent

    output = arguments.output
    contract = DeliveryContract()
    trace, model_trace, active = [], [], None
    fault_applied = False
    registry = collaboration()
    connection = registry._get_conn()
    connection.ping()
    lock = connection.lock('openflex_delivery_single_scheduler', timeout=1800, blocking=False)
    if not lock.acquire(blocking=False):
        raise RuntimeError('Another bounded RoboOS delivery scheduler owns the Redis lease')
    from openflex_runtime import save as atomic_save, RUNTIME
    atomic_save(RUNTIME / 'current_scheduler.json', {'pid': os.getpid(), 'output': str(output.resolve()),
                                                   'task_id': arguments.task_id})
    process = None
    pubsub = connection.pubsub()
    pubsub.subscribe('roboos_to_openflex')
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        acknowledgement = pubsub.get_message(timeout=0.5)
        if acknowledgement and acknowledgement['type'] == 'subscribe':
            break
    else:
        raise RuntimeError('Worker subscription was not acknowledged')
    result = {'success': False, 'status': 'not_completed', 'task_id': arguments.task_id}
    try:
        async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
            async with streamable_http_client('http://127.0.0.1:18556/mcp', http_client=client) as (reader, writer, _):
                async with ClientSession(reader, writer) as session:
                    await session.initialize()
                    advertised = {tool.name: tool for tool in (await session.list_tools()).tools}
                    names = ['get_robot_state', 'observe_scene', 'navigate_to_target', 'grasp_object', 'place_to_affordance']
                    tools = []
                    for name in names:
                        tool = advertised[name]
                        schema = deepcopy(tool.inputSchema)
                        schema['properties'].pop('request_id', None)
                        schema['properties'].pop('timeout_sec', None)
                        schema['required'] = [key for key in schema.get('required', []) if key not in ('request_id', 'timeout_sec')]
                        schema['additionalProperties'] = False
                        descriptions = {'get_robot_state': 'Read actual fresh state; also required after placement to independently verify completion.',
                            'observe_scene': 'Read actual fresh camera image plus explicitly labeled simulation Oracle object.',
                            'navigate_to_target': 'Navigate to pickup_zone before grasp, delivery_zone only after verified grasp. Blocks until measured terminal result.',
                            'grasp_object': 'Guarded grasp of thesis_item_01 after pickup observation. Blocks for full checked grasp and transport posture.',
                            'place_to_affordance': 'Actually place thesis_item_01 on delivery_zone after verified delivery navigation. Blocks through contact/settling/retraction.'}
                        tools.append({'type': 'function', 'function': {'name': name, 'description': descriptions[name], 'parameters': schema}})
                    save(output / 'model_tool_contracts.json', tools)
                    registry.register_agent('openflex', json.dumps({'robot_name': 'openflex', 'robot_state': 'idle',
                        'robot_tool': names, 'physical_verification': False}), expire_second=1800)
                    environment = dict(os.environ)
                    for key in list(environment):
                        if key.lower().endswith('_proxy'):
                            environment.pop(key)
                    if arguments.policy == 'model':
                        process = subprocess.Popen([sys.executable, '-I', str(Path(__file__).resolve()), '--role', 'master',
                            '--output', str(output), '--task-id', arguments.task_id, '--task', arguments.task],
                            cwd=ROOT, env=environment, stdout=(output / 'master_stdout.log').open('w'), stderr=subprocess.STDOUT)
                        message = None
                        for attempt in range(240):
                            message = pubsub.get_message(timeout=0.1)
                            if message and message['type'] == 'message':
                                break
                            if process.poll() is not None:
                                raise RuntimeError('Master exited before dispatch; inspect master_stdout.log')
                            await asyncio.sleep(0.15)
                        if not message or message['type'] != 'message':
                            raise TimeoutError('No model-planned subtask received via actual Redis pub/sub')
                        subtask = json.loads(message['data'])
                        if subtask.get('task_id') != arguments.task_id:
                            raise RuntimeError('Unexpected task ID on isolated delivery channel')
                        save(output / 'worker_received_subtask.json', subtask)

                    async def call_mcp(name, parameters):
                        response = await session.call_tool(name, parameters)
                        if response.isError:
                            raise RuntimeError(f'MCP error: {response.content}')
                        return response

                    def extract(response):
                        return response.structuredContent or json.loads(next(block.text for block in response.content if block.type == 'text'))

                    async def execute(name, parameters):
                        nonlocal active, fault_applied
                        check_stop(output)
                        validate_selection(tools, name, parameters)
                        try:
                            contract.validate(name, parameters)
                        except ValueError as error:
                            rejected = {'status': 'rejected_before_execution', 'success': False,
                                'detail': str(error), 'no_motion_sent': True}
                            trace.append({'selected_tool': name, 'model_arguments': parameters,
                                'raw_result': rejected})
                            save(output / 'tool_trace.json', trace)
                            return CallToolResult(content=[TextContent(type='text', text=json.dumps(rejected))])
                        backend = dict(parameters)
                        if name in MOTION_TOOLS:
                            backend.update(request_id=arguments.task_id + f'_skill_{len(trace):02d}',
                                           timeout_sec=90.0 if name == 'navigate_to_target' else 600.0)
                            active = backend['request_id']
                        response = await call_mcp(name, backend)
                        raw = extract(response)
                        image_blocks = [block for block in response.content if block.type == 'image']
                        if name == 'observe_scene' and arguments.inject_stale_once and not fault_applied:
                            rejected = reject_stale_frame(raw, len(image_blocks))
                            fault_applied = True
                            trace.append({'time': datetime.now().astimezone().isoformat(), 'selected_tool': name,
                                'model_arguments': parameters, 'backend_arguments': backend, 'raw_result': rejected,
                                'pre_injection_camera_status': 'fresh', 'image_count': 0})
                            save(output / 'tool_trace.json', trace)
                            return CallToolResult(content=[TextContent(type='text', text=json.dumps(rejected))])
                        trace.append({'time': datetime.now().astimezone().isoformat(), 'selected_tool': name,
                            'model_arguments': parameters, 'backend_arguments': backend, 'raw_result': raw,
                            'image_count': len(image_blocks)})
                        save(output / 'tool_trace.json', trace)
                        if name in MOTION_TOOLS:
                            limit = time.monotonic() + backend['timeout_sec'] + 30
                            while raw.get('status') == 'running' and time.monotonic() < limit:
                                check_stop(output)
                                await asyncio.sleep(0.5)
                                raw = extract(await call_mcp('get_skill_result', {'request_id': active}))
                            trace[-1]['terminal_result'] = raw
                            save(output / 'tool_trace.json', trace)
                            if raw.get('status') not in ('succeeded', 'failed', 'timed_out', 'canceled') or raw.get('stopped_confirmed') is not True:
                                raise RuntimeError('Unknown terminal state: hold same request ID; do not retry')
                            active = None
                        contract.record(name, parameters, raw, len(image_blocks))
                        if name == 'observe_scene':
                            camera = raw.get('sources', {}).get('camera', {})
                            compact = {'actual_camera': camera.get('status'), 'object_id': 'thesis_item_01',
                                'identity_basis': 'fresh registered ROS object, Oracle pose, NOT visual recognition'}
                        else:
                            compact = terminal_summary(raw) if name in MOTION_TOOLS else state_summary(raw)
                        return CallToolResult(content=[TextContent(type='text', text=json.dumps(compact, ensure_ascii=False)), *image_blocks])

                    if arguments.policy == 'fixed':
                        result = await run_fixed(execute, contract)
                        result.update(task_id=arguments.task_id, physical_verification=False,
                            master_worker_redis=False, final_measured_completion=contract.completed,
                            passed=contract.completed, policy='fixed',
                            scope='fixed_dispatch_same_MCP_ROS_MuJoCo_skills_NOT_PHYSICAL')
                        return result

                    model = OpenAIServerModel(model_id=MODEL, api_base='http://127.0.0.1:4567/v1/', api_key='EMPTY',
                        support_tool_calls=True, profiling=False, max_tokens=256,
                        parallel_tool_calls=False, tool_choice='required',
                        client_kwargs={'http_client': httpx.Client(trust_env=False, timeout=120), 'max_retries': 0})

                    def bounded_model(**kwargs):
                        check_stop(output)
                        history = kwargs['messages']
                        history = [item for index, item in enumerate(history)
                            if not (isinstance(item.get('content'), list) and index < len(history) - 4)]
                        kwargs['messages'] = history
                        inference_started = time.monotonic()
                        answer = model(**kwargs)
                        inference_seconds = time.monotonic() - inference_started
                        check_stop(output)
                        model_trace.append({'time': datetime.now().astimezone().isoformat(), 'messages': history,
                            'inference_seconds': inference_seconds,
                            'content': answer.content, 'tool_calls': [{'name': call.function.name,
                                'arguments': call.function.arguments, 'id': call.id} for call in answer.tool_calls or []],
                            'dispatch_policy': 'first_model_call_only_discard_rest_then_reinfer_from_actual_feedback',
                            'usage': model.get_token_counts()})
                        save(output / 'model_trace.json', model_trace)
                        return serial_selection(answer)

                    prompt = '''You control ONLY isolated OpenFlex MuJoCo, never physical hardware.
Complete one-item delivery using actual tool feedback. Select exactly ONE tool per response, never parallel calls.
Required safety order: get_robot_state, observe_scene (actual image), navigate_to_target(pickup_zone),
grasp_object(thesis_item_01), navigate_to_target(delivery_zone),
place_to_affordance(thesis_item_01, delivery_zone), get_robot_state for independent final confirmation.
These are prerequisites, not evidence of success: every move must return succeeded, success=true and stopped_confirmed=true.
Motion tools BLOCK until terminal feedback. Do not generate request_id or timeout_sec; the host creates auditable IDs.
Unknown, failed, timed_out, canceled, missing or stale evidence means stop, never success or implicit retries.
Object identity comes from registered simulation ROS ID, not visual recognition; Oracle is not a detector.
Do not imagine updates. Only the external final measured completion verifier can end the task successfully.'''
                    instance = ToolCallingAgent(tools=tools, model=bounded_model, model_path=MODEL,
                        collaborator=registry, tool_executor=execute, robot_name='openflex', max_steps=12,
                        verbosity_level=0, log_file=str(output / 'worker.log'),
                        completion_verifier=lambda: contract.completed, system_prompt=prompt)
                    try:
                        result = await instance.run(subtask['task'])
                        result.update(task_id=arguments.task_id, physical_verification=False,
                            master_worker_redis=True, final_measured_completion=contract.completed)
                    finally:
                        model.client.close()
    except BaseException as error:
        (output / 'failure_traceback.txt').write_text(traceback.format_exc())
        result.update(success=False, status='failed', error=failure_detail(error), final_measured_completion=False)
        if active is not None:
            async with httpx.AsyncClient(trust_env=False, timeout=10) as cleanup:
                audit = await cancel_and_audit(cleanup, active, output)
                result['stopped_confirmed'] = audit['stopped_confirmed']
                if not audit['stopped_confirmed']:
                    result['status'] = 'unknown'
        if isinstance(error, (KeyboardInterrupt, asyncio.CancelledError)) and result['status'] != 'unknown':
            result['status'] = 'interrupted'
    finally:
        save(output / 'worker_result.json', result)
        if arguments.policy == 'model':
            registry.send('openflex_to_RoboOS', json.dumps({'robot_name': 'openflex', 'task_id': arguments.task_id,
                'subtask_handle': arguments.task, 'subtask_result': result}))
        if process is not None:
            for attempt in range(40):
                if process.poll() is not None:
                    break
                await asyncio.sleep(0.1)
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        pubsub.close()
        lock.release()
        connection.close()
    master_result = output / 'master_received_result.json'
    received = json.loads(master_result.read_text()) if master_result.exists() else {}
    result['passed'] = bool(result.get('success') and result.get('final_measured_completion')
        and (arguments.policy == 'fixed' or (received.get('subtask_result', {}).get('success')
        and (output / 'master_plan.json').exists())))
    result['policy'] = arguments.policy
    result['fault_applied'] = fault_applied
    result['scope'] = ('actual_RoboOS_GlobalAgent_ToolCallingAgent_Redis_MCP_ROS_MuJoCo_NOT_PHYSICAL'
                       if arguments.policy == 'model' else 'fixed_dispatch_same_MCP_ROS_MuJoCo_skills_NOT_PHYSICAL')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=['worker', 'master'], default='worker')
    parser.add_argument('--task', default='请把货架上的 thesis_item_01 商品取下，送到 delivery_zone 交付桌面并放稳。')
    parser.add_argument('--task-id')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--execute-isolated-simulation', action='store_true')
    parser.add_argument('--policy', choices=['model', 'fixed'], default='model')
    parser.add_argument('--inject-stale-once', action='store_true')
    arguments = parser.parse_args()
    if arguments.role == 'master':
        if arguments.output is None or arguments.task_id is None:
            raise RuntimeError('Master is launched only with a bounded run directory and task correlation')
        master(arguments)
        return
    if not arguments.execute_isolated_simulation:
        raise RuntimeError('Explicit --execute-isolated-simulation required; this runs real simulation motion')
    if arguments.inject_stale_once and arguments.policy != 'model':
        parser.error('The one-shot observation fault is only for the model recovery trial')
    arguments.task_id = arguments.task_id or datetime.now().strftime('roboos_delivery_%Y%m%d_%H%M%S')
    arguments.output = arguments.output or ROOT / 'local_dependencies/openflex_roboos/runs' / arguments.task_id
    arguments.output.mkdir(parents=True, exist_ok=False)
    save(arguments.output / 'manifest.json', {'task': arguments.task, 'task_id': arguments.task_id,
        'ros_domain': 83, 'model': MODEL if arguments.policy == 'model' else None, 'policy': arguments.policy,
        'inject_stale_once': arguments.inject_stale_once, 'roboos_revision': subprocess.check_output(
            ['git', '-C', str(ROOT / 'local_dependencies/RoboOS'), 'rev-parse', 'HEAD'], text=True).strip(),
        'adapter_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'worker_patch_sha256': hashlib.sha256((ROOT / 'scripts/patches/roboos-verified-worker.patch').read_bytes()).hexdigest(),
        'physical_verification': False})
    report = {'passed': False}
    try:
        report = asyncio.run(worker(arguments))
        if not report['passed']:
            raise RuntimeError('RoboOS delivery not independently verified; inspect recorded feedback')
    finally:
        save(arguments.output / 'result.json', report)
        print(arguments.output / 'result.json')


if __name__ == '__main__':
    main()
