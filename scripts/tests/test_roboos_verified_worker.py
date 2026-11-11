import asyncio
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import httpx
from mcp.types import CallToolResult, ImageContent, TextContent


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'local_dependencies/roboos_client'))
sys.path.insert(0, str(ROOT / 'local_dependencies/RoboOS/slaver'))
from agents.models import ChatMessage, ChatMessageToolCall, ChatMessageToolCallDefinition
from agents.slaver_agent import ToolCallingAgent
from tools.memory import ActionStep


class CollaboratorFixture:
    def __init__(self):
        self.observations = []

    def read_agent_status(self, name):
        return self.observations

    def record_agent_status(self, name, value):
        self.observations.append(value)


def message(name='get_robot_state'):
    return ChatMessage(role='assistant', content=None, tool_calls=[ChatMessageToolCall(
        function=ChatMessageToolCallDefinition(name=name, arguments='{}'), id='actual_call', type='function')])


def agent(model, executor, **kwargs):
    return ToolCallingAgent(tools=[], model=model, model_path='fixture', collaborator=CollaboratorFixture(),
        tool_executor=executor, robot_name='fixture', verbosity_level=0, max_steps=2, **kwargs)


def test_no_tool_never_becomes_mission_accomplished():
    async def unused(*arguments):
        raise AssertionError('No tool should execute')
    instance = agent(lambda **arguments: ChatMessage(role='assistant', content='Mission accomplished'), unused)
    result = asyncio.run(instance.run('fixture'))
    assert result['status'] == 'not_completed' and result['success'] is False


def test_identical_read_tool_calls_are_feedback_not_false_completion():
    calls = []
    async def execute(name, arguments):
        calls.append(name)
        return CallToolResult(content=[TextContent(type='text', text='{"status":"unknown"}')])
    instance = agent(lambda **arguments: message(), execute)
    result = asyncio.run(instance.run('fixture'))
    assert calls == ['get_robot_state', 'get_robot_state']
    assert not result['success']


def test_real_text_image_and_history_reach_the_next_model_call():
    requests = []
    def model(**arguments):
        requests.append(json.loads(json.dumps(arguments['messages'])))
        return message('observe_scene')
    async def execute(name, arguments):
        return CallToolResult(content=[TextContent(type='text', text='{"source":"actual-camera"}'),
            ImageContent(type='image', mimeType='image/jpeg', data='Zml4dHVyZQ==')])
    instance = agent(model, execute)
    asyncio.run(instance.run('fixture'))
    history = requests[1]
    assert history[2]['tool_calls'][0]['id'] == history[3]['tool_call_id']
    assert history[3]['content'] == '{"source":"actual-camera"}'
    assert history[4]['content'][1]['image_url']['url'] == 'data:image/jpeg;base64,Zml4dHVyZQ=='


def test_external_verified_completion_is_the_only_success_path():
    async def unused(*arguments):
        raise AssertionError('Already independently verified')
    instance = agent(lambda **arguments: None, unused, completion_verifier=lambda: True)
    result = asyncio.run(instance.run('fixture'))
    assert result == {'status': 'succeeded', 'success': True, 'completion_verified': True}


def test_tool_error_is_not_observation_or_success():
    async def execute(name, arguments):
        return CallToolResult(isError=True, content=[TextContent(type='text', text='fixture failure')])
    instance = agent(lambda **arguments: message(), execute)
    with pytest.raises(RuntimeError, match='MCP tool error'):
        asyncio.run(instance.run('fixture'))


def test_predicted_scene_mutation_is_disabled():
    instance = agent(lambda **arguments: None, None)
    with pytest.raises(RuntimeError, match='Predicted scene mutation is disabled'):
        asyncio.run(instance.memory_predict({}))


def test_malformed_model_json_never_reaches_tool_executor():
    def model(**arguments):
        answer = message('grasp_object')
        answer.tool_calls[0].function.arguments = '{not valid JSON'
        return answer

    async def execute(name, arguments):
        pytest.fail('Malformed model arguments must not dispatch any tool')

    with pytest.raises(json.JSONDecodeError):
        asyncio.run(agent(model, execute).run('injected malformed arguments'))


def test_disconnected_model_never_dispatches_or_claims_success():
    def model(**arguments):
        raise httpx.ConnectError('injected model service disconnect')

    async def execute(name, arguments):
        pytest.fail('Disconnected model must not dispatch any tool')

    with pytest.raises(httpx.ConnectError):
        asyncio.run(agent(model, execute).run('injected model disconnect'))


def test_disconnected_skill_is_not_retried_under_a_new_request():
    calls = []

    async def execute(name, arguments):
        calls.append(name)
        raise httpx.ConnectError('injected skill service disconnect')

    with pytest.raises(httpx.ConnectError):
        asyncio.run(agent(lambda **arguments: message('grasp_object'), execute).run('injected skill disconnect'))
    assert calls == ['grasp_object']


def test_budget_exhaustion_explicitly_reports_not_completed():
    calls = []

    async def execute(name, arguments):
        calls.append(name)
        return CallToolResult(content=[TextContent(type='text', text='{"status":"unknown"}')])

    result = asyncio.run(agent(lambda **arguments: message(), execute).run('injected bounded budget'))
    assert result['status'] == 'not_completed' and result['success'] is False
    assert result['reason'] == 'bounded model step budget exhausted'
    assert len(calls) == 2


def test_stale_camera_aborts_real_worker_before_any_motion():
    sys.path.insert(0, str(ROOT / 'scripts'))
    from openflex_delivery_contract import DeliveryContract

    contract = DeliveryContract()
    calls = []

    async def execute(name, arguments):
        calls.append(name)
        contract.record(name, arguments, {'sources': {'camera': {'status': 'stale'}}}, image_count=1)
        pytest.fail('Stale image must not enter model history as fresh evidence')

    with pytest.raises(RuntimeError, match='fresh camera image'):
        asyncio.run(agent(lambda **arguments: message('observe_scene'), execute).run('injected stale camera'))
    assert calls == ['observe_scene'] and not contract.observed and not contract.completed


def test_repeated_model_motion_is_rejected_before_second_dispatch():
    sys.path.insert(0, str(ROOT / 'scripts'))
    from openflex_delivery_contract import DeliveryContract

    contract = DeliveryContract()
    contract.checked_state = contract.observed = True
    dispatched = []

    def model(**arguments):
        answer = message('navigate_to_target')
        answer.tool_calls[0].function.arguments = '{"target_id":"pickup_zone"}'
        return answer

    async def execute(name, arguments):
        contract.validate(name, arguments)
        dispatched.append((name, arguments))
        terminal = {'status': 'succeeded', 'success': True, 'stopped_confirmed': True}
        contract.record(name, arguments, terminal)
        return CallToolResult(content=[TextContent(type='text', text=json.dumps(terminal))])

    with pytest.raises(ValueError, match='Repeated motion'):
        asyncio.run(agent(model, execute).run('injected repeated model motion'))
    assert len(dispatched) == 1 and not contract.completed
