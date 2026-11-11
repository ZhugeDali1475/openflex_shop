import ast
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import httpx
import pytest
from openai import OpenAI


ROOT = Path(__file__).resolve().parents[2]


def load_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    import sys

    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def upstream():
    return load_file("roboos_models_test", ROOT / "local_dependencies/RoboOS/slaver/agents/models.py")


@pytest.fixture
def probe():
    return load_file("roboos_probe_test", ROOT / "scripts/validate_roboos_gguf_integration.py")


def response_data(finish_reason="tool_calls", arguments="{}", tool_name="get_robot_state"):
    return {
        "id": "contract-unit-test", "object": "chat.completion", "created": 0,
        "model": "robobrain2.5-4b-gguf", "choices": [{
            "index": 0, "finish_reason": finish_reason,
            "message": {"role": "assistant", "content": "", "tool_calls": [{
                "id": "fixture_1", "type": "function",
                "function": {"name": tool_name, "arguments": arguments},
            }]},
        }],
        "usage": {"prompt_tokens": 10, "completion_tokens": 5, "total_tokens": 15},
    }


def make_model(upstream, data, requests):
    def handler(request):
        requests.append(json.loads(request.content))
        return httpx.Response(200, json=data)

    model = upstream.OpenAIServerModel(
        "robobrain2.5-4b-gguf", api_base="http://127.0.0.1:4567/v1/", api_key="EMPTY",
        profiling=False, support_tool_calls=True, max_tokens=512,
        client_kwargs={"http_client": httpx.Client(transport=httpx.MockTransport(handler), trust_env=False)},
    )
    return model


def test_worker_preserves_native_tool_call_and_usage(upstream):
    requests = []
    model = make_model(upstream, response_data(), requests)
    try:
        message = model("查询状态", [], model.model_id)
        assert message.tool_calls[0].function.name == "get_robot_state"
        assert message.tool_calls[0].function.arguments == "{}"
        assert model.get_token_counts() == {"input_token_count": 10, "output_token_count": 5}
        assert requests[0]["max_tokens"] == 512
    finally:
        model.client.close()


def test_worker_forwards_multimodal_and_tool_history(upstream):
    requests = []
    model = make_model(upstream, response_data(), requests)
    messages = [
        {"role": "user", "content": [
            {"type": "text", "text": "查看场景"},
            {"type": "image_url", "image_url": {"url": "data:image/png;base64,fixture"}},
        ]},
        {"role": "assistant", "content": None, "tool_calls": [{
            "id": "fixture_1", "type": "function",
            "function": {"name": "get_robot_state", "arguments": "{}"},
        }]},
        {"role": "tool", "tool_call_id": "fixture_1", "content": '{"success":false}'},
    ]
    try:
        model("检查反馈", [], model.model_id, messages=messages)
        assert requests[0]["messages"] == messages
    finally:
        model.client.close()


def test_worker_sends_serial_tool_call_constraints_to_inference_server(upstream):
    requests = []
    model = make_model(upstream, response_data(), requests)
    model.kwargs.update(parallel_tool_calls=False, tool_choice='required')
    tools = [{'type': 'function', 'function': {'name': 'get_robot_state',
        'parameters': {'type': 'object', 'properties': {}}}}]
    try:
        model('查询状态', [], model.model_id, tools_to_call_from=tools)
        assert requests[0]['parallel_tool_calls'] is False
        assert requests[0]['tool_choice'] == 'required'
        assert requests[0]['tools'] == tools
    finally:
        model.client.close()


@pytest.mark.parametrize("finish_reason", ["length", "content_filter"])
def test_worker_rejects_abnormal_termination(upstream, finish_reason):
    model = make_model(upstream, response_data(finish_reason=finish_reason), [])
    try:
        with pytest.raises(RuntimeError, match="terminate normally"):
            model("查询状态", [], model.model_id)
    finally:
        model.client.close()


def test_actual_mcp_schema_comprehension():
    tree = ast.parse((ROOT / "local_dependencies/RoboOS/slaver/run.py").read_text())
    assignment = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Attribute) and target.attr == "tools" for target in node.targets)
        and isinstance(node.value, ast.ListComp)
    )
    schema = {"type": "object", "properties": {"object_id": {"type": "string"}}}
    response = SimpleNamespace(tools=[SimpleNamespace(name="grasp_object", description="抓取", inputSchema=schema)])
    tools = eval(compile(ast.Expression(assignment.value), "schema_probe", "eval"), {"response": response})
    assert tools == [{"type": "function", "function": {
        "name": "grasp_object", "description": "抓取", "parameters": schema,
    }}]


@pytest.mark.parametrize("tool_name, arguments", [
    ("unregistered_tool", "{}"),
    ("navigate_to_target", '{"target_id":"unknown_station"}'),
    ("navigate_to_target", '{"target_id":"delivery_zone"}'),
    ("navigate_to_target", "not JSON"),
])
def test_probe_rejects_invalid_or_wrong_navigation(probe, upstream, tool_name, arguments):
    model = make_model(upstream, response_data(tool_name=tool_name, arguments=arguments), [])
    args = SimpleNamespace(model=model.model_id)
    try:
        result = probe.check_selection(
            model, args, "navigate", "前往取货位", ["navigate_to_target"],
            expected_arguments={"target_id": "pickup_zone"},
        )
        assert not result["valid"]
    finally:
        model.client.close()
