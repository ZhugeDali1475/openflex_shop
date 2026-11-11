#!/usr/bin/env python3
"""Validate RoboOS's OpenAI-compatible model contract without ROS, MCP, Redis, or motion."""

import argparse
import base64
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
ROBOOS_SLAVER = ROOT / "local_dependencies/RoboOS/slaver"
ROBOOS_MASTER = ROOT / "local_dependencies/RoboOS/master"
DEFAULT_MODEL = "robobrain2.5-4b-gguf"


def available_tools():
    contracts = [
        ("get_robot_state", "查询机器人状态，未知或超时后先查询，不执行运动。", {}),
        ("observe_scene", "重新观察场景和目标；抓取失败后优先重新观察，不执行运动。", {}),
        ("navigate_to_target", "移动到登记站点，不接受自由坐标。", {
            "target_id": {"type": "string", "enum": ["pickup_zone", "delivery_zone"]},
        }),
        ("grasp_object", "在停稳且观察确认可抓取后抓取登记商品。", {
            "object_id": {"type": "string", "enum": ["thesis_item_01"]},
        }),
        ("place_to_affordance", "仅在持物且已到交付站点时放置。", {
            "affordance_id": {"type": "string", "enum": ["delivery_zone"]},
        }),
        ("cancel_current_task", "请求取消当前任务，不等于已停止或已成功。", {}),
    ]
    return [{
        "type": "function", "function": {
            "name": name, "description": description,
            "parameters": {
                "type": "object", "properties": properties,
                "required": list(properties), "additionalProperties": False,
            },
        },
    } for name, description, properties in contracts]


def serialize_message(message):
    return {
        "role": message.role, "content": message.content,
        "tool_calls": [{
            "id": call.id, "type": call.type,
            "function": {"name": call.function.name, "arguments": call.function.arguments},
        } for call in message.tool_calls or []],
    }


def check_selection(model, args, name, task, expected, messages=None, expected_arguments=None):
    from jsonschema import validate

    tools = available_tools()
    started = time.perf_counter()
    message = model(
        task=task, current_status=[], model_path=args.model,
        tools_to_call_from=tools, messages=messages,
    )
    record = {
        "seconds": time.perf_counter() - started,
        "message": serialize_message(message), "expected": expected,
        "expected_arguments": expected_arguments,
        "finish_reason": message.raw.choices[0].finish_reason,
        "usage": message.raw.usage.model_dump(), "executed": False, "valid": False,
    }
    calls = message.tool_calls or []
    if len(calls) != 1:
        return record
    call = calls[0]
    schema = next((tool["function"]["parameters"] for tool in tools if tool["function"]["name"] == call.function.name), None)
    try:
        arguments = json.loads(call.function.arguments)
        if schema is None:
            raise ValueError("Unregistered tool")
        validate(arguments, schema)
        record["arguments"] = arguments
        record["valid"] = call.function.name in expected and (
            expected_arguments is None or arguments == expected_arguments
        )
    except Exception as error:
        record["validation_error"] = str(error)
    print(f"{name}: {'PASS' if record['valid'] else 'FAIL'}", flush=True)
    return record


def parse_args():
    parser = argparse.ArgumentParser(description="RoboOS ↔ local RoboBrain2.5 GGUF contract test")
    parser.add_argument("--base-url", default="http://127.0.0.1:4567/v1/")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--output-dir", type=Path)
    return parser.parse_args()


def load_roboos_model(args):
    sys.path.insert(0, str(ROBOOS_SLAVER))
    from agents.models import OpenAIServerModel
    import httpx
    import yaml

    config = yaml.safe_load((ROBOOS_SLAVER / "config.yaml").read_text())
    candidate = config["model"]["model_dict"]
    if candidate["cloud_server"] != args.base_url or candidate["cloud_model"] != args.model:
        raise ValueError("Worker configuration does not match the requested endpoint/model")

    return OpenAIServerModel(
        model_id=args.model,
        api_base=args.base_url,
        api_key="EMPTY",
        support_tool_calls=config["tool"]["support_tool_calls"],
        profiling=False,
        max_tokens=512,
        client_kwargs={"http_client": httpx.Client(trust_env=False, timeout=120), "max_retries": 0},
    )


def load_roboos_planner(args):
    class PlannerRegistryFixture:
        def read_all_agents_name(self):
            return ["openflex"]

        def read_all_agents_info(self):
            return {
                "openflex": (
                    "get_robot_state (read-only), observe_scene (read-only), "
                    "navigate_to_target, grasp_object, place_to_affordance"
                )
            }

        def read_environment(self, name=None):
            return "Contract-test scene fixture only; robot state unknown. Item: thesis_item_01."

    sys.path.insert(0, str(ROOT / "local_dependencies/roboos_client"))
    sys.path.insert(0, str(ROBOOS_MASTER))
    from agents.planner import GlobalTaskPlanner
    import yaml

    planner = GlobalTaskPlanner.__new__(GlobalTaskPlanner)
    config = yaml.safe_load((ROBOOS_MASTER / "config.yaml").read_text())
    candidate = config["model"]["model_dict"]
    if candidate["cloud_server"] != args.base_url or candidate["cloud_model"] != args.model:
        raise ValueError("Master configuration does not match the requested endpoint/model")
    planner.global_model, planner.model_name = planner._get_model_info_from_config(
        config["model"]
    )
    planner.global_model = planner.global_model.with_options(timeout=120, max_retries=0)
    planner.collaborator = PlannerRegistryFixture()
    planner.profiling = False
    return planner


def main():
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_dir = args.output_dir or ROOT / "local_dependencies/robobrain25_gguf/runs" / f"roboos_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "running",
        "utc": datetime.now(timezone.utc).isoformat(),
        "base_url": args.base_url,
        "model": args.model,
        "scope": "RoboOS model-client and prompt contract only; no Redis, MCP, ROS, MuJoCo, or motion",
        "metadata_source": "Explicit planner registry / scene test fixture, not real robot state",
        "station_ids": "pickup_zone / delivery_zone are contract-test fixtures; ROS mapping is not implemented",
        "master_execution": "Real GlobalTaskPlanner.forward with injected read-only registry fixture; no real Redis registry",
        "roboos_revision": "1c39b85ef5c9dcb9d5da235665c19af21824e545 plus local GGUF contract patch",
        "cases": {},
    }
    try:
        model = load_roboos_model(args)
        tool = {
            "type": "function",
            "function": {
                "name": "get_robot_state",
                "description": "查询 OpenFlex 当前位姿、导航状态、持物状态和安全状态。只读，不执行运动。",
                "parameters": {"type": "object", "properties": {}, "required": []},
            },
        }

        started = time.perf_counter()
        worker_message = model(
            task="请先查询 OpenFlex 当前状态，暂时不要执行导航、抓取或放置。",
            current_status="",
            model_path=args.model,
            tools_to_call_from=[tool],
        )
        elapsed = time.perf_counter() - started
        worker_calls = worker_message.tool_calls or []
        worker_case = {
            "seconds": elapsed,
            "message": {
                "role": worker_message.role,
                "content": worker_message.content,
                "tool_calls": [
                    {
                        "id": call.id,
                        "type": call.type,
                        "function": {
                            "name": call.function.name,
                            "arguments": call.function.arguments,
                        },
                    }
                    for call in worker_calls
                ],
            },
            "expected_tool": "get_robot_state",
            "valid": len(worker_calls) == 1 and worker_calls[0].function.name == "get_robot_state",
        }
        worker_case["parsed_arguments"] = None
        try:
            worker_case["parsed_arguments"] = json.loads(worker_calls[0].function.arguments) if worker_calls else None
        except json.JSONDecodeError as error:
            worker_case["valid"] = False
            worker_case["argument_error"] = str(error)
        worker_case["valid"] = worker_case["valid"] and worker_case["parsed_arguments"] == {}
        report["cases"]["worker_tool_call"] = worker_case
        if not worker_case["valid"]:
            raise RuntimeError("RoboOS Worker model contract did not return the expected tool call")

        feedback_messages = [
            {
                "role": "system",
                "content": "你是 OpenFlex 机器人任务执行器。没有真实验证就不能宣称成功。freshness_ms 表示状态样本距现在的毫秒数，不是机器人空闲时长。",
            },
            {
                "role": "user",
                "content": "查询机器人状态。",
            },
            {
                "role": "assistant",
                "content": None,
                "tool_calls": [{
                    "id": "call_state_1",
                    "type": "function",
                    "function": {"name": "get_robot_state", "arguments": "{}"},
                }],
            },
            {
                "role": "tool",
                "tool_call_id": "call_state_1",
                "content": json.dumps({
                    "navigation": "idle",
                    "holding": None,
                    "safety": "ok",
                    "freshness_ms": 120,
                }, ensure_ascii=False),
            },
            {
                "role": "user",
                "content": "状态已返回。现在只报告下一步建议，不要调用运动工具。",
            },
        ]
        started = time.perf_counter()
        feedback_message = model(
            task="查询机器人状态后只报告下一步建议，不执行运动。",
            current_status=[],
            model_path=args.model,
            messages=feedback_messages,
        )
        feedback_response = feedback_message.raw
        report["cases"]["tool_feedback"] = {
            "seconds": time.perf_counter() - started,
            "content": feedback_message.content,
            "tool_calls": feedback_message.tool_calls,
            "finish_reason": feedback_response.choices[0].finish_reason,
            "usage": feedback_response.usage.model_dump() if feedback_response.usage else None,
            "valid": bool(feedback_message.content) and not feedback_message.tool_calls,
        }
        if not report["cases"]["tool_feedback"]["valid"]:
            raise RuntimeError("Tool feedback did not produce a final non-tool response")

        image_path = ROOT / "supermarket_scene/mujoco/screenshots/v5_overview.png"
        image_url = "data:image/png;base64," + base64.b64encode(image_path.read_bytes()).decode("ascii")
        started = time.perf_counter()
        vision_message = model(
            task="描述图片", current_status=[], model_path=args.model,
            messages=[{"role": "user", "content": [
                {"type": "text", "text": "请用中文50字以内描述图中的主要场景，不调用任何工具。"},
                {"type": "image_url", "image_url": {"url": image_url}},
            ]}],
        )
        report["cases"]["worker_vision"] = {
            "seconds": time.perf_counter() - started, "message": serialize_message(vision_message),
            "valid": bool(vision_message.content) and any(word in vision_message.content for word in ["货架", "超市"]),
            "image": str(image_path), "image_source": "saved screenshot, not a live camera",
        }

        scenarios = [
            ("navigate", "登记站点为 pickup_zone（取货）与 delivery_zone（交付）。当前在起点、空手，目标 thesis_item_01 位于 pickup_zone。请只选择下一步导航技能，不输出完整计划。", ["navigate_to_target"], {"target_id": "pickup_zone"}),
            ("grasp", "当前已在取货位停稳，状态安全，空手，最新观察已确认 thesis_item_01 可抓取。请只选择下一步抓取技能。", ["grasp_object"], {"object_id": "thesis_item_01"}),
            ("place", "当前已在 delivery_zone 停稳，状态安全，已验证持有 thesis_item_01，交付位空闲。请只选择下一步放置技能。", ["place_to_affordance"], {"affordance_id": "delivery_zone"}),
        ]
        for name, task, expected, arguments in scenarios:
            report["cases"][name] = check_selection(
                model, args, name, task, expected, expected_arguments=arguments,
            )
        failed_history = [
            {"role": "system", "content": "你是 OpenFlex 技能调度器。工具结果是唯一事实依据。抓取失败后必须先重新观察，不得直接重试、交付或宣布成功。"},
            {"role": "user", "content": "抓取 thesis_item_01 并交付。"},
            {"role": "assistant", "content": None, "tool_calls": [{
                "id": "fixture_grasp_1", "type": "function",
                "function": {"name": "grasp_object", "arguments": '{"object_id":"thesis_item_01"}'},
            }]},
            {"role": "tool", "tool_call_id": "fixture_grasp_1", "content": json.dumps({
                "source": "contract_test_fixture", "success": False, "error_code": "GRASP_FAILED",
                "holding": None, "grasp_verified": False, "retry_requires_observation": True,
            })},
            {"role": "user", "content": "根据上述测试失败反馈，仅选择下一步安全技能。"},
        ]
        report["cases"]["grasp_failure_feedback"] = check_selection(
            model, args, "grasp_failure_feedback", "抓取失败后重新观察", ["observe_scene"], failed_history,
        )
        unknown_history = [
            {"role": "system", "content": "工具结果未知时，必须先查询新鲜机器人状态，不得假定运动已停止、已抓取或已交付。"},
            {"role": "user", "content": "导航请求超时，执行结果未知。请仅选择下一步安全查询技能，不能宣称成功。"},
        ]
        report["cases"]["unknown_result_feedback"] = check_selection(
            model, args, "unknown_result_feedback", "导航超时后查询状态", ["get_robot_state"], unknown_history,
        )

        planner = load_roboos_planner(args)
        started = time.perf_counter()
        master_content = planner.forward(
            "把货架上的 thesis_item_01 取下并送到交付台；先观察并查询状态，失败时不得宣称成功。"
        )
        master_case = {
            "seconds": time.perf_counter() - started,
            "raw_content": master_content,
            "valid": False,
        }
        try:
            normalized_content = master_content.strip()
            if normalized_content.startswith("```json") and normalized_content.endswith("```"):
                normalized_content = normalized_content[len("```json"): -len("```")].strip()
            plan = json.loads(normalized_content)
            subtasks = plan.get("subtask_list")
            master_case["parsed"] = plan
            master_case["valid"] = (
                isinstance(plan.get("reasoning_explanation"), str)
                and len(plan["reasoning_explanation"]) > 0
                and isinstance(subtasks, list)
                and len(subtasks) > 0
                and all(
                    isinstance(item, dict)
                    and item.get("robot_name") == "openflex"
                    and isinstance(item.get("subtask"), str) and bool(item["subtask"].strip())
                    and type(item.get("subtask_order")) is int and item["subtask_order"] >= 1
                    for item in subtasks
                )
            )
        except json.JSONDecodeError as error:
            master_case["parse_error"] = str(error)
        report["cases"]["master_plan"] = master_case
        if not master_case["valid"]:
            raise RuntimeError("Master planner did not return the expected JSON plan")
        if not all(case.get("valid") for case in report["cases"].values()):
            raise RuntimeError("One or more image / skill selection / feedback checks failed")
        report["status"] = "passed"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = str(error)
        print(f"ERROR: {error}", file=sys.stderr)
    report_path = output_dir / "result.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str) + "\n")
    print(f"Report: {report_path}")
    for name, case in report["cases"].items():
        print(f"{name}: {'PASS' if case.get('valid') else 'FAIL'} ({case.get('seconds', 0):.2f}s)")
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
