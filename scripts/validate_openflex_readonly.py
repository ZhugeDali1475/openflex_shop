#!/usr/bin/env python3
import argparse
import asyncio
import base64
from datetime import datetime
import hashlib
import json
from pathlib import Path

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client


ROOT = Path(__file__).resolve().parents[1]


def text_result(result):
    if result.isError:
        raise RuntimeError(f"MCP tool failed: {result.content}")
    return json.loads(next(block.text for block in result.content if block.type == "text"))


def save_image(result, destination):
    images = [block for block in result.content if block.type == "image"]
    if not images:
        return None
    if len(images) != 1 or images[0].mimeType != "image/jpeg":
        raise RuntimeError("Unexpected image content")
    image = images[0]
    payload = base64.b64decode(image.data, validate=True)
    destination.write_bytes(payload)
    return {"path": str(destination.relative_to(ROOT)), "bytes": len(payload),
            "sha256": hashlib.sha256(payload).hexdigest(), "mime_type": image.mimeType}


async def run(args, output):
    async with httpx.AsyncClient(trust_env=False, timeout=60) as transport:
        async with streamable_http_client(args.url, http_client=transport) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                initialized = await session.initialize()
                tools = (await session.list_tools()).tools
                if {tool.name for tool in tools} != {"get_robot_state", "observe_scene"}:
                    raise RuntimeError("Read-only server must expose exactly the two approved tools")
                if not all(tool.annotations and tool.annotations.readOnlyHint for tool in tools):
                    raise RuntimeError("Missing read-only tool annotations")
                state = text_result(await session.call_tool("get_robot_state", {}))
                observation_result = await session.call_tool("observe_scene", {})
                observation = text_result(observation_result)
                image = save_image(observation_result, output / "camera.jpg")
                rejected = await session.call_tool("navigate_to_target", {"target_id": "pickup_zone"})
                if not rejected.isError:
                    raise RuntimeError("Unregistered motion tool was not rejected")
                if args.expect_live:
                    for key in ("odometry", "joint_states", "grasp_status"):
                        if state["sources"].get(key, {}).get("status") != "fresh":
                            raise RuntimeError(f"Real ROS telemetry not fresh: {key}")
                    if observation.get("camera_status") != "fresh" or image is None:
                        raise RuntimeError("No fresh real camera frame")
                    if observation["sources"]["object_observation"]["status"] != "fresh":
                        raise RuntimeError("No fresh real object observation")
                if args.expect_missing:
                    if state["status"] != "unavailable" or image is not None or state.get("holding") is not None:
                        raise RuntimeError("Unavailable ROS must not produce images, holding or available state")
                report = {
                    "created_at": datetime.now().astimezone().isoformat(), "mcp_url": args.url,
                    "server": initialized.serverInfo.model_dump(),
                    "tools": [tool.model_dump(mode="json", exclude_none=True) for tool in tools],
                    "state": state, "observation": observation, "image": image,
                    "motion_tool_rejected": True, "motion_executed": False,
                }
                if args.model:
                    report["model_probe"] = await model_probe(session, tools, output, args.model_url)
                return report


async def model_probe(session, tools, output, model_url):
    contracts = [{"type": "function", "function": {
        "name": tool.name, "description": tool.description, "parameters": tool.inputSchema,
    }} for tool in tools]
    messages = [
        {"role": "system", "content": "你是 OpenFlex 只读观察助手。仅能查询状态、观察图像，不能运动。工具缺失或过期时必须说明未知，不能宣称抓取或交付成功。图像是仿真渲染，不是物理验证。"},
        {"role": "user", "content": "请调用 observe_scene 读取当前头部相机，然后简短说明图像里能看到什么。不要执行动作。"},
    ]
    body = {"model": "robobrain2.5-4b-gguf", "messages": messages, "tools": contracts,
            "tool_choice": "required", "temperature": 0, "max_tokens": 256}
    async with httpx.AsyncClient(trust_env=False, timeout=120) as client:
        response = await client.post(model_url + "/chat/completions", json=body)
        response.raise_for_status()
        first = response.json()
        choice = first["choices"][0]
        calls = choice["message"].get("tool_calls", [])
        if choice["finish_reason"] != "tool_calls" or len(calls) != 1:
            raise RuntimeError("Model did not produce one complete read-only tool call")
        call = calls[0]
        if call["function"]["name"] != "observe_scene" or json.loads(call["function"]["arguments"]) != {}:
            raise RuntimeError("Model did not select expected camera tool")
        result = await session.call_tool("observe_scene", {})
        observation = text_result(result)
        image_evidence = save_image(result, output / "model_camera.jpg")
        if observation.get("camera_status") != "fresh" or image_evidence is None:
            raise RuntimeError("Refusing model image test without a fresh real ROS image")
        image = next(block for block in result.content if block.type == "image")
        metadata = {"camera_status": observation["camera_status"],
                    "camera": observation["sources"]["camera"],
                    "provenance": observation["provenance"]["camera"],
                    "oracle_object_data_sent_to_model": False}
        messages.extend([
            choice["message"],
            {"role": "tool", "tool_call_id": call["id"], "content": json.dumps(metadata)},
            {"role": "user", "content": [
                {"type": "text", "text": "这是刚才工具返回的实时仿真相机图像。请只描述可见内容，不推断物理成功或画面外物体。"},
                {"type": "image_url", "image_url": {"url": f"data:{image.mimeType};base64,{image.data}"}},
            ]},
        ])
        response = await client.post(model_url + "/chat/completions", json={
            "model": body["model"], "messages": messages, "temperature": 0, "max_tokens": 256,
        })
        response.raise_for_status()
        final = response.json()
        if final["choices"][0]["finish_reason"] != "stop":
            raise RuntimeError("Model final description did not finish normally")
        return {"tool_call_response": first, "observation_metadata": metadata,
                "image": image_evidence, "description_response": final,
                "scope": "Direct GGUF API + real MCP/ROS; not the complete RoboOS Worker loop"}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:18551/mcp")
    expectation = parser.add_mutually_exclusive_group()
    expectation.add_argument("--expect-live", action="store_true")
    expectation.add_argument("--expect-missing", action="store_true")
    parser.add_argument("--model", action="store_true")
    parser.add_argument("--model-url", default="http://127.0.0.1:4567/v1")
    args = parser.parse_args()
    output = ROOT / "local_dependencies/openflex_roboos/runs" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    try:
        report = asyncio.run(run(args, output))
        report["passed"] = True
    except Exception as error:
        report = {"passed": False, "error": str(error)}
        raise
    finally:
        (output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(output / "result.json")


if __name__ == "__main__":
    main()
