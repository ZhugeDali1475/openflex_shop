#!/usr/bin/env python3
import asyncio
from datetime import datetime
import json
from pathlib import Path
import time

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client


ROOT = Path(__file__).resolve().parents[1]


async def main_async(output):
    prefix = datetime.now().strftime("nav_%Y%m%d_%H%M%S")
    events = []
    async with httpx.AsyncClient(trust_env=False, timeout=10) as transport:
        async with streamable_http_client("http://127.0.0.1:18554/mcp", http_client=transport) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                await session.initialize()

                async def call(tool, arguments):
                    result = await session.call_tool(tool, arguments)
                    if result.isError:
                        raise RuntimeError(str(result.content))
                    data = result.structuredContent or json.loads(next(block.text for block in result.content if block.type == "text"))
                    events.append({"time": datetime.now().astimezone().isoformat(), "tool": tool,
                                   "arguments": arguments, "result": data})
                    (output / "events.json").write_text(json.dumps(events, ensure_ascii=False, indent=2))
                    return data

                async def wait(request_id, timeout=90):
                    started = time.monotonic()
                    while time.monotonic() - started < timeout:
                        result = await call("get_navigation_result", {"request_id": request_id})
                        if result["status"] != "running":
                            return result
                        await asyncio.sleep(0.5)
                    raise RuntimeError("Validator timed out; do not infer operation stopped")

                async def navigate(name, station, timeout=60):
                    request_id = prefix + "_" + name
                    arguments = {"request_id": request_id, "target_id": station, "timeout_sec": timeout}
                    await call("navigate_to_target", arguments)
                    return request_id, arguments

                request_id, arguments = await navigate("pickup", "pickup_zone")
                result = await wait(request_id)
                if not result["success"] or not result["stopped_confirmed"]:
                    raise RuntimeError(f"Pickup verification failed: {result}")
                repeated = await call("navigate_to_target", arguments)
                if repeated != result:
                    raise RuntimeError("Same request_id did not return the existing terminal result")
                request_id, _ = await navigate("cancel", "delivery_zone")
                busy = await call("navigate_to_target", {"request_id": prefix + "_busy", "target_id": "pickup_zone", "timeout_sec": 30})
                if busy["status"] != "rejected_busy":
                    raise RuntimeError("Concurrent navigation was not rejected")
                await asyncio.sleep(1.5)
                await call("cancel_current_task", {"request_id": request_id})
                canceled = await wait(request_id)
                if canceled["status"] != "canceled" or canceled["success"] or not canceled["stopped_confirmed"]:
                    raise RuntimeError(f"Cancel not confirmed: {canceled}")
                request_id, _ = await navigate("timeout", "delivery_zone", 1.2)
                expired = await wait(request_id)
                if expired["status"] != "timed_out" or expired["success"] or not expired["stopped_confirmed"]:
                    raise RuntimeError(f"Timeout not confirmed: {expired}")
                request_id, _ = await navigate("delivery", "delivery_zone")
                delivered = await wait(request_id)
                if not delivered["success"] or not delivered["stopped_confirmed"]:
                    raise RuntimeError(f"Delivery approach navigation failed: {delivered}")
                request_id, _ = await navigate("return", "pickup_zone")
                returned = await wait(request_id)
                if not returned["success"] or not returned["stopped_confirmed"]:
                    raise RuntimeError(f"Return to pickup failed: {returned}")
                return {"passed": True, "scope": "empty_robot_sim_navigation_NOT_Nav2_not_transport_or_delivery",
                        "checks": ["measured_pose_and_stop", "cancel", "timeout", "mutex", "idempotency", "delivery_approach", "return_pickup"],
                        "events": events}


def main():
    output = ROOT / "local_dependencies/openflex_roboos/runs" / datetime.now().strftime("navigation_%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True)
    try:
        report = asyncio.run(main_async(output))
    except Exception as error:
        report = {"passed": False, "error": str(error)}
        raise
    finally:
        (output / "result.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(output / "result.json")


if __name__ == "__main__":
    main()
