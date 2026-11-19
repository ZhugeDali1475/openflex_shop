#!/usr/bin/env python3
"""Drive the online ``search_and_pick`` composite over the isolated MCP gateway.

Each round starts from an independently reset scene (a full stop/start of the
isolated stack rebuilds the MJCF, so the red object is back at its shelf
nominal pose and the base is back at the observation pose).  A round then runs
through the real MCP skill gateway:

* round 1 only: a *mismatch* operation - asking for a colour that is not at the
  registered slot while the object is still on the shelf - which must fail
  closed with no motion;
* every round: a *match* operation - ``search_and_pick(query, color=<target>)``
  must observe the target, authorise it from image-derived candidates and grasp
  it with an independently verified attachment.

Failures stay in the denominator and are never rewritten.  Isolated MuJoCo
only; ``physical_verification=false``; no hardware.
"""

import argparse
import asyncio
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import openflex_runtime  # noqa: E402

DEFAULT_SCENE_DIR = (ROOT / "local_dependencies/openflex_thesis/"
                     "retail_fixture_single_red_runtime_20261116_left_v9")


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def probe_navigation(port=18553):
    """A fresh node answers a harmless ``/status`` read with ``{"status": ...}``.

    The HTTP server is created only *after* every skill (including ``SearchSkill``)
    is constructed, so a well-formed reply proves ``/search_and_pick`` exists.
    """
    try:
        reply = openflex_runtime.request_json(f"http://127.0.0.1:{port}/status",
                                              {"request_id": "__readiness_probe__"})
    except Exception:
        return False
    return isinstance(reply, dict) and "status" in reply


def probe_mcp(port=18556):
    """The MCP gateway answers ``/mcp`` (200/406/...) once uvicorn is listening.

    ``urllib`` raises ``HTTPError`` for the expected 4xx probe reply, which still
    proves the socket is serving; a refused connection raises without ``.code``.
    """
    try:
        openflex_runtime.request_json(f"http://127.0.0.1:{port}/mcp")
        return True
    except Exception as error:
        return getattr(error, "code", None) is not None


def wait_stack_ready(wait=240.0):
    """Wait until the bridge, the navigation HTTP API and the MCP gateway all serve.

    The old gate only polled the read-only bridge snapshot on 18552, so a reset
    could "succeed" while the navigation node (18553) or the MCP gateway (18556)
    were still coming up; the first operation then failed closed with a bare
    ``Connection refused``.  All three surfaces must answer before a round starts.
    """
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        snapshot = None
        try:
            snapshot = openflex_runtime.request_json("http://127.0.0.1:18552/state")
        except Exception:
            snapshot = None
        hold = False
        if snapshot is not None:
            try:
                hold = openflex_runtime.measured_hold(snapshot)
            except Exception:
                hold = False
        if hold and probe_navigation() and probe_mcp():
            return snapshot
        time.sleep(1.5)
    raise RuntimeError("scene did not become ready after reset")


def scene_reset(arguments, wait=240.0):
    scene = arguments.scene_dir
    environment = os.environ.copy()
    environment.update({"OPENFLEX_SIM_SCENE_MODEL": str(scene / "scene.xml"),
                        "OPENFLEX_SIM_SCENE_CONFIG": str(scene / "runtime.yaml"),
                        "OPENFLEX_SIM_SHELF_SCENE_CONFIG": str(scene / "shelf_scene.yaml")})
    if any(openflex_runtime.active(unit) for unit in openflex_runtime.UNITS.values()) \
            or openflex_runtime.pending_operations():
        subprocess.run([sys.executable, str(ROOT / "scripts/openflex_runtime.py"), "stop",
                        "--timeout", str(arguments.stop_timeout), "--shutdown-services"],
                       env=environment, check=True, capture_output=True)
    subprocess.run([sys.executable, str(ROOT / "scripts/openflex_runtime.py"), "start",
                    "--enable-simulation-motion"],
                   env=environment, check=True, capture_output=True)
    wait_stack_ready(wait)


async def run_operation(session, events, tool, arguments, timeout):
    response = await session.call_tool(tool, arguments)
    if response.isError:
        raise RuntimeError(f"{tool} returned MCP error: {response.content}")
    started = (response.structuredContent if response.structuredContent is not None
               else json.loads(next(block.text for block in response.content if block.type == 'text')))
    events.append({"time": time.time(), "tool": tool, "arguments": arguments, "result": started})
    request_id = arguments["request_id"]
    deadline = time.monotonic() + timeout
    terminal = started
    while terminal.get("status") == "running":
        if time.monotonic() > deadline:
            raise RuntimeError(f"{request_id} timed out while running")
        await asyncio.sleep(0.5)
        reply = await session.call_tool("get_skill_result", {"request_id": request_id})
        terminal = (reply.structuredContent if reply.structuredContent is not None
                    else json.loads(next(block.text for block in reply.content if block.type == 'text')))
        events.append({"time": time.time(), "tool": "get_skill_result", "request_id": request_id,
                       "result": terminal})
    return terminal


def judge(record, expectation):
    terminal = record.get("terminal")
    if not terminal:
        return False
    search = terminal.get("search") or {}
    if expectation == "authorized":
        return bool(terminal.get("status") == "succeeded" and terminal.get("success")
                    and terminal.get("stopped_confirmed")
                    and search.get("outcome") == "authorized"
                    and search.get("attachment_verified"))
    return bool(terminal.get("status") == "failed" and terminal.get("success") in (False, None)
                and terminal.get("stopped_confirmed")
                and not search.get("attachment_verified") and search.get("outcome") == "failed")


async def main_async(arguments):
    output = arguments.output_root or (ROOT / "local_dependencies/openflex_roboos/runs" /
                                       datetime.now().strftime("search_and_pick_%Y%m%d_%H%M%S_%f"))
    output.mkdir(parents=True, exist_ok=True)
    events = []
    summary = {"scope": "P4.3_search_and_pick_isolated_MuJoCo_NOT_physical",
               "physical_verification": False, "query": arguments.query,
               "object_id": arguments.object_id, "scene_dir": str(arguments.scene_dir),
               "started_at": datetime.now().astimezone().isoformat(), "rounds": []}
    save(output / "summary.json", summary)

    plan = []
    for index in range(1, arguments.rounds + 1):
        if index == 1 and arguments.only != "match":
            plan.append((index, "mismatch", arguments.mismatch_color, "fail_closed"))
        plan.append((index, "match", arguments.color, "authorized"))

    pending_resets = {item[0] for item in plan} if arguments.reset_each else {plan[0][0]}
    if arguments.no_reset_first:
        pending_resets.discard(plan[0][0])

    for index, name, color, expectation in plan:
        record = {"round": index, "name": name, "query": arguments.query, "color": color,
                  "expectation": expectation}
        if index in pending_resets:
            pending_resets.discard(index)
            print(json.dumps({"reset": "scene", "round": index}), flush=True)
            try:
                await asyncio.get_running_loop().run_in_executor(None, scene_reset, arguments)
            except Exception as error:
                record.update(error=f"reset_failed: {error}", passed=False)
                summary["rounds"].append(record)
                save(output / "summary.json", summary)
                continue
        # Every reset stops/starts the MCP unit, so open a fresh session per operation.
        # Retries reuse the *same* request_id: the server store is idempotent per
        # request_id+fingerprint, so a dropped session re-attaches to the in-flight
        # operation instead of double-submitting it.
        request_id = f"search_{name}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
        record["request_id"] = request_id
        payload = {"query": arguments.query, "request_id": request_id,
                   "timeout_sec": arguments.timeout, "color": color}
        for attempt in range(1, 4):
            try:
                async with httpx.AsyncClient(trust_env=False, timeout=15) as client:
                    async with streamable_http_client(f"http://127.0.0.1:{arguments.port}/mcp",
                                                      http_client=client) as (reader, writer, _):
                        async with ClientSession(reader, writer) as session:
                            await session.initialize()
                            record["terminal"] = await run_operation(
                                session, events, "search_and_pick", payload,
                                arguments.timeout + 120)
                record.pop("error", None)
                break
            except Exception as error:
                record["error"] = f"attempt{attempt}: {type(error).__name__}: {error}"
                if attempt < 3:
                    await asyncio.sleep(3.0)
        record["passed"] = judge(record, expectation)
        save(output / "events.json", events)
        summary["rounds"].append(record)
        save(output / "summary.json", summary)
        print(json.dumps({k: v for k, v in record.items() if k != "terminal"},
                         ensure_ascii=False), flush=True)

    passed = sum(1 for item in summary["rounds"] if item.get("passed"))
    summary["passed_rounds"] = passed
    summary["total_rounds"] = len(summary["rounds"])
    summary["finished_at"] = datetime.now().astimezone().isoformat()
    save(output / "summary.json", summary)
    print(json.dumps({"output": str(output), "passed": passed, "total": summary["total_rounds"]},
                     ensure_ascii=False))
    return 0 if passed == summary["total_rounds"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=18556)
    parser.add_argument("--query", default="红色的快递盒")
    parser.add_argument("--object-id", default="thesis_item_01")
    parser.add_argument("--color", default="red")
    parser.add_argument("--mismatch-color", default="blue")
    parser.add_argument("--timeout", type=float, default=300.0)
    parser.add_argument("--rounds", type=int, default=3)
    parser.add_argument("--only", choices=["both", "match", "mismatch"], default="both")
    parser.add_argument("--scene-dir", type=Path, default=DEFAULT_SCENE_DIR)
    parser.add_argument("--stop-timeout", type=float, default=90.0)
    parser.add_argument("--reset-each", action="store_true",
                        help="reset the scene before every round (default: only before the first)")
    parser.add_argument("--no-reset-first", action="store_true",
                        help="reuse the already audited running scene for the first operation")
    parser.add_argument("--output-root", type=Path)
    arguments = parser.parse_args()
    if not 1.0 <= arguments.timeout <= 600.0:
        raise SystemExit("--timeout must be 1-600 seconds (gateway limit)")
    for path in (arguments.scene_dir / "scene.xml", arguments.scene_dir / "runtime.yaml",
                 arguments.scene_dir / "shelf_scene.yaml"):
        if not path.exists():
            raise SystemExit(f"missing scene asset: {path}")
    return asyncio.run(main_async(arguments))


if __name__ == "__main__":
    raise SystemExit(main())
