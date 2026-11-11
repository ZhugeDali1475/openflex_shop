#!/usr/bin/env python3
import argparse
import base64
from datetime import datetime, timezone
import json
import mimetypes
from pathlib import Path
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "local_dependencies/robobrain25_gguf"
MODEL = "robobrain2.5-4b-gguf"


def parse_args():
    parser = argparse.ArgumentParser(description="ROS-free local GGUF text/vision smoke test")
    parser.add_argument("--base-url", default="http://127.0.0.1:4567")
    parser.add_argument("--image", type=Path, default=(
        ROOT / "local_dependencies/RoboBrain2.5/assets/demo/navigation.jpg"
    ))
    parser.add_argument("--tool-probe", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not args.image.is_file():
        parser.error(f"Image does not exist: {args.image}")
    return args


def request_json(opener, url, payload=None):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url, data=data, headers={"Content-Type": "application/json"}
    )
    try:
        with opener.open(request, timeout=180) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        raise RuntimeError(f"HTTP {error.code}: {error.read().decode('utf-8')}") from error


def sample_gpu(stop_event, samples, errors):
    while not stop_event.is_set():
        try:
            output = subprocess.check_output([
                "nvidia-smi", "--query-gpu=index,memory.used,memory.total",
                "--format=csv,noheader,nounits",
            ], text=True, timeout=10)
            for line in output.strip().splitlines():
                device, used, total = [int(value.strip()) for value in line.split(",")]
                samples.append({
                    "utc": datetime.now(timezone.utc).isoformat(),
                    "device": device, "used_mib": used, "total_mib": total,
                })
        except (OSError, ValueError, subprocess.SubprocessError) as error:
            errors.append(str(error))
            return
        stop_event.wait(0.5)


def run_case(opener, args, output_dir, name, messages, **extra):
    payload = {
        "model": MODEL, "messages": messages, "max_tokens": 512,
        "temperature": 0, "seed": 0, "stream": False,
        "reasoning_effort": "none", **extra,
    }
    started = time.perf_counter()
    response = request_json(opener, args.base_url.rstrip("/") + "/v1/chat/completions", payload)
    elapsed = time.perf_counter() - started
    response_path = output_dir / f"{name}.json"
    response_path.write_text(json.dumps(response, ensure_ascii=False, indent=2) + "\n")
    choice = response["choices"][0]
    message = choice["message"]
    if choice.get("finish_reason") == "length":
        raise RuntimeError(f"{name}: output truncated; inspect {response_path}")
    if not message.get("content") and not message.get("tool_calls"):
        raise RuntimeError(f"{name}: no final content or tool_calls; inspect {response_path}")
    print(f"{name} ({elapsed:.2f}s): {json.dumps(message, ensure_ascii=False)}", flush=True)
    return {
        "seconds": elapsed, "message": message, "usage": response.get("usage"),
        "timings": response.get("timings"), "finish_reason": choice.get("finish_reason"),
        "response_file": str(response_path),
    }


def main():
    args = parse_args()
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_dir = args.output_dir or RUNTIME / "runs" / stamp
    output_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "status": "running", "utc": datetime.now(timezone.utc).isoformat(),
        "base_url": args.base_url, "model": MODEL, "image": str(args.image.resolve()),
        "scope": "Text/vision API smoke test only; no ROS, MCP, robot actions or MuJoCo concurrency",
        "semantic_quality": "Requires human review of the saved image and responses",
        "cases": {},
    }
    samples = []
    sampling_errors = []
    stop_event = threading.Event()
    sampler = threading.Thread(target=sample_gpu, args=(stop_event, samples, sampling_errors))
    sampler.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        report["health"] = request_json(opener, args.base_url.rstrip("/") + "/health")
        report["models"] = request_json(opener, args.base_url.rstrip("/") + "/v1/models")
        report["cases"]["text"] = run_case(opener, args, output_dir, "text", [{
            "role": "user", "content": "请用中文简短回答：机器人执行抓取失败后，为什么不能直接宣称任务成功？",
        }])
        media_type = mimetypes.guess_type(args.image.name)[0] or "image/jpeg"
        encoded = base64.b64encode(args.image.read_bytes()).decode("ascii")
        report["cases"]["vision"] = run_case(opener, args, output_dir, "vision", [{
            "role": "user", "content": [
                {"type": "text", "text": "请用中文、不超过100字、一个自然段描述这张图中实际可见的场景和主要物体，不要编造看不见的内容。"},
                {"type": "image_url", "image_url": {"url": f"data:{media_type};base64,{encoded}"}},
            ],
        }])
        if args.tool_probe:
            tools = [{
                "type": "function", "function": {
                    "name": "get_robot_state",
                    "description": "查询机器人当前状态。仅返回状态，不执行运动。",
                    "parameters": {"type": "object", "properties": {}, "required": []},
                },
            }]
            result = run_case(opener, args, output_dir, "tool_probe", [
                {"role": "system", "content": "你是机器人任务规划助手。未知的状态必须通过工具查询，不得编造。"},
                {"role": "user", "content": "请先查询机器人当前状态，暂时不要执行任何运动。"},
            ], tools=tools, tool_choice="auto")
            calls = result["message"].get("tool_calls", [])
            valid = len(calls) == 1 and calls[0].get("function", {}).get("name") == "get_robot_state"
            if valid:
                valid = json.loads(calls[0]["function"]["arguments"]) == {}
            result["valid_expected_tool_call"] = valid
            result["executed"] = False
            report["cases"]["tool_probe"] = result
            if not valid:
                raise RuntimeError("Tool probe did not return the expected native tool_calls")
        report["status"] = "smoke_passed"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = str(error)
        print(str(error), file=sys.stderr)
    finally:
        stop_event.set()
        sampler.join(timeout=15)
        report["gpu_samples"] = samples
        report["gpu_sampling_errors"] = sampling_errors
        report["sampled_peak_total_gpu_used_mib"] = max(
            (sample["used_mib"] for sample in samples), default=None
        )
        report["gpu_measurement_scope"] = "Total GPU usage, includes desktop and other processes; sampled, not allocator peak"
        report_path = output_dir / "result.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(f"Report: {report_path}")
    return 0 if report["status"] == "smoke_passed" else 1


if __name__ == "__main__":
    sys.exit(main())
