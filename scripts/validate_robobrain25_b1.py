#!/usr/bin/env python3
import argparse
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback


ROOT = Path(__file__).resolve().parents[1]
MODEL_ID = "BAAI/RoboBrain2.5-4B"
REVISION = "ec4b3bf5e969b48ad17df432b479c10402a0079b"
WEIGHT_BYTES = 9653631848
WEIGHT_SHA256 = "e2f6e3c372f4ef76afb5e9e28ec56679944fcc3b4f2ce2ebda8b21264e96eb2f"
RUNTIME = ROOT / "local_dependencies" / "robobrain25_b1"


def parse_args():
    parser = argparse.ArgumentParser(description="Isolated, ROS-free RoboBrain2.5 B1 probe")
    parser.add_argument("--download-only", action="store_true")
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--image", type=Path, default=(
        ROOT / "local_dependencies/RoboBrain2.5/assets/demo/navigation.jpg"
    ))
    parser.add_argument("--max-pixels", type=int, default=262144)
    parser.add_argument("--max-new-tokens", type=int, default=160)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if args.max_pixels < 1024 or args.max_new_tokens < 1:
        parser.error("max-pixels must be >= 1024 and max-new-tokens must be positive")
    return args


def system_snapshot():
    result = {"utc": datetime.now(timezone.utc).isoformat()}
    result["meminfo"] = Path("/proc/meminfo").read_text()
    try:
        result["nvidia_smi"] = subprocess.check_output(
            ["nvidia-smi"], text=True, timeout=15
        )
    except (OSError, subprocess.SubprocessError) as error:
        result["nvidia_smi_error"] = str(error)
    return result


def write_report(path, report):
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")


def verify_snapshot(snapshot):
    snapshot = Path(snapshot)
    for name in (
        "config.json", "preprocessor_config.json", "tokenizer.json",
        "tokenizer_config.json", "chat_template.json", "model.safetensors",
    ):
        if not (snapshot / name).is_file():
            raise FileNotFoundError(snapshot / name)
    weight = snapshot / "model.safetensors"
    if weight.stat().st_size != WEIGHT_BYTES:
        raise RuntimeError("Pinned weight size does not match the official metadata")
    digest = hashlib.sha256()
    with weight.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != WEIGHT_SHA256:
        raise RuntimeError("Pinned weight SHA256 does not match the official LFS metadata")
    return {"weight_bytes": WEIGHT_BYTES, "weight_sha256": digest.hexdigest()}


def run_inference(args, snapshot, report, output_path):
    import torch
    import bitsandbytes
    from PIL import Image
    from transformers import AutoModelForImageTextToText, AutoProcessor, BitsAndBytesConfig

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is unavailable; CPU fallback is not a successful B1 GPU run")
    if not args.image.is_file():
        raise FileNotFoundError(args.image)
    report["dependencies"] = {
        name: version(name) for name in (
            "torch", "torchvision", "transformers", "accelerate", "bitsandbytes",
            "huggingface-hub", "numpy", "Pillow", "safetensors",
        )
    }
    report["cuda"] = {
        "runtime": torch.version.cuda,
        "device": torch.cuda.get_device_name(0),
        "capability": torch.cuda.get_device_capability(0),
        "free_and_total_bytes_before_load": torch.cuda.mem_get_info(),
    }
    dtype = torch.bfloat16 if torch.cuda.is_bf16_supported() else torch.float16
    quantization = BitsAndBytesConfig(
        load_in_4bit=True,
        bnb_4bit_quant_type="nf4",
        bnb_4bit_use_double_quant=True,
        bnb_4bit_compute_dtype=dtype,
    )
    report["quantization"] = quantization.to_dict()
    report["attention_implementation"] = "sdpa"
    torch.manual_seed(0)
    torch.cuda.reset_peak_memory_stats()
    started = time.perf_counter()
    print("Loading pinned model with NF4 double quantization on cuda:0", flush=True)
    model = AutoModelForImageTextToText.from_pretrained(
        snapshot,
        local_files_only=True,
        trust_remote_code=False,
        quantization_config=quantization,
        dtype=dtype,
        device_map={"": 0},
        low_cpu_mem_usage=True,
        attn_implementation="sdpa",
    ).eval()
    torch.cuda.synchronize()
    report["load_seconds"] = time.perf_counter() - started
    report["load_peak_allocated_bytes"] = torch.cuda.max_memory_allocated()
    report["load_peak_reserved_bytes"] = torch.cuda.max_memory_reserved()
    report["model_memory_footprint_bytes"] = model.get_memory_footprint()
    report["is_loaded_in_4bit"] = bool(getattr(model, "is_loaded_in_4bit", False))
    report["linear4bit_modules"] = sum(
        isinstance(module, bitsandbytes.nn.Linear4bit) for module in model.modules()
    )
    report["device_map"] = {name: str(device) for name, device in model.hf_device_map.items()}
    if not report["is_loaded_in_4bit"] or not report["linear4bit_modules"]:
        raise RuntimeError("No actual 4-bit modules found")
    processor = AutoProcessor.from_pretrained(
        snapshot, local_files_only=True, trust_remote_code=False,
        min_pixels=4096, max_pixels=args.max_pixels,
    )
    with Image.open(args.image) as source:
        image = source.convert("RGB")
    report["image"] = {
        "path": str(args.image.resolve()),
        "sha256": hashlib.sha256(args.image.read_bytes()).hexdigest(),
        "size": image.size,
        "source": "upstream demo; not a live OpenFlex camera observation",
        "max_pixels": args.max_pixels,
    }
    cases = [
        ("text", "请用一句中文说明机器人为什么应先观察再抓取。不要调用工具，不要声称已执行动作。", None),
        ("image", "请用中文简短描述图像中可见的主要物体和空间关系。只依据图像，不推测机器人已完成任何操作。", image),
    ]
    report["cases"] = []
    write_report(output_path, report)
    for name, prompt, case_image in cases:
        print(f"Running {name} inference", flush=True)
        torch.cuda.reset_peak_memory_stats()
        started = time.perf_counter()
        content = []
        if case_image is not None:
            content.append({"type": "image"})
        content.append({"type": "text", "text": prompt})
        messages = [{"role": "user", "content": content}]
        rendered = processor.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        inputs = processor(
            text=[rendered],
            images=[case_image] if case_image is not None else None,
            return_tensors="pt",
        ).to("cuda:0")
        if case_image is not None and "pixel_values" not in inputs:
            raise RuntimeError("Image case did not produce pixel_values")
        image_tokens = int((inputs.input_ids == model.config.image_token_id).sum().item())
        if case_image is not None and image_tokens == 0:
            raise RuntimeError("Image case contains no image tokens")
        torch.cuda.synchronize()
        prepared = time.perf_counter()
        with torch.inference_mode():
            generated = model.generate(
                **inputs,
                max_new_tokens=args.max_new_tokens,
                do_sample=False,
                temperature=None,
                top_p=None,
                top_k=None,
                use_cache=True,
            )
        torch.cuda.synchronize()
        finished = time.perf_counter()
        output_ids = generated[:, inputs.input_ids.shape[1]:]
        answer = processor.batch_decode(output_ids, skip_special_tokens=True)[0]
        if not answer.strip():
            raise RuntimeError(f"{name} inference returned empty text")
        generated_tokens = output_ids.shape[1]
        case = {
            "name": name,
            "prompt": prompt,
            "response": answer,
            "input_tokens": inputs.input_ids.shape[1],
            "image_tokens": image_tokens,
            "image_grid_thw": inputs.image_grid_thw.tolist() if case_image is not None else None,
            "generated_tokens": generated_tokens,
            "last_token_id": output_ids[0, -1].item(),
            "hit_token_limit": generated_tokens >= args.max_new_tokens,
            "preprocess_seconds": prepared - started,
            "generation_seconds": finished - prepared,
            "total_seconds": finished - started,
            "generation_tokens_per_second": generated_tokens / (finished - prepared),
            "peak_allocated_bytes": torch.cuda.max_memory_allocated(),
            "peak_reserved_bytes": torch.cuda.max_memory_reserved(),
            "system_after": system_snapshot(),
        }
        report["cases"].append(case)
        write_report(output_path, report)
        print(json.dumps(case, ensure_ascii=False, indent=2), flush=True)
        del inputs, generated, output_ids


def main():
    args = parse_args()
    os.environ.setdefault("HF_HOME", str(RUNTIME / "hf_cache"))
    os.environ.setdefault("HF_HUB_DISABLE_XET", "1")
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
    os.environ.setdefault("HF_HUB_DOWNLOAD_TIMEOUT", "120")
    output_dir = args.output_dir or RUNTIME / "runs" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_dir.mkdir(parents=True, exist_ok=False)
    output_path = output_dir / "result.json"
    report = {
        "status": "running",
        "model_id": MODEL_ID,
        "revision": REVISION,
        "command": sys.argv,
        "python": sys.version,
        "executable": sys.executable,
        "platform": platform.platform(),
        "system_before": system_snapshot(),
        "scope": "B1 loading and generation only; no ROS calls or tool execution",
        "max_new_tokens": args.max_new_tokens,
    }
    write_report(output_path, report)
    try:
        from huggingface_hub import snapshot_download

        started = time.perf_counter()
        snapshot = snapshot_download(
            MODEL_ID, revision=REVISION, local_files_only=args.offline,
            allow_patterns=["*.json", "*.safetensors", "*.txt", "*.md"],
            max_workers=1,
        )
        report["snapshot"] = snapshot
        report["download_or_cache_lookup_seconds"] = time.perf_counter() - started
        print("Checking pinned weight size and SHA256", flush=True)
        started = time.perf_counter()
        report["snapshot_integrity"] = verify_snapshot(snapshot)
        report["integrity_check_seconds"] = time.perf_counter() - started
        if args.download_only:
            report["status"] = "downloaded_not_inference_validated"
        else:
            run_inference(args, snapshot, report, output_path)
            report["status"] = "passed_loading_and_generation_only"
    except KeyboardInterrupt:
        report["status"] = "interrupted"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = str(error)
        report["traceback"] = traceback.format_exc()
        traceback.print_exc()
    finally:
        report["system_after"] = system_snapshot()
        write_report(output_path, report)
        print(f"Evidence: {output_path}", flush=True)
    if report["status"] == "interrupted":
        return 130
    return 1 if report["status"] == "failed" else 0


if __name__ == "__main__":
    raise SystemExit(main())
