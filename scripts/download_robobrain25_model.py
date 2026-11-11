#!/usr/bin/env python3
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import threading
import time

import requests


ROOT = Path(__file__).resolve().parents[1]
REVISION = "ec4b3bf5e969b48ad17df432b479c10402a0079b"
WEIGHT_SHA256 = "e2f6e3c372f4ef76afb5e9e28ec56679944fcc3b4f2ce2ebda8b21264e96eb2f"
RUNTIME = ROOT / "local_dependencies/robobrain25_b1"
CHUNK_BYTES = 16000000
REQUIRED = (
    "chat_template.json", "config.json", "generation_config.json", "merges.txt",
    "preprocessor_config.json", "tokenizer.json", "tokenizer_config.json",
    "video_preprocessor_config.json", "vocab.json", "model.safetensors",
)


def digest_file(path, algorithm="sha256"):
    digest = hashlib.new(algorithm)
    if algorithm == "sha1":
        digest.update(f"blob {path.stat().st_size}\0".encode())
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_file(path, metadata):
    if not path.is_file() or path.stat().st_size != metadata["size"]:
        return False
    lfs = metadata.get("lfs")
    expected = lfs["sha256"] if lfs else metadata["blobId"]
    return digest_file(path, "sha256" if lfs else "sha1") == expected


def main():
    parser = argparse.ArgumentParser(description="Resumable, hash-verified RoboBrain2.5 mirror download")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--proxy", action="store_true")
    parser.add_argument("--local-dir", type=Path, default=ROOT / "local_dependencies/models/RoboBrain2.5-4B")
    args = parser.parse_args()
    if not 1 <= args.workers <= 16:
        parser.error("workers must be between 1 and 16")
    official = json.loads((ROOT / "local_dependencies/review_metadata/robobrain25_4b_metadata.json").read_text())
    if official["sha"] != REVISION or official["id"] != "BAAI/RoboBrain2.5-4B":
        raise RuntimeError("Official model metadata does not match the pinned model")
    expected = {item["rfilename"]: item for item in official["siblings"]}
    if expected["model.safetensors"]["lfs"]["sha256"] != WEIGHT_SHA256:
        raise RuntimeError("Official weight hash changed")
    client = requests.Session()
    client.trust_env = args.proxy
    api = "https://modelscope.cn/api/v1/models/BAAI/RoboBrain2.5-4B/repo"
    response = client.get(api + "/files", params={"Revision": "master", "Recursive": "true"}, timeout=(10, 40))
    response.raise_for_status()
    listing = response.json()
    if listing.get("Code") != 200:
        raise RuntimeError(f"ModelScope listing failed: {listing.get('Message')}")
    mirror = {item["Path"]: item for item in listing["Data"]["Files"] if item["Type"] == "blob"}
    for name in REQUIRED:
        if mirror[name]["Size"] != expected[name]["size"]:
            raise RuntimeError(f"Mirror size differs from pinned HF metadata: {name}")
        if "lfs" in expected[name] and mirror[name]["Sha256"] != expected[name]["lfs"]["sha256"]:
            raise RuntimeError(f"Mirror weight hash differs from pinned HF metadata: {name}")
    args.local_dir.mkdir(parents=True, exist_ok=True)
    parts = args.local_dir / (".download_parts_" + REVISION[:12])
    parts.mkdir(exist_ok=True)
    (parts / "modelscope_files.json").write_text(json.dumps(listing, ensure_ascii=False, indent=2))
    started = time.monotonic()
    manifest = {
        "status": "downloading", "hf_model": official["id"], "hf_revision": REVISION,
        "source": "BAAI/RoboBrain2.5-4B on ModelScope; each file pinned to its listed commit",
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "workers": args.workers, "proxy": args.proxy, "chunk_bytes": CHUNK_BYTES,
        "files": {},
    }
    manifest_path = args.local_dir / "download_manifest.json"

    def save_manifest():
        manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")

    def get_file(name, headers=None, session=None):
        return (session or client).get(
            api, params={"Revision": mirror[name]["Revision"], "FilePath": name},
            headers=headers, stream=True, timeout=(15, 60),
        )

    save_manifest()
    for name in REQUIRED:
        if name == "model.safetensors":
            continue
        target = args.local_dir / name
        if not validate_file(target, expected[name]):
            cache = RUNTIME / "hf_cache/hub/models--BAAI--RoboBrain2.5-4B/snapshots" / REVISION / name
            temporary = target.with_name(name + ".downloading")
            if validate_file(cache, expected[name]):
                shutil.copyfile(cache, temporary)
            else:
                with get_file(name) as response:
                    response.raise_for_status()
                    with temporary.open("wb") as stream:
                        for block in response.iter_content(1024 * 1024):
                            stream.write(block)
            if not validate_file(temporary, expected[name]):
                raise RuntimeError(f"File differs from pinned Hugging Face content: {name}")
            temporary.replace(target)
        manifest["files"][name] = {"verified": True, "size": target.stat().st_size, "sha256": digest_file(target)}
        save_manifest()
        print(f"Verified {name}", flush=True)

    weight = args.local_dir / "model.safetensors"
    total = expected["model.safetensors"]["size"]
    count = (total + CHUNK_BYTES - 1) // CHUNK_BYTES
    seed = RUNTIME / "hf_cache/hub/models--BAAI--RoboBrain2.5-4B/blobs" / (WEIGHT_SHA256 + ".incomplete")
    if not weight.exists() and seed.is_file():
        with seed.open("rb") as stream:
            for index in range(min(seed.stat().st_size // CHUNK_BYTES, count)):
                part = parts / f"{index:05d}.part"
                block = stream.read(CHUNK_BYTES)
                if not part.exists():
                    part.write_bytes(block)
        print(f"Seeded complete chunks from existing {seed.stat().st_size} byte HF partial; final hash remains mandatory", flush=True)
    lock = threading.Lock()
    completed = sum(part.stat().st_size for part in parts.glob("*.part"))
    initially_complete = completed
    thread_state = threading.local()

    def fetch(index):
        nonlocal completed
        start = index * CHUNK_BYTES
        end = min(total, start + CHUNK_BYTES) - 1
        part = parts / f"{index:05d}.part"
        needed = end - start + 1
        if part.is_file() and part.stat().st_size == needed:
            return
        if not hasattr(thread_state, "session"):
            thread_state.session = requests.Session()
            thread_state.session.trust_env = args.proxy
        partial = parts / f"{index:05d}.partial"
        for attempt in range(8):
            offset = partial.stat().st_size if partial.exists() else 0
            if offset == needed:
                partial.replace(part)
                with lock:
                    completed += needed
                return
            if offset > needed:
                raise RuntimeError(f"Oversized partial: {partial}")
            try:
                expected_range = f"bytes {start + offset}-{end}/{total}"
                with get_file("model.safetensors", {"Range": f"bytes={start + offset}-{end}"}, thread_state.session) as response:
                    response.raise_for_status()
                    if response.status_code != 206 or response.headers.get("Content-Range") != expected_range:
                        raise RuntimeError(f"Server returned wrong range: {response.status_code} {response.headers.get('Content-Range')}")
                    with partial.open("ab") as stream:
                        for block in response.iter_content(256 * 1024):
                            stream.write(block)
                if partial.stat().st_size != needed:
                    raise RuntimeError(f"Incomplete range {index}")
                partial.replace(part)
                with lock:
                    completed += needed
                    elapsed = time.monotonic() - started
                    speed = (completed - initially_complete) / elapsed
                    eta = (total - completed) / speed if speed > 0 else None
                    print(json.dumps({"completed_bytes": completed, "total_bytes": total, "percent": round(completed / total * 100, 2), "new_MiB_per_second": round(speed / 1024**2, 2), "eta_minutes": round(eta / 60, 1) if eta else None}), flush=True)
                return
            except (requests.RequestException, RuntimeError) as error:
                print(f"Retry range {index}, attempt {attempt + 1}: {error}", flush=True)
                if attempt == 7:
                    raise
                time.sleep(min(2**attempt, 30))

    try:
        if not weight.exists():
            with ThreadPoolExecutor(max_workers=args.workers) as pool:
                for future in as_completed([pool.submit(fetch, index) for index in range(count)]):
                    future.result()
            print("Assembling and hashing all ranges", flush=True)
            assembled = weight.with_name("model.safetensors.assembling")
            digest = hashlib.sha256()
            with assembled.open("wb") as output:
                for index in range(count):
                    with (parts / f"{index:05d}.part").open("rb") as stream:
                        for block in iter(lambda: stream.read(1024 * 1024), b""):
                            digest.update(block)
                            output.write(block)
            if assembled.stat().st_size != total or digest.hexdigest() != WEIGHT_SHA256:
                raise RuntimeError("Final weight SHA256 mismatch; assembled file preserved, NOT accepted")
            assembled.replace(weight)
        elif not validate_file(weight, expected["model.safetensors"]):
            raise RuntimeError("Existing model.safetensors fails validation; not overwriting it")
        manifest["files"]["model.safetensors"] = {"verified": True, "size": total, "sha256": WEIGHT_SHA256}
        manifest["status"] = "downloaded_and_hash_verified_not_inference_validated"
        manifest["elapsed_seconds"] = time.monotonic() - started
        print(f"SUCCESS: {weight}, SHA256={WEIGHT_SHA256}", flush=True)
    except BaseException as error:
        manifest["status"] = "interrupted" if isinstance(error, KeyboardInterrupt) else "failed"
        manifest["error"] = str(error)
        raise
    finally:
        save_manifest()


if __name__ == "__main__":
    main()
