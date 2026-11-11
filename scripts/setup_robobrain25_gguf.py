#!/usr/bin/env python3
import argparse
import fcntl
import hashlib
from pathlib import Path
import platform
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "local_dependencies/robobrain25_gguf"
RELEASE_URL = "https://github.com/ggml-org/llama.cpp/releases/download/b12110/"
ARTIFACTS = (
    (
        RELEASE_URL + "llama-b12110-bin-ubuntu-cuda-12.8-x64.tar.gz",
        "4af1e1da23661db4a5b84b29aae16ebc053df689c5b8b1427c4670f2c6406d8c",
        "runtime",
    ),
    (
        RELEASE_URL + "cudart-llama-b12110-bin-ubuntu-cuda-12.8-x64.tar.gz",
        "6f1d562086d84ae85df880d3711079ec225602b51d530608e5001600625067dd",
        "runtime",
    ),
    (
        "https://archive.ubuntu.com/ubuntu/pool/main/g/glibc/libc6_2.39-0ubuntu8.9_amd64.deb",
        "ff5557d99b51f761c4b7c92368b9cc45565eda17df9bf9eb4b134d09825008be",
        "compat",
    ),
    (
        "https://archive.ubuntu.com/ubuntu/pool/main/g/gcc-14/libstdc++6_14.2.0-4ubuntu2~24.04.1_amd64.deb",
        "a51f8de7829211db961a31f02158058ad1a95f92ac6d0a5dff6350e2821c54c0",
        "compat",
    ),
)
MODELS = {
    "RoboBrain2.5-4B.Q4_K_M.gguf": "9d83cfe5f74148ec334caa5a0f02ebce9d18fab54f0f5c28436096e15d046108",
    "RoboBrain2.5-4B.mmproj-f16.gguf": "7ef45bb8e9db278c6a0b2a1b6131e8fcc967e734ea424d2f617ee92b47218bdf",
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description="Project-local pinned GGUF runtime; no sudo or system installation")
    parser.add_argument("--models-from", type=Path, default=Path.home() / "下载")
    args = parser.parse_args()
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        parser.error("This pinned deployment targets x86_64 Ubuntu with an NVIDIA CUDA 12.8-compatible driver")
    for command in ("curl", "tar", "dpkg-deb", "nvidia-smi"):
        if shutil.which(command) is None:
            parser.error(f"Missing prerequisite: {command}")
    for name in ("downloads", "runtime", "compat", "runs"):
        (RUNTIME / name).mkdir(parents=True, exist_ok=True)
    runtime_lock = (RUNTIME / "runtime.lock").open("a")
    try:
        fcntl.flock(runtime_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as error:
        raise RuntimeError("Stop the GGUF server before installing its runtime") from error
    for url, expected, destination in ARTIFACTS:
        archive = RUNTIME / "downloads" / url.rsplit("/", 1)[1]
        if archive.exists():
            if sha256(archive) != expected:
                raise RuntimeError(f"Existing archive is incomplete or corrupt: {archive}; move it aside and retry")
        else:
            partial = archive.with_name(archive.name + ".partial")
            subprocess.run([
                "curl", "-fL", "--retry", "4", "--connect-timeout", "20",
                "--max-time", "1800", "-C", "-", url, "-o", str(partial),
            ], check=True)
            if sha256(partial) != expected:
                raise RuntimeError(f"Downloaded SHA256 mismatch: {partial}")
            partial.replace(archive)
        if destination == "compat":
            command = ["dpkg-deb", "-x", str(archive), str(RUNTIME / destination)]
        else:
            command = ["tar", "-xzf", str(archive), "-C", str(RUNTIME / destination)]
        subprocess.run(command, check=True)
        print(f"Verified and extracted: {archive.name}", flush=True)
    model_dir = ROOT / "local_dependencies/models/RoboBrain2.5-4B-GGUF"
    model_dir.mkdir(parents=True, exist_ok=True)
    for name, expected in MODELS.items():
        target = model_dir / name
        if target.exists():
            if target.is_symlink() or sha256(target) != expected:
                raise RuntimeError(f"Existing model must be an independent verified copy: {target}")
        else:
            source = args.models_from / name
            if not source.is_file() or sha256(source) != expected:
                raise RuntimeError(f"Missing or corrupt model: {source}")
            temporary = target.with_name(target.name + ".copying")
            shutil.copyfile(source, temporary)
            if sha256(temporary) != expected:
                raise RuntimeError(f"Model copy SHA256 mismatch: {temporary}")
            temporary.replace(target)
        print(f"Verified model: {target}", flush=True)
    (ROOT / "local_dependencies/COLCON_IGNORE").touch()
    subprocess.run(["bash", str(ROOT / "scripts/run_robobrain25_gguf.sh"), "--version"], check=True)
    print("Ready. Start with: bash scripts/run_robobrain25_gguf.sh")


if __name__ == "__main__":
    main()
