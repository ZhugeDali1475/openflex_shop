#!/usr/bin/env python3
import hashlib
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
PIP_SHA256 = "be510f6d23d345eeabdd3eec2599a3ca2df3cbc98c4afbe2631561ca12e744d7"


def main():
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("Use Ubuntu 22.04 /usr/bin/python3 (3.10)")
    archive = ROOT / "local_dependencies/roboos_client/downloads/pip.pyz"
    if not archive.is_file() or hashlib.sha256(archive.read_bytes()).hexdigest() != PIP_SHA256:
        raise RuntimeError("Verified pip.pyz missing; first run scripts/setup_roboos_gguf_client.py")
    environment = ROOT / ".venvs/openflex-mcp"
    if not (environment / "bin/python").is_file():
        subprocess.run([sys.executable, "-m", "venv", "--without-pip", str(environment)], check=True)
    python = str(environment / "bin/python")
    subprocess.run([python, "-I", str(archive), "install", "--disable-pip-version-check",
                    "-r", str(ROOT / "scripts/requirements-openflex-mcp.txt")], check=True)
    subprocess.run([python, "-I", str(archive), "check"], check=True)
    print("MCP environment ready. ROS uses system Python, not this environment.")


if __name__ == "__main__":
    main()
