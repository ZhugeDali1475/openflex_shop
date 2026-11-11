#!/usr/bin/env python3
import hashlib
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / "local_dependencies/roboos_client"
ROBOOS = ROOT / "local_dependencies/RoboOS"
REVISION = "1c39b85ef5c9dcb9d5da235665c19af21824e545"
FLAG_REVISION = "3fc2037f90917227bd4aebabd9d7b330523f437c"
ARTIFACTS = [
    (
        "https://bootstrap.pypa.io/pip/pip.pyz",
        "pip.pyz",
        "be510f6d23d345eeabdd3eec2599a3ca2df3cbc98c4afbe2631561ca12e744d7",
    ),
    (
        f"https://codeload.github.com/FlagOpen/FlagScale/tar.gz/{FLAG_REVISION}",
        "flagscale-3fc2037.tar.gz",
        "7614831d618032101ebcfdd65732120c952a09d5bb92faccc75d4e4abd2ac2ba",
    ),
]


def main():
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError("The pinned client environment targets Python 3.10")
    actual_revision = subprocess.check_output(["git", "-C", str(ROBOOS), "rev-parse", "HEAD"], text=True).strip()
    if actual_revision != REVISION:
        raise RuntimeError(f"Expected RoboOS {REVISION}, got {actual_revision}; do not apply the patch blindly")
    downloads = RUNTIME / "downloads"
    downloads.mkdir(parents=True, exist_ok=True)
    for url, filename, expected in ARTIFACTS:
        target = downloads / filename
        if not target.exists():
            partial = target.with_name(target.name + ".partial")
            subprocess.run([
                "curl", "-fL", "--retry", "3", "--connect-timeout", "15", "--max-time", "180",
                url, "-o", str(partial),
            ], check=True)
            if hashlib.sha256(partial.read_bytes()).hexdigest() != expected:
                raise RuntimeError(f"SHA256 mismatch: {partial}; upstream may have changed, review before updating")
            partial.replace(target)
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise RuntimeError(f"SHA256 mismatch: {target}")
    environment = ROOT / ".venvs/roboos-client"
    python = environment / "bin/python"
    if not python.exists():
        subprocess.run([sys.executable, "-m", "venv", "--without-pip", str(environment)], check=True)
    subprocess.run([
        str(python), "-I", str(downloads / "pip.pyz"), "install", "--disable-pip-version-check",
        "-r", str(ROOT / "scripts/requirements-roboos-client.txt"),
    ], check=True)
    source_root = RUNTIME / f"FlagScale-{FLAG_REVISION}"
    if not source_root.exists():
        subprocess.run(["tar", "-xzf", str(downloads / "flagscale-3fc2037.tar.gz"), "-C", str(RUNTIME)], check=True)
    alias = RUNTIME / "flag_scale"
    if not alias.exists():
        alias.symlink_to(source_root.name, target_is_directory=True)
    elif alias.resolve() != source_root.resolve():
        raise RuntimeError("Existing FlagScale alias points to a different source; not overwriting")
    for name in ['roboos-standalone-gguf.patch', 'roboos-verified-worker.patch']:
        patch = ROOT / 'scripts/patches' / name
        check = subprocess.run(["git", "-C", str(ROBOOS), "apply", "--check", str(patch)], capture_output=True, text=True)
        if check.returncode == 0:
            subprocess.run(["git", "-C", str(ROBOOS), "apply", str(patch)], check=True)
        else:
            reverse = subprocess.run([
                "git", "-C", str(ROBOOS), "apply", "--reverse", "--check", str(patch),
            ], capture_output=True, text=True)
            if reverse.returncode != 0:
                raise RuntimeError(f"RoboOS patch {name} conflicts with local changes; inspect manually, no files overwritten")
    print("Client environment and reversible RoboOS patch are ready.")
    print("Validate without robot actions: bash scripts/run_roboos_gguf_validation.sh")


if __name__ == "__main__":
    main()
