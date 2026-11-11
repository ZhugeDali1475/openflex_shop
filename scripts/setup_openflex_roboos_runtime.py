#!/usr/bin/env python3
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'local_dependencies/roboos_runtime'
PACKAGES = ['redis-server', 'redis-tools', 'liblzf1', 'lua-cjson', 'lua-bitop', 'liblua5.1-0', 'libjemalloc2']


def main():
    if sys.version_info[:2] != (3, 10):
        raise RuntimeError('The reviewed ROS-adjacent runtime targets Python 3.10')
    for name in ['roboos-standalone-gguf.patch', 'roboos-verified-worker.patch']:
        patch = ROOT / 'scripts/patches' / name
        checked = subprocess.run(['git', '-C', str(ROOT / 'local_dependencies/RoboOS'), 'apply',
            '--reverse', '--check', str(patch)], capture_output=True, text=True)
        if checked.returncode:
            raise RuntimeError(f'First apply/review pinned RoboOS integration: {name}; do not overwrite local source')
    environment = ROOT / '.venvs/roboos-runtime'
    if not (environment / 'bin/python').exists():
        subprocess.run([sys.executable, '-m', 'venv', '--without-pip', str(environment)], check=True)
    subprocess.run([str(environment / 'bin/python'), '-I', str(ROOT / 'local_dependencies/roboos_client/downloads/pip.pyz'),
        'install', '--disable-pip-version-check', '-r', str(ROOT / 'scripts/requirements-roboos-runtime.txt')], check=True)
    packages = RUNTIME / 'redis'
    packages.mkdir(parents=True, exist_ok=True)
    if not list(packages.glob('redis-server_*.deb')):
        subprocess.run(['apt-get', 'download', *PACKAGES], cwd=packages, check=True)
    manifest = []
    for archive in sorted(packages.glob('*.deb')):
        metadata = subprocess.check_output(['dpkg-deb', '-f', str(archive), 'Package', 'Version'], text=True)
        manifest.append({'file': archive.name, 'metadata': metadata,
            'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()})
        subprocess.run(['dpkg-deb', '-x', str(archive), str(packages / 'root')], check=True)
    executable = packages / 'root/usr/bin/redis-server'
    environment_variables = dict(os.environ, LD_LIBRARY_PATH=str(packages / 'root/usr/lib/x86_64-linux-gnu'))
    subprocess.run([str(executable), '--version'], env=environment_variables, check=True)
    (RUNTIME / 'deployment.json').write_text(json.dumps({'packages': manifest, 'redis_bind': '127.0.0.1',
        'redis_port': 16383, 'redis_persistence': False, 'modifies_system_packages': False}, indent=2) + '\n')
    print('Bounded RoboOS runtime ready; no motion or scheduler started.')


if __name__ == '__main__':
    main()
