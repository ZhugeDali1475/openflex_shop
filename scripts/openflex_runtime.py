#!/usr/bin/env python3
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from urllib.request import ProxyHandler, Request, build_opener


ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / 'local_dependencies/openflex_roboos'
UNITS = {
    'model': 'openflex-robobrain25-gguf',
    'redis': 'openflex-roboos-redis',
    'readonly': 'openflex-roboos-readonly',
    'simulation': 'openflex-roboos-navigation',
    'planning': 'openflex-manipulation-preflight',
    'skills': 'openflex-roboos-delivery-mcp',
    'viewer': 'openflex-live-viewer',
    'scheduler': 'openflex-roboos-scheduler',
}


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def request_json(url, payload=None):
    body = None if payload is None else json.dumps(payload).encode()
    request = Request(url, data=body, headers={'Content-Type': 'application/json'})
    with build_opener(ProxyHandler({})).open(request, timeout=5) as response:
        return json.load(response)


def active(unit):
    return subprocess.run(['systemctl', '--user', 'is-active', '--quiet', unit], check=False).returncode == 0


def pending_operations(path=None):
    path = path or RUNTIME / 'navigation_operations.jsonl'
    records = {}
    if path.exists():
        for line in path.read_text().splitlines():
            record = json.loads(line)
            records[record['request_id']] = record
    return [record for record in records.values() if record['status'] in ('running', 'unknown')]


def measured_hold(snapshot):
    try:
        if snapshot['environment'] != 'mujoco_simulation' or snapshot['physical_verification'] is not False:
            return False
        sources = snapshot['sources']
        for name in ('simulation_diagnostic', 'odometry', 'joint_states'):
            if sources[name]['status'] != 'fresh':
                return False
        diagnostic = sources['simulation_diagnostic']['data']
        acknowledgement = diagnostic['control_ack']
        if (diagnostic['source'] != 'mujoco_diagnostic_only_NOT_PHYSICAL'
                or acknowledgement['mode'] != 0 or acknowledgement['fresh'] is not True
                or acknowledgement['inputs_healthy'] is not True):
            return False
        twist = sources['odometry']['data']['twist']['twist']
        base = [twist['linear'][axis] for axis in ('x', 'y', 'z')]
        angular = [twist['angular'][axis] for axis in ('x', 'y', 'z')]
        joints = sources['joint_states']['data']['velocity']
        return bool(joints and all(math.isfinite(value) and abs(value) <= 0.02 for value in base)
                    and all(math.isfinite(value) and abs(value) <= 0.03 for value in angular + joints))
    except (KeyError, TypeError, ValueError):
        return False


def scheduler_process():
    path = RUNTIME / 'current_scheduler.json'
    if not path.exists():
        return None
    record = json.loads(path.read_text())
    try:
        command = Path(f'/proc/{int(record["pid"])}/cmdline').read_bytes().split(b'\0')
    except FileNotFoundError:
        return None
    if str(ROOT / 'scripts/run_openflex_roboos_delivery.py').encode() not in command:
        return None
    return record


def status():
    report = {'scope': 'isolated_MuJoCo_domain_83_NOT_PHYSICAL',
              'units': {name: active(unit) for name, unit in UNITS.items()},
              'scheduler': scheduler_process(), 'pending_operations': pending_operations()}
    try:
        snapshot = request_json('http://127.0.0.1:18552/state')
        report['measured_hold'] = measured_hold(snapshot)
        report['source_status'] = {name: value['status'] for name, value in snapshot['sources'].items()}
        report['object'] = snapshot['sources']['simulation_diagnostic'].get('data', {}).get('object_pose')
    except Exception as error:
        report.update(measured_hold=False, snapshot_error=str(error))
    return report


def start_unit(name, command, environment=None):
    unit = UNITS[name]
    if active(unit):
        return
    subprocess.run(['systemctl', '--user', 'reset-failed', unit], check=False, capture_output=True)
    clean = ['/usr/bin/env', '-u', 'ALL_PROXY', '-u', 'all_proxy', '-u', 'HTTP_PROXY', '-u', 'http_proxy',
             '-u', 'HTTPS_PROXY', '-u', 'https_proxy', 'ROS_DOMAIN_ID=83', *(environment or [])]
    subprocess.run(['systemd-run', '--user', '--unit=' + unit, '--property=TimeoutStopSec=150',
                    '--property=SendSIGKILL=no', '--property=KillSignal=SIGINT',
                    '--property=SuccessExitStatus=130 143', *clean, *command], check=True)
    time.sleep(0.25)
    if not active(unit):
        raise RuntimeError(f'{unit} did not stay active; inspect journalctl --user -u {unit}')


def start(arguments):
    if not arguments.enable_simulation_motion:
        raise RuntimeError('start requires --enable-simulation-motion; never starts physical hardware')
    if pending_operations() or scheduler_process() or active(UNITS['scheduler']):
        raise RuntimeError('Pending operation or scheduler exists; audit/stop it before lifecycle changes')
    if not active(UNITS['simulation']) and active(UNITS['planning']):
        raise RuntimeError('Orphan planning clock: stop/audit existing planning stack before a fresh simulation')
    for name, filename in [('model', 'run_robobrain25_gguf.sh'), ('redis', 'run_openflex_roboos_redis.sh'),
                           ('readonly', 'run_openflex_readonly_bridge.sh'),
                           ('simulation', 'run_openflex_sim_navigation.sh'),
                           ('planning', 'run_openflex_manipulation_preflight.sh')]:
        environment = ['OPENFLEX_ENABLE_SIM_NAV=1', 'OPENFLEX_ENABLE_SIM_MANIPULATION=1',
                       'OPENFLEX_ENABLE_SIM_PLACE=1'] if name == 'simulation' else []
        start_unit(name, ['/bin/bash', str(ROOT / 'scripts' / filename)], environment)
    start_unit('skills', [str(ROOT / '.venvs/openflex-mcp/bin/python'), '-I',
                         str(ROOT / 'scripts/openflex_mcp_entry.py'), '--port', '18556',
                         '--enable-navigation', '--enable-manipulation', '--enable-placement'])
    if arguments.gui:
        start_unit('viewer', ['/bin/bash', str(ROOT / 'scripts/run_openflex_live_viewer.sh')],
                   ['DISPLAY=' + os.environ.get('DISPLAY', ':0'),
                    'XAUTHORITY=' + os.environ.get('XAUTHORITY', '/run/user/1000/gdm/Xauthority')])
    return status()


def safe_stop(arguments):
    output = RUNTIME / 'runs' / datetime.now().strftime('safe_stop_%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True)
    report = {'success': False, 'physical_verification': False, 'operations': [], 'snapshots': []}
    try:
        scheduler = scheduler_process()
        if scheduler:
            save(Path(scheduler['output']) / 'STOP', {'reason': 'operator_safe_stop'})
            os.kill(scheduler['pid'], signal.SIGINT)
        deadline = time.monotonic() + arguments.timeout
        stable_stamps = []
        canceled_ids = set()
        while time.monotonic() < deadline:
            pending = pending_operations()
            for operation in pending:
                request_id = operation['request_id']
                if request_id not in canceled_ids:
                    reply = request_json('http://127.0.0.1:18553/cancel', {'request_id': request_id})
                    report['operations'].append({'cancel': reply, 'request_id': request_id})
                    canceled_ids.add(request_id)
                observed = request_json('http://127.0.0.1:18553/status', {'request_id': request_id})
                report['operations'].append({'observed': observed, 'request_id': request_id})
            snapshot = request_json('http://127.0.0.1:18552/state')
            save(output / 'latest_snapshot.json', snapshot)
            stamp = snapshot['sources']['simulation_diagnostic'].get('source_stamp_ns')
            holding = measured_hold(snapshot)
            report['snapshots'].append({'stamp_ns': stamp, 'measured_hold': holding})
            if holding and stamp is not None and not pending:
                if not stable_stamps or stamp > stable_stamps[-1]:
                    stable_stamps.append(stamp)
                elif stamp < stable_stamps[-1]:
                    stable_stamps.clear()
            else:
                stable_stamps.clear()
            if len(stable_stamps) >= 3 and not scheduler_process() and not active(UNITS['scheduler']):
                for request_id in canceled_ids:
                    terminal = request_json('http://127.0.0.1:18553/status', {'request_id': request_id})
                    if terminal.get('status') not in ('succeeded', 'failed', 'canceled', 'timed_out') or terminal.get('stopped_confirmed') is not True:
                        raise RuntimeError('Cancel request is not independent stopped confirmation')
                report.update(success=True, stopped_confirmed=True)
                break
            time.sleep(0.25)
        if not report['success']:
            raise RuntimeError('Stopped state not confirmed; services left running, no force kill or false success')
        if arguments.shutdown_services:
            units = [UNITS[name] for name in ('viewer', 'skills', 'planning', 'simulation', 'readonly', 'model', 'redis')]
            units += ['openflex-roboos-navigation-mcp', 'openflex-roboos-manipulation-mcp']
            for unit in units:
                subprocess.run(['systemctl', '--user', 'stop', unit], check=True)
        return report
    except BaseException as error:
        report['error'] = str(error)
        raise
    finally:
        save(output / 'result.json', report)
        print('Safe-stop evidence:', output)


def run_delivery(arguments):
    if not arguments.execute_isolated_simulation:
        raise RuntimeError('run requires --execute-isolated-simulation; actual bounded simulation motion')
    if scheduler_process() or pending_operations() or active(UNITS['scheduler']):
        raise RuntimeError('Another scheduler or unresolved operation exists; no retry/new request permitted')
    required = ('model', 'redis', 'readonly', 'simulation', 'planning', 'skills')
    if not all(active(UNITS[name]) for name in required):
        raise RuntimeError('Required services are not active; use status/start first')
    snapshot = request_json('http://127.0.0.1:18552/state')
    if not measured_hold(snapshot):
        raise RuntimeError('Fresh healthy measured HOLD required before scheduling')
    diagnostic = snapshot['sources']['simulation_diagnostic']['data']
    nominal = snapshot['scene_registry']['objects']['thesis_item_01']['nominal_pose_xyzyaw'][:3]
    if diagnostic.get('attached_arm') is not None or math.dist(diagnostic['object_pose']['position_m'], nominal) > 0.04:
        raise RuntimeError('Object is not at shelf origin: current delivered scene is preserved; no automatic reset')
    command = [str(ROOT / '.venvs/roboos-runtime/bin/python'), '-I',
               str(ROOT / 'scripts/run_openflex_roboos_delivery.py'), '--execute-isolated-simulation']
    if arguments.task:
        command += ['--task', arguments.task]
    start_unit('scheduler', command)
    return {'status': 'started', 'unit': UNITS['scheduler'], 'physical_verification': False}


def main():
    parser = argparse.ArgumentParser(description='Isolated OpenFlex lifecycle; no scene reset and no hardware')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status')
    starter = commands.add_parser('start')
    starter.add_argument('--enable-simulation-motion', action='store_true')
    starter.add_argument('--gui', action='store_true')
    stopper = commands.add_parser('stop')
    stopper.add_argument('--timeout', type=float, default=150)
    stopper.add_argument('--shutdown-services', action='store_true')
    runner = commands.add_parser('run')
    runner.add_argument('--execute-isolated-simulation', action='store_true')
    runner.add_argument('--task')
    arguments = parser.parse_args()
    result = {'status': status, 'start': lambda: start(arguments), 'stop': lambda: safe_stop(arguments),
              'run': lambda: run_delivery(arguments)}[arguments.command]()
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
