#!/usr/bin/env python3
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'openflex_ws/src/openflex_thesis/openflex_thesis_mujoco'))

from openflex_runtime import ROOT, RUNTIME, UNITS, active, measured_hold, pending_operations, request_json, safe_stop, save, scheduler_process, start
from openflex_delivery_contract import DeliveryContract
from openflex_motion_profile import validate_profile


def independent_completion(snapshot):
    contract = DeliveryContract()
    contract.checked_state = True
    contract.place = True
    try:
        contract.record('get_robot_state', {}, snapshot)
        return contract.completed
    except (RuntimeError, KeyError, TypeError, ValueError):
        return False


def initial_sources_ready(snapshot):
    return bool(measured_hold(snapshot)
                and snapshot.get('sources', {}).get('object_memory', {}).get('status') == 'fresh')


def digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def source_hashes():
    directories = [ROOT / 'scripts', ROOT / 'openflex_ws/src/openflex_thesis']
    suffixes = {'.py', '.yaml', '.yml', '.json', '.sh', '.patch', '.xml', '.msg', '.srv', '.action', '.cfg'}
    return {str(path.relative_to(ROOT)): digest(path) for directory in directories
            for path in sorted(directory.rglob('*'))
            if path.is_file() and path.suffix in suffixes and '__pycache__' not in path.parts}


def validate_protocol(protocol):
    if protocol.get('scope') != 'isolated_MuJoCo_domain_83_NOT_PHYSICAL' or protocol.get('automatic_retry') is not False:
        raise ValueError('Only the isolated no-retry protocol is permitted')
    identifiers = [trial['id'] for trial in protocol['trials']]
    if len(identifiers) != len(set(identifiers)) or len(identifiers) != protocol['denominators']['all_scheduled']:
        raise ValueError('Trial IDs and frozen denominator must match')
    if not 60 <= protocol['max_trial_seconds'] <= 1200:
        raise ValueError('A bounded trial deadline is required')
    for trial in protocol['trials']:
        validate_profile(trial.get('motion_profile', 'baseline'))
        if (not trial['id'].replace('_', '').isalnum() or trial['policy'] not in ('model', 'fixed')
                or (trial['inject_stale_once'] and trial['policy'] != 'model')):
            raise ValueError('Invalid trial declaration')


def freeze(output, protocol_path):
    protocol = json.loads(protocol_path.read_text())
    validate_protocol(protocol)
    output.mkdir(parents=True, exist_ok=False)
    manifest = {'frozen_at': datetime.now().astimezone().isoformat(), 'protocol': protocol,
                'git_head': subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                'git_status': subprocess.check_output(['git', '-C', str(ROOT), 'status', '--short'], text=True),
                'source_sha256': source_hashes(),
                'roboos_revision': subprocess.check_output(['git', '-C', str(ROOT / 'local_dependencies/RoboOS'),
                                                          'rev-parse', 'HEAD'], text=True).strip(),
                'model_files': {}}
    for filename in ('RoboBrain2.5-4B.Q4_K_M.gguf', 'RoboBrain2.5-4B.mmproj-f16.gguf'):
        path = Path('/home/wgb/下载') / filename
        manifest['model_files'][filename] = {'bytes': path.stat().st_size, 'sha256': digest(path)}
    save(output / 'frozen_protocol.json', manifest)
    save(output / 'freeze_seal.json', {'frozen_protocol_sha256': digest(output / 'frozen_protocol.json')})
    print(output / 'frozen_protocol.json')


def load_frozen(output):
    seal = json.loads((output / 'freeze_seal.json').read_text())
    if digest(output / 'frozen_protocol.json') != seal['frozen_protocol_sha256']:
        raise RuntimeError('Frozen protocol has changed; do not run this batch')
    manifest = json.loads((output / 'frozen_protocol.json').read_text())
    validate_protocol(manifest['protocol'])
    if source_hashes() != manifest['source_sha256']:
        raise RuntimeError('Execution sources changed since freeze; make a separately identified new batch')
    return manifest


def prepare_trial(output, tolerance, motion_profile='baseline'):
    safe_stop(SimpleNamespace(timeout=150, shutdown_services=False))
    save(output / 'previous_scene.json', request_json('http://127.0.0.1:18552/state'))
    if pending_operations() or scheduler_process() or active(UNITS['scheduler']):
        raise RuntimeError('Unresolved execution; no scene reset permitted')
    subprocess.run(['systemctl', '--user', 'stop', UNITS['planning'], UNITS['simulation'], UNITS['readonly']], check=True)
    start(SimpleNamespace(enable_simulation_motion=True, gui=True, motion_profile=motion_profile))
    deadline = time.monotonic() + 75
    while time.monotonic() < deadline:
        try:
            snapshot = request_json('http://127.0.0.1:18552/state')
            sources = snapshot['sources']
            ready = initial_sources_ready(snapshot)
            if ready:
                from math import dist
                nominal = snapshot['scene_registry']['objects']['thesis_item_01']['nominal_pose_xyzyaw'][:3]
                diagnostic = sources['simulation_diagnostic']['data']
                if (diagnostic.get('safety_stop') is not None or diagnostic.get('attached_arm') is not None
                        or dist(diagnostic['object_pose']['position_m'], nominal) > tolerance):
                    raise RuntimeError('Measured initial conditions do not match the frozen protocol')
                save(output / 'initial_snapshot.json', snapshot)
                return
        except (OSError, KeyError, TypeError):
            pass
        time.sleep(1)
    raise RuntimeError('Fresh initial scene/planning stack not ready; trial remains in the denominator')


def resource_sample():
    memory = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        fields = line.split()
        if fields[0].rstrip(':') in ('MemAvailable', 'SwapTotal', 'SwapFree'):
            memory[fields[0].rstrip(':')] = int(fields[1])
    result = {'time': datetime.now().astimezone().isoformat(), 'memory_kib': memory,
              'load_average': os.getloadavg()}
    try:
        result['gpu_memory_mib'] = subprocess.check_output(['nvidia-smi', '--query-gpu=memory.used,memory.free',
            '--format=csv,noheader,nounits'], text=True, timeout=4).strip()
    except (OSError, subprocess.SubprocessError) as error:
        result['gpu_error'] = str(error)
    return result


def terminal_records(task_id):
    records = {}
    for line in (RUNTIME / 'navigation_operations.jsonl').read_text().splitlines():
        record = json.loads(line)
        if record['request_id'].startswith(task_id + '_'):
            records[record['request_id']] = record
    return list(records.values())


def run_trial(output, trial, protocol):
    report = {'trial': trial, 'success': False, 'status': 'not_started', 'physical_verification': False}
    task_id = output.parent.name + '_' + trial['id']
    task_dir = output / 'task'
    runner = recorder = None
    started = time.monotonic()
    try:
        prepare_trial(output, protocol['initial_object_tolerance_m'], trial.get('motion_profile', 'baseline'))
        with (output / 'recording.log').open('w') as log:
            recorder = subprocess.Popen(['/bin/bash', str(ROOT / 'scripts/run_openflex_recording.sh'),
                '--output', str(output / 'video'), '--duration', str(protocol['max_trial_seconds']),
                '--fps', str(protocol['video_fps'])], stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 35
        while not (output / 'video/first_frame.png').exists():
            if recorder.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Recorder did not produce a measured first frame')
            time.sleep(0.25)
        environment = {key: value for key, value in os.environ.items() if not key.lower().endswith('_proxy')}
        environment.update(ROS_DOMAIN_ID='83')
        command = [str(ROOT / '.venvs/roboos-runtime/bin/python'), '-I',
            str(ROOT / 'scripts/run_openflex_roboos_delivery.py'), '--task-id', task_id,
            '--task', trial['task'], '--output', str(task_dir), '--policy', trial['policy'],
            '--execute-isolated-simulation']
        if trial['inject_stale_once']:
            command.append('--inject-stale-once')
        with (output / 'runner.log').open('w') as log:
            runner = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
        executing = time.monotonic()
        with (output / 'resources.jsonl').open('w') as samples:
            while runner.poll() is None:
                samples.write(json.dumps(resource_sample()) + '\n')
                samples.flush()
                if time.monotonic() - executing > protocol['max_trial_seconds']:
                    save(task_dir / 'STOP', {'reason': 'frozen_trial_deadline'})
                    runner.send_signal(signal.SIGINT)
                    raise TimeoutError('Frozen trial deadline reached; no automatic retry')
                time.sleep(2)
        report.update(status='finished', runner_exit=runner.returncode,
                      execution_seconds=time.monotonic() - executing)
        result = json.loads((task_dir / 'result.json').read_text())
        report['task_result'] = result
        snapshot = request_json('http://127.0.0.1:18552/state')
        save(output / 'independent_final_snapshot.json', snapshot)
        report['measured_hold'] = measured_hold(snapshot)
        report['independent_completion'] = independent_completion(snapshot)
        report['operations'] = terminal_records(task_id)
        trace = json.loads((task_dir / 'tool_trace.json').read_text())
        report['tool_calls'] = len(trace)
        report['rejected_before_execution'] = sum(item['raw_result'].get('status') == 'rejected_before_execution'
                                                   for item in trace)
        report['motion_calls'] = sum(item['selected_tool'] in ('navigate_to_target', 'grasp_object',
                                                               'place_to_affordance')
                                     and 'request_id' in item.get('backend_arguments', {}) for item in trace)
        model_trace = task_dir / 'model_trace.json'
        report['worker_inferences'] = len(json.loads(model_trace.read_text())) if model_trace.exists() else 0
        report['fault_reobserved'] = bool(trial['inject_stale_once'] and sum(
            item['selected_tool'] == 'observe_scene' for item in trace) >= 2)
        report['final_safety_stop'] = snapshot['sources']['simulation_diagnostic']['data'].get('safety_stop')
        report['success'] = bool(result.get('passed') is True and runner.returncode == 0
                                  and report['measured_hold'] and report['independent_completion'])
    except BaseException as error:
        report.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed',
                      error=f'{type(error).__name__}: {error}')
        if runner is not None and runner.poll() is None:
            if task_dir.exists():
                save(task_dir / 'STOP', {'reason': 'experiment_failed'})
            runner.send_signal(signal.SIGINT)
            try:
                runner.wait(timeout=150)
            except subprocess.TimeoutExpired:
                report['stop_unknown'] = True
    finally:
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=30)
            except subprocess.TimeoutExpired:
                report['recorder_stop_unknown'] = True
        video_manifest = output / 'video/manifest.json'
        if video_manifest.exists():
            report['video'] = json.loads(video_manifest.read_text())
        report['total_seconds'] = time.monotonic() - started
        save(output / 'trial_result.json', report)
    return report


def run_batch(output):
    manifest = load_frozen(output)
    marker = output / 'collection_started.json'
    with marker.open('x') as stream:
        json.dump({'started_at': datetime.now().astimezone().isoformat()}, stream)
    protocol = manifest['protocol']
    reports = []
    try:
        for trial in protocol['trials']:
            if source_hashes() != manifest['source_sha256']:
                raise RuntimeError('Source drift during batch; stop collecting, preserve all scheduled trials')
            trial_dir = output / trial['id']
            trial_dir.mkdir(exist_ok=False)
            print('START', trial['id'], flush=True)
            report = run_trial(trial_dir, trial, protocol)
            reports.append(report)
            print('END', trial['id'], 'success=', report['success'], flush=True)
            safe_stop(SimpleNamespace(timeout=150, shutdown_services=False))
            if report['status'] == 'interrupted':
                break
    finally:
        collected = {report['trial']['id']: report for report in reports}
        summary = {'protocol_id': protocol['protocol_id'], 'scheduled': len(protocol['trials']),
                   'collected': len(reports), 'successes': sum(report['success'] for report in reports),
                   'not_collected': [trial['id'] for trial in protocol['trials'] if trial['id'] not in collected],
                   'trials': [{'id': report['trial']['id'], 'policy': report['trial']['policy'],
                               'success': report['success'], 'status': report['status'],
                               'execution_seconds': report.get('execution_seconds'),
                               'worker_inferences': report.get('worker_inferences'),
                               'tool_calls': report.get('tool_calls'),
                               'motion_calls': report.get('motion_calls'),
                               'rejected_before_execution': report.get('rejected_before_execution'),
                               'fault_reobserved': report.get('fault_reobserved'),
                               'final_safety_stop': report.get('final_safety_stop'),
                               'valid_video': report.get('video', {}).get('valid_visual_evidence', False)} for report in reports],
                   'limitations': protocol['limitations'], 'physical_verification': False}
        save(output / 'summary.json', summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description='Freeze then collect one no-retry isolated evaluation batch')
    parser.add_argument('command', choices=['freeze', 'run'])
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--protocol', type=Path, default=ROOT / 'scripts/openflex_delivery_protocol.json')
    parser.add_argument('--execute-isolated-simulation', action='store_true')
    arguments = parser.parse_args()
    if arguments.command == 'freeze':
        freeze(arguments.output.resolve(), arguments.protocol)
    else:
        if not arguments.execute_isolated_simulation:
            parser.error('run requires --execute-isolated-simulation')
        run_batch(arguments.output.resolve())


if __name__ == '__main__':
    main()
