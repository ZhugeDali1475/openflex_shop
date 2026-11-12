import argparse
from datetime import datetime
import json
from pathlib import Path


def analyze_trial(directory):
    report = json.loads((directory / 'trial_result.json').read_text())
    if report.get('status') != 'finished' or 'execution_seconds' not in report:
        return {'id': directory.name, 'status': report['status'], 'success': report['success'],
                'timing_available': False}
    operations = report['operations']
    motion_seconds = 0.0
    phases = []
    skills = []
    probe_seconds = []
    for operation in operations:
        duration = operation['finished_at'] - operation['started_at']
        if duration < 0:
            raise ValueError('Operation wall clock moved backwards')
        motion_seconds += duration
        skills.append({'tool': operation['tool'], 'request_id': operation['request_id'],
                       'wall_seconds': duration, 'status': operation['status'],
                       'stopped_confirmed': operation['stopped_confirmed']})
        started = None
        for event in operation.get('events', []):
            if event['kind'] == 'planning_probe' and 'elapsed_seconds' in event['data']:
                probe_seconds.append(event['data']['elapsed_seconds'])
            if event['kind'] == 'trajectory_start':
                if started is not None:
                    raise ValueError('Overlapping trajectory events cannot be summed')
                started = event
            elif event['kind'] == 'trajectory_executed':
                if started is None or started['data']['phase'] != event['data']['phase']:
                    raise ValueError('Trajectory completion has no matching start')
                wall = event['time'] - started['time']
                if wall < 0:
                    raise ValueError('Trajectory wall clock moved backwards')
                phases.append({'phase': event['data']['phase'], 'planned_seconds': started['data']['duration_sec'],
                               'wall_seconds': wall, 'tool': operation['tool']})
                started = None
        if started is not None:
            raise ValueError('Unfinished trajectory is not a measured successful interval')
    elapsed = report['execution_seconds']
    if motion_seconds > elapsed + 1:
        raise ValueError('Operation duration exceeds process duration; clocks or overlapping records need review')
    model_path = directory / 'task/model_trace.json'
    model = json.loads(model_path.read_text()) if model_path.exists() else []
    inference = sum(item['inference_seconds'] for item in model) if all('inference_seconds' in item for item in model) else None
    trajectory_seconds = sum(phase['wall_seconds'] for phase in phases)
    return {'id': directory.name, 'success': report['success'], 'timing_available': True,
            'execution_seconds': elapsed, 'trial_including_preparation_seconds': report.get('total_seconds'),
            'motion_operation_seconds': motion_seconds, 'trajectory_wall_seconds': trajectory_seconds,
            'motion_nontrajectory_seconds': motion_seconds - trajectory_seconds,
            'outside_motion_seconds': elapsed - motion_seconds,
            'worker_inference_seconds': inference, 'worker_inferences': report['worker_inferences'],
            'probe_measured_seconds': sum(probe_seconds) if probe_seconds else None,
            'probe_measurements': len(probe_seconds), 'phases': phases, 'skills': skills,
            'measurement_note': 'Process duration is monotonic; operation/event durations use recorded wall clock. '
                                'Outside-motion time is NOT pure model inference; trajectory intervals include controller/stopped waits.'}


def main():
    parser = argparse.ArgumentParser(description='Read-only decomposition of recorded simulation trial timing')
    parser.add_argument('batch', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    arguments = parser.parse_args()
    protocol = json.loads((arguments.batch / 'frozen_protocol.json').read_text())['protocol']
    trials = []
    for trial in protocol['trials']:
        folder = arguments.batch / trial['id']
        if (folder / 'trial_result.json').exists():
            trials.append(analyze_trial(folder))
        else:
            trials.append({'id': trial['id'], 'status': 'not_collected', 'timing_available': False})
    result = {'created_at': datetime.now().astimezone().isoformat(), 'source_batch': str(arguments.batch.resolve()),
              'physical_verification': False, 'scheduled': len(protocol['trials']), 'trials': trials}
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    with arguments.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    for trial in trials:
        print(trial['id'], {key: round(value, 2) if isinstance(value, float) else value
                           for key, value in trial.items() if key not in ('phases', 'skills', 'measurement_note')})


if __name__ == '__main__':
    main()
