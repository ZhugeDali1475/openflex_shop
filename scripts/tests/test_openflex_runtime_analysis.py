import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from analyze_openflex_runtime import analyze_trial


def fixture(tmp_path, events):
    report = {'status': 'finished', 'success': True, 'execution_seconds': 20, 'total_seconds': 25,
              'worker_inferences': 1, 'operations': [{'tool': 'grasp_object', 'request_id': 'test',
               'started_at': 100, 'finished_at': 115, 'status': 'succeeded', 'stopped_confirmed': True, 'events': events}]}
    (tmp_path / 'trial_result.json').write_text(json.dumps(report))


def test_distinguishes_motion_trajectory_and_unknown_model_time(tmp_path):
    fixture(tmp_path, [{'kind': 'trajectory_start', 'time': 101, 'data': {'phase': 'transport', 'duration_sec': 10}},
                       {'kind': 'trajectory_executed', 'time': 112, 'data': {'phase': 'transport'}}])
    (tmp_path / 'task').mkdir()
    (tmp_path / 'task/model_trace.json').write_text('[{"time":"old_record_without_duration"}]')
    result = analyze_trial(tmp_path)
    assert result['trajectory_wall_seconds'] == 11
    assert result['outside_motion_seconds'] == 5
    assert result['motion_nontrajectory_seconds'] == 4
    assert result['worker_inference_seconds'] is None


def test_missing_completion_cannot_be_counted_as_successful_timing(tmp_path):
    fixture(tmp_path, [{'kind': 'trajectory_start', 'time': 101, 'data': {'phase': 'transport'}}])
    with pytest.raises(ValueError, match='Unfinished'):
        analyze_trial(tmp_path)


def test_retains_failed_trial_without_fabricating_duration(tmp_path):
    (tmp_path / 'trial_result.json').write_text('{"status":"failed","success":false}')
    assert analyze_trial(tmp_path)['timing_available'] is False
