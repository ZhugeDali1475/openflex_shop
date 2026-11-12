from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_method_counterexamples import evaluate


def test_actual_region_code_rejects_constructed_mixed_conditions():
    report = evaluate()
    assert report['variants']['condition_and_component'] == [1, 2]
    assert report['variants']['condition_only'] == [1, 2, 6]
    assert report['variants']['pooled_high_score'] == [1, 2, 3, 4, 5, 6, 7]
    assert all(not item['feasible'] for item in report['scores'] if item['candidate_id'] == 9)
    assert 'NOT_robot_experiment' in report['evidence_type']
