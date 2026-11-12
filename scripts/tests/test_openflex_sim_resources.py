import os
from pathlib import Path
import subprocess


def test_native_thread_budget_is_explicit_and_does_not_change_ros_safety_settings():
    root = Path(__file__).resolve().parents[2]
    command = 'source scripts/openflex_sim_resources.sh; printf "%s %s %s %s" "$OMP_NUM_THREADS" "$OPENBLAS_NUM_THREADS" "$ROS_DOMAIN_ID" "$OPENFLEX_ENABLE_SIM_MANIPULATION"'
    environment = dict(os.environ, ROS_DOMAIN_ID='83', OPENFLEX_ENABLE_SIM_MANIPULATION='1', OMP_NUM_THREADS='64')
    result = subprocess.check_output(['bash', '-c', command], cwd=root, env=environment, text=True)
    assert result == '1 1 83 1'
