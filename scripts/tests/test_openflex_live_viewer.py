import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


pytest.importorskip('rclpy')
pytest.importorskip('mujoco')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import view_openflex_live_sim as live


def diagnostic(second):
    return {'source': 'mujoco_diagnostic_only_NOT_PHYSICAL', 'stamp': {'sec': second, 'nanosec': 0},
        'joint_state': {'names': [], 'position_rad_or_m': []},
        'object_pose': {'position_m': [second, 0.0, 0.0], 'orientation_xyzw': [0.0, 0.0, 0.0, 1.0]},
        'simulation_time_sec': float(second)}


def odometry():
    return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        orientation=SimpleNamespace(w=1.0, z=0.0), position=SimpleNamespace(x=0.0, y=0.0))))


def test_display_consumes_newest_coherent_frame_without_replaying_older_matches(monkeypatch):
    monkeypatch.setattr(live.mujoco, 'mj_name2id', lambda model, kind, name: 0 if name == 'thesis_object_free' else -1)
    monkeypatch.setattr(live.mujoco, 'mj_forward', lambda model, data: None)
    node = SimpleNamespace(model=SimpleNamespace(jnt_qposadr=[0]), data=SimpleNamespace(qpos=[0.0] * 7, time=0.0),
        frames={(second, 0): diagnostic(second) for second in [1, 2]},
        odometry={(second, 0): odometry() for second in [1, 2]}, frame_count=1)
    live.LiveView.update(node)
    assert node.data.time == 2.0 and node.data.qpos[0] == 2.0
    live.LiveView.update(node)
    assert node.data.time == 2.0 and not node.frames and not node.odometry


def test_simulation_restart_discards_old_clock_epoch_buffers():
    node = SimpleNamespace(latest_diagnostic_stamp=(100, 0), frames={(100, 0): diagnostic(100)},
        odometry={(100, 0): odometry()})
    live.LiveView.receive_diagnostic(node, SimpleNamespace(data=json.dumps(diagnostic(1))))
    assert list(node.frames) == [(1, 0)] and not node.odometry
