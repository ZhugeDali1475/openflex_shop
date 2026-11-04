from pathlib import Path
import sys

from nav_msgs.msg import Odometry


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from safe_odom_mux import odometry_is_fresh  # noqa: E402


def odom_at(seconds: float) -> Odometry:
    msg = Odometry()
    msg.header.stamp.sec = int(seconds)
    msg.header.stamp.nanosec = int((seconds - int(seconds)) * 1_000_000_000)
    return msg


def test_fresh_header_and_recent_callback_are_accepted():
    assert odometry_is_fresh(
        odom_at(10.8),
        now_ns=11_000_000_000,
        receive_age=0.01,
        timeout=0.30,
        future_tolerance=0.10,
    )


def test_delayed_header_is_rejected_even_when_callback_just_arrived():
    assert not odometry_is_fresh(
        odom_at(5.0),
        now_ns=11_000_000_000,
        receive_age=0.01,
        timeout=0.30,
        future_tolerance=0.10,
    )


def test_future_header_and_old_callback_are_rejected():
    assert not odometry_is_fresh(
        odom_at(11.2),
        now_ns=11_000_000_000,
        receive_age=0.01,
        timeout=0.30,
        future_tolerance=0.10,
    )
    assert not odometry_is_fresh(
        odom_at(10.9),
        now_ns=11_000_000_000,
        receive_age=0.31,
        timeout=0.30,
        future_tolerance=0.10,
    )
