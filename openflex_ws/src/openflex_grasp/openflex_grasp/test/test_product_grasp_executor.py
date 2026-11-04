import copy
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import PoseStamped

from openflex_grasp.product_grasp_executor import (
    MotionStateUncertainError,
    ProductGraspExecutorNode,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    bounded_corridor_max_path_length,
    duplicate_grasp_pipeline_nodes,
    gripper_goal_accepted,
    joint_posture_spec,
    ordered_joint_positions,
    pose_distance,
    project_pose_onto_line,
    require_arm_only_stage,
    require_pregrasp_stage,
    require_path_fraction,
    resolve_pregrasp_max_dof,
    robot_gripper_commands,
    single_finger_position,
    translated_pose,
    reduced_basket_descent_delta,
    vertical_lift_feed_target,
)


def test_single_finger_position_uses_half_total_opening():
    assert single_finger_position(0.072) == pytest.approx(0.036)
    assert single_finger_position(0.088) == pytest.approx(0.044)


def test_single_finger_position_uses_robot_calibration():
    assert single_finger_position(
        0.04,
        joint_max_position=0.04,
        opening_scale=1.0,
    ) == pytest.approx(0.04)


def test_robot_gripper_uses_full_open_and_physical_zero_close():
    open_position, close_position, candidate_position, grasp_width = (
        robot_gripper_commands(0.084)
    )
    assert open_position == pytest.approx(0.044)
    assert close_position == pytest.approx(0.0)
    assert candidate_position == pytest.approx(0.042)
    assert grasp_width == pytest.approx(0.080)


def test_robot_gripper_retains_optional_width_derived_close_mode():
    _, close_position, _, _ = robot_gripper_commands(
        0.084,
        configured_close_position=-1.0,
    )
    assert close_position == pytest.approx(0.0385)


def test_robot_gripper_clamps_oversized_estimate_to_full_open():
    open_position, close_position, candidate_position, grasp_width = (
        robot_gripper_commands(
            0.111,
            joint_max_position=0.043,
            opening_scale=2.0,
        )
    )
    assert open_position == pytest.approx(0.043)
    assert candidate_position == pytest.approx(0.043)
    assert close_position == pytest.approx(0.0)
    assert grasp_width == pytest.approx(0.107)


def test_open_gripper_accepts_nearby_mechanical_limit_stall():
    assert gripper_goal_accepted(
        0.043,
        0.0429,
        reached_goal=False,
        stalled=True,
        accept_stalled=True,
        stalled_tolerance=0.002,
    )
    assert not gripper_goal_accepted(
        0.043,
        0.038,
        reached_goal=False,
        stalled=True,
        accept_stalled=True,
        stalled_tolerance=0.002,
    )


@pytest.mark.parametrize("width", [-0.001, 0.089, float("nan")])
def test_single_finger_position_rejects_invalid_width(width):
    with pytest.raises(ValueError):
        single_finger_position(width)


def test_pose_distance():
    first = PoseStamped()
    second = PoseStamped()
    second.pose.position.x = 0.12
    assert pose_distance(first, second) == pytest.approx(0.12)


def test_translated_pose_preserves_orientation_and_offsets_position():
    source = PoseStamped()
    source.header.frame_id = "odom"
    source.pose.position.x = 0.4
    source.pose.position.y = -0.2
    source.pose.position.z = 0.7
    source.pose.orientation.w = 1.0
    translated = translated_pose(source, x=0.1, y=0.02, z=0.05)
    assert translated.header.frame_id == "odom"
    assert translated.pose.position.x == pytest.approx(0.5)
    assert translated.pose.position.y == pytest.approx(-0.18)
    assert translated.pose.position.z == pytest.approx(0.75)
    assert translated.pose.orientation.w == pytest.approx(1.0)
    assert source.pose.position.z == pytest.approx(0.7)


def test_held_object_return_lifts_then_feeds_pregrasp_then_photo():
    class Harness:
        def __init__(self):
            self.current = PoseStamped()
            self.current.header.frame_id = "odom"
            self.current.pose.position.x = 0.6
            self.current.pose.position.y = -0.1
            self.current.pose.position.z = 0.8
            self.current.pose.orientation.w = 1.0
            self.executions = []
            self.postures = []

        def _lookup_current_tcp_pose(self):
            return copy.deepcopy(self.current)

        def _publish(self, *_args, **_kwargs):
            pass

        def _bool_param(self, name):
            return name == "unconstrained_moveit"

        def _checked_cartesian(
            self, start, end, *, execute, label, path_mode, end_joint_positions=None
        ):
            if execute:
                self.executions.append((label, copy.deepcopy(start), copy.deepcopy(end)))
                self.current = copy.deepcopy(end)
            return 1.0

        def _checked_joint_posture(
            self, arm_positions, *, lift_position, execute, label
        ):
            assert execute
            self.postures.append((label, tuple(arm_positions), lift_position))

    photo = PoseStamped()
    photo.header.frame_id = "odom"
    photo.pose.position.x = 0.3
    photo.pose.position.y = 0.2
    photo.pose.position.z = 0.7
    photo.pose.orientation.w = 1.0
    pregrasp = PoseStamped()
    pregrasp.header.frame_id = "odom"
    pregrasp.pose.position.x = 0.45
    pregrasp.pose.position.y = 0.05
    pregrasp.pose.position.z = 0.78
    pregrasp.pose.orientation.w = 1.0
    harness = Harness()

    ProductGraspExecutorNode._return_held_object_to_photo_pose(
        harness,
        photo,
        pregrasp,
        (0.0,) * 7,
        0.05,
        photo_lift_position=None,
        label_prefix="test",
    )

    assert len(harness.executions) == 2
    lift_end = harness.executions[0][2]
    pregrasp_end = harness.executions[1][2]
    assert lift_end.pose.position.x == pytest.approx(0.6)
    assert lift_end.pose.position.y == pytest.approx(-0.1)
    assert lift_end.pose.position.z == pytest.approx(0.85)
    assert pregrasp_end.pose.position.x == pytest.approx(0.45)
    assert pregrasp_end.pose.position.y == pytest.approx(0.05)
    assert pregrasp_end.pose.position.z == pytest.approx(0.78)
    assert harness.postures == [
        ("test pregrasp-to-photo execution", (0.0,) * 7, None)
    ]


def test_orders_right_arm_joint_positions_by_name():
    names = ["unused", "openarmx_right_joint2", "openarmx_right_joint1"] + [
        f"openarmx_right_joint{index}" for index in range(3, 8)
    ]
    positions = [99.0, 2.0, 1.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    assert ordered_joint_positions(names, positions) == pytest.approx(
        (1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0)
    )


def test_rejects_incomplete_right_arm_joint_positions():
    with pytest.raises(ValueError, match="joint7"):
        ordered_joint_positions(
            [f"openarmx_right_joint{index}" for index in range(1, 7)],
            [0.0] * 6,
        )


def test_projects_tracking_error_onto_straight_grasp_line():
    pregrasp = PoseStamped()
    grasp = PoseStamped()
    grasp.pose.position.x = 0.04
    current = PoseStamped()
    current.pose.position.x = 0.003
    current.pose.position.y = 0.0175
    projected, lateral = project_pose_onto_line(current, pregrasp, grasp)
    assert projected.pose.position.x == pytest.approx(0.003)
    assert projected.pose.position.y == pytest.approx(0.0)
    assert lateral == pytest.approx(0.0175)


def test_cartesian_fraction_must_meet_client_threshold():
    assert require_path_fraction(0.99, 0.99, "approach") == pytest.approx(0.99)
    with pytest.raises(RuntimeError, match="below required"):
        require_path_fraction(0.4167, 0.99, "approach")


def test_bounded_corridor_limits_path_length():
    assert bounded_corridor_max_path_length(0.20) == pytest.approx(0.50)
    assert bounded_corridor_max_path_length(0.04) == pytest.approx(0.14)
    with pytest.raises(ValueError, match="limits are invalid"):
        bounded_corridor_max_path_length(0.0)


def test_vertical_lift_feed_uses_tcp_height_delta_and_speed():
    target, duration = vertical_lift_feed_target(
        0.128,
        -0.135,
        speed_m_s=0.04,
    )
    assert target == pytest.approx(-0.007)
    assert duration == pytest.approx(3.375)


def test_vertical_lift_feed_rejects_joint_limit_violation():
    with pytest.raises(ValueError, match="outside"):
        vertical_lift_feed_target(-0.60, -0.10)


def test_basket_final_descent_uses_full_nominal_distance_without_reduction():
    assert reduced_basket_descent_delta(-0.095, 0.0) == pytest.approx(-0.095)


def test_basket_final_descent_reduction_never_reverses_upward():
    assert reduced_basket_descent_delta(-0.095, 0.10) == pytest.approx(0.0)


def test_detects_duplicate_grasp_pipeline_nodes_by_full_name():
    duplicates = duplicate_grasp_pipeline_nodes([
        ("product_pointcloud", "/"),
        ("product_pointcloud", "/"),
        ("straight_grasp", "/"),
        ("right_product_grasp_executor", "/"),
        ("unrelated", "/"),
        ("unrelated", "/"),
    ])
    assert duplicates == ("/product_pointcloud",)


def test_only_stage_a_is_accepted():
    require_arm_only_stage(0, "right arm")
    with pytest.raises(RuntimeError, match="only right-arm STAGE_A"):
        require_arm_only_stage(1, "right arm")


def test_pregrasp_dof_auto_uses_lift_only_for_basket():
    assert resolve_pregrasp_max_dof("auto", "shelf") == 7
    assert resolve_pregrasp_max_dof("auto", "basket") == 8
    assert resolve_pregrasp_max_dof("7", "basket") == 7
    with pytest.raises(ValueError, match="base motion is forbidden"):
        resolve_pregrasp_max_dof("11", "basket")


def test_arm_with_lift_accepts_stage_b_but_rejects_stage_c():
    require_pregrasp_stage(0, 8, "basket pregrasp")
    require_pregrasp_stage(1, 8, "basket pregrasp")
    with pytest.raises(RuntimeError, match="forbidden MoveIt stage"):
        require_pregrasp_stage(2, 8, "basket pregrasp")


def test_arm_with_lift_goal_keeps_base_fixed():
    class Parameter:
        def __init__(self, value):
            self.value = value

    class Harness:
        def __init__(self):
            self._pregrasp_client = object()
            self.goal = None

        def get_parameter(self, name):
            return Parameter({
                "arm": "right",
                "cartesian_velocity_scale": 0.04,
                "cartesian_acceleration_scale": 0.04,
                "corridor_planning_timeout_s": 8.0,
                "pregrasp_planning_attempts": 1,
            }[name])

        def _send_action(self, client, goal, label):
            assert client is self._pregrasp_client
            assert label == "basket pregrasp"
            self.goal = goal
            return SimpleNamespace(
                error_code=SimpleNamespace(val=1),
                message="ok",
                selected_stage=1,
            )

        def _require_moveit_success(self, result, label):
            assert result.error_code.val == 1
            assert label == "basket pregrasp"

    target = PoseStamped()
    target.header.frame_id = "odom"
    harness = Harness()
    ProductGraspExecutorNode._checked_pregrasp(
        harness,
        target,
        max_dof=8,
        execute=False,
        label="basket pregrasp",
    )
    assert harness.goal.max_dof == 8
    assert harness.goal.base_yaw_mode == harness.goal.CURRENT_STATE
    assert harness.goal.base_yaw_samples == 0
    assert harness.goal.execute is False


@pytest.mark.parametrize("arm,joint_names", [
    ("left", LEFT_ARM_JOINTS),
    ("right", RIGHT_ARM_JOINTS),
])
@pytest.mark.parametrize("lift_position", [None, 0.42])
def test_joint_posture_spec_supports_both_arms_and_optional_lift(
    arm, joint_names, lift_position
):
    group, names, positions = joint_posture_spec(
        arm, range(7), lift_position=lift_position
    )
    if lift_position is None:
        assert group == f"{arm}_arm"
        assert names == joint_names
        assert positions == pytest.approx(tuple(range(7)))
    else:
        assert group == f"{arm}_arm_with_lift"
        assert names == ("lift_joint", *joint_names)
        assert positions == pytest.approx((lift_position, *range(7)))


def test_exact_photo_return_uses_joint_posture_action():
    class Parameter:
        def __init__(self, value):
            self.value = value

    class Harness:
        def __init__(self):
            self._joint_posture_client = object()
            self.goal = None

        def get_parameter(self, name):
            return Parameter({
                "arm": "right",
                "cartesian_velocity_scale": 0.04,
                "cartesian_acceleration_scale": 0.04,
                "corridor_planning_timeout_s": 8.0,
                "pregrasp_planning_attempts": 1,
            }[name])

        def _send_action(self, client, goal, _label):
            assert client is self._joint_posture_client
            self.goal = goal
            return SimpleNamespace(error_code=SimpleNamespace(val=1), message="ok")

        def _require_moveit_success(self, _result, _label):
            pass

    arm_positions = tuple(0.1 * index for index in range(7))
    harness = Harness()
    ProductGraspExecutorNode._checked_joint_posture(
        harness,
        arm_positions,
        lift_position=0.42,
        execute=True,
        label="strict photo return",
    )

    assert harness.goal.planning_group == "right_arm_with_lift"
    assert harness.goal.joint_goal.name == ["lift_joint", *RIGHT_ARM_JOINTS]
    assert harness.goal.joint_goal.position == pytest.approx(
        [0.42, *arm_positions]
    )


def test_pregrasp_retries_once_after_no_ik(monkeypatch):
    class Parameter:
        def __init__(self, value):
            self.value = value

    class Logger:
        def __init__(self):
            self.warnings = []

        def warning(self, message):
            self.warnings.append(message)

    class Harness:
        def __init__(self):
            self._pregrasp_client = object()
            self.calls = 0
            self.logger = Logger()

        def get_parameter(self, name):
            return Parameter({
                "arm": "right",
                "cartesian_velocity_scale": 0.04,
                "cartesian_acceleration_scale": 0.04,
                "corridor_planning_timeout_s": 8.0,
                "pregrasp_planning_attempts": 2,
            }[name])

        def _send_action(self, _client, _goal, _label):
            self.calls += 1
            return SimpleNamespace(
                error_code=SimpleNamespace(val=-31 if self.calls == 1 else 1),
                message="no IK" if self.calls == 1 else "ok",
                selected_stage=1,
            )

        def _require_moveit_success(self, result, _label):
            assert result.error_code.val == 1

        def get_logger(self):
            return self.logger

    monkeypatch.setattr(
        "openflex_grasp.product_grasp_executor.time.sleep", lambda _seconds: None
    )
    harness = Harness()
    ProductGraspExecutorNode._checked_pregrasp(
        harness,
        PoseStamped(),
        max_dof=8,
        execute=False,
        label="basket pregrasp",
    )
    assert harness.calls == 2
    assert len(harness.logger.warnings) == 1


def test_joint_goal_tolerance_failure_marks_motion_state_uncertain():
    result = SimpleNamespace(
        error_code=SimpleNamespace(val=-5),
        message="joint target did not settle",
    )
    with pytest.raises(MotionStateUncertainError, match="did not settle"):
        ProductGraspExecutorNode._require_moveit_success(result, "Cartesian segment")
