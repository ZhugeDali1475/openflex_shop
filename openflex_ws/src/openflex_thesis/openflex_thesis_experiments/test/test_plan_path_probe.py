from copy import deepcopy

import pytest
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import (
    AttachedCollisionObject, CollisionObject, ContactInformation, MoveItErrorCodes,
    PlanningScene, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint, MultiDOFJointTrajectoryPoint

from openflex_thesis_experiments.plan_path_probe import (
    SERVICES,
    collect_plan_evidence,
    main,
    positions_from_state,
    scene_geometry_hash,
    trajectory_states,
)


def full_state(position=0.0, is_diff=False):
    state = RobotState()
    state.is_diff = is_diff
    state.joint_state.name = ["lift_joint", "openarmx_right_joint1"]
    state.joint_state.position = [position, position + 1.0]
    return state


def trajectory(points):
    result = RobotTrajectory()
    result.joint_trajectory.joint_names = ["lift_joint", "openarmx_right_joint1"]
    result.joint_trajectory.points = points
    return result


def point(lift, arm, sec):
    result = JointTrajectoryPoint()
    result.positions = [lift, arm]
    result.time_from_start = Duration(sec=sec, nanosec=0)
    return result


def test_positions_from_state_requires_matching_unique_arrays():
    state = full_state()
    assert positions_from_state(state) == {
        "lift_joint": 0.0, "openarmx_right_joint1": 1.0
    }
    state.joint_state.position = []
    with pytest.raises(ValueError, match="mismatch"):
        positions_from_state(state)


def test_trajectory_states_merges_partial_trajectory_into_full_start():
    start = full_state()
    states = trajectory_states(start, trajectory([point(0.5, 1.5, 1)]))
    assert len(states) == 1
    assert positions_from_state(states[0]) == {
        "lift_joint": 0.5, "openarmx_right_joint1": 1.5
    }


@pytest.mark.parametrize("bad_start", [full_state(is_diff=True)])
def test_trajectory_states_rejects_non_full_start(bad_start):
    with pytest.raises(ValueError, match="diff"):
        trajectory_states(bad_start, trajectory([point(0.5, 1.5, 1)]))


def test_trajectory_states_rejects_non_monotonic_time_and_shape_errors():
    with pytest.raises(ValueError, match="strictly increasing"):
        trajectory_states(full_state(), trajectory([
            point(0.5, 1.5, 1), point(0.6, 1.6, 1),
        ]))
    malformed = trajectory([point(0.5, 1.5, 1)])
    malformed.joint_trajectory.points[0].positions = [0.5]
    with pytest.raises(ValueError, match="match joint names"):
        trajectory_states(full_state(), malformed)


def test_trajectory_states_rejects_multidof_and_unknown_joints():
    multi = trajectory([point(0.5, 1.5, 1)])
    multi.multi_dof_joint_trajectory.points.append(MultiDOFJointTrajectoryPoint())
    with pytest.raises(ValueError, match="multi-DOF"):
        trajectory_states(full_state(), multi)
    unknown = trajectory([point(0.5, 1.5, 1)])
    unknown.joint_trajectory.joint_names = ["lift_joint", "unknown_joint"]
    with pytest.raises(ValueError, match="unknown"):
        trajectory_states(full_state(), unknown)


@pytest.fixture
def service_fixture():
    scene = PlanningScene()
    scene.robot_state.joint_state.name = ["lift_joint"] + [
        f"openarmx_{side}_joint{index}" for side in ("left", "right") for index in range(1, 8)
    ]
    scene.robot_state.joint_state.position = [0.0] * 15
    scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
        link_name="openarmx_right_hand", object=CollisionObject(id="item"))]
    scene.world.collision_objects = [CollisionObject(id="shelf")]
    target = {name: 0.1 for name in scene.robot_state.joint_state.name
              if name == "lift_joint" or "right" in name}
    plan = GetMotionPlan.Response()
    plan.motion_plan_response.trajectory_start = deepcopy(scene.robot_state)
    plan.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
    planned = plan.motion_plan_response.trajectory.joint_trajectory
    planned.joint_names = list(target)
    planned.points = [
        JointTrajectoryPoint(positions=[value] * 8, time_from_start=Duration(sec=index))
        for index, value in enumerate([0.0, 0.05, 0.1])
    ]
    calls = []

    def call(kind, request):
        calls.append((kind, deepcopy(request)))
        if kind == "scene":
            return GetPlanningScene.Response(scene=deepcopy(scene))
        if kind == "plan":
            return deepcopy(plan)
        return GetStateValidity.Response(valid=True)

    return scene, target, plan, calls, call


def test_service_probe_uses_only_plan_and_read_services_and_retains_all_points(service_fixture):
    scene, target, plan, calls, call = service_fixture
    original = deepcopy(scene)
    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert report["status"] == "returned_discrete_points_checked"
    assert report["returned_point_count"] == 3
    assert report["path_audit"]["category"] == "valid"
    assert report["returned_start_matches_requested_joints"]
    assert report["discrete_diagnostic_passed"]
    assert not report["continuous_safety_proven"]
    assert not report["scene_atomicity_proven"]
    assert not report["scene_geometry_changed"]
    assert set(SERVICES) == {"scene", "plan", "validity"}
    planning_request = next(request for kind, request in calls if kind == "plan")
    assert isinstance(planning_request, GetMotionPlan.Request)
    assert planning_request.motion_plan_request.start_state == original.robot_state
    for kind, request in calls:
        if kind == "validity":
            assert request.group_name == ""
            assert not request.robot_state.is_diff
            assert request.robot_state.attached_collision_objects[0].object.id == "item"
            assert positions_from_state(request.robot_state)["openarmx_left_joint1"] == 0.0
    assert len(report["path_samples"]) == 3
    assert report["calls"][3]["response"]["motion_plan_response"]["trajectory"]
    assert scene == original


@pytest.mark.parametrize("invalid_call", [1, 2])
def test_invalid_endpoint_never_requests_planning(service_fixture, invalid_call):
    _, target, _, calls, base_call = service_fixture
    count = 0

    def call(kind, request):
        nonlocal count
        if kind == "validity":
            count += 1
            return GetStateValidity.Response(valid=count != invalid_call)
        return base_call(kind, request)

    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert report["status"] == "endpoint_rejected_no_planning_request"
    assert not report["discrete_diagnostic_passed"]
    assert all(kind != "plan" for kind, _ in calls)
    assert len(report["endpoint_checks"]) == 2


def test_missing_required_scene_prevents_planning(service_fixture):
    _, target, _, calls, call = service_fixture
    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["missing"])
    assert "required collision objects missing" in report["error"]
    assert all(kind == "scene" for kind, _ in calls)


def test_no_trajectory_keeps_planner_error_and_does_not_invent_points(service_fixture):
    _, target, plan, _, call = service_fixture
    plan.motion_plan_response.error_code.val = MoveItErrorCodes.PLANNING_FAILED
    plan.motion_plan_response.trajectory.joint_trajectory.points = []
    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert report["status"] == "no_returned_trajectory"
    assert not report["planning_succeeded"]
    assert report["path_samples"] == []
    assert "path_audit" not in report


def test_failed_planner_with_returned_path_is_audited_not_called_success(service_fixture):
    _, target, plan, _, base_call = service_fixture
    plan.motion_plan_response.error_code.val = MoveItErrorCodes.INVALID_MOTION_PLAN
    validity_count = 0

    def call(kind, request):
        nonlocal validity_count
        if kind == "validity":
            validity_count += 1
            if validity_count == 5:
                contact = ContactInformation(contact_body_1="shelf", contact_body_2="finger")
                return GetStateValidity.Response(valid=False, contacts=[contact])
        return base_call(kind, request)

    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert not report["planning_succeeded"]
    assert report["path_audit"]["category"] == "intermediate"
    assert report["path_audit"]["invalid_indices"] == [1]
    assert not report["discrete_diagnostic_passed"]
    assert report["path_samples"][1]["contacts"] == ["shelf<->finger"]


def test_validity_timeout_preserves_partial_unknown_sample_and_raw_request(service_fixture):
    _, target, _, _, base_call = service_fixture
    validity_count = 0

    def call(kind, request):
        nonlocal validity_count
        if kind == "validity":
            validity_count += 1
            if validity_count == 5:
                raise TimeoutError("test point response missing")
        return base_call(kind, request)

    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert report["status"] == "incomplete"
    assert "TimeoutError" in report["error"]
    assert report["path_samples"][1]["valid"] is None
    assert report["path_audit"]["unknown_indices"] == [1]
    assert not report["path_audit"]["complete_sample_evidence"]
    assert not report["path_audit"]["complete_returned_path_checked"]
    assert not report["discrete_diagnostic_passed"]
    assert any("error" in entry and entry["request"] for entry in report["calls"])


def test_scene_changes_are_flagged_and_both_snapshots_retained(service_fixture):
    scene, target, _, _, base_call = service_fixture
    count = 0

    def call(kind, request):
        nonlocal count
        if kind == "scene":
            count += 1
            if count == 2:
                scene.allowed_collision_matrix.default_entry_names = ["shelf"]
                scene.allowed_collision_matrix.default_entry_values = [True]
        return base_call(kind, request)

    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert report["scene_geometry_changed"]
    assert not report["discrete_diagnostic_passed"]
    assert report["scene_geometry_hash_before"] != report["scene_geometry_hash_after"]


def test_oversize_trajectory_keeps_raw_response_without_partial_checks(service_fixture):
    _, target, _, _, call = service_fixture
    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"], max_points=2)
    assert "audit limit" in report["error"]
    assert report["returned_point_count"] == 3
    assert report["path_samples"] == []
    assert next(entry for entry in report["calls"] if entry["service"] == "/plan_kinematic_path")[
        "response"] is not None


def test_scene_hash_preserves_attachment_and_acm_changes(service_fixture):
    scene, _, _, _, _ = service_fixture
    first = scene_geometry_hash(scene)
    scene.robot_state.joint_state.position[0] = 0.5
    assert scene_geometry_hash(scene) == first
    scene.robot_state.attached_collision_objects[0].object.id = "different"
    assert scene_geometry_hash(scene) != first


def test_missing_inactive_arm_joints_prevents_any_validity_checks(service_fixture):
    scene, target, _, calls, call = service_fixture
    scene.robot_state.joint_state.name = scene.robot_state.joint_state.name[7:]
    scene.robot_state.joint_state.position = [0.0] * len(scene.robot_state.joint_state.name)
    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert "both arms and lift" in report["error"]
    assert all(kind == "scene" for kind, _ in calls)


def test_cli_requires_localhost_before_ros_initialization(tmp_path, monkeypatch):
    monkeypatch.delenv("ROS_LOCALHOST_ONLY", raising=False)
    with pytest.raises(SystemExit):
        main(["--target", str(tmp_path / "not_read.json"), "--group", "right_arm_with_lift",
              "--required-object-id", "shelf", "--domain-id", "174", "--confirm-simulation",
              "--output", str(tmp_path / "not_created.json")])
    assert not (tmp_path / "not_created.json").exists()


@pytest.mark.parametrize("change", ["start", "first_point", "goal", "attachment", "joints"])
def test_trajectory_mismatches_cannot_pass_discrete_diagnostic(service_fixture, change):
    _, target, plan, _, call = service_fixture
    response = plan.motion_plan_response
    if change == "start":
        response.trajectory_start.joint_state.position[1] = 0.2
    elif change == "first_point":
        response.trajectory.joint_trajectory.points[0].positions[0] = 0.02
    elif change == "goal":
        response.trajectory.joint_trajectory.points[-1].positions[0] = 0.2
    elif change == "attachment":
        response.trajectory_start.attached_collision_objects = []
    else:
        response.trajectory.joint_trajectory.joint_names[-1] = "openarmx_left_joint1"
    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert not report["discrete_diagnostic_passed"]


def test_final_scene_readback_failure_keeps_checks_but_cannot_pass(service_fixture):
    _, target, _, _, base_call = service_fixture
    count = 0

    def call(kind, request):
        nonlocal count
        if kind == "scene":
            count += 1
            if count == 2:
                raise TimeoutError("scene unavailable after checking")
        return base_call(kind, request)

    report = collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert report["path_audit"]["category"] == "valid"
    assert report["scene_readback_after_error"]
    assert not report["discrete_diagnostic_passed"]


def test_probe_rejects_invalid_target_before_calling_services(service_fixture):
    _, target, _, calls, call = service_fixture
    target.pop("lift_joint")
    with pytest.raises(ValueError, match="exactly"):
        collect_plan_evidence(call, target, "right_arm_with_lift", ["shelf"])
    assert not calls
