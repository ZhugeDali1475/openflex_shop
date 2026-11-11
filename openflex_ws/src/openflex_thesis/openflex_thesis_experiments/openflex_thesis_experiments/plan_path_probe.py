"""Simulation-only MoveIt service probe; no execution or scene-write clients."""

import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import time

from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, PlanningSceneComponents
from moveit_msgs.srv import GetMotionPlan, GetPlanningScene, GetStateValidity
from rosidl_runtime_py.convert import message_to_ordereddict

from .path_audit import audit_discrete_path, joint_positions


SERVICES = {
    "scene": (GetPlanningScene, "/get_planning_scene"),
    "plan": (GetMotionPlan, "/plan_kinematic_path"),
    "validity": (GetStateValidity, "/check_state_validity"),
}


def positions_from_state(state):
    names, positions = list(state.joint_state.name), list(state.joint_state.position)
    if len(names) != len(positions) or len(set(names)) != len(names):
        raise ValueError("robot state joint names/positions mismatch or duplicate names")
    return joint_positions(dict(zip(names, positions)))


def scene_geometry_hash(scene):
    payload = message_to_ordereddict(scene)
    payload["robot_state"].pop("joint_state", None)
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()


def trajectory_states(start, trajectory):
    if trajectory.multi_dof_joint_trajectory.points:
        raise ValueError("multi-DOF trajectories are unsupported; refusing partial audit")
    if start.is_diff:
        raise ValueError("returned trajectory_start is a diff, not a full starting state")
    full = positions_from_state(start)
    names = list(trajectory.joint_trajectory.joint_names)
    if not names or len(set(names)) != len(names) or not set(names) <= set(full):
        raise ValueError("trajectory joint names duplicate, absent or unknown in returned start")
    states = []
    previous_ns = -1
    for point in trajectory.joint_trajectory.points:
        if len(point.positions) != len(names):
            raise ValueError("trajectory positions do not match joint names")
        stamp = point.time_from_start
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000 or stamp_ns <= previous_ns:
            raise ValueError("trajectory times must be non-negative and strictly increasing")
        previous_ns = stamp_ns
        positions = joint_positions(dict(zip(names, point.positions)))
        state = deepcopy(start)
        merged = {**full, **positions}
        state.joint_state.position = [merged[name] for name in state.joint_state.name]
        state.joint_state.velocity = []
        state.joint_state.effort = []
        states.append(state)
    return states


def collect_plan_evidence(call, target, group, required_objects, max_points=1000):
    """Use only read/plan services and retain every service request and response."""
    target = joint_positions(target)
    if group not in ("left_arm_with_lift", "right_arm_with_lift"):
        raise ValueError("only explicit lift+arm planning groups are supported")
    arm = group.split("_", 1)[0]
    expected = {"lift_joint"} | {f"openarmx_{arm}_joint{index}" for index in range(1, 8)}
    if set(target) != expected:
        raise ValueError("target must contain exactly lift_joint and seven active-arm joints")
    if not required_objects or any(
        not isinstance(name, str) or not name for name in required_objects
    ):
        raise ValueError("expected scene object IDs must be explicit and non-empty")
    if type(max_points) is not int or max_points < 1:
        raise ValueError("max_points must be positive")
    report = {
        "schema_version": 1,
        "analysis_type": "simulation_plan_only_service_probe_NOT_A_TASK_TRIAL",
        "status": "incomplete", "calls": [], "endpoint_checks": [], "path_samples": [],
        "planning_group": group, "target_positions": target,
        "required_scene_objects": list(required_objects),
        "continuous_safety_proven": False, "scene_atomicity_proven": False,
        "limitations": [
            "No execution clients exist in this tool; other processes may still move the robot.",
            "Sequential validity queries use the live scene, not an atomic frozen scene.",
            "Equal before/after hashes do not exclude transient scene changes.",
            "Discrete returned points are not continuous collision safety evidence.",
            "A new plan cannot recover historical seed 6301 trajectory point 61.",
            "Scene IDs alone do not prove calibrated geometry or complete robot collision meshes.",
        ],
    }

    def query(kind, request):
        entry = {"service": SERVICES[kind][1], "request": message_to_ordereddict(request),
                 "started_monotonic_sec": time.monotonic(), "response": None}
        report["calls"].append(entry)
        try:
            response = call(kind, request)
            if response is None:
                raise RuntimeError(f"{kind} service returned no response")
            entry["response"] = message_to_ordereddict(response)
            return response
        except Exception as error:
            entry["error"] = str(error)
            raise
        finally:
            entry["finished_monotonic_sec"] = time.monotonic()

    def check(state, label, index=None):
        request = GetStateValidity.Request()
        request.group_name = ""
        request.robot_state = deepcopy(state)
        record = {"label": label, "index": index, "positions": positions_from_state(state),
                  "valid": None, "contacts": []}
        if index is None:
            report["endpoint_checks"].append(record)
        else:
            report["path_samples"].append(record)
        response = query("validity", request)
        record["valid"] = bool(response.valid)
        record["contacts"] = [
            f"{contact.contact_body_1}<->{contact.contact_body_2}"
            for contact in response.contacts
        ]
        return response.valid

    scene_request = GetPlanningScene.Request()
    scene_request.components.components = (
        PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.WORLD_OBJECT_NAMES | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        | PlanningSceneComponents.OCTOMAP | PlanningSceneComponents.TRANSFORMS
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        | PlanningSceneComponents.LINK_PADDING_AND_SCALING | PlanningSceneComponents.OBJECT_COLORS
    )
    before = None
    try:
        before = query("scene", scene_request).scene
        observed = {item.id for item in before.world.collision_objects}
        if not set(required_objects) <= observed:
            raise ValueError("required collision objects missing from scene readback")
        start = deepcopy(before.robot_state)
        if start.is_diff:
            raise ValueError("scene robot state is not a full state")
        full = positions_from_state(start)
        both_arms = {f"openarmx_{side}_joint{index}"
                     for side in ("left", "right") for index in range(1, 8)}
        if not (both_arms | {"lift_joint"}) <= set(full):
            raise ValueError("both arms and lift joints must be present in the scene state")
        goal_state = deepcopy(start)
        merged = {**full, **target}
        goal_state.joint_state.position = [merged[name] for name in goal_state.joint_state.name]
        goal_state.joint_state.velocity = []
        goal_state.joint_state.effort = []
        current_valid = check(start, "current")
        target_valid = check(goal_state, "target")
        if not current_valid or not target_valid:
            report["status"] = "endpoint_rejected_no_planning_request"
        else:
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name = group
            motion.start_state = start
            motion.allowed_planning_time = 5.0
            motion.num_planning_attempts = 1
            motion.max_velocity_scaling_factor = 0.1
            motion.max_acceleration_scaling_factor = 0.1
            constraints = Constraints(name="diagnostic_park_target")
            for name, position in target.items():
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=name, position=position, tolerance_above=0.01,
                    tolerance_below=0.01, weight=1.0))
            motion.goal_constraints = [constraints]
            response = query("plan", request).motion_plan_response
            report["planning_error_code"] = response.error_code.val
            report["planning_succeeded"] = response.error_code.val == MoveItErrorCodes.SUCCESS
            point_count = len(response.trajectory.joint_trajectory.points)
            report["returned_point_count"] = point_count
            if not point_count:
                report["status"] = "no_returned_trajectory"
            elif point_count > max_points:
                raise ValueError("trajectory exceeds point audit limit; raw response retained")
            else:
                states = trajectory_states(response.trajectory_start, response.trajectory)
                if set(response.trajectory.joint_trajectory.joint_names) != expected:
                    raise ValueError(
                        "returned trajectory must cover exactly the active arm and lift")
                returned_full = positions_from_state(response.trajectory_start)
                if not (both_arms | {"lift_joint"}) <= set(returned_full):
                    raise ValueError("returned trajectory start omits robot joints")
                report["returned_start_matches_requested_joints"] = (
                    set(returned_full) == set(full) and all(
                        abs(returned_full[name] - value) <= 1e-9
                        for name, value in full.items()))
                report["returned_start_preserves_attachments"] = (
                    response.trajectory_start.attached_collision_objects
                    == start.attached_collision_objects)
                report["returned_start_preserves_multi_dof_state"] = (
                    response.trajectory_start.multi_dof_joint_state == start.multi_dof_joint_state)
                first_positions = positions_from_state(states[0])
                last_positions = positions_from_state(states[-1])
                report["first_point_matches_returned_start"] = all(
                    abs(first_positions[name] - value) <= 1e-9
                    for name, value in returned_full.items())
                report["last_point_within_goal_tolerance"] = all(
                    abs(last_positions[name] - value) <= 0.01 + 1e-12
                    for name, value in target.items())
                check(response.trajectory_start, "returned_trajectory_start")
                for index, state in enumerate(states):
                    check(state, "trajectory_point", index)
                report["path_audit"] = audit_discrete_path(report["path_samples"])
                report["status"] = "returned_discrete_points_checked"
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        if report["path_samples"]:
            report["path_audit"] = audit_discrete_path(report["path_samples"])
            report["path_audit"]["complete_returned_path_checked"] = (
                len(report["path_samples"]) == report.get("returned_point_count")
                and report["path_audit"]["complete_sample_evidence"])
        if before is not None:
            report["scene_geometry_hash_before"] = scene_geometry_hash(before)
            try:
                after = query("scene", scene_request).scene
                report["scene_geometry_hash_after"] = scene_geometry_hash(after)
                report["scene_geometry_changed"] = (
                    report["scene_geometry_hash_before"] != report["scene_geometry_hash_after"])
            except Exception as error:
                report["scene_readback_after_error"] = str(error)
    report["discrete_diagnostic_passed"] = bool(
        report["status"] == "returned_discrete_points_checked"
        and report.get("planning_succeeded")
        and report.get("returned_start_matches_requested_joints")
        and report.get("returned_start_preserves_attachments")
        and report.get("returned_start_preserves_multi_dof_state")
        and report.get("first_point_matches_returned_start")
        and report.get("last_point_within_goal_tolerance")
        and report.get("path_audit", {}).get("complete_returned_path_checked")
        and report.get("path_audit", {}).get("category") == "valid"
        and all(record["valid"] is True for record in report["endpoint_checks"])
        and report.get("scene_geometry_changed") is False
        and "scene_readback_after_error" not in report
        and "error" not in report
    )
    return report


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=Path, required=True,
                        help="JSON object of 8 joint positions")
    parser.add_argument("--group", choices=["left_arm_with_lift", "right_arm_with_lift"],
                        required=True)
    parser.add_argument("--required-object-id", action="append", required=True)
    parser.add_argument("--domain-id", type=int, required=True)
    parser.add_argument("--confirm-simulation", action="store_true", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=60.0)
    options = parser.parse_args(args)
    if os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        parser.error("ROS_LOCALHOST_ONLY=1 is required; this does not certify server identity")
    if not 1 <= options.domain_id <= 232:
        parser.error("use an explicitly isolated simulation domain in [1,232]")
    if not math.isfinite(options.timeout_sec) or not 0 < options.timeout_sec <= 300:
        parser.error("timeout must be finite and in (0,300] seconds")
    raw = options.target.read_bytes()
    target = json.loads(raw)
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node

    with options.output.open("x") as stream:
        context = Context()
        node = None
        report = {"status": "initialization_failed"}
        try:
            rclpy.init(context=context, domain_id=options.domain_id)
            node = Node("simulation_plan_path_probe", context=context,
                        enable_rosout=False, start_parameter_services=False)
            clients = {key: node.create_client(kind, name)
                       for key, (kind, name) in SERVICES.items()}
            deadline = time.monotonic() + options.timeout_sec

            def call(kind, request):
                remaining = min(10.0, deadline - time.monotonic())
                if remaining <= 0 or not clients[kind].wait_for_service(timeout_sec=remaining):
                    raise TimeoutError(f"{kind} service unavailable or probe deadline exceeded")
                remaining = min(10.0, deadline - time.monotonic())
                if remaining <= 0:
                    raise TimeoutError("probe deadline exceeded")
                future = clients[kind].call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=remaining)
                if not future.done():
                    future.cancel()
                    raise TimeoutError(f"{kind} response exceeded time bound")
                return future.result()

            report = collect_plan_evidence(call, target, options.group, options.required_object_id)
        except Exception as error:
            report["error"] = f"{type(error).__name__}: {error}"
        finally:
            report["target_sha256"] = hashlib.sha256(raw).hexdigest()
            report["probe_source_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            report["domain_id"] = options.domain_id
            report["server_identity_verified"] = False
            try:
                encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write(encoded + "\n")
            finally:
                if node is not None:
                    node.destroy_node()
                if context.ok():
                    rclpy.shutdown(context=context)
    if not report.get("discrete_diagnostic_passed", False):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
