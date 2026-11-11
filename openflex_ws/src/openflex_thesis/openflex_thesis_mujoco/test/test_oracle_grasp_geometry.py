"""Production oracle and jaw-gate contracts, without nodes or task execution."""

import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from openflex_thesis_msgs.msg import AnyGraspRequest, GraspCandidate
import pytest

from openflex_thesis_mujoco.bridge_node import MujocoBridgeNode
from openflex_thesis_mujoco.math_utils import relative_pose, rotate_vector
from openflex_thesis_mujoco.oracle_anygrasp_node import OracleAnyGraspNode


def generated_grasps(offsets, yaw=math.pi, stage=GraspCandidate.STAGE_REFINED,
                     top_k=3):
    parameters = {
        "coarse_lateral_spread_m": 0.025,
        "refined_lateral_spread_m": 0.010,
        "pregrasp_distance_m": 0.12,
        "refined_candidate_offsets_m": offsets,
    }
    published = []
    stamp = Time(sec=12, nanosec=34)
    oracle = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: stamp)
        ),
        yaw_from_quaternion=OracleAnyGraspNode.yaw_from_quaternion,
        publisher=SimpleNamespace(publish=published.append),
    )
    request = AnyGraspRequest()
    request.request_id = "offline-geometry"
    request.stage = stage
    request.top_k = top_k
    request.target.header.frame_id = "odom"
    request.target.object_id = "thesis_item_01"
    request.target.map_version = 1
    request.target.observation_sequence = 7
    position = request.target.pose.pose.position
    position.x, position.y, position.z = 1.12, 0.0, 0.84
    orientation = request.target.pose.pose.orientation
    orientation.z, orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    OracleAnyGraspNode.on_request(oracle, request)
    assert len(published) == 1
    return request, published[0]


def position_tuple(position):
    return (position.x, position.y, position.z)


def quaternion_tuple(orientation):
    return (orientation.x, orientation.y, orientation.z, orientation.w)


@pytest.mark.parametrize("offsets,laterals", [
    ([0.0], [-0.01, 0.0, 0.01]),
    ([0.0, 0.15, 0.0], [-0.01, 0.15, 0.01]),
])
def test_refined_offsets_preserve_id_quality_and_metadata(offsets, laterals):
    request, output = generated_grasps(offsets)
    assert output.request_id == request.request_id
    assert output.object_id == request.target.object_id
    assert output.map_version == 1
    assert output.observation_sequence == 7
    assert output.header.frame_id == "odom"
    assert output.header.stamp == Time(sec=12, nanosec=34)
    assert output.stage == GraspCandidate.STAGE_REFINED
    assert not output.physical_perception
    assert [grasp.grasp_id for grasp in output.grasps] == [200100, 200101, 200102]
    assert [grasp.quality for grasp in output.grasps] == pytest.approx([.92, .98, .92])
    assert max(output.grasps, key=lambda grasp: grasp.quality).grasp_id == 200101
    for grasp, lateral, height in zip(output.grasps, laterals, [-.006, 0., .006]):
        delta = tuple(
            target - origin for target, origin in zip(
                position_tuple(grasp.grasp_pose.pose.position),
                position_tuple(request.target.pose.pose.position),
            )
        )
        assert delta == pytest.approx((0.0, -lateral, height), abs=1e-12)
        assert grasp.grasp_pose.header.frame_id == "odom"
        assert grasp.pregrasp_pose.header.frame_id == "odom"
        assert grasp.observation_sequence == 7
        assert not grasp.physical_perception
        assert grasp.source == "mujoco_ground_truth_oracle_NOT_PHYSICAL"


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 2, 0.37])
def test_pregrasp_and_tcp_positive_z_share_inward_approach(yaw):
    _, output = generated_grasps([0.0, 0.15, 0.0], yaw=yaw)
    inward = (-math.cos(yaw), -math.sin(yaw), 0.0)
    for grasp in output.grasps:
        final = grasp.grasp_pose.pose
        pregrasp = grasp.pregrasp_pose.pose
        quaternion = quaternion_tuple(final.orientation)
        assert sum(value ** 2 for value in quaternion) == pytest.approx(1.0)
        assert rotate_vector(quaternion, (0., 0., 1.)) == pytest.approx(inward)
        assert position_tuple(grasp.approach) == pytest.approx(inward)
        displacement = tuple(
            target - start for target, start in zip(
                position_tuple(final.position), position_tuple(pregrasp.position)
            )
        )
        assert displacement == pytest.approx(tuple(.12 * value for value in inward))
        assert quaternion_tuple(pregrasp.orientation) == pytest.approx(quaternion)


@pytest.mark.parametrize("yaw", [0.0, math.pi])
@pytest.mark.parametrize("offsets,rejected", [
    ([0.0], False), ([0.0, 0.15, 0.0], True),
])
def test_ideal_center_candidate_uses_production_relative_pose_and_jaw_gate(
        yaw, offsets, rejected):
    request, output = generated_grasps(offsets, yaw=yaw)
    grasp_pose = output.grasps[1].grasp_pose.pose
    object_pose = request.target.pose.pose
    offset, _ = relative_pose(
        position_tuple(grasp_pose.position), quaternion_tuple(grasp_pose.orientation),
        position_tuple(object_pose.position), quaternion_tuple(object_pose.orientation),
    )
    expected_y = (0.15 if yaw == math.pi else -0.15) if rejected else 0.0
    assert offset == pytest.approx((0.0, expected_y, 0.0), abs=1e-12)
    bridge = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value=0.020)
    )
    reason = MujocoBridgeNode.attach_rejection(bridge, offset, (-.069, .016, .0855 / 2))
    if rejected:
        assert "outside the jaw span" in reason
    else:
        assert reason == ""


def test_coarse_candidates_do_not_receive_refined_fault_injection():
    request, output = generated_grasps(
        [0.0, 0.15, 0.0], stage=GraspCandidate.STAGE_COARSE
    )
    assert [grasp.grasp_id for grasp in output.grasps] == [100100, 100101, 100102]
    object_y = request.target.pose.pose.position.y
    assert [grasp.grasp_pose.pose.position.y - object_y for grasp in output.grasps] == (
        pytest.approx([.025, 0., -.025])
    )


def test_zero_top_k_still_generates_one_candidate():
    _, output = generated_grasps([0.0], top_k=0)
    assert len(output.grasps) == 1
    assert output.grasps[0].grasp_id == 200100
