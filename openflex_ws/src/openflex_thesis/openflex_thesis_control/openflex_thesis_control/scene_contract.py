"""Box geometry and readback checks for the execution planning scene."""

from copy import deepcopy
import math

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from .attached_object import quaternion_multiply, rotate


def normalized_quaternion(values):
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must contain four finite components")
    norm = math.hypot(*values)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion must have a finite positive norm")
    return tuple(value / norm for value in values)


def shelf_collision_objects(scene, transform, base_frame):
    specifications = scene.get("collision_objects", [])
    if not scene.get("frame_id") or not specifications:
        raise ValueError("shelf scene needs a frame and collision objects")
    rotation = transform.transform.rotation
    translation = transform.transform.translation
    if not all(math.isfinite(value) for value in (translation.x, translation.y, translation.z)):
        raise ValueError("shelf transform must have a finite translation")
    quaternion = normalized_quaternion((rotation.x, rotation.y, rotation.z, rotation.w))
    objects = []
    identifiers = set()
    for specification in specifications:
        identifier = specification.get("id", "")
        dimensions = list(map(float, specification.get("dimensions", [])))
        position = list(map(float, specification.get("position", [])))
        orientation = list(map(float, specification.get("orientation_xyzw", [])))
        if (not identifier or identifier in identifiers or specification.get("shape") != "box"
                or len(dimensions) != 3 or len(position) != 3 or len(orientation) != 4
                or not all(math.isfinite(value) for value in dimensions + position + orientation)
                or any(value <= 0 for value in dimensions)):
            raise ValueError(f"invalid or duplicate shelf box: {identifier}")
        identifiers.add(identifier)
        orientation = normalized_quaternion(orientation)
        rotated = rotate(quaternion, position)
        collision = CollisionObject()
        collision.id = identifier
        collision.header.frame_id = base_frame
        collision.operation = CollisionObject.ADD
        collision.pose.orientation.w = 1.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions
        collision.primitives = [primitive]
        pose = PoseStamped().pose
        pose.position.x = rotated[0] + translation.x
        pose.position.y = rotated[1] + translation.y
        pose.position.z = rotated[2] + translation.z
        result = quaternion_multiply(quaternion, orientation)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = result
        collision.primitive_poses = [pose]
        objects.append(collision)
    return objects


def primitive_pose(collision):
    if len(collision.primitives) != 1 or len(collision.primitive_poses) != 1:
        raise ValueError(f"expected one box for {collision.id}")
    parent = collision.pose
    child = collision.primitive_poses[0]
    orientation = normalized_quaternion((parent.orientation.x, parent.orientation.y,
                                        parent.orientation.z, parent.orientation.w))
    position = rotate(orientation, (child.position.x, child.position.y, child.position.z))
    pose = PoseStamped()
    pose.header = deepcopy(collision.header)
    pose.pose.position.x = parent.position.x + position[0]
    pose.pose.position.y = parent.position.y + position[1]
    pose.pose.position.z = parent.position.z + position[2]
    child_orientation = normalized_quaternion((child.orientation.x, child.orientation.y,
                                              child.orientation.z, child.orientation.w))
    result = quaternion_multiply(orientation, child_orientation)
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = result
    return pose


def scene_readback_matches(expected, actual, to_base, tolerance=1.0e-5):
    by_id = {collision.id: collision for collision in actual}
    for requested in expected:
        observed = by_id.get(requested.id)
        if observed is None or observed.meshes or observed.planes:
            return False
        if (len(observed.primitives) != 1 or len(observed.primitive_poses) != 1
                or observed.primitives[0].type != SolidPrimitive.BOX
                or len(observed.primitives[0].dimensions) != 3):
            return False
        if any(abs(wanted - received) > tolerance or not math.isfinite(received)
               for wanted, received in zip(requested.primitives[0].dimensions,
                                           observed.primitives[0].dimensions)):
            return False
        try:
            observed_pose = to_base(primitive_pose(observed))
            requested_pose = primitive_pose(requested)
        except ValueError:
            return False
        if observed_pose is None or observed_pose.header.frame_id != requested_pose.header.frame_id:
            return False
        wanted = requested_pose.pose
        received = observed_pose.pose
        differences = [wanted.position.x - received.position.x,
                       wanted.position.y - received.position.y,
                       wanted.position.z - received.position.z]
        if any(not math.isfinite(value) or abs(value) > tolerance for value in differences):
            return False
        wanted_orientation = (wanted.orientation.x, wanted.orientation.y,
                              wanted.orientation.z, wanted.orientation.w)
        received_orientation = (received.orientation.x, received.orientation.y,
                                received.orientation.z, received.orientation.w)
        if not all(math.isfinite(value) for value in received_orientation):
            return False
        if min(max(abs(left - sign * right) for left, right in zip(wanted_orientation, received_orientation))
               for sign in (1, -1)) > tolerance:
            return False
    return bool(expected)
