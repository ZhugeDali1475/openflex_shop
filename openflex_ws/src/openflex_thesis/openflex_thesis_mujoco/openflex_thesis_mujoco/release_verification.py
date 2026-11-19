import math

from .math_utils import rotate_vector


def box_world_extents(half_size, orientation):
    if (len(half_size) != 3 or len(orientation) != 4
            or not all(math.isfinite(value) for value in (*half_size, *orientation))
            or any(value <= 0 for value in half_size)
            or abs(sum(value * value for value in orientation) - 1.0) > 0.01):
        raise ValueError('Expected finite positive geometry and a normalized orientation')
    axes = [rotate_vector(orientation, direction) for direction in [(1, 0, 0), (0, 1, 0), (0, 0, 1)]]
    return tuple(sum(abs(axis[coordinate]) * size for axis, size in zip(axes, half_size))
                 for coordinate in range(3))


def release_request_rejection(position, orientation, target, half_size, surface_center, surface_half_size):
    values = (*position, *target, *surface_center, *surface_half_size)
    if not all(math.isfinite(value) for value in values) or any(value <= 0 for value in surface_half_size):
        return 'non-finite or invalid release geometry'
    try:
        extent = box_world_extents(half_size, orientation)
    except ValueError as error:
        return str(error)
    upright = rotate_vector(orientation, (0, 0, 1))[2]
    if upright < math.cos(0.15):
        return 'object must be upright before release'
    surface_z = surface_center[2] + surface_half_size[2]
    if abs(target[2] - (surface_z + extent[2])) > 0.005:
        return 'requested object center does not match the measured delivery surface'
    if any(abs(target[axis] - surface_center[axis]) + extent[axis] > surface_half_size[axis] - 0.01
           for axis in (0, 1)):
        return 'object footprint is not fully inside the delivery surface'
    if math.hypot(position[0] - target[0], position[1] - target[1]) > 0.025:
        return 'arm has not brought the object to the delivery target; teleportation is forbidden'
    if not -0.001 <= position[2] - target[2] <= 0.03:
        return 'object release height is not within the verified short-drop range'
    if any(abs(position[axis] - surface_center[axis]) + extent[axis] > surface_half_size[axis] - 0.01
           for axis in (0, 1)):
        return 'actual object footprint is outside the support surface'
    return ''


def object_settled_on_surface(position, orientation, half_size, target, surface_center, surface_half_size,
                              velocity, supported, attached, gripper_open):
    reason = release_request_rejection(position, orientation, target, half_size, surface_center, surface_half_size)
    if reason or attached or not gripper_open or not supported or len(velocity) != 6:
        return False
    if not all(math.isfinite(value) for value in velocity):
        return False
    extent = box_world_extents(half_size, orientation)
    bottom = position[2] - extent[2]
    surface_z = surface_center[2] + surface_half_size[2]
    return bool(abs(bottom - surface_z) <= 0.002
                and math.hypot(*velocity[:3]) <= 0.01 and math.hypot(*velocity[3:]) <= 0.05)


# Measured limits of the isolated fixture (ROS domain 83, MuJoCo only).
#
# The delivery counter centre sits ~1.1 m from the registered delivery approach
# pose.  A planning-only probe of the real right-arm grasp (``place_trajectory``)
# showed that MoveIt ``/compute_ik`` returns NO_IK_SOLUTION for a release centre
# 0.62 m from the base, and that for this grasp the arm-feasible band is a narrow
# strip near the table's near edge: map x in [-1.860, -1.845] for the left
# delivery pose ``(-1.30, 3.60, pi)`` (i.e. 0.545-0.560 m of reach).  The centre
# of that strip is used here.  ``SURFACE_EDGE_MARGIN_M`` keeps the object (half
# extent 0.035 m) fully on the surface with a 1 cm buffer, which is the same
# bound the independent ``release_request_rejection`` footprint check enforces.
RELEASE_REACH_M = 0.5525
SURFACE_EDGE_MARGIN_M = 0.045
RELEASE_DROP_CLEARANCE_M = 0.015


def reachable_release_center(base_xy, surface_center_xyz, half_size,
                             reach=RELEASE_REACH_M, edge_margin=SURFACE_EDGE_MARGIN_M):
    """Point on the delivery surface that a measured base can actually release onto.

    Walks ``reach`` metres from the base towards the surface centre and rejects the
    result when it falls outside the surface footprint (minus ``edge_margin``). The
    returned ``z`` carries the small short-drop clearance above the settled height.
    """
    delta_x, delta_y = surface_center_xyz[0] - base_xy[0], surface_center_xyz[1] - base_xy[1]
    distance = math.hypot(delta_x, delta_y)
    if distance <= 1.0e-6:
        return None
    step = min(reach, distance)
    point = [base_xy[0] + delta_x / distance * step, base_xy[1] + delta_y / distance * step]
    if (abs(point[0] - surface_center_xyz[0]) > half_size[0] - edge_margin
            or abs(point[1] - surface_center_xyz[1]) > half_size[1] - edge_margin):
        return None
    return [point[0], point[1], surface_center_xyz[2] + 0.08 + RELEASE_DROP_CLEARANCE_M]
