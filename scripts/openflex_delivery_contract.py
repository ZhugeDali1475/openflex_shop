import math


OBJECT = 'thesis_item_01'
READ_TOOLS = {'get_robot_state', 'observe_scene'}
MOTION_TOOLS = {'navigate_to_target', 'grasp_object', 'place_to_affordance'}


def fresh_data(snapshot, name):
    source = snapshot.get('sources', {}).get(name, {})
    if source.get('status') != 'fresh' or source.get('data') is None:
        raise RuntimeError(f'Fresh independent evidence required: {name}')
    return source['data']


def state_summary(snapshot):
    result = {key: snapshot.get(key) for key in ['environment', 'physical_verification', 'holding', 'holding_evidence']}
    result['sources'] = {key: source.get('status') for key, source in snapshot.get('sources', {}).items()}
    for name, field in [('odometry', 'pose'), ('object_observation', 'pose'), ('grasp_status', 'verified')]:
        source = snapshot.get('sources', {}).get(name, {})
        result[name] = source.get('data', {}).get(field) if source.get('data') else None
    diagnostic = snapshot.get('sources', {}).get('simulation_diagnostic', {}).get('data')
    if diagnostic:
        result['diagnostic'] = {key: diagnostic.get(key) for key in ['object_pose', 'attached_arm',
            'delivery_support_contact', 'object_velocity_linear_angular', 'safety_stop']}
    result['object_identity'] = 'registered ROS simulation object; visual identity NOT verified'
    return result


def terminal_summary(result):
    return {key: result.get(key) for key in ['request_id', 'tool', 'arguments', 'status', 'success',
        'detail', 'stopped_confirmed', 'grasp_verified', 'transport_safe', 'place_verified', 'physical_verification']
        if key in result}


class DeliveryContract:
    def __init__(self):
        self.checked_state = False
        self.observed = False
        self.pickup = False
        self.grasp = False
        self.delivery = False
        self.place = False
        self.completed = False
        self.snapshot = None
        self.motion_calls = set()

    def validate(self, name, arguments):
        if name in READ_TOOLS:
            if arguments:
                raise ValueError('Observation tools take no arguments')
            return
        if name not in MOTION_TOOLS:
            raise ValueError('Unregistered model-selected skill')
        expected = {'navigate_to_target': {'target_id'}, 'grasp_object': {'object_id'},
                    'place_to_affordance': {'object_id', 'affordance_id'}}[name]
        if set(arguments) != expected:
            raise ValueError('Model may not inject request IDs, timeouts, coordinates or extra arguments')
        if not self.checked_state or not self.observed:
            raise ValueError('Fresh initial state and actual camera observation required before motion')
        identity = (name, tuple(sorted(arguments.items())))
        if identity in self.motion_calls:
            raise ValueError('Repeated motion is not recovery; original operation must be audited')
        if name == 'navigate_to_target':
            target = arguments['target_id']
            if target == 'pickup_zone' and not self.grasp and not self.place:
                return
            if target == 'delivery_zone' and self.grasp and not self.place:
                return
            raise ValueError('Navigation does not match verified pickup/delivery phase')
        if name == 'grasp_object' and arguments['object_id'] == OBJECT and self.pickup and not self.grasp:
            return
        if (name == 'place_to_affordance' and arguments == {'object_id': OBJECT, 'affordance_id': 'delivery_zone'}
                and self.grasp and self.delivery and not self.place):
            return
        raise ValueError('Skill prerequisites are not independently verified')

    def record(self, name, arguments, result, image_count=0):
        if name == 'observe_scene':
            camera = result.get('sources', {}).get('camera', {})
            if image_count != 1 or camera.get('status') != 'fresh':
                raise RuntimeError('Actual fresh camera image required; no imagined substitute')
            self.observed = True
            return
        if name == 'get_robot_state':
            self.snapshot = result
            diagnostic = fresh_data(result, 'simulation_diagnostic')
            fresh_data(result, 'odometry')
            fresh_data(result, 'joint_states')
            if (result.get('environment') != 'mujoco_simulation' or result.get('physical_verification') is not False
                    or diagnostic.get('source') != 'mujoco_diagnostic_only_NOT_PHYSICAL'
                    or diagnostic.get('safety_stop') is not None):
                raise RuntimeError('Only healthy isolated simulation evidence is permitted')
            if not self.checked_state:
                if diagnostic.get('attached_arm') is not None:
                    raise RuntimeError('New delivery demonstration requires a measured unheld initial object')
                nominal = result['scene_registry']['objects'][OBJECT]['nominal_pose_xyzyaw'][:3]
                if (diagnostic.get('object_id') != OBJECT
                        or math.dist(diagnostic['object_pose']['position_m'], nominal) > 0.04):
                    raise RuntimeError('New delivery must start with the actual registered object at its shelf origin')
                self.checked_state = True
            if self.place:
                from openflex_thesis_mujoco.release_verification import object_settled_on_surface

                joints = fresh_data(result, 'joint_states')
                fingers = dict(zip(joints['name'], joints['position']))
                affordance = result['scene_registry']['affordances']['delivery_zone']
                surface = affordance['surface_center_xyz']
                width, depth = affordance['surface_dimensions_xy']
                settled = object_settled_on_surface(diagnostic['object_pose']['position_m'],
                    diagnostic['object_pose']['orientation_xyzw'], [0.035, 0.03, 0.08],
                    [surface[0], surface[1], surface[2] + 0.08],
                    [surface[0], surface[1], surface[2] - 0.02], [width / 2, depth / 2, 0.02],
                    diagnostic['object_velocity_linear_angular'], diagnostic['delivery_support_contact'],
                    diagnostic['attached_arm'] is not None, fingers['openarmx_right_finger_joint1'] >= 0.03)
                twist = fresh_data(result, 'odometry')['twist']['twist']
                self.completed = bool(settled and math.hypot(twist['linear']['x'], twist['linear']['y']) <= 0.02
                    and abs(twist['angular']['z']) <= 0.03 and all(math.isfinite(value) and abs(value) <= 0.03
                        for value in joints['velocity']))
                if not self.completed:
                    raise RuntimeError('Final independent measured support, release, pose and stopped evidence disagree')
            return
        if result.get('status') != 'succeeded' or result.get('success') is not True or result.get('stopped_confirmed') is not True:
            raise RuntimeError(f'Failed/uncertain motion must stop the delivery loop: {terminal_summary(result)}')
        self.motion_calls.add((name, tuple(sorted(arguments.items()))))
        if name == 'navigate_to_target':
            if arguments['target_id'] == 'pickup_zone':
                self.pickup = True
            else:
                self.delivery = True
        elif name == 'grasp_object':
            if not result.get('grasp_verified') or not result.get('transport_safe'):
                raise RuntimeError('Successful controller action is not verified grasp/transport')
            self.grasp = True
        elif name == 'place_to_affordance':
            if not result.get('place_verified'):
                raise RuntimeError('Release is not independently verified placement')
            self.place = True
