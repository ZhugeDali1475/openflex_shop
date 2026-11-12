from copy import deepcopy


FIXED_SEQUENCE = (
    ('get_robot_state', {}),
    ('observe_scene', {}),
    ('navigate_to_target', {'target_id': 'pickup_zone'}),
    ('grasp_object', {'object_id': 'thesis_item_01'}),
    ('navigate_to_target', {'target_id': 'delivery_zone'}),
    ('place_to_affordance', {'object_id': 'thesis_item_01', 'affordance_id': 'delivery_zone'}),
    ('get_robot_state', {}),
)


def reject_stale_frame(raw, image_count):
    camera = raw.get('sources', {}).get('camera', {})
    if camera.get('status') != 'fresh' or image_count != 1:
        raise RuntimeError('Fault trial requires an actual fresh image to withhold')
    return {'status': 'rejected_before_execution', 'success': False, 'no_motion_sent': True,
            'actual_camera': 'stale', 'fault': 'experiment_injected_stale_observation_once',
            'detail': 'Experiment withheld this frame as stale. No image can be used; no motion sent. '
                      'Select observe_scene again for a fresh observation, or stop safely.'}


async def run_fixed(execute, contract):
    for name, arguments in FIXED_SEQUENCE:
        response = await execute(name, deepcopy(arguments))
        if response.isError:
            raise RuntimeError('Fixed dispatcher received an MCP error')
        if name == 'observe_scene' and not contract.observed:
            raise RuntimeError('Fixed dispatcher stops on unusable observation')
        if name == 'get_robot_state' and not contract.checked_state:
            raise RuntimeError('Fixed dispatcher stops on unusable state')
        phase = {'grasp_object': 'grasp', 'place_to_affordance': 'place'}.get(name)
        if name == 'navigate_to_target':
            phase = 'pickup' if arguments['target_id'] == 'pickup_zone' else 'delivery'
        if phase and not getattr(contract, phase):
            raise RuntimeError(f'Fixed dispatcher stops on unverified {phase}')
    return {'status': 'succeeded' if contract.completed else 'not_completed',
            'success': contract.completed, 'completion_verified': contract.completed}
