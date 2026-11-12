import os


PROFILES = ('baseline', 'balanced_v1')
FASTER_PHASES = frozenset({'pregrasp', 'transport', 'preplace', 'postplace_retract'})
KNOWN_PHASES = FASTER_PHASES | {'grasp', 'clearance_lift', 'shelf_withdrawal', 'place', 'postplace_clearance'}


def validate_profile(profile):
    if profile not in PROFILES:
        raise ValueError('Unreviewed simulation motion profile')
    return profile


def selected_profile():
    return validate_profile(os.environ.get('OPENFLEX_MOTION_PROFILE', 'baseline'))


def scaling_for_phase(phase, profile=None):
    profile = validate_profile(profile if profile is not None else selected_profile())
    if phase not in KNOWN_PHASES:
        raise ValueError('Unknown motion phase cannot receive a speed setting')
    if profile == 'balanced_v1':
        if os.environ.get('ROS_DOMAIN_ID') != '83' or os.environ.get('OPENFLEX_ENABLE_SIM_MANIPULATION') != '1':
            raise RuntimeError('Faster profile requires explicit isolated simulation manipulation')
        if phase in FASTER_PHASES:
            return 0.10, 0.05
    return 0.05, 0.05
