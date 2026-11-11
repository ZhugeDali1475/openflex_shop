import math


def sample_joint_path(joint_names, points, maximum_samples=10000):
    names = list(joint_names)
    if not names or len(set(names)) != len(names):
        raise ValueError('Expected distinct joint names')
    waypoints = [list(point) for point in points]
    if not waypoints or any(len(point) != len(names) for point in waypoints):
        raise ValueError('Joint path dimensions differ')
    if not all(math.isfinite(value) for point in waypoints for value in point):
        raise ValueError('Joint path is not finite')
    sampled = [waypoints[0]]
    for previous, following in zip(waypoints, waypoints[1:]):
        segments = max(1, *(math.ceil(abs(end - start) / (0.005 if name == 'lift_joint' else 0.03))
                            for name, start, end in zip(names, previous, following)))
        if len(sampled) + segments > maximum_samples:
            raise ValueError('Joint path exceeds safety sampling budget')
        sampled.extend([[start + (end - start) * index / segments for start, end in zip(previous, following)]
                        for index in range(1, segments + 1)])
    return sampled
