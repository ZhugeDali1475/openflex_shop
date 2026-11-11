"""Forbidden robot contacts with the fixed task environment."""


def forbidden_environment_contact(body_ids, distance, robot_body_ids, environment_body_ids):
    first, second = body_ids
    return bool(distance <= 0.0 and (
        (first in robot_body_ids and second in environment_body_ids)
        or (second in robot_body_ids and first in environment_body_ids)
    ))


def forbidden_payload_contact(body_ids, distance, attached_object_body, environment_body_ids):
    if attached_object_body is None or distance > 0:
        return False
    first, second = body_ids
    obstacles = set(environment_body_ids) | {0}
    return bool((first == attached_object_body and second in obstacles)
                or (second == attached_object_body and first in obstacles))
