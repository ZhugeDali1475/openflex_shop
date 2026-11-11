"""Serialize validity failures while preserving sampling limitations."""

from rosidl_runtime_py.convert import message_to_ordereddict


def ros_message(message):
    return None if message is None else message_to_ordereddict(message)


def stamp_ns(stamp):
    if isinstance(stamp, dict):
        seconds, nanoseconds = stamp["sec"], stamp["nanosec"]
    else:
        seconds, nanoseconds = stamp.sec, stamp.nanosec
    return int(seconds) * 1_000_000_000 + int(nanoseconds)


def sample_pairing(joint_state, simulation, received_age_sec, object_id):
    result = {
        "received_age_sec": received_age_sec,
        "stamp_delta_ns": None,
        "same_publish_stamp": False,
        "same_object_id": None,
        "synchronized_with_moveit": False,
    }
    if joint_state is not None and simulation is not None:
        delta = stamp_ns(simulation["stamp"]) - stamp_ns(
            joint_state.header.stamp
        )
        result["stamp_delta_ns"] = delta
        result["same_publish_stamp"] = delta == 0
        result["same_object_id"] = simulation.get("object_id") == object_id
    return result


def failure_category(checks):
    if any(
        check["response"] is None or check.get("error")
        for check in checks
    ):
        return "validity_service_failure"
    invalid = [
        check["label"]
        for check in checks
        if not check["response"]["valid"]
    ]
    if len(invalid) == 2:
        return "current_and_target_invalid"
    if "current" in invalid:
        return "current_invalid"
    if invalid:
        return "target_invalid"
    return "valid"
