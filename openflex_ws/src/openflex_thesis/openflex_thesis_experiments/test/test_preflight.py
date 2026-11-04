from openflex_thesis_experiments.preflight import (
    gate_checks,
    physical_readiness_checks,
)


def ready_configs():
    shelf = {"calibrated": True}
    perception = {
        "perception": {
            "backend": "test",
            "physical_candidates_available": True,
        }
    }
    control = {
        "velocity_arbiter": {"ros__parameters": {"require_physical_affordance": True}},
        "moveit_grasp_executor": {
            "ros__parameters": {"object_geometry_calibrated": True}
        },
        "mission_orchestrator": {
            "ros__parameters": {
                "require_physical_affordance": True,
                "require_physical_perception": True,
            }
        },
    }
    schema = {
        "frozen": True,
        "scope": {"act_may_command_lift_arm_or_gripper": False},
        "numeric_observation": {"dimension": 24},
        "action": {
            "dimension": 3,
            "semantics": "body_twist_only",
            "order": ["base_vx", "base_vy", "base_wz"],
        },
    }
    return shelf, perception, control, schema


def test_physical_readiness_accepts_explicit_frozen_configuration():
    assert all(
        result.passed
        for result in physical_readiness_checks(
            *ready_configs(), experiment_protocol={"frozen": True}
        )
    )


def test_physical_readiness_rejects_unconfirmed_perception_mapping():
    shelf, perception, control, schema = ready_configs()
    perception["perception"]["physical_candidates_available"] = False
    failed = [
        result.name
        for result in physical_readiness_checks(
            shelf,
            perception,
            control,
            schema,
            experiment_protocol={"frozen": True},
        )
        if not result.passed
    ]
    assert failed == ["physical_perception_candidates"]


def test_gate_checks_enforce_order():
    config = {
        "gates": {"G0": {"pass": True}, "G1": {"pass": False}},
        "ordering": ["G0", "G1"],
    }
    results = gate_checks(config, "G1")
    assert [item.passed for item in results] == [True, False]
