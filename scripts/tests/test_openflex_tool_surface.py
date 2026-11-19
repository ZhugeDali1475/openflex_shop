from copy import deepcopy
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))
from openflex_roboos_bridge.tool_surface import (  # noqa: E402
    FORBIDDEN_ARGUMENT_NAMES,
    HOST_MANAGED_TOOLS,
    HOST_TIMEOUT_BOUNDS,
    MODEL_FACING_ARGUMENTS,
    MODEL_FACING_TOOLS,
    REGISTERED_VALUES,
    check_argument_names,
    model_facing_tools,
    project_schema,
    unauthorized_values,
    validate_advertised_surface,
    validate_model_facing_surface,
)
import openflex_delivery_contract  # noqa: E402


def advertised():
    """Fixture shaped like the real FastMCP advertisement for the six tools."""
    schema = lambda properties, required=(): {  # noqa: E731
        'type': 'object', 'properties': properties, 'required': list(required)}
    return {
        'get_robot_state': schema({}),
        'observe_scene': schema({}),
        'navigate_to_target': schema({
            'target_id': {'anyOf': [{'const': 'pickup_zone'}, {'const': 'delivery_zone'}], 'type': 'string'},
            'request_id': {'type': 'string'}, 'timeout_sec': {'type': 'number'}},
            ('target_id', 'request_id')),
        'grasp_object': schema({
            'object_id': {'type': 'string'}, 'request_id': {'type': 'string'},
            'timeout_sec': {'type': 'number'}}, ('object_id', 'request_id')),
        'place_to_affordance': schema({
            'object_id': {'type': 'string'}, 'affordance_id': {'type': 'string'},
            'request_id': {'type': 'string'}, 'timeout_sec': {'type': 'number'}},
            ('object_id', 'affordance_id', 'request_id')),
        'search_and_pick': schema({
            'query': {'type': 'string'}, 'request_id': {'type': 'string'},
            'timeout_sec': {'type': 'number'}, 'color': {'type': 'string'}},
            ('query', 'request_id')),
    }


def test_model_facing_surface_is_one_source_of_truth_with_the_contract():
    assert set(MODEL_FACING_TOOLS) == (openflex_delivery_contract.READ_TOOLS
                                       | openflex_delivery_contract.COMMANDING_TOOLS)
    assert not set(HOST_MANAGED_TOOLS) & set(MODEL_FACING_TOOLS)
    for name in openflex_delivery_contract.COMMANDING_TOOLS:
        assert 0.0 < HOST_TIMEOUT_BOUNDS[name][0] < HOST_TIMEOUT_BOUNDS[name][1]


def test_projection_drops_host_arguments_and_narrows_registered_enums():
    surface = model_facing_tools(advertised())
    assert set(surface) == set(MODEL_FACING_TOOLS)
    place = surface['place_to_affordance']
    assert place['additionalProperties'] is False
    assert set(place['properties']) == {'object_id', 'affordance_id'}
    assert set(place['required']) == {'object_id', 'affordance_id'}
    assert place['properties']['affordance_id']['enum'] == list(REGISTERED_VALUES['affordance_id'])
    navigate = surface['navigate_to_target']
    assert navigate['properties']['target_id']['enum'] == list(REGISTERED_VALUES['target_id'])
    assert 'anyOf' not in navigate['properties']['target_id']
    assert 'sim_delivery_top_right' not in str(surface)


def test_unauthorized_delivery_table_is_never_offered_or_registered():
    assert 'sim_delivery_top_right' not in str(REGISTERED_VALUES)
    assert not any('sim_delivery_top_right' in values for values in REGISTERED_VALUES.values())
    assert unauthorized_values() == ('sim_delivery_top_right',)


def test_free_text_argument_stays_bounded():
    surface = model_facing_tools(advertised())
    query = surface['search_and_pick']['properties']['query']
    assert query['type'] == 'string'
    assert query['minLength'] == 1 and query['maxLength'] == 200
    assert 'enum' not in query


@pytest.mark.parametrize('tool', ['navigate_to_target', 'grasp_object', 'place_to_affordance', 'search_and_pick'])
def test_request_id_and_timeout_are_host_managed_not_model_chosen(tool):
    surface = model_facing_tools(advertised())
    assert 'request_id' not in surface[tool]['properties']
    assert 'timeout_sec' not in surface[tool]['properties']


def test_missing_model_facing_tool_fails_closed_instead_of_shrinking():
    incomplete = advertised()
    incomplete.pop('grasp_object')
    with pytest.raises(ValueError, match='missing from the server'):
        model_facing_tools(incomplete)


def test_server_surface_must_not_leak_coordinates_paths_or_debug_handles():
    leaked = advertised()
    leaked['teleport_object'] = {'type': 'object', 'properties': {
        'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'number'}}}
    with pytest.raises(ValueError, match='free-coordinate/debug'):
        validate_advertised_surface(leaked)
    for name in ('position', 'trajectory', 'controller', 'shell'):
        with pytest.raises(ValueError):
            check_argument_names('fixture', {name: {'type': 'string'}})
    assert 'x' in FORBIDDEN_ARGUMENT_NAMES and 'path' in FORBIDDEN_ARGUMENT_NAMES


def test_declared_model_arguments_cannot_grow_without_the_projection_noticing():
    widened = advertised()
    widened['grasp_object']['properties']['pose'] = {'type': 'array'}
    with pytest.raises(ValueError, match='free-coordinate/debug'):
        model_facing_tools(widened)
    narrowed = advertised()
    narrowed['grasp_object']['properties'].pop('object_id')
    with pytest.raises(ValueError, match='do not match the declared surface'):
        model_facing_tools(narrowed)


def test_a_projected_surface_that_advertises_the_right_table_is_rejected():
    tampered = model_facing_tools(advertised())
    tampered['navigate_to_target'] = deepcopy(tampered['navigate_to_target'])
    tampered['navigate_to_target']['properties']['target_id']['enum'] = [
        'pickup_zone', 'delivery_zone', 'sim_delivery_top_right']
    with pytest.raises(ValueError, match='not the registered value set|unauthorized delivery table'):
        validate_model_facing_surface(tampered)


def test_composite_skill_is_model_facing_and_declared_as_a_commanding_tool():
    assert 'search_and_pick' in MODEL_FACING_TOOLS
    assert MODEL_FACING_ARGUMENTS['search_and_pick'] == ('query', 'color')
    assert 'search_and_pick' in openflex_delivery_contract.COMPOSITE_TOOLS
    assert 'search_and_pick' in openflex_delivery_contract.COMMANDING_TOOLS
    assert 'search_and_pick' not in openflex_delivery_contract.MOTION_TOOLS
