#!/usr/bin/env python3
"""End-to-end composite Skill closed loop (P4) on the isolated MuJoCo gateway.

Runs ``fetch_and_deliver(thesis_item_01, sim_delivery_top_left)`` for one or
more rounds.  Every round starts from an independently reset isolated scene: a
full stop/start of the ROS_DOMAIN_ID=83 stack reloads the MJCF so the single red
object is back at its shelf nominal pose and is not attached.  The reset is a
scene rebuild, never a robot "put-back" action.

The round then executes the composite steps through the real MCP skill gateway,
ordered by the ``CompositeExecution`` state machines declared in
``skill_contracts.py`` (observe -> navigate(pickup_zone) -> grasp ->
verify_attachment -> navigate_with_payload(left table) -> place ->
verify_release_and_settle -> retreat -> get_state), plus an independent final
readback.  A failure is terminal for the round and stays in the denominator;
nothing is rewritten into a success.

Isolated MuJoCo only; ``physical_verification=false``.  Not physical hardware.
"""
import argparse
import asyncio
from datetime import datetime
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import httpx
from mcp import ClientSession
from mcp.client.streamable_http import streamable_http_client

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))

import openflex_runtime  # noqa: E402  (scripts/ on sys.path)
from openflex_roboos_bridge.competition_registry import TARGET_OBJECT_ID  # noqa: E402
from openflex_roboos_bridge.single_red_lifecycle import (  # noqa: E402
    EvidenceReference, attach_target, begin_transport, confirm_approach,
    confirm_placed, initial_single_red_lifecycle, select_target)
from openflex_roboos_bridge.skill_contracts import (  # noqa: E402
    COMPOSITE_SKILLS, CompositeExecution, SkillResult, SkillStatus)

DEFAULT_SCENE_DIR = (ROOT / 'local_dependencies/openflex_thesis/'
                     'retail_fixture_single_red_runtime_20261116_left_v9')
LEFT_DELIVERY_ID = 'sim_delivery_top_left'
DELIVERY_ALIAS = 'delivery_zone'  # online registry id the canonical left id maps to
PICKUP_ID = 'pickup_zone'
SCENE_HASH = '6db3a9bfad3fa6f937856521ece259e06fe1efc578db858251cb630a82257995'


class RoundError(RuntimeError):
    """A terminal failure for the current round (kept in the denominator)."""


def spec_for(name):
    for spec in COMPOSITE_SKILLS:
        if spec.name == name:
            return spec
    raise KeyError(name)


def error_detail(error):
    nested = getattr(error, 'exceptions', None)
    if nested:
        return '; '.join(error_detail(item) for item in nested)
    return f'{type(error).__name__}: {error}'


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def load_scene_snapshot():
    return openflex_runtime.request_json('http://127.0.0.1:18552/state')


def wait_endpoints(ports, timeout):
    """Block until every isolated-stack HTTP port accepts a TCP connection.

    The readonly snapshot (18552) can be fresh before the navigation gateway
    (18553) or the MCP gateway (18556) finish binding, so a measured-hold check
    alone is not enough to start a round.
    """
    deadline = time.monotonic() + timeout
    pending = list(ports)
    while pending:
        if time.monotonic() > deadline:
            raise RoundError(f'isolated stack ports not reachable: {pending}')
        still = []
        for port in pending:
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=2):
                    pass
            except OSError:
                still.append(port)
        pending = still
        if pending:
            time.sleep(1.0)


def scene_reset(arguments, log):
    """Independently rebuild the isolated scene; return the ready initial snapshot."""
    environment = os.environ.copy()
    environment.update({
        'OPENFLEX_SIM_SCENE_MODEL': str(arguments.scene_xml),
        'OPENFLEX_SIM_SCENE_CONFIG': str(arguments.scene_yaml),
        'OPENFLEX_SIM_SHELF_SCENE_CONFIG': str(arguments.shelf_scene),
        'OPENFLEX_SIM_SHELF_SCENE': str(arguments.shelf_scene),
    })
    active = any(openflex_runtime.active(unit) for unit in openflex_runtime.UNITS.values())
    if active or openflex_runtime.pending_operations():
        log({'reset_step': 'stop', 'timeout': arguments.stop_timeout})
        subprocess.run([sys.executable, str(ROOT / 'scripts/openflex_runtime.py'), 'stop',
                        '--timeout', str(arguments.stop_timeout), '--shutdown-services'],
                       env=environment, check=True)
    log({'reset_step': 'start'})
    start_command = [sys.executable, str(ROOT / 'scripts/openflex_runtime.py'), 'start',
                     '--enable-simulation-motion']
    if getattr(arguments, 'gui', False):
        start_command.append('--gui')
    subprocess.run(start_command, env=environment, check=True)
    deadline = time.monotonic() + arguments.ready_timeout
    nominal = None
    while time.monotonic() < deadline:
        try:
            snapshot = load_scene_snapshot()
        except Exception as error:  # gateway not up yet
            log({'reset_step': 'wait', 'detail': str(error)})
            time.sleep(2.0)
            continue
        if openflex_runtime.measured_hold(snapshot):
            diagnostic = snapshot['sources']['simulation_diagnostic']['data']
            nominal = snapshot['scene_registry']['objects'][arguments.object_id]['nominal_pose_xyzyaw'][:3]
            offset = math.dist(diagnostic['object_pose']['position_m'], nominal)
            if diagnostic.get('attached_arm') is None and offset <= 0.04:
                return snapshot
            log({'reset_step': 'wait', 'attached_arm': diagnostic.get('attached_arm'),
                 'object_offset_m': offset})
        time.sleep(2.0)
    raise RoundError('scene did not become ready after reset')


async def run_round(arguments, output, round_index, model):
    events = []

    async def call(session, name, parameters):
        response = await session.call_tool(name, parameters)
        if response.isError:
            raise RoundError(f'{name} returned MCP error: {response.content}')
        if response.structuredContent is not None:
            result = response.structuredContent
        else:
            result = json.loads(next(block.text for block in response.content if block.type == 'text'))
        events.append({'time': datetime.now().astimezone().isoformat(), 'tool': name,
                       'arguments': parameters, 'result': result})
        save(output / 'events.json', events)
        return result

    async def poll(session, request_id, timeout):
        deadline = time.monotonic() + timeout + 30
        while True:
            result = await call(session, 'get_skill_result', {'request_id': request_id})
            if result['status'] != 'running':
                return result
            if time.monotonic() > deadline:
                raise RoundError('observer timed out; robot status UNKNOWN, no implicit stop or retry')
            await asyncio.sleep(0.5)

    def result(name, status, *, success=None, facts=None, evidence=(), children=(),
               source='mcp_skill_gateway', stopped=None, collision=None,
               independent=None, message=''):
        return SkillResult(request_id=f'{model.request_id}_{name}', skill_name=name,
                           status=status, success=success, source=source,
                           evidence_refs=tuple(evidence) or (str(output / 'events.json'),),
                           child_request_ids=tuple(children), facts=facts or {},
                           stopped_confirmed=stopped, collision_verified=collision,
                           independent_verification_ref=independent or str(output / 'events.json'),
                           safety_verified=(status is SkillStatus.SUCCEEDED),
                           freshness_verified=(status is SkillStatus.SUCCEEDED),
                           message=message)

    wait_endpoints([18552, 18553, arguments.port], arguments.ready_timeout)

    async with httpx.AsyncClient(trust_env=False, timeout=10) as client:
        async with streamable_http_client(f'http://127.0.0.1:{arguments.port}/mcp', http_client=client) as (reader, writer, _):
            async with ClientSession(reader, writer) as session:
                await session.initialize()

                # ---- pick_object: observe -> navigate -> execute_grasp -> verify_attachment
                pick = CompositeExecution(spec_for('pick_object'), f'{model.request_id}_pick')
                pick.start_step('observe')
                observation = await call(session, 'observe_scene', {})
                observed = json.loads(observation[0]) if isinstance(observation, list) else observation
                camera_fresh = observed.get('sources', {}).get('camera', {}).get('status') == 'fresh'
                pick.record_result('observe', result('observe', SkillStatus.SUCCEEDED if camera_fresh else SkillStatus.FAILED,
                                                     success=camera_fresh or None,
                                                     facts={'camera_fresh': camera_fresh,
                                                            'object_id': arguments.object_id}))

                pick.start_step('navigate')
                if arguments.pickup_nav == 'on':
                    pickup_id = f'{model.request_id}_nav_pickup'
                    started = await call(session, 'navigate_to_target',
                                         {'target_id': PICKUP_ID, 'request_id': pickup_id,
                                          'timeout_sec': arguments.navigate_timeout})
                    arrival = started if started['status'] != 'running' else await poll(session, pickup_id, arguments.navigate_timeout)
                    pick.record_result('navigate', result('navigate', SkillStatus.SUCCEEDED if (arrival.get('success') and arrival.get('stopped_confirmed')) else SkillStatus.FAILED,
                                                          success=bool(arrival.get('success') and arrival.get('stopped_confirmed')) or None,
                                                          facts={'station_id': PICKUP_ID, 'status': arrival.get('status')},
                                                          children=(pickup_id,), stopped=arrival.get('stopped_confirmed'),
                                                          independent=arrival.get('fingerprint')))
                    if pick.halted_reason:
                        if pick.requires_stop_confirmation and arrival.get('stopped_confirmed'):
                            pick.confirm_stop(f'{pickup_id}:stopped_confirmed')
                        raise RoundError(f'navigate halted: {pick.halted_reason}')
                else:
                    # Runtime mapping: the deployed grasp skill performs the pickup-station
                    # base alignment internally (control_acquired/measured_goal).  Issuing a
                    # separate navigate_to_target(pickup_zone) first destabilizes the grasp
                    # MOVEIT acknowledgement (measured 3/3 revoked); the alignment evidence
                    # is asserted from the execute_grasp result below.
                    pickup_id = None
                    pick.record_result('navigate', result('navigate', SkillStatus.SUCCEEDED, success=True,
                                                          facts={'station_id': PICKUP_ID,
                                                                 'realized_by': 'execute_grasp_internal_base_alignment',
                                                                 'explicit_navigation_called': False},
                                                          source='composite_runtime_mapping'))

                pick.start_step('execute_grasp')
                grasp_id = f'{model.request_id}_grasp'
                started = await call(session, 'grasp_object',
                                     {'object_id': arguments.object_id, 'request_id': grasp_id,
                                      'timeout_sec': arguments.grasp_timeout})
                grasp = started if started['status'] != 'running' else await poll(session, grasp_id, arguments.grasp_timeout)
                grasp_ok = bool(grasp.get('status') == 'succeeded' and grasp.get('success')
                                and grasp.get('stopped_confirmed') and grasp.get('grasp_verified')
                                and grasp.get('transport_safe'))
                pick.record_result('execute_grasp', result('execute_grasp', SkillStatus.SUCCEEDED if grasp_ok else SkillStatus.FAILED,
                                                           success=grasp_ok or None,
                                                           facts={'grasp_verified': grasp.get('grasp_verified'),
                                                                  'transport_safe': grasp.get('transport_safe'),
                                                                  'status': grasp.get('status')},
                                                           children=(grasp_id,), stopped=grasp.get('stopped_confirmed'),
                                                           independent=grasp.get('fingerprint'),
                                                           message=grasp.get('detail', '')))
                if pick.halted_reason:
                    if pick.requires_stop_confirmation and grasp.get('stopped_confirmed'):
                        pick.confirm_stop(f'{grasp_id}:stopped_confirmed')
                    raise RoundError(f'execute_grasp halted: {pick.halted_reason}')
                if arguments.pickup_nav == 'off':
                    grasp_kinds = {event.get('kind') for event in grasp.get('events', [])}
                    if not {'control_acquired', 'measured_goal'} <= grasp_kinds:
                        raise RoundError('grasp succeeded without the internal pickup base alignment evidence')

                pick.start_step('verify_attachment')
                state = await call(session, 'get_robot_state', {})
                state_diag = state.get('sources', {}).get('simulation_diagnostic', {}).get('data', {})
                attached = (state.get('holding') == arguments.object_id
                            and state_diag.get('attached_arm') not in (None, 'none')
                            and state_diag.get('object_id') == arguments.object_id)
                pick.record_result('verify_attachment', result('verify_attachment', SkillStatus.SUCCEEDED if attached else SkillStatus.FAILED,
                                                               success=attached or None,
                                                               facts={'holding': state.get('holding'),
                                                                      'attached_arm': state_diag.get('attached_arm'),
                                                                      'object_id': state_diag.get('object_id')}))

                if not pick.succeeded:
                    raise RoundError(f'pick_object halted: {pick.halted_reason or "incomplete"}')

                # ---- deliver_object: navigate_with_payload -> execute_place -> verify_release_and_settle -> retreat
                deliver = CompositeExecution(spec_for('deliver_object'), f'{model.request_id}_deliver')
                deliver.start_step('navigate_with_payload')
                carry_id = f'{model.request_id}_carry'
                started = await call(session, 'navigate_to_target',
                                     {'target_id': LEFT_DELIVERY_ID, 'request_id': carry_id,
                                      'timeout_sec': arguments.navigate_timeout})
                carry = started if started['status'] != 'running' else await poll(session, carry_id, arguments.navigate_timeout)
                carry_ok = bool(carry.get('success') and carry.get('stopped_confirmed'))
                deliver.record_result('navigate_with_payload', result('navigate_with_payload', SkillStatus.SUCCEEDED if carry_ok else SkillStatus.FAILED,
                                                                      success=carry_ok or None,
                                                                      facts={'affordance_id': LEFT_DELIVERY_ID,
                                                                             'status': carry.get('status')},
                                                                      children=(carry_id,), stopped=carry.get('stopped_confirmed'),
                                                                      independent=carry.get('fingerprint')))
                if deliver.halted_reason:
                    if deliver.requires_stop_confirmation and carry.get('stopped_confirmed'):
                        deliver.confirm_stop(f'{carry_id}:stopped_confirmed')
                    raise RoundError(f'navigate_with_payload halted: {deliver.halted_reason}')

                deliver.start_step('execute_place')
                place_id = f'{model.request_id}_place'
                started = await call(session, 'place_to_affordance',
                                     {'object_id': arguments.object_id, 'affordance_id': LEFT_DELIVERY_ID,
                                      'request_id': place_id, 'timeout_sec': arguments.place_timeout})
                place = started if started['status'] != 'running' else await poll(session, place_id, arguments.place_timeout)
                place_events = {event.get('kind') for event in place.get('events', [])}
                release_ok = 'actual_release_settled' in place_events
                retreat_ok = 'post_retreat_placement_verified' in place_events
                place_ok = bool(place.get('status') == 'succeeded' and place.get('success')
                                and place.get('stopped_confirmed') and place.get('place_verified')
                                and release_ok and retreat_ok)
                deliver.record_result('execute_place', result('execute_place', SkillStatus.SUCCEEDED if place_ok else SkillStatus.FAILED,
                                                              success=place_ok or None,
                                                              facts={'place_verified': place.get('place_verified'),
                                                                     'release_verified': release_ok, 'retreat_verified': retreat_ok,
                                                                     'status': place.get('status')},
                                                              children=(place_id,), stopped=place.get('stopped_confirmed'),
                                                              independent=place.get('fingerprint'),
                                                              message=place.get('detail', '')))
                if deliver.halted_reason:
                    if deliver.requires_stop_confirmation and place.get('stopped_confirmed'):
                        deliver.confirm_stop(f'{place_id}:stopped_confirmed')
                    raise RoundError(f'execute_place halted: {deliver.halted_reason}')

                deliver.start_step('verify_release_and_settle')
                after = await call(session, 'get_robot_state', {})
                diagnostic = after.get('sources', {}).get('simulation_diagnostic', {}).get('data', {})
                settled = (after.get('holding') is None
                           and diagnostic.get('delivery_support_contact') is True
                           and diagnostic.get('delivery_support_destination_id') == LEFT_DELIVERY_ID)
                deliver.record_result('verify_release_and_settle', result('verify_release_and_settle', SkillStatus.SUCCEEDED if settled else SkillStatus.FAILED,
                                                                          success=settled or None,
                                                                          facts={'holding': after.get('holding'),
                                                                                 'delivery_support_contact': diagnostic.get('delivery_support_contact'),
                                                                                 'destination_id': diagnostic.get('delivery_support_destination_id')}))
                if deliver.halted_reason:
                    raise RoundError(f'verify_release_and_settle halted: {deliver.halted_reason}')

                deliver.start_step('retreat')
                deliver.record_result('retreat', result('retreat', SkillStatus.SUCCEEDED if retreat_ok else SkillStatus.FAILED,
                                                        success=retreat_ok or None,
                                                        facts={'retreat_bundled_in_execute_place': True,
                                                               'post_retreat_placement_verified': retreat_ok},
                                                        independent=place.get('fingerprint')))

                if not deliver.succeeded:
                    raise RoundError(f'deliver_object halted: {deliver.halted_reason or "incomplete"}')

                model.start_step('pick_object')
                model.record_result('pick_object', result('pick_object', SkillStatus.SUCCEEDED, success=True,
                                                          facts={'sub_steps': {k: v.value for k, v in pick.step_status.items()}},
                                                          children=tuple(x for x in (pickup_id, grasp_id) if x)))
                model.start_step('deliver_object')
                model.record_result('deliver_object', result('deliver_object', SkillStatus.SUCCEEDED, success=True,
                                                             facts={'sub_steps': {k: v.value for k, v in deliver.step_status.items()}},
                                                             children=(carry_id, place_id)))

                model.start_step('get_state')
                final = await call(session, 'get_robot_state', {})
                final_diag = final.get('sources', {}).get('simulation_diagnostic', {}).get('data', {})
                final_ok = (final.get('holding') is None and final_diag.get('attached_arm') is None
                            and final_diag.get('delivery_support_destination_id') == LEFT_DELIVERY_ID)
                model.record_result('get_state', result('get_state', SkillStatus.SUCCEEDED if final_ok else SkillStatus.FAILED,
                                                        success=final_ok or None,
                                                        facts={'holding': final.get('holding'),
                                                               'attached_arm': final_diag.get('attached_arm'),
                                                               'destination_id': final_diag.get('delivery_support_destination_id'),
                                                               'object_position_m': final_diag.get('object_pose', {}).get('position_m')}))

                round_result = {'round': round_index, 'run_id': model.request_id,
                                'passed': bool(model.succeeded), 'composite': model.snapshot(),
                                'sub_composites': {'pick_object': pick.snapshot(), 'deliver_object': deliver.snapshot()},
                                'final_state': {'holding': final.get('holding'),
                                                'delivery_support_contact': final_diag.get('delivery_support_contact'),
                                                'destination_id': final_diag.get('delivery_support_destination_id'),
                                                'object_pose': final_diag.get('object_pose'),
                                                'attached_arm': final_diag.get('attached_arm')},
                                'request_ids': {'pickup_nav': pickup_id, 'grasp': grasp_id,
                                                'carry_nav': carry_id, 'place': place_id},
                                'scene_sha256': SCENE_HASH, 'physical_verification': False,
                                'pickup_nav_mode': arguments.pickup_nav,
                                'scope': 'real_MCP_MuJoCo_composite_fetch_and_deliver_NOT_physical'}
                return round_result


def _terminal_results(events):
    """Terminal result per request_id from recorded get_skill_result polls."""
    terminal = {}
    for event in events:
        if event['tool'] != 'get_skill_result':
            continue
        result = event['result']
        request_id = result.get('request_id')
        if request_id and result.get('status') not in (None, 'running'):
            terminal[request_id] = result
    return terminal


def lifecycle_evidence(round_result, output):
    """Build the offline single-red lifecycle transitions from measured round facts."""
    steps = json.loads((output / 'events.json').read_text())
    terminal = _terminal_results(steps)
    ids = round_result.get('request_ids', {})
    grasp = terminal.get(ids.get('grasp'), {})
    place = terminal.get(ids.get('place'), {})
    run_id = round_result['run_id']
    state = initial_single_red_lifecycle(run_id)
    select = select_target(state, EvidenceReference('observe_scene', 'mcp_observe_scene',
                                                    {'camera_fresh': True, 'object_id': TARGET_OBJECT_ID}))
    if round_result.get('pickup_nav_mode', 'on') == 'off':
        approach = confirm_approach(select, EvidenceReference('grasp_internal_alignment', 'mcp_grasp_object',
                                                              {'success': True, 'stopped_confirmed': True,
                                                               'realized_by': 'execute_grasp_internal_base_alignment'}))
    else:
        approach = confirm_approach(select, EvidenceReference('navigate_pickup', 'mcp_navigate_to_target',
                                                              {'success': True, 'stopped_confirmed': True}))
    attached = attach_target(approach, EvidenceReference('execute_grasp', 'mcp_grasp_object',
                                                         {'grasp_precheck_passed': bool(grasp.get('grasp_verified')),
                                                          'transport_safe': bool(grasp.get('transport_safe'))}))
    transport = begin_transport(attached, EvidenceReference('verify_attachment', 'mcp_get_robot_state',
                                                            {'attachment_independently_verified': True}))
    kinds = {event.get('kind') for event in place.get('events', [])}
    placed = confirm_placed(transport, EvidenceReference('execute_place', 'mcp_place_to_affordance',
                                                         {'release_verified': 'actual_release_settled' in kinds,
                                                          'settled_on_registered_surface': 'post_retreat_placement_verified' in kinds,
                                                          'retreat_verified': 'post_retreat_placement_verified' in kinds,
                                                          'destination_id': LEFT_DELIVERY_ID}))
    return {'run_id': run_id, 'phase': placed.phase,
            'transitions': [ev.evidence_id for ev in placed.evidence]}


def main():
    parser = argparse.ArgumentParser(description='Isolated MuJoCo composite fetch_and_deliver closed loop; no hardware')
    parser.add_argument('--rounds', type=int, default=3)
    parser.add_argument('--port', type=int, default=18556)
    parser.add_argument('--object-id', default=TARGET_OBJECT_ID)
    parser.add_argument('--scene-dir', type=Path, default=DEFAULT_SCENE_DIR)
    parser.add_argument('--output-root', type=Path)
    parser.add_argument('--navigate-timeout', type=float, default=90.0,
                        help='gateway requires 0.1-90s')
    parser.add_argument('--grasp-timeout', type=float, default=600.0,
                        help='gateway requires 1-600s')
    parser.add_argument('--place-timeout', type=float, default=600.0,
                        help='gateway requires 1-600s')
    parser.add_argument('--stop-timeout', type=float, default=90.0)
    parser.add_argument('--ready-timeout', type=float, default=240.0)
    parser.add_argument('--round-start', type=int, default=1)
    parser.add_argument('--gui', action='store_true',
                        help='start the isolated stack with the MuJoCo live viewer')
    parser.add_argument('--pickup-nav', choices=['on', 'off'], default='off',
                        help='on: explicit navigate_to_target(pickup_zone) before grasp; off: use the grasp skill internal base alignment (validated pipeline)')
    parser.add_argument('--no-reset-first', action='store_true',
                        help='reuse the currently running (already audited) scene for the first round')
    arguments = parser.parse_args()

    if not 0.1 <= arguments.navigate_timeout <= 90.0:
        raise SystemExit('--navigate-timeout must be within 0.1-90 seconds (gateway limit)')
    for name in ('grasp_timeout', 'place_timeout'):
        if not 1.0 <= getattr(arguments, name) <= 600.0:
            raise SystemExit(f'--{name.replace("_", "-")} must be within 1-600 seconds (gateway limit)')

    arguments.scene_xml = arguments.scene_dir / 'scene.xml'
    arguments.scene_yaml = arguments.scene_dir / 'runtime.yaml'
    arguments.shelf_scene = arguments.scene_dir / 'shelf_scene.yaml'
    for path in (arguments.scene_xml, arguments.scene_yaml, arguments.shelf_scene):
        if not path.exists():
            raise SystemExit(f'missing scene asset: {path}')

    root = arguments.output_root or (ROOT / 'local_dependencies/openflex_roboos/runs' /
                                     datetime.now().strftime('composite_fetch_deliver_%Y%m%d_%H%M%S_%f'))
    root.mkdir(parents=True, exist_ok=True)
    resumed = root / 'summary.json'
    if arguments.output_root and resumed.exists():
        summary = json.loads(resumed.read_text())
        summary['rounds'] = [item for item in summary.get('rounds', [])
                             if item.get('round') not in range(arguments.round_start,
                                                              arguments.round_start + arguments.rounds)]
    else:
        summary = {'scope': 'P4_composite_fetch_and_deliver_isolated_MuJoCo_NOT_physical',
                   'object_id': arguments.object_id, 'destination_id': LEFT_DELIVERY_ID,
                   'affordance_alias': DELIVERY_ALIAS, 'scene_dir': str(arguments.scene_dir),
                   'scene_sha256': SCENE_HASH, 'physical_verification': False,
                   'started_at': datetime.now().astimezone().isoformat(), 'rounds': []}
    save(root / 'summary.json', summary)

    for index in range(arguments.round_start, arguments.round_start + arguments.rounds):
        round_dir = root / f'round_{index:02d}'
        round_dir.mkdir(parents=True, exist_ok=True)
        log_lines = []

        def log(entry):
            log_lines.append(entry)
            save(round_dir / 'reset_log.json', log_lines)

        record = {'round': index, 'dir': str(round_dir), 'passed': False}
        if index == arguments.round_start and arguments.no_reset_first:
            log({'reset_step': 'reuse_running_scene'})
        else:
            try:
                initial = scene_reset(arguments, log)
                save(round_dir / 'initial_state.json', initial)
            except Exception as error:
                record['error'] = f'reset_failed: {error}'
                summary['rounds'].append(record)
                save(root / 'summary.json', summary)
                print(json.dumps(record, ensure_ascii=False))
                continue
        run_id = datetime.now().strftime('composite_%Y%m%d_%H%M%S_%f')
        model = CompositeExecution(spec_for('fetch_and_deliver'), run_id)
        try:
            round_result = asyncio.run(run_round(arguments, round_dir, index, model))
            save(round_dir / 'round.json', round_result)
            if round_result['passed']:
                try:
                    round_result['lifecycle'] = lifecycle_evidence(round_result, round_dir)
                except Exception as error:  # never let evidence assembly flip a real success
                    round_result['lifecycle_error'] = error_detail(error)
                save(round_dir / 'round.json', round_result)
            record.update(passed=round_result['passed'], run_id=run_id,
                          request_ids=round_result['request_ids'])
        except Exception as error:
            record.update(error=error_detail(error), run_id=run_id,
                          composite=model.snapshot(), events=str(round_dir / 'events.json'))
            save(round_dir / 'round.json', record)
        summary['rounds'].append(record)
        save(root / 'summary.json', summary)
        print(json.dumps(record, ensure_ascii=False))

    passed = sum(1 for item in summary['rounds'] if item.get('passed'))
    summary['passed_rounds'] = passed
    summary['total_rounds'] = len(summary['rounds'])
    summary['finished_at'] = datetime.now().astimezone().isoformat()
    save(root / 'summary.json', summary)
    print(json.dumps({'output': str(root), 'passed': passed, 'total': len(summary['rounds'])}, ensure_ascii=False))
    return 0 if passed == len(summary['rounds']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
