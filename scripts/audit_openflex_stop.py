#!/usr/bin/env python3
import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'openflex_ws/src/openflex_thesis/openflex_roboos_bridge'))

from openflex_runtime import measured_hold, request_json, save


ENDPOINTS = {
    '/navigate_to_pose': 'navigation',
    '/lift_joint_controller/follow_joint_trajectory': 'trajectory',
    '/right_joint_trajectory_controller/follow_joint_trajectory': 'trajectory',
    '/right_gripper_controller/gripper_cmd': 'gripper',
}


def gap_summary(samples):
    gaps = [following - previous for previous, following in zip(samples, samples[1:])]
    ordered = sorted(gaps)
    return {'messages': len(samples), 'max_gap_seconds': max(gaps) if gaps else None,
            'p95_gap_seconds': ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)] if ordered else None,
            'gaps_over_0_5_seconds': sum(value > 0.5 for value in gaps)}


def terminal_evidence(operation, results):
    from openflex_roboos_bridge.action_journal import goal_receipt_summary

    durable = goal_receipt_summary(operation)
    queried = [{'endpoint': item['endpoint'], 'goal_uuid': item['goal_uuid'], 'status': item['status'],
                'terminal_confirmed': item['status'] in (4, 5, 6)} for item in results]
    return {'durable_receipts': durable, 'read_only_result_queries': queried,
            'unknown_result_queries': sum(item['status'] == 0 for item in queried),
            'historical_status_is_not_a_fresh_result_response': True,
            'safe_to_restart': False, 'journal_modified': False,
            'explanation': 'This read-only audit never unlocks. Missing UUIDs, expired results or latched status alone '
                           'cannot reconstruct the original request terminal contract.'}


def run(arguments):
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Read-only audit must explicitly use ROS_DOMAIN_ID=83')
    if not math.isfinite(arguments.duration) or not 3 <= arguments.duration <= 300:
        raise ValueError('Audit duration must be 3-300 seconds')
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from control_msgs.action import FollowJointTrajectory, GripperCommand
    from nav2_msgs.action import NavigateToPose
    from openflex_thesis_msgs.msg import ControlMode
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_action_status_default, qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    before = request_json('http://127.0.0.1:18553/status', {'request_id': arguments.request_id})
    if before.get('request_id') != arguments.request_id or before.get('status') != 'unknown':
        raise RuntimeError('Audit requires the original existing UNKNOWN request')
    save(output / 'operation_before.json', before)
    rclpy.init()
    node = Node('openflex_read_only_stop_audit', start_parameter_services=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    lock = threading.RLock()
    receipts = {'mode': [], 'diagnostic': [], 'clock': []}
    statuses = {}
    current_clock = [None]
    diagnostics = []
    subscriptions = []
    stream = (output / 'telemetry_trace.jsonl').open('w')
    started = time.monotonic()

    def telemetry(key, message):
        received = time.monotonic()
        with lock:
            if key == 'clock':
                stamp = message.clock
                current_clock[0] = stamp.sec + stamp.nanosec / 1e9
                record = {'simulation_clock_sec': current_clock[0]}
            elif key == 'mode':
                stamp = message.header.stamp
                record = {'mode': int(message.mode), 'epoch': int(message.epoch),
                          'inputs_healthy': bool(message.inputs_healthy),
                          'source_stamp_sec': stamp.sec + stamp.nanosec / 1e9}
            else:
                try:
                    data = json.loads(message.data)
                    stamp = data['stamp']
                    record = {'source_stamp_sec': stamp['sec'] + stamp['nanosec'] / 1e9,
                              'control_ack': data.get('control_ack'), 'safety_stop': data.get('safety_stop'),
                              'execution_state': data.get('execution_state')}
                    diagnostics.append(record)
                except (ValueError, TypeError, KeyError):
                    record = {'invalid_diagnostic': True}
            if key != 'clock' and 'source_stamp_sec' in record:
                record['source_age_in_observer_clock_sec'] = (None if current_clock[0] is None
                    else current_clock[0] - record['source_stamp_sec'])
            receipts[key].append(received)
            stream.write(json.dumps({'topic': key, 'elapsed_seconds': received - started, **record}) + '\n')

    subscriptions.append(node.create_subscription(Clock, '/clock', lambda msg: telemetry('clock', msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(ControlMode, '/thesis/control_mode', lambda msg: telemetry('mode', msg), qos_profile_action_status_default))
    subscriptions.append(node.create_subscription(String, '/thesis_sim/diagnostic_state', lambda msg: telemetry('diagnostic', msg), qos_profile_sensor_data))
    for endpoint in ENDPOINTS:
        def receive_status(message, endpoint=endpoint):
            with lock:
                statuses[endpoint] = {'received_elapsed_seconds': time.monotonic() - started,
                    'latched_snapshot_only': True,
                    'goals': [{'goal_uuid': bytes(item.goal_info.goal_id.uuid).hex(), 'status': int(item.status),
                               'stamp': {'sec': item.goal_info.stamp.sec, 'nanosec': item.goal_info.stamp.nanosec}}
                              for item in message.status_list]}

        subscriptions.append(node.create_subscription(GoalStatusArray, endpoint + '/_action/status',
                                                       receive_status, qos_profile_action_status_default))
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    snapshots = []
    queried = []
    try:
        while time.monotonic() - started < arguments.duration:
            try:
                value = request_json('http://127.0.0.1:18552/state')
                save(output / f'snapshot_{len(snapshots):04d}.json', value)
                diagnostic = value['sources']['simulation_diagnostic'].get('data') or {}
                snapshots.append({'elapsed_seconds': time.monotonic() - started,
                                  'measured_hold': measured_hold(value), 'stamp': diagnostic.get('stamp'),
                                  'control_ack': diagnostic.get('control_ack'),
                                  'attached_arm': diagnostic.get('attached_arm')})
            except Exception as error:
                snapshots.append({'elapsed_seconds': time.monotonic() - started,
                                  'measured_hold': False, 'error': str(error)})
            time.sleep(0.1)
        with lock:
            status_copy = dict(statuses)
        save(output / 'action_status_latched.json', status_copy)
        for endpoint, kind in ENDPOINTS.items():
            action = {'navigation': NavigateToPose, 'trajectory': FollowJointTrajectory, 'gripper': GripperCommand}[kind]
            service = action.Impl.GetResultService
            client = node.create_client(service, endpoint + '/_action/get_result')
            identifiers = {goal['goal_uuid'] for goal in status_copy.get(endpoint, {}).get('goals', [])}
            for event in before.get('events', []):
                if event.get('kind') == 'controller_goal_intent' and event['data'].get('endpoint') == endpoint:
                    identifiers.add(event['data']['goal_uuid'])
            for identifier in sorted(identifiers):
                record = {'endpoint': endpoint, 'goal_uuid': identifier, 'status': None}
                if client.wait_for_service(timeout_sec=1):
                    request = service.Request()
                    request.goal_id.uuid = list(bytes.fromhex(identifier))
                    future = client.call_async(request)
                    deadline = time.monotonic() + 2
                    while not future.done() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    if future.done() and future.exception() is None:
                        response = future.result()
                        record.update(status=int(response.status), result=str(response.result))
                queried.append(record)
                save(output / 'action_results.json', queried)
            node.destroy_client(client)
        after = request_json('http://127.0.0.1:18553/status', {'request_id': arguments.request_id})
        save(output / 'operation_after.json', after)
        with lock:
            report = {'created_at': datetime.now().astimezone().isoformat(), 'read_only': True,
                'physical_verification': False, 'request_id': arguments.request_id,
                'observation_context': 'Current HOLD only; cannot diagnose the earlier moving failure retroactively',
                'operation_unchanged': before == after, 'operation_status': after.get('status'),
                'all_samples_measured_hold': bool(snapshots) and all(item['measured_hold'] for item in snapshots),
                'snapshots': snapshots, 'telemetry': {key: gap_summary(values) for key, values in receipts.items()},
                'stale_executor_ack_samples': sum((record.get('control_ack') or {}).get('fresh') is not True for record in diagnostics),
                'terminal_evidence': terminal_evidence(before, queried)}
        save(output / 'audit.json', report)
        print(json.dumps({'output': str(output), 'operation_unchanged': report['operation_unchanged'],
                          'all_samples_measured_hold': report['all_samples_measured_hold'],
                          'telemetry': report['telemetry'],
                          'terminal_evidence': report['terminal_evidence']}, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown(timeout_sec=3)
        spin.join(timeout=3)
        stream.close()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Only query UNKNOWN operations, snapshots, ROS status and result services; never command or unlock')
    parser.add_argument('--request-id', required=True)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    run(parser.parse_args())


if __name__ == '__main__':
    main()
