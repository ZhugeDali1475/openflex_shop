import math
import os
import threading
import time

from action_msgs.msg import GoalStatus
from openflex_thesis_msgs.msg import ControlMode, FixedPlaceControl, PlaceStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from openflex_thesis_mujoco.release_verification import object_settled_on_surface

from .action_journal import cancel_to_hold_and_audit


class PlaceSkill:
    def __init__(self, node):
        if os.environ.get('OPENFLEX_ENABLE_SIM_PLACE') != '1' or node.manipulation is None:
            raise RuntimeError('Separate explicit isolated simulation place opt-in required')
        self.node, self.motion = node, node.manipulation
        self.publisher = node.create_publisher(FixedPlaceControl, '/thesis/place/control', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.listener = node.create_subscription(PlaceStatus, '/thesis/place/status',
            lambda message: node.receive('place', message), qos_profile_sensor_data, callback_group=node.group)

    def submit(self, arguments):
        if (set(arguments) != {'object_id', 'affordance_id', 'request_id', 'timeout_sec'}
                or arguments['object_id'] != 'thesis_item_01' or arguments['affordance_id'] != 'delivery_zone'):
            raise ValueError('Only the registered single object and delivery table are supported')
        timeout = arguments['timeout_sec']
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or not 1 <= timeout <= 600:
            raise ValueError('timeout_sec must be a finite JSON number in 1-600')
        arguments = dict(arguments, timeout_sec=float(timeout))
        record, accepted = self.node.store.begin(arguments['request_id'], 'place_to_affordance', arguments)
        if accepted:
            threading.Thread(target=self.execute, args=(arguments,), daemon=True).start()
        return record

    def placement_still_verified(self, target):
        diagnostic = self.node.healthy(allow_attached=False)
        affordance = self.node.registry['affordances']['delivery_zone']
        surface = affordance['surface_center_xyz']
        width, depth = affordance['surface_dimensions_xy']
        return object_settled_on_surface(diagnostic['object_pose']['position_m'],
            diagnostic['object_pose']['orientation_xyzw'], [0.035, 0.03, 0.08], target,
            [surface[0], surface[1], surface[2] - 0.02], [width / 2, depth / 2, 0.02],
            diagnostic['object_velocity_linear_angular'], diagnostic['delivery_support_contact'],
            diagnostic['attached_arm'] is not None,
            dict(zip(self.node.sample('joints').name, self.node.sample('joints').position))['openarmx_right_finger_joint1'] >= 0.03)

    def execute(self, arguments):
        request_id = arguments['request_id']
        self.motion.deadline = time.monotonic() + arguments['timeout_sec']
        status, success, detail, stopped, verified, uncertain = 'failed', False, '', False, False, False
        command = FixedPlaceControl()
        command.object_id, command.active_arm = arguments['object_id'], 2
        command.header.frame_id = command.delivery_pose.header.frame_id = 'map'
        command.reason = request_id
        try:
            self.motion.gate(attached=True)
            if not self.motion.retained():
                raise RuntimeError('Fresh independently retained object required before placing')
            self.node.set_mode(ControlMode.HOLD)
            if not self.node.stopped() or not self.motion.joints_stopped():
                raise RuntimeError('Measured base and arm stop required before place planning')
            planned, _ = self.motion.probe(request_id, 'probe_openflex_place_trajectory.py', ['--search-base'])
            if not planned.get('place_paths_valid'):
                raise RuntimeError(f'Complete carried-object place path rejected: {planned.get("blocked_at")}')
            approach = self.node.execute({'request_id': request_id,
                'timeout_sec': min(90, self.motion.deadline - time.monotonic())},
                target_override=planned['selected_base_xyyaw'], finalize=False, allow_attached=True)
            if not approach['success']:
                if approach['status'] == 'canceled':
                    raise InterruptedError('Place approach canceled')
                if approach['status'] == 'timed_out':
                    raise TimeoutError('Place approach timed out')
                uncertain = approach['status'] == 'unknown'
                raise RuntimeError(f'Place approach failed: {approach}')
            acquired = self.node.set_mode(ControlMode.MOVEIT)
            self.motion.base_lock_epoch = acquired['epoch']
            self.motion.base_lock = self.node.base_pose()
            fresh, _ = self.motion.probe(request_id, 'probe_openflex_place_trajectory.py', [])
            if not fresh.get('place_paths_valid'):
                raise RuntimeError(f'Measured-base place path rejected: {fresh.get("blocked_at")}')
            for phase in fresh['phases']:
                self.motion.execute_trajectory(request_id, phase, True)
            if not self.motion.retained():
                raise RuntimeError('Object lost before authorized release')
            self.motion.base_lock = None
            release_control = self.node.set_mode(ControlMode.PLACE)
            command.enabled = True
            command.header.stamp = command.delivery_pose.header.stamp = self.node.get_clock().now().to_msg()
            target = list(fresh['release_center_map'])
            target[2] -= 0.015
            command.delivery_pose.pose.position.x, command.delivery_pose.pose.position.y, command.delivery_pose.pose.position.z = target
            command.delivery_pose.pose.orientation.w = 1.0
            self.publisher.publish(command)
            self.node.store.event(request_id, 'actual_pose_release_requested', {'target_map': target, 'teleportation': False})
            release_deadline = min(self.motion.deadline, time.monotonic() + 12)
            while time.monotonic() < release_deadline:
                self.motion.gate(attached=True)
                self.node.require_control_ack(ControlMode.PLACE, release_control['epoch'])
                if not self.node.stopped(timeout=0.6):
                    raise RuntimeError('Exclusive release mode or base-stop revoked')
                try:
                    observed = self.node.sample('place')
                except RuntimeError:
                    time.sleep(0.02)
                    continue
                stamp = observed.header.stamp
                if observed.object_id != command.object_id or (stamp.sec, stamp.nanosec) < (command.header.stamp.sec, command.header.stamp.nanosec):
                    time.sleep(0.02)
                    continue
                if observed.failure_code:
                    raise RuntimeError(f'Release backend rejected: {observed.failure_code}: {observed.detail}')
                if observed.target_reached and observed.released and observed.verified:
                    verified = True
                    self.node.store.event(request_id, 'actual_release_settled', {'detail': observed.detail, 'physical_verification': False})
                    break
                time.sleep(0.02)
            if not verified:
                raise TimeoutError('Actual contact and settled release not verified before deadline')
            self.motion.released_target = target
            acquired = self.node.set_mode(ControlMode.MOVEIT)
            self.motion.base_lock_epoch = acquired['epoch']
            self.motion.base_lock = self.node.base_pose()
            retreat, _ = self.motion.probe(request_id, 'probe_openflex_place_trajectory.py', ['--retreat'])
            if not retreat.get('place_paths_valid'):
                raise RuntimeError(f'Post-release robot retreat rejected: {retreat.get("blocked_at")}')
            for phase in retreat['phases']:
                self.motion.execute_trajectory(request_id, phase, False)
            self.motion.gate(attached=False)
            if not self.placement_still_verified(target):
                raise RuntimeError('Fresh post-retreat object support, pose, openness or velocity disagrees with placement')
            self.node.store.event(request_id, 'post_retreat_placement_verified', {'target_map': target})
            status, success = 'succeeded', True
        except InterruptedError as error:
            status, detail = 'canceled', str(error)
        except TimeoutError as error:
            status, detail = 'timed_out', str(error)
        except Exception as error:
            detail = str(error)
            uncertain |= 'Unknown goal acceptance' in detail
        finally:
            self.motion.released_target = None
            try:
                command.enabled = False
                self.publisher.publish(command)
                cancel_to_hold_and_audit(self.node, request_id, self.motion.handles, ControlMode.HOLD)
                stopped = self.node.stopped() and self.motion.joints_stopped()
                if not stopped or uncertain:
                    status, success = 'unknown', False
            except Exception as error:
                status, success = 'unknown', False
                detail += f'; terminal stop audit failed: {error}'
            self.motion.handles.clear()
            self.motion.base_lock = None
            self.motion.base_lock_epoch = None
            self.node.store.finish(request_id, status, success=success and stopped, detail=detail,
                stopped_confirmed=stopped, place_verified=verified, physical_verification=False,
                backend='actual_MuJoCo_pose_release_contact_settling_NOT_PHYSICAL')
