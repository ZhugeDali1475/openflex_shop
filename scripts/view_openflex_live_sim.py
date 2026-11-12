#!/usr/bin/env python3
import json
import math
import os
from pathlib import Path
import time

import mujoco
import mujoco.viewer
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]


def viewer_period(value):
    rate = float(value)
    if not math.isfinite(rate) or not 5 <= rate <= 60:
        raise ValueError('Read-only viewer FPS must be finite and between 5 and 60')
    return 1.0 / rate


class LiveView(Node):
    def __init__(self):
        super().__init__('openflex_live_readonly_viewer', start_parameter_services=False)
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / 'local_dependencies/openflex_roboos/readonly_scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.odometry = {}
        self.frames = {}
        self.last_sample = 0.0
        self.last_diagnostic = None
        self.frame_count = 0
        self.latest_diagnostic_stamp = None
        self.create_subscription(Odometry, '/swerve_drive_controller/odom', self.receive_odometry, qos_profile_sensor_data)
        self.create_subscription(String, '/thesis_sim/diagnostic_state', self.receive_diagnostic, qos_profile_sensor_data)

    def receive_odometry(self, message):
        self.odometry[(message.header.stamp.sec, message.header.stamp.nanosec)] = message
        if len(self.odometry) > 60:
            self.odometry.pop(next(iter(self.odometry)))

    def receive_diagnostic(self, message):
        try:
            diagnostic = json.loads(message.data)
            if diagnostic['source'] != 'mujoco_diagnostic_only_NOT_PHYSICAL':
                return
            stamp = diagnostic['stamp']
            key = (stamp['sec'], stamp['nanosec'])
            if self.latest_diagnostic_stamp is not None and key < self.latest_diagnostic_stamp:
                self.frames.clear()
                self.odometry.clear()
            self.latest_diagnostic_stamp = key
            self.frames[key] = diagnostic
            if len(self.frames) > 60:
                self.frames.pop(next(iter(self.frames)))
        except (ValueError, KeyError, TypeError):
            return

    def update(self):
        matching = self.odometry.keys() & self.frames.keys()
        if not matching:
            return
        stamp = max(matching)
        diagnostic, odometry = self.frames.pop(stamp), self.odometry.pop(stamp)
        self.frames = {key: value for key, value in self.frames.items() if key > stamp}
        self.odometry = {key: value for key, value in self.odometry.items() if key > stamp}
        joints = diagnostic['joint_state']
        positions = dict(zip(joints['names'], joints['position_rad_or_m']))
        orientation = odometry.pose.pose.orientation
        yaw = math.atan2(2 * orientation.w * orientation.z, 1 - 2 * orientation.z ** 2)
        positions.update(base_x=odometry.pose.pose.position.x, base_y=odometry.pose.pose.position.y, base_yaw=yaw)
        for name, position in positions.items():
            joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint >= 0:
                self.data.qpos[self.model.jnt_qposadr[joint]] = position
        joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'thesis_object_free')
        address = self.model.jnt_qposadr[joint]
        pose = diagnostic['object_pose']
        rotation = pose['orientation_xyzw']
        self.data.qpos[address:address + 7] = [*pose['position_m'], rotation[3], *rotation[:3]]
        self.data.time = diagnostic['simulation_time_sec']
        mujoco.mj_forward(self.model, self.data)
        self.last_sample, self.last_diagnostic = time.monotonic(), diagnostic
        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info('Live GUI synchronized to measured ROS state; read-only viewer, no second physics loop or commands')


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '83':
        raise RuntimeError('Read-only live viewer requires isolated ROS domain 83')
    rclpy.init()
    period = viewer_period(os.environ.get('OPENFLEX_VIEWER_FPS', '30'))
    node = LiveView()
    try:
        with mujoco.viewer.launch_passive(node.model, node.data, show_left_ui=False, show_right_ui=False) as viewer:
            viewer.cam.lookat[:] = [0.0, -0.15, 0.75]
            viewer.cam.distance = 3.8
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
            while viewer.is_running() and rclpy.ok():
                frame_started = time.monotonic()
                for callback_index in range(16):
                    rclpy.spin_once(node, timeout_sec=0.0)
                with viewer.lock():
                    node.update()
                fresh = time.monotonic() - node.last_sample <= 0.5
                diagnostic = node.last_diagnostic or {}
                viewer.set_texts((None, None,
                    'OpenFlex LIVE / read-only ROS mirror\nState\nSim time\nHolding\nTable contact\nSafety stop',
                    f'ROS domain 83 / NOT PHYSICAL\n{"LIVE" if fresh else "STALE - frozen, not current"}\n'
                    f'{node.data.time:.2f}s\n{diagnostic.get("attached_arm")}\n'
                    f'{diagnostic.get("delivery_support_contact")}\n{diagnostic.get("safety_stop")}'))
                viewer.sync()
                time.sleep(max(0.0, period - (time.monotonic() - frame_started)))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
