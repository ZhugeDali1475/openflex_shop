import math

import mujoco
import numpy as np


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class NavigationGuard:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.robot = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "openflex_robot_yaw_root")
        self.environment = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                            for name in ("thesis_shelf", "thesis_delivery_table")}
        self.base_offsets = [self.offset(name) for name in ("base_x", "base_y", "base_yaw")]

    def offset(self, name):
        joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint < 0:
            raise ValueError(f"Missing model joint: {name}")
        return self.model.jnt_qposadr[joint]

    def check(self, joint_states, start, target, payload=None):
        if len(joint_states.name) != len(joint_states.position):
            raise ValueError("Invalid joint state lengths")
        mujoco.mj_resetData(self.model, self.data)
        measured = dict(zip(joint_states.name, joint_states.position))
        for joint in range(self.model.njnt):
            if self.model.jnt_type[joint] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
            if name in ("base_x", "base_y", "base_yaw"):
                continue
            if name not in measured or not math.isfinite(measured[name]):
                raise ValueError(f"Missing finite measured joint: {name}")
            self.data.qpos[self.model.jnt_qposadr[joint]] = measured[name]
        start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
        if start.shape != (3,) or target.shape != (3,) or not np.isfinite([start, target]).all():
            raise ValueError("Expected finite base poses")
        self.data.qpos[self.base_offsets] = start
        mujoco.mj_forward(self.model, self.data)
        attachment = None
        if payload is not None:
            if payload.get('frame_id') != 'map':
                raise ValueError('Measured payload must be in map')
            position = np.asarray(payload['position_m'], dtype=float)
            rotation = np.asarray(payload['orientation_xyzw'], dtype=float)
            if (position.shape != (3,) or rotation.shape != (4,) or not np.isfinite(position).all()
                    or not np.isfinite(rotation).all() or abs(np.linalg.norm(rotation) - 1) > 0.001):
                raise ValueError('Expected finite measured payload pose with unit quaternion')
            hand = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'openarmx_right_hand')
            body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'thesis_object')
            address = self.offset('thesis_object_free')
            matrix = np.empty(9)
            mujoco.mju_quat2Mat(matrix, np.array([rotation[3], *rotation[:3]]))
            hand_rotation = self.data.xmat[hand].reshape(3, 3)
            attachment = (hand, body, address, hand_rotation.T @ (position - self.data.xpos[hand]),
                          hand_rotation.T @ matrix.reshape(3, 3))
        waypoints = [[float(target[0]), float(target[1]), float(start[2])], target.tolist()]
        previous = start
        samples = 0
        for waypoint in waypoints:
            end = np.asarray(waypoint)
            delta = end - previous
            delta[2] = angle_difference(end[2], previous[2])
            count = max(1, math.ceil(np.linalg.norm(delta[:2]) / 0.01), math.ceil(abs(delta[2]) / 0.02))
            for amount in np.linspace(0, 1, count + 1):
                self.data.qpos[self.base_offsets] = previous + delta * amount
                mujoco.mj_forward(self.model, self.data)
                if attachment is not None:
                    hand, body, address, offset, relative_rotation = attachment
                    hand_rotation = self.data.xmat[hand].reshape(3, 3)
                    quaternion = np.empty(4)
                    mujoco.mju_mat2Quat(quaternion, (hand_rotation @ relative_rotation).reshape(9))
                    self.data.qpos[address:address + 7] = [*(self.data.xpos[hand] + hand_rotation @ offset), *quaternion]
                    mujoco.mj_forward(self.model, self.data)
                samples += 1
                for contact in self.data.contact[:self.data.ncon]:
                    first, second = (int(self.model.geom_bodyid[index]) for index in (contact.geom1, contact.geom2))
                    forbidden = ((self.model.body_rootid[first] == self.robot and second in self.environment)
                                 or (self.model.body_rootid[second] == self.robot and first in self.environment))
                    if attachment is not None:
                        touch_links = {'openarmx_right_hand', 'openarmx_right_left_finger', 'openarmx_right_right_finger'}
                        for robot_body, obstacle in ((first, second), (second, first)):
                            if self.model.body_rootid[robot_body] == self.robot and obstacle == 0:
                                forbidden |= not mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, robot_body).endswith('_wheel_link')
                        if body in (first, second):
                            other = second if first == body else first
                            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, other)
                            forbidden |= other in self.environment | {0}
                            forbidden |= self.model.body_rootid[other] == self.robot and name not in touch_links
                    if contact.dist <= 0 and forbidden:
                        return {"accepted": False, "samples": samples, "reason": "robot_environment_contact",
                                "distance_m": float(contact.dist), "bodies": [
                                    mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body)
                                    for body in (first, second)]}
            previous = end
        return {"accepted": True, "samples": samples, "waypoints": waypoints,
                "scope": ("measured_rigid_payload_static_scene_sampled_translation_then_rotation" if payload is not None
                          else "empty_robot_static_scene_sampled_translation_then_rotation"),
                "not_continuous_collision_proof": True, "physical_navigation_validated": False}
