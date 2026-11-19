import math

import mujoco
import numpy as np


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


# Mirrors the deployed simulator base controller: openflex_thesis_mujoco
# sim_navigation_node.velocity() as launched by scripts/run_openflex_sim_navigation.sh,
# the 50 Hz velocity arbiter limits and the bridge's first-order twist lag.
BASE_CONTROLLER = {
    "linear_gain": 0.9,
    "angular_gain": 1.5,
    "max_linear_mps": 0.15,
    "max_angular_rps": 0.25,
    "position_tolerance_m": 0.008,
    "yaw_tolerance_rad": 0.01,
    "velocity_tau_sec": 0.08,
    "simulation_step_sec": 0.01,
    "leg_timeout_sec": 30.0,
}


class NavigationGuard:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.robot = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "openflex_robot_yaw_root")
        robot_roots = {self.robot, 0}
        excluded = {"thesis_object", "thesis_occluder", "floor"}
        self.environment = {
            body_id for body_id in range(1, self.model.nbody)
            if int(self.model.body_rootid[body_id]) not in robot_roots
            and mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) not in excluded
        }
        self.base_offsets = [self.offset(name) for name in ("base_x", "base_y", "base_yaw")]

    def offset(self, name):
        joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint < 0:
            raise ValueError(f"Missing model joint: {name}")
        return self.model.jnt_qposadr[joint]

    def _attachment(self, payload):
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
        return (hand, body, address, hand_rotation.T @ (position - self.data.xpos[hand]),
                hand_rotation.T @ matrix.reshape(3, 3))

    def _sample(self, pose, attachment):
        """Place the base (and rigid payload) at one posture and report a real contact."""
        self.data.qpos[self.base_offsets] = pose
        mujoco.mj_forward(self.model, self.data)
        body = None
        if attachment is not None:
            hand, body, address, offset, relative_rotation = attachment
            hand_rotation = self.data.xmat[hand].reshape(3, 3)
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, (hand_rotation @ relative_rotation).reshape(9))
            self.data.qpos[address:address + 7] = [*(self.data.xpos[hand] + hand_rotation @ offset), *quaternion]
            mujoco.mj_forward(self.model, self.data)
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
                return {"accepted": False, "reason": "robot_environment_contact",
                        "distance_m": float(contact.dist), "bodies": [
                            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                            for body_id in (first, second)]}
        return None

    def _controller_path(self, start, target, attachment):
        """Replay the deployed base controller law so the preflight predicts the real path.

        The simulator integrates commanded body-frame velocity in the world frame with a
        first-order lag, so an unmodelled yaw wrap can curve the base into scenery. This
        walk uses the same law, tolerances and limits as the running nodes.
        """
        config = BASE_CONTROLLER
        step = float(config["simulation_step_sec"])
        x, y, yaw = (float(value) for value in start)
        target = [float(value) for value in target]
        twist = [0.0, 0.0, 0.0]
        samples = 0
        elapsed = 0.0
        alpha = 1.0 - math.exp(-step / max(1.0e-4, float(config["velocity_tau_sec"])))
        while elapsed <= float(config["leg_timeout_sec"]):
            dx, dy = target[0] - x, target[1] - y
            distance = math.hypot(dx, dy)
            body_x = math.cos(yaw) * dx + math.sin(yaw) * dy
            body_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
            yaw_error = wrap_angle(target[2] - yaw)
            if distance <= float(config["position_tolerance_m"]):
                body_x = body_y = 0.0
            if abs(yaw_error) <= float(config["yaw_tolerance_rad"]):
                yaw_error = 0.0
            command = [
                max(-float(config["max_linear_mps"]), min(float(config["max_linear_mps"]),
                                                         float(config["linear_gain"]) * body_x)),
                max(-float(config["max_linear_mps"]), min(float(config["max_linear_mps"]),
                                                         float(config["linear_gain"]) * body_y)),
                max(-float(config["max_angular_rps"]), min(float(config["max_angular_rps"]),
                                                          float(config["angular_gain"]) * yaw_error)),
            ]
            twist = [value + alpha * (wanted - value) for value, wanted in zip(twist, command)]
            x += step * (math.cos(yaw) * twist[0] - math.sin(yaw) * twist[1])
            y += step * (math.sin(yaw) * twist[0] + math.cos(yaw) * twist[1])
            yaw = wrap_angle(yaw + step * twist[2])
            elapsed += step
            samples += 1
            violation = self._sample([x, y, yaw], attachment)
            if violation is not None:
                return violation, samples, False
            if (distance <= float(config["position_tolerance_m"])
                    and abs(yaw_error) <= float(config["yaw_tolerance_rad"])):
                return None, samples, True
        return ({"accepted": False, "reason": "controller_path_did_not_converge",
                 "timeout_sec": float(config["leg_timeout_sec"]),
                 "pose": [x, y, yaw], "target": list(target)}, samples, False)

    def check(self, joint_states, start, target, payload=None, via=None):
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
        attachment = self._attachment(payload) if payload is not None else None
        if via is None:
            waypoints = [[float(target[0]), float(target[1]), float(start[2])], target.tolist()]
        else:
            waypoints = []
            for waypoint in via:
                waypoint = np.asarray(waypoint, dtype=float)
                if waypoint.shape != (3,) or not np.isfinite(waypoint).all():
                    raise ValueError("Expected finite navigation via waypoint")
                waypoints.append(waypoint.tolist())
            waypoints.append(target.tolist())
        previous = start
        samples = 0
        for waypoint in waypoints:
            end = np.asarray(waypoint)
            delta = end - previous
            delta[2] = angle_difference(end[2], previous[2])
            count = max(1, math.ceil(np.linalg.norm(delta[:2]) / 0.01), math.ceil(abs(delta[2]) / 0.02))
            for amount in np.linspace(0, 1, count + 1):
                samples += 1
                violation = self._sample(previous + delta * amount, attachment)
                if violation is not None:
                    return {"accepted": False, "samples": samples,
                            **{key: violation[key] for key in ("reason", "distance_m", "bodies")}}
            previous = end
        controller_samples = 0
        previous = start
        for waypoint in waypoints:
            violation, walked, converged = self._controller_path(previous, waypoint, attachment)
            controller_samples += walked
            if violation is not None:
                return {"accepted": False, "samples": samples + controller_samples,
                        "controller_path_samples": controller_samples,
                        **{key: violation[key] for key in violation if key != "accepted"}}
            if not converged:
                return {"accepted": False, "samples": samples + controller_samples,
                        "controller_path_samples": controller_samples,
                        "reason": "controller_path_did_not_converge", "waypoints": waypoints}
            previous = np.asarray(waypoint)
        return {"accepted": True, "samples": samples + controller_samples,
                "controller_path_samples": controller_samples, "waypoints": waypoints,
                "scope": ("measured_rigid_payload_static_scene_sampled_translation_then_rotation" if payload is not None
                          else "empty_robot_static_scene_sampled_translation_then_rotation"),
                "not_continuous_collision_proof": True, "physical_navigation_validated": False}
