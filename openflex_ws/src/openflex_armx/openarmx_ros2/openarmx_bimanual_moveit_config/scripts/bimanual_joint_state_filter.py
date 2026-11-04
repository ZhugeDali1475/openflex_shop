#!/usr/bin/env python3

"""Publish only joints represented by the bimanual MoveIt robot model."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


ARM_JOINTS = {
    *(f"openarmx_left_joint{index}" for index in range(1, 8)),
    *(f"openarmx_right_joint{index}" for index in range(1, 8)),
    "openarmx_left_finger_joint1",
    "openarmx_right_finger_joint1",
}


def filtered_joint_state(message: JointState) -> JointState:
    indices = [index for index, name in enumerate(message.name) if name in ARM_JOINTS]

    result = JointState()
    result.header = message.header
    result.name = [message.name[index] for index in indices]

    for field in ("position", "velocity", "effort"):
        values = getattr(message, field)
        if values:
            if len(values) != len(message.name):
                raise ValueError(
                    f"JointState {field} has {len(values)} values for "
                    f"{len(message.name)} names"
                )
            setattr(result, field, [values[index] for index in indices])

    return result


class BimanualJointStateFilter(Node):
    def __init__(self) -> None:
        super().__init__("bimanual_joint_state_filter")
        self.publisher = self.create_publisher(JointState, "/bimanual_joint_states", 10)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            qos_profile_sensor_data,
        )

    def _on_joint_state(self, message: JointState) -> None:
        try:
            filtered = filtered_joint_state(message)
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return

        if filtered.name:
            self.publisher.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BimanualJointStateFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
