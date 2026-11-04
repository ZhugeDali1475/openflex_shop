#!/usr/bin/env python3

"""Wait for MoveIt's planning-scene service before starting clients."""

import argparse
import sys
import time

import rclpy
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    rclpy.init()
    node = Node("wait_for_moveit")
    client = node.create_client(GetPlanningScene, "/get_planning_scene")
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=0.25):
                node.get_logger().info("MoveIt planning scene service is ready")
                return 0
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().error(
            f"Timed out waiting for MoveIt planning scene service ({args.timeout:.1f}s)"
        )
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
