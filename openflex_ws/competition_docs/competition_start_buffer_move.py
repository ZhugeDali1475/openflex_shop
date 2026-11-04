#!/usr/bin/env python3
"""
比赛启动缓冲移动节点

功能：在比赛开始时，机器人从START点直接前进到PRE_START点，避开后方围栏

使用方法：
    ros2 run <package_name> competition_start_buffer_move.py

或直接运行：
    python3 competition_start_buffer_move.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class CompetitionStartBufferMove(Node):
    """比赛启动缓冲移动控制器"""

    def __init__(self):
        super().__init__('competition_start_buffer_move')

        # 参数配置
        self.declare_parameter('start_y', -0.278702)
        self.declare_parameter('target_distance', 0.30)  # 前进30cm
        self.declare_parameter('linear_speed', 0.05)  # 5cm/s
        self.declare_parameter('timeout_sec', 15.0)  # 超时15秒
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')

        self.start_y = self.get_parameter('start_y').value
        self.target_distance = self.get_parameter('target_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.timeout = self.get_parameter('timeout_sec').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        # 发布速度命令
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # 订阅里程计
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            qos
        )

        # 状态变量
        self.current_y = None
        self.initial_y = None
        self.distance_moved = 0.0
        self.is_moving = False
        self.start_time = None

        self.get_logger().info('=' * 60)
        self.get_logger().info('比赛启动缓冲移动节点已启动')
        self.get_logger().info(f'起点Y坐标: {self.start_y:.4f}m')
        self.get_logger().info(f'目标移动距离: {self.target_distance:.3f}m')
        self.get_logger().info(f'前进速度: {self.linear_speed:.3f}m/s')
        self.get_logger().info(f'超时设置: {self.timeout:.1f}秒')
        self.get_logger().info('=' * 60)
        self.get_logger().info('等待里程计数据...')

    def odom_callback(self, msg):
        """里程计回调"""
        self.current_y = msg.pose.pose.position.y

        # 记录初始位置
        if self.initial_y is None:
            self.initial_y = self.current_y
            self.get_logger().info(f'✅ 里程计已就绪，当前Y坐标: {self.current_y:.4f}m')

        # 计算已移动距离
        if self.initial_y is not None:
            self.distance_moved = self.current_y - self.initial_y

    def move_forward(self):
        """执行前进到缓冲区的动作"""
        if self.current_y is None or self.initial_y is None:
            self.get_logger().error('❌ 未接收到里程计数据，无法移动')
            return False

        self.get_logger().info('')
        self.get_logger().info('🚀 开始前进到缓冲区...')
        self.get_logger().info(f'初始位置: Y = {self.initial_y:.4f}m')
        self.get_logger().info(f'目标位置: Y = {self.initial_y + self.target_distance:.4f}m')
        self.get_logger().info('')

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.is_moving = True
        self.start_time = time.time()
        rate = self.create_rate(20)  # 20Hz

        try:
            while rclpy.ok() and self.is_moving:
                # 检查是否到达目标
                if self.distance_moved >= self.target_distance:
                    self.get_logger().info('')
                    self.get_logger().info('=' * 60)
                    self.get_logger().info('✅ 已到达缓冲区！')
                    self.get_logger().info(f'最终位置: Y = {self.current_y:.4f}m')
                    self.get_logger().info(f'实际移动: {self.distance_moved:.4f}m')
                    self.get_logger().info('=' * 60)
                    break

                # 检查超时
                elapsed = time.time() - self.start_time
                if elapsed > self.timeout:
                    self.get_logger().error('❌ 移动超时！')
                    self.get_logger().error(f'已移动: {self.distance_moved:.4f}m / {self.target_distance:.4f}m')
                    break

                # 发布速度命令
                self.cmd_pub.publish(cmd)

                # 每秒输出一次进度
                if int(elapsed * 10) % 10 == 0:
                    progress = (self.distance_moved / self.target_distance) * 100
                    self.get_logger().info(
                        f'进度: {progress:.1f}% | '
                        f'已移动: {self.distance_moved*100:.1f}cm / {self.target_distance*100:.0f}cm | '
                        f'当前Y: {self.current_y:.4f}m'
                    )

                rate.sleep()

        except KeyboardInterrupt:
            self.get_logger().warn('⚠️  用户中断')
        finally:
            # 停止机器人
            stop_cmd = Twist()
            for _ in range(10):
                self.cmd_pub.publish(stop_cmd)
                time.sleep(0.05)
            self.is_moving = False

        return self.distance_moved >= self.target_distance * 0.95  # 允许5%误差

    def wait_for_odom(self, timeout_sec=5.0):
        """等待里程计数据"""
        start = time.time()
        rate = self.create_rate(10)

        while rclpy.ok() and (time.time() - start) < timeout_sec:
            if self.current_y is not None:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            rate.sleep()

        return False


def main(args=None):
    rclpy.init(args=args)

    node = CompetitionStartBufferMove()

    try:
        # 等待里程计数据
        if not node.wait_for_odom(timeout_sec=10.0):
            node.get_logger().error('❌ 等待里程计超时')
            return

        # 等待2秒让系统稳定
        node.get_logger().info('系统稳定中，2秒后开始移动...')
        time.sleep(2.0)

        # 执行前进
        success = node.move_forward()

        if success:
            node.get_logger().info('')
            node.get_logger().info('✅ 缓冲移动完成！')
            node.get_logger().info('📍 现在可以初始化定位到PRE_START点')
            node.get_logger().info('')
            node.get_logger().info('PRE_START坐标:')
            node.get_logger().info(f'  X: 0.0031m')
            node.get_logger().info(f'  Y: {node.current_y:.4f}m (当前位置)')
            node.get_logger().info(f'  Yaw: 1.5687 rad (90°)')
        else:
            node.get_logger().error('❌ 缓冲移动失败')

    except Exception as e:
        node.get_logger().error(f'❌ 错误: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
