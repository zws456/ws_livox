#!/usr/bin/env python3
"""
Waypoint Navigator Node

提供特定点导航功能：
- 订阅 /goto_waypoint (std_msgs/String)，内容为 waypoints.yaml 中的 key
- 读取 waypoints.yaml 获取坐标
- 调用 Nav2 的 /navigate_to_pose action 进行导航
- 发布 /waypoint_status (std_msgs/String) 报告导航状态

使用方式：
    ros2 topic pub /goto_waypoint std_msgs/msg/String "data: vending_machine" --once
"""

import os
import sys
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # 参数：waypoints 文件路径
        self.declare_parameter(
            'waypoints_file',
            os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'waypoints',
                'waypoints.yaml'
            )
        )
        waypoints_file = self.get_parameter('waypoints_file').value

        # 加载路标点
        self.waypoints = self.load_waypoints(waypoints_file)
        if not self.waypoints:
            self.get_logger().warn(f'No waypoints loaded from {waypoints_file}')
        else:
            self.get_logger().info(f'Loaded waypoints: {list(self.waypoints.keys())}')

        # Action Client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 /navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available! Is Nav2 running?')
        else:
            self.get_logger().info('Nav2 action server connected.')

        # 订阅触发指令
        self._sub = self.create_subscription(
            String,
            '/goto_waypoint',
            self._on_goto_waypoint,
            10
        )

        # 发布状态
        self._status_pub = self.create_publisher(String, '/waypoint_status', 10)

        self._current_goal_future = None

    def load_waypoints(self, filepath):
        """从 yaml 文件加载路标点"""
        if not os.path.exists(filepath):
            self.get_logger().error(f'Waypoints file not found: {filepath}')
            return {}
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            if not data:
                return {}
            # 支持两种格式：直接是字典，或嵌套在 waypoints 下
            return data
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')
            return {}

    def _publish_status(self, msg):
        self._status_pub.publish(String(data=msg))
        self.get_logger().info(msg)

    def _on_goto_waypoint(self, msg):
        name = msg.data.strip()
        if not name:
            self._publish_status('Error: empty waypoint name')
            return

        if name not in self.waypoints:
            available = ', '.join(self.waypoints.keys()) if self.waypoints else 'none'
            self._publish_status(f'Error: waypoint "{name}" not found. Available: {available}')
            return

        wp = self.waypoints[name]
        x = float(wp.get('x', 0.0))
        y = float(wp.get('y', 0.0))
        yaw = float(wp.get('yaw', 0.0))

        self._publish_status(f'Navigating to "{name}" -> ({x}, {y}, {yaw})')
        self._send_goal(x, y, yaw, name)

    def _send_goal(self, x, y, yaw, name):
        # 构造 PoseStamped
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # yaw -> quaternion (z, w)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = sy
        pose.pose.orientation.w = cy

        goal_msg.pose = pose

        # 发送 action goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, name)
        )

    def _goal_response_callback(self, future, name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._publish_status(f'Goal "{name}" rejected by Nav2')
            return

        self._publish_status(f'Goal "{name}" accepted, navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            lambda future: self._get_result_callback(future, name)
        )

    def _get_result_callback(self, future, name):
        result = future.result().result
        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self._publish_status(f'Goal "{name}" reached successfully!')
        else:
            self._publish_status(f'Goal "{name}" failed or cancelled (status={status})')

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # 可选：发布剩余距离等信息
        remaining = feedback.distance_remaining
        self.get_logger().debug(f'Distance remaining: {remaining:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
