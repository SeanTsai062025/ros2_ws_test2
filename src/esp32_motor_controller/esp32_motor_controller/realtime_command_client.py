#!/usr/bin/env python3
"""Publish one motor command through a short-lived ROS 2 client."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RealtimeCommandClient(Node):
    """Publish motor commands with minimal latency."""

    def __init__(self):
        super().__init__('realtime_command_client')
        self.publisher = self.create_publisher(String, '/motor_command', 10)
        self.get_logger().info('Real-time command client initialized')

    def send_command(self, command: str):
        """Publish a command immediately and allow DDS to process it."""
        message = String()
        message.data = command
        self.publisher.publish(message)
        self.get_logger().info(f'Command sent: {command}')
        time.sleep(0.01)


def main(args=None):
    """Run the one-shot command client."""
    del args
    if len(sys.argv) < 2:
        print("Usage: realtime_command_client 'servo1 90 15'")
        raise SystemExit(1)

    rclpy.init()
    client = RealtimeCommandClient()
    client.send_command(sys.argv[1])
    time.sleep(0.1)
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
