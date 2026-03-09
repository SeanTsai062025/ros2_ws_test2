#!/usr/bin/env python3
"""
Real-time Motor Command Client

Publishes motor commands without waiting for subscribers.
Ideal for real-time control applications.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class RealtimeCommandClient(Node):
    """Publishes motor commands with minimal latency."""

    def __init__(self):
        super().__init__('realtime_command_client')
        
        # Create publisher with transient_local QoS for guaranteed delivery
        # or use BEST_EFFORT for ultra-low latency
        self.publisher = self.create_publisher(
            String,
            '/motor_command',
            qos_profile=10
        )
        
        self.get_logger().info('Real-time Command Client initialized')
        self.get_logger().info('Usage: python3 realtime_command_client.py "servo1 90 15"')

    def send_command(self, command: str):
        """Send command immediately without waiting.
        
        Args:
            command: Command string (e.g., "servo1 90 15")
        """
        msg = String()
        msg.data = command
        
        # Publish immediately - no waiting for subscribers
        self.publisher.publish(msg)
        self.get_logger().info(f'Command sent (real-time): {command}')
        
        # Small delay to ensure message is processed
        time.sleep(0.01)


def main(args=None):
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage: python3 realtime_command_client.py 'servo1 90 15'")
        print("       python3 realtime_command_client.py 'servo2 180 30'")
        sys.exit(1)
    
    rclpy.init(args=args)
    
    client = RealtimeCommandClient()
    command = sys.argv[1]
    
    # Send command and exit immediately
    client.send_command(command)
    
    # Give it a moment to publish
    time.sleep(0.1)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
