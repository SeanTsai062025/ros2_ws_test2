#!/usr/bin/env python3
"""Send a joint command to the commander node."""
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64MultiArray
import sys, time

def main():
    rclpy.init()
    node = Node('test_publisher')
    pub = node.create_publisher(Float64MultiArray, '/joint_command', 10)
    
    # Wait for subscriber
    time.sleep(2.0)
    
    msg = Float64MultiArray()
    msg.data = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    print(f"Publishing joint command: {msg.data}")
    pub.publish(msg)
    
    # Give time for message to be received
    time.sleep(1.0)
    print("Done!")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
