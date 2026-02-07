#!/usr/bin/env python3
"""Dexter Hardware Motor Controller Node"""

import rclpy
from rclpy.node import Node
import can
import sys
import os

# Add the mks_servo_can package to path
# Use absolute path to ensure it works from any location
workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_servo_can_path = os.path.join(workspace_root, 'src/dexter_hardware')

if mks_servo_can_path not in sys.path:
    sys.path.insert(0, mks_servo_can_path)

from mks_servo_can import MksServo
from dexter_hardware_interfaces.msg import MotorAbsoluteCommand


class MotorControllerNode(Node):
    """ROS 2 Node for controlling MKS Servo motors via CAN bus"""

    def __init__(self):
        super().__init__('motor_controller')
        
        # Get parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('bitrate', 1000000)
        
        can_interface = self.get_parameter('can_interface').value
        bitrate = self.get_parameter('bitrate').value
        
        self.get_logger().info(f'Initializing CAN interface: {can_interface} at {bitrate} bps')
        
        try:
            # Initialize CAN bus
            self.bus = can.interface.Bus(interface='socketcan', channel=can_interface, bitrate=bitrate)
            self.get_logger().info('CAN bus initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            self.get_logger().error('Make sure the CAN interface is properly configured.')
            self.get_logger().error('You can test it with: sudo ip link show can0')
            raise
        
        # Create notifier for CAN bus
        self.notifier = can.Notifier(self.bus, [])
        
        # Dictionary to store servo instances by motor ID
        self.servos = {}
        
        # Subscribe to motor commands
        self.motor_cmd_sub = self.create_subscription(
            MotorAbsoluteCommand,
            'motor_absolute_command',
            self.motor_command_callback,
            10
        )
        
        self.get_logger().info('Motor controller node initialized. Waiting for commands on /motor_absolute_command')

    def get_or_create_servo(self, motor_id):
        """Get or create a servo instance for the given motor ID"""
        if motor_id not in self.servos:
            try:
                servo = MksServo(self.bus, self.notifier, motor_id)
                self.servos[motor_id] = servo
                self.get_logger().info(f'Created servo instance for motor ID {motor_id}')
            except Exception as e:
                self.get_logger().error(f'Failed to create servo instance for motor ID {motor_id}: {e}')
                return None
        return self.servos[motor_id]

    def motor_command_callback(self, msg):
        """Callback for motor absolute command messages"""
        motor_id = msg.motor_id
        position = msg.absolute_position
        speed = msg.speed
        acceleration = msg.acceleration
        
        self.get_logger().info(
            f'Received motor command: ID={motor_id}, position={position}, speed={speed}, acc={acceleration}'
        )
        
        try:
            servo = self.get_or_create_servo(motor_id)
            if servo is None:
                self.get_logger().error(f'Failed to get servo for motor ID {motor_id}')
                return
            
            # Send absolute position command
            self.get_logger().info(f'Sending absolute motion command to motor {motor_id}')
            result = servo.run_motor_absolute_motion_by_pulses(speed, acceleration, position)
            
            if result is not None:
                self.get_logger().info(f'Motor {motor_id} command sent successfully. Result: {result}')
            else:
                self.get_logger().warn(f'Motor {motor_id} command sent but no clear response received')
                
        except Exception as e:
            self.get_logger().error(f'Error sending command to motor {motor_id}: {e}')

    def destroy_node(self):
        """Cleanup on node destruction"""
        try:
            if hasattr(self, 'bus'):
                self.bus.shutdown()
            if hasattr(self, 'notifier'):
                self.notifier.stop()
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MotorControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
