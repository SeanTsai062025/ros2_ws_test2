#!/usr/bin/env python3
"""Dexter Hardware Motor Controller Node

Uses the mks-servo-can-main library (correct MKS SERVO42D/57D protocol)
"""

import rclpy
from rclpy.node import Node
import can
import sys
import os

# Add the mks-servo-can-main library to path (the correct GitHub library)
workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_servo_can_path = os.path.join(workspace_root, 'src/dexter_hardware/mks-servo-can-main')

if mks_servo_can_path not in sys.path:
    sys.path.insert(0, mks_servo_can_path)

from mks_servo_can.mks_enums import MksCommands
from dexter_hardware_interfaces.msg import MotorAbsoluteCommand


class MotorControllerNode(Node):
    """ROS 2 Node for controlling MKS Servo motors via CAN bus
    
    Uses the correct MKS SERVO42D/57D CAN protocol:
    - CAN ID = motor_id (standard 11-bit)
    - Data includes CRC checksum as last byte
    - CRC = (CAN_ID + sum(data_bytes)) & 0xFF
    """

    def __init__(self):
        super().__init__('motor_controller')
        
        # Get parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('bitrate', 500000)
        
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
            self.get_logger().error('Run: sudo ip link set can0 up type can bitrate 500000')
            raise
        
        # Subscribe to motor commands
        self.motor_cmd_sub = self.create_subscription(
            MotorAbsoluteCommand,
            'motor_absolute_command',
            self.motor_command_callback,
            10
        )
        
        self.get_logger().info('Motor controller node initialized. Waiting for commands on /motor_absolute_command')

    def create_can_message(self, motor_id: int, data: list) -> can.Message:
        """Create a CAN message with CRC checksum (MKS protocol)
        
        Args:
            motor_id: Motor CAN ID (1-255)
            data: Data bytes without CRC (max 7 bytes)
            
        Returns:
            CAN message with CRC appended
        """
        # Calculate CRC: (CAN_ID + sum(data)) & 0xFF
        crc = (motor_id + sum(data)) & 0xFF
        full_data = bytearray(data) + bytes([crc])
        
        return can.Message(arbitration_id=motor_id, data=full_data, is_extended_id=False)

    def send_absolute_motion_command(self, motor_id: int, speed: int, acceleration: int, position: int) -> bool:
        """Send absolute motion command to motor (0xFE command)
        
        Frame format: [0xFE] [SPEED_H] [SPEED_L] [ACC] [POS_H] [POS_M] [POS_L] [CRC]
        
        Args:
            motor_id: Motor CAN ID (1-255)
            speed: Speed in RPM (0-3000)
            acceleration: Acceleration (0-255, 0=fastest)
            position: Absolute position in pulses (-8388607 to +8388607)
            
        Returns:
            True if sent successfully
        """
        # Handle negative positions (24-bit signed)
        if position < 0:
            position = position & 0xFFFFFF
        
        # Build command data
        data = [
            MksCommands.RUN_MOTOR_ABSOLUTE_MOTION_BY_PULSES_COMMAND.value,  # 0xFE
            (speed >> 8) & 0xFF,       # Speed high byte
            speed & 0xFF,              # Speed low byte
            acceleration & 0xFF,       # Acceleration
            (position >> 16) & 0xFF,   # Position high byte (24-bit)
            (position >> 8) & 0xFF,    # Position mid byte
            position & 0xFF,           # Position low byte
        ]
        
        msg = self.create_can_message(motor_id, data)
        
        try:
            self.bus.send(msg)
            # Log the frame for debugging
            hex_data = ' '.join(f'{b:02X}' for b in msg.data)
            self.get_logger().info(f'Sent CAN frame: ID=0x{motor_id:03X} Data=[{hex_data}]')
            return True
        except can.CanError as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')
            return False

    def motor_command_callback(self, msg):
        """Callback for motor absolute command messages"""
        motor_id = msg.motor_id
        position = msg.absolute_position
        speed = msg.speed
        acceleration = msg.acceleration
        
        self.get_logger().info(
            f'Received motor command: ID={motor_id}, position={position}, speed={speed}, acc={acceleration}'
        )
        
        # Send the command
        success = self.send_absolute_motion_command(motor_id, speed, acceleration, position)
        
        if success:
            self.get_logger().info(f'Motor {motor_id} command sent successfully')
        else:
            self.get_logger().error(f'Failed to send command to motor {motor_id}')

    def destroy_node(self):
        """Cleanup on node destruction"""
        try:
            if hasattr(self, 'bus'):
                self.bus.shutdown()
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
