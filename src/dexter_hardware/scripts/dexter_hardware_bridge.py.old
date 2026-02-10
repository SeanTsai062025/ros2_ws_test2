#!/usr/bin/env python3
"""
Dexter Hardware Interface for ros2_control

This node acts as a bridge between ros2_control's JointTrajectoryController
and the Servo42D motors via CAN bus.

Architecture:
- Subscribes to /arm_controller/joint_trajectory (velocity commands from JTC)
- Publishes to /joint_states (encoder feedback for JTC)
- Communicates with Servo42D motors via CAN (Speed Mode 0xF6, Encoder Read 0x31)

Note: This is a "bridge" approach. For true ros2_control integration, 
a C++ hardware interface plugin would be needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import can
import sys
import os
import math
import struct
from typing import Dict, List, Optional
from dataclasses import dataclass

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Add mks-servo-can-main to path
workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_servo_can_path = os.path.join(workspace_root, 'src/dexter_hardware/mks-servo-can-main')
if mks_servo_can_path not in sys.path:
    sys.path.insert(0, mks_servo_can_path)

from mks_servo_can.mks_enums import MksCommands


@dataclass
class MotorConfig:
    """Configuration for a single motor"""
    joint_name: str
    can_id: int
    encoder_ticks_per_rev: int = 16384  # 14-bit encoder
    gear_ratio: float = 1.0  # Motor revolutions per joint revolution
    direction: int = 1  # 1 or -1 for direction inversion
    max_rpm: int = 300  # Maximum RPM limit


class DexterHardwareBridge(Node):
    """
    Bridge node connecting ros2_control to Servo42D motors
    
    This implements the read/write cycle:
    - read(): Query encoder positions from all motors (0x31 command)
    - write(): Send velocity commands to all motors (0xF6 command)
    """

    def __init__(self):
        super().__init__('dexter_hardware_bridge')
        
        # Declare parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('control_rate', 100.0)  # Hz
        self.declare_parameter('use_sim', False)  # Simulation mode - no CAN hardware
        
        # Motor configuration (joint_name -> MotorConfig)
        # These map to your URDF joint names
        self.motors: Dict[str, MotorConfig] = {
            'base': MotorConfig(joint_name='base', can_id=1, direction=1),
            'part1': MotorConfig(joint_name='part1', can_id=2, direction=1),
            'part2': MotorConfig(joint_name='part2', can_id=3, direction=1),
            'part3': MotorConfig(joint_name='part3', can_id=4, direction=1),
            'part4': MotorConfig(joint_name='part4', can_id=5, direction=1),
            'part5': MotorConfig(joint_name='part5', can_id=6, direction=1),
        }
        
        # State storage
        self.joint_positions: Dict[str, float] = {name: 0.0 for name in self.motors}
        self.joint_velocities: Dict[str, float] = {name: 0.0 for name in self.motors}
        self.velocity_commands: Dict[str, float] = {name: 0.0 for name in self.motors}
        
        # Simulation mode flag
        self.use_sim = self.get_parameter('use_sim').value
        self.bus = None
        
        # Initialize CAN bus (only if not in simulation mode)
        if not self.use_sim:
            can_interface = self.get_parameter('can_interface').value
            can_bitrate = self.get_parameter('can_bitrate').value
            
            try:
                self.bus = can.interface.Bus(
                    interface='socketcan', 
                    channel=can_interface, 
                    bitrate=can_bitrate
                )
                self.get_logger().info(f'CAN bus initialized: {can_interface} at {can_bitrate} bps')
            except Exception as e:
                self.get_logger().error(f'Failed to initialize CAN bus: {e}')
                raise
        else:
            self.get_logger().info('Running in SIMULATION mode - no CAN hardware')
            self.last_sim_time = self.get_clock().now()
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        # Subscribers - velocity commands from JTC
        # JTC publishes to /arm_controller/commands when using forward_command_controller
        # For JTC, we need to listen to the internal command interface
        # Alternative: Use a forward_velocity_controller
        self.velocity_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',  # We'll create a velocity controller
            self.velocity_command_callback,
            10
        )
        
        # Control loop timer
        control_rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )
        
        # Statistics
        self.loop_count = 0
        self.get_logger().info('Dexter Hardware Bridge initialized')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info(f'Motors: {list(self.motors.keys())}')

    def create_can_message(self, motor_id: int, data: list) -> can.Message:
        """Create CAN message with CRC checksum"""
        crc = (motor_id + sum(data)) & 0xFF
        full_data = bytearray(data) + bytes([crc])
        return can.Message(arbitration_id=motor_id, data=full_data, is_extended_id=False)

    def send_velocity_command(self, motor_id: int, velocity_rpm: float, acceleration: int = 0):
        """
        Send Speed Mode command (0xF6) to motor
        
        MKS protocol for 0xF6 speed mode (from mks-servo-can-main library):
          Data: [0xF6] [DIR | SPEED_H] [SPEED_L] [ACC] [CRC]
          - DIR: 0x80 = CW, 0x00 = CCW
          - SPEED_H: high nibble of speed (0-0x0F)
          - SPEED_L: low byte of speed
          - ACC: acceleration 0-255
          - CRC: (CAN_ID + sum(data_bytes)) & 0xFF
          Total frame: 5 bytes (4 data + 1 CRC)
        
        Args:
            motor_id: CAN ID of motor
            velocity_rpm: Desired velocity in RPM (positive = CCW, negative = CW)
            acceleration: Acceleration rate (0-255, 0 = fastest)
        """
        # Clamp velocity to max RPM
        speed = min(abs(velocity_rpm), 3000)
        speed_int = int(speed)
        
        # Determine direction: 0x80 = CW (negative), 0x00 = CCW (positive)
        direction_bit = 0x80 if velocity_rpm < 0 else 0x00
        
        # Build command data — NO padding bytes!
        # Must match library: [0xF6, dir|speed_h, speed_l, acc]
        data = [
            MksCommands.RUN_MOTOR_SPEED_MODE_COMMAND.value,  # 0xF6
            direction_bit | ((speed_int >> 8) & 0x0F),       # Dir + Speed high nibble
            speed_int & 0xFF,                                # Speed low byte
            acceleration & 0xFF,                             # Acceleration
        ]
        
        msg = self.create_can_message(motor_id, data)
        
        try:
            self.bus.send(msg)
            # Debug: log frame details occasionally
            if self.loop_count % 100 == 0 and speed_int > 0:
                hex_data = ' '.join(f'{b:02X}' for b in msg.data)
                self.get_logger().info(
                    f'CAN TX: ID=0x{motor_id:03X} DLC={len(msg.data)} Data=[{hex_data}]'
                )
        except can.CanError as e:
            self.get_logger().error(f'Failed to send velocity command to motor {motor_id}: {e}')

    def read_encoder_position(self, motor_id: int) -> Optional[int]:
        """
        Read encoder position using 0x31 command
        
        Returns:
            Encoder position in ticks, or None if failed
        """
        # Send read command
        data = [MksCommands.READ_ENCODED_VALUE_ADDITION.value]  # 0x31
        msg = self.create_can_message(motor_id, data)
        
        try:
            self.bus.send(msg)
            
            # Wait for response (with short timeout)
            response = self.bus.recv(timeout=0.005)  # 5ms timeout
            
            if response and response.arbitration_id == motor_id:
                if len(response.data) >= 7 and response.data[0] == 0x31:
                    # Parse 48-bit signed position (bytes 1-6)
                    pos_bytes = response.data[1:7]
                    position = int.from_bytes(pos_bytes, byteorder='big', signed=True)
                    return position
            
            return None
            
        except can.CanError as e:
            self.get_logger().warn(f'Failed to read encoder from motor {motor_id}: {e}')
            return None

    def velocity_command_callback(self, msg: Float64MultiArray):
        """Handle velocity commands from controller"""
        joint_names = list(self.motors.keys())
        
        if len(msg.data) != len(joint_names):
            self.get_logger().warn(
                f'Velocity command size mismatch: got {len(msg.data)}, expected {len(joint_names)}'
            )
            return
        
        for i, joint_name in enumerate(joint_names):
            self.velocity_commands[joint_name] = msg.data[i]

    def control_loop(self):
        """
        Main control loop - runs at control_rate Hz
        
        1. Read encoder positions from all motors
        2. Publish joint states
        3. Send velocity commands to all motors
        """
        self.loop_count += 1
        
        if self.use_sim:
            # --- SIMULATION MODE ---
            # Simulate motor dynamics: integrate velocity to get position
            current_time = self.get_clock().now()
            dt = (current_time - self.last_sim_time).nanoseconds / 1e9
            self.last_sim_time = current_time
            
            for joint_name, motor in self.motors.items():
                # Integrate velocity to get position
                velocity = self.velocity_commands[joint_name]
                self.joint_positions[joint_name] += velocity * dt
                self.joint_velocities[joint_name] = velocity
        else:
            # --- REAL HARDWARE MODE ---
            # Read encoder positions from all motors
            for joint_name, motor in self.motors.items():
                encoder_ticks = self.read_encoder_position(motor.can_id)
                
                if encoder_ticks is not None:
                    # Convert ticks to radians
                    revolutions = encoder_ticks / motor.encoder_ticks_per_rev / motor.gear_ratio
                    position_rad = revolutions * 2 * math.pi * motor.direction
                    self.joint_positions[joint_name] = position_rad
        
        # --- PUBLISH: Joint states ---
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = list(self.motors.keys())
        joint_state_msg.position = [self.joint_positions[name] for name in joint_state_msg.name]
        joint_state_msg.velocity = [self.joint_velocities[name] for name in joint_state_msg.name]
        joint_state_msg.effort = []  # Not measured
        
        self.joint_state_pub.publish(joint_state_msg)
        
        if not self.use_sim:
            # --- WRITE: Send velocity commands to real hardware ---
            for joint_name, motor in self.motors.items():
                velocity_rad_s = self.velocity_commands[joint_name]
                
                # Convert rad/s to RPM
                velocity_rpm = (velocity_rad_s * 60) / (2 * math.pi) * motor.direction
                
                # Clamp to motor limits
                velocity_rpm = max(-motor.max_rpm, min(motor.max_rpm, velocity_rpm))
                
                # Send command
                self.send_velocity_command(motor.can_id, velocity_rpm)
                
                # Debug logging for non-zero commands
                if self.loop_count % 100 == 0 and abs(velocity_rpm) > 0.01:
                    self.get_logger().info(
                        f'Motor {motor.can_id} ({joint_name}): {velocity_rad_s:.3f} rad/s = {velocity_rpm:.1f} RPM'
                    )
        
        # Log occasionally
        if self.loop_count % 500 == 0:  # Every 5 seconds at 100Hz
            mode = "SIM" if self.use_sim else "HW"
            self.get_logger().info(
                f'[{mode}] Control loop running. Positions: {[f"{p:.2f}" for p in self.joint_positions.values()]}'
            )

    def destroy_node(self):
        """Cleanup"""
        if not self.use_sim and self.bus:
            # Stop all motors
            self.get_logger().info('Stopping all motors...')
            for motor in self.motors.values():
                self.send_velocity_command(motor.can_id, 0.0)
            
            # Close CAN bus
            self.bus.shutdown()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DexterHardwareBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
