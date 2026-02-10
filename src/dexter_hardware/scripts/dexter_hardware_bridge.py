#!/usr/bin/env python3
"""
Dexter Hardware Bridge v2 - Full MoveIt to Servo42D Pipeline.

Subscribes to /arm_controller/controller_state from the JTC.
The JTC interpolates the trajectory at 100Hz and publishes
reference.velocities (the planned velocity at this instant).
We send that velocity directly to motors via 0xF6 speed-mode CAN.
We read encoders via 0x31 and publish /joint_states.
"""

import rclpy
from rclpy.node import Node
import can
import sys
import os
import math
from typing import Dict, Optional
from dataclasses import dataclass

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import JointTrajectoryControllerState

# Add mks-servo-can-main to path
workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_path = os.path.join(workspace_root, 'src/dexter_hardware/mks-servo-can-main')
if mks_path not in sys.path:
    sys.path.insert(0, mks_path)

from mks_servo_can.mks_enums import MksCommands

JOINT_NAMES = ['base', 'part1', 'part2', 'part3', 'part4', 'part5']


@dataclass
class MotorConfig:
    joint_name: str
    can_id: int
    encoder_ticks_per_rev: int = 16384
    gear_ratio: float = 1.0
    direction: int = 1
    max_rpm: int = 300


class DexterHardwareBridge(Node):

    def __init__(self):
        super().__init__('dexter_hardware_bridge')

        # Parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('use_sim', False)

        self.use_sim = self.get_parameter('use_sim').value
        self.dt = 1.0 / self.get_parameter('control_rate').value

        # Motor map
        self.motors = {
            'base':  MotorConfig(joint_name='base',  can_id=1),
            'part1': MotorConfig(joint_name='part1', can_id=2),
            'part2': MotorConfig(joint_name='part2', can_id=3),
            'part3': MotorConfig(joint_name='part3', can_id=4),
            'part4': MotorConfig(joint_name='part4', can_id=5),
            'part5': MotorConfig(joint_name='part5', can_id=6),
        }

        # State
        self.joint_positions = {n: 0.0 for n in JOINT_NAMES}
        self.joint_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._ref_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._manual_mode = False
        self._manual_vels = {n: 0.0 for n in JOINT_NAMES}
        self._jtc_active = False
        self._encoder_read_index = 0  # round-robin: read 1 motor per tick

        # CAN bus
        self.bus = None
        if not self.use_sim:
            iface = self.get_parameter('can_interface').value
            baud = self.get_parameter('can_bitrate').value
            try:
                self.bus = can.interface.Bus(
                    interface='socketcan', channel=iface, bitrate=baud)
                self.get_logger().info(
                    'CAN bus ready: {} @ {} bps'.format(iface, baud))
            except Exception as e:
                self.get_logger().error('CAN init failed: {}'.format(e))
                raise
        else:
            self.get_logger().info('SIMULATION mode - no CAN hardware')

        # Publisher: encoder feedback on separate topic (diagnostics only).
        # The joint_state_broadcaster already publishes /joint_states from
        # mock hardware — we must NOT conflict with it or JTC gets confused.
        # In sim mode we DO publish /joint_states (no mock hw feedback).
        if self.use_sim:
            self.joint_state_pub = self.create_publisher(
                JointState, '/joint_states', 10)
        else:
            self.joint_state_pub = self.create_publisher(
                JointState, '/encoder_joint_states', 10)

        # Subscriber: JTC controller state (100 Hz from JTC)
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self._on_jtc_state,
            10)

        # Subscriber: manual velocity override for testing
        self.vel_sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self._on_velocity_cmd,
            10)

        # Control loop
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.loop_count = 0

        mode_str = 'SIM' if self.use_sim else 'HARDWARE'
        rate = self.get_parameter('control_rate').value
        self.get_logger().info('=' * 50)
        self.get_logger().info('  Dexter Hardware Bridge v2 - READY')
        self.get_logger().info('  Mode    : {}'.format(mode_str))
        self.get_logger().info('  Rate    : {} Hz'.format(rate))
        self.get_logger().info('  Joints  : {}'.format(JOINT_NAMES))
        self.get_logger().info('  Listens : /arm_controller/controller_state')
        self.get_logger().info('            /velocity_controller/commands')
        js_topic = '/joint_states' if self.use_sim else '/encoder_joint_states'
        self.get_logger().info('  Publishes: {}'.format(js_topic))
        self.get_logger().info('=' * 50)

    # -- Callbacks --

    def _on_jtc_state(self, msg):
        """Extract reference velocities from JTC state at each tick."""
        names = list(msg.joint_names)
        ref_vel = msg.reference.velocities

        if len(ref_vel) == 0:
            return

        any_nonzero = False
        for i, name in enumerate(names):
            if name in self._ref_velocities and i < len(ref_vel):
                self._ref_velocities[name] = ref_vel[i]
                if abs(ref_vel[i]) > 1e-6:
                    any_nonzero = True

        if any_nonzero and not self._jtc_active:
            self._jtc_active = True
            self._manual_mode = False
            self.get_logger().info('JTC trajectory active')
        elif not any_nonzero and self._jtc_active:
            self._jtc_active = False
            self.get_logger().info('JTC trajectory idle - holding')

    def _on_velocity_cmd(self, msg):
        """Manual velocity override for testing without MoveIt."""
        if len(msg.data) != len(JOINT_NAMES):
            self.get_logger().warn(
                'Velocity size mismatch: {} != {}'.format(
                    len(msg.data), len(JOINT_NAMES)))
            return
        self._manual_mode = True
        for i, name in enumerate(JOINT_NAMES):
            self._manual_vels[name] = msg.data[i]

    # -- Control Loop (100 Hz) --

    def _control_loop(self):
        self.loop_count += 1

        # 1. READ: current joint positions
        if not self.use_sim:
            self._hw_read_encoders()

        # 2. DETERMINE: velocity commands
        if self._manual_mode:
            vel_cmds = dict(self._manual_vels)
        else:
            vel_cmds = dict(self._ref_velocities)

        # 3. WRITE: send velocity to motors
        if self.use_sim:
            for name in JOINT_NAMES:
                self.joint_positions[name] += vel_cmds[name] * self.dt
        else:
            self._hw_write_velocities(vel_cmds)

        self.joint_velocities = vel_cmds

        # 4. PUBLISH: /joint_states
        self._publish_joint_states()

        # 5. Periodic log
        if self.loop_count % 500 == 0:
            mode = 'SIM' if self.use_sim else 'HW'
            state = ('MANUAL' if self._manual_mode
                     else ('JTC' if self._jtc_active else 'IDLE'))
            pos = ', '.join(
                '{:+.3f}'.format(self.joint_positions[n]) for n in JOINT_NAMES)
            vel = ', '.join(
                '{:+.3f}'.format(vel_cmds[n]) for n in JOINT_NAMES)
            self.get_logger().info(
                '[{}/{}] pos=[{}] vel=[{}]'.format(mode, state, pos, vel))

    # -- Hardware CAN --

    def _hw_read_encoders(self):
        """Round-robin: read only ONE motor per tick to avoid flooding CAN.
        Each motor gets read every 6 ticks = 60ms at 100Hz. Still plenty fast."""
        name = JOINT_NAMES[self._encoder_read_index]
        motor = self.motors[name]
        self._encoder_read_index = (self._encoder_read_index + 1) % len(JOINT_NAMES)

        ticks = self._can_read_encoder(motor.can_id)
        if ticks is not None:
            revs = ticks / motor.encoder_ticks_per_rev / motor.gear_ratio
            self.joint_positions[name] = (
                revs * 2.0 * math.pi * motor.direction)

    def _hw_write_velocities(self, vel_cmds):
        for name, motor in self.motors.items():
            vel_rad_s = vel_cmds[name]
            rpm = (vel_rad_s * 60.0) / (2.0 * math.pi) * motor.direction
            rpm = max(-motor.max_rpm, min(motor.max_rpm, rpm))
            self._can_send_speed(motor.can_id, rpm, accel=0)

            if self.loop_count % 200 == 0 and abs(rpm) > 0.01:
                self.get_logger().info(
                    '  Motor {} ({}): {:.3f} rad/s -> {:.1f} RPM'.format(
                        motor.can_id, name, vel_rad_s, rpm))

    # -- CAN primitives --

    def _can_msg(self, motor_id, data):
        """Build CAN message with MKS CRC."""
        crc = (motor_id + sum(data)) & 0xFF
        return can.Message(
            arbitration_id=motor_id,
            data=bytearray(data) + bytes([crc]),
            is_extended_id=False)

    def _can_send_speed(self, motor_id, rpm, accel=0):
        """
        0xF6 Speed Mode - 5-byte frame.
        [0xF6] [DIR|SPD_H] [SPD_L] [ACC] [CRC]
        """
        speed = min(int(abs(rpm)), 3000)
        dir_bit = 0x80 if rpm < 0 else 0x00

        data = [
            MksCommands.RUN_MOTOR_SPEED_MODE_COMMAND.value,  # 0xF6
            dir_bit | ((speed >> 8) & 0x0F),
            speed & 0xFF,
            accel & 0xFF,
        ]
        msg = self._can_msg(motor_id, data)

        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(
                'CAN TX fail motor {}: {}'.format(motor_id, e))

    def _can_read_encoder(self, motor_id):
        """
        0x31 Read Encoder Value Addition.
        TX: [0x31] [CRC] = 2 bytes
        RX: [0x31] [6 bytes signed pos big-endian] [CRC] = 8 bytes
        """
        data = [MksCommands.READ_ENCODED_VALUE_ADDITION.value]  # 0x31
        msg = self._can_msg(motor_id, data)

        try:
            self.bus.send(msg)
            resp = self.bus.recv(timeout=0.002)

            if resp and resp.arbitration_id == motor_id:
                if len(resp.data) >= 7 and resp.data[0] == 0x31:
                    return int.from_bytes(
                        resp.data[1:7], 'big', signed=True)
            return None
        except can.CanError as e:
            self.get_logger().warn(
                'Encoder read fail motor {}: {}'.format(motor_id, e))
            return None

    # -- Publish --

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [self.joint_positions[n] for n in JOINT_NAMES]
        msg.velocity = [self.joint_velocities[n] for n in JOINT_NAMES]
        msg.effort = []
        self.joint_state_pub.publish(msg)

    # -- Shutdown --

    def destroy_node(self):
        if not self.use_sim and self.bus:
            self.get_logger().info('Stopping all motors...')
            for motor in self.motors.values():
                self._can_send_speed(motor.can_id, 0.0)
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
        print('Fatal: {}'.format(e))
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
