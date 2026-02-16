#!/usr/bin/env python3
"""
Dexter Hardware Bridge v3.1.

Architecture
────────────
MoveIt → Commander → JTC (arm_controller, mock hardware)
                        │
   /arm_controller/controller_state (100 Hz, reference.positions + velocities)
                        │
             Hardware Bridge (this node)
   ├─ Down-samples to 30 Hz synchronous loop
   ├─ READ  : 0x31 encoder (always, even during motion)
   ├─ UPDATE: publish /encoder_joint_states
   └─ WRITE : 0xF5 absolute-position by axis (encoder ticks)

Control mode: 0xF5 — Absolute Motion by Axis (encoder ticks)
Encoder read:  0x31  (addition mode, signed 48-bit ticks)
"""

import rclpy
from rclpy.node import Node
import can
import sys
import os
import math
import time
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

# ── 0xF5 absolute-position-by-axis constants ──────────────────
TICKS_PER_REV = 16384   # 14-bit encoder
MAX_AXIS = 8388607      # 0x7FFFFF  (24-bit signed max)

# ── Velocity look-ahead parameters ────────────────────────────
#
@dataclass
class MotorConfig:
    joint_name: str
    can_id: int
    encoder_ticks_per_rev: int = TICKS_PER_REV
    gear_ratio: float = 1.0
    # cmd_direction:  1 → positive rad → positive pulses (CCW default)
    #                -1 → positive rad → negative pulses
    cmd_direction: int = 1
    # enc_direction:  1 → positive encoder ticks = positive rad
    #                -1 → positive encoder ticks = negative rad
    enc_direction: int = 1


class DexterHardwareBridge(Node):

    def __init__(self):
        super().__init__('dexter_hardware_bridge')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('control_rate', 100.0)   # ROS timer Hz
        self.declare_parameter('bus_rate', 30.0)         # CAN I/O Hz
        self.declare_parameter('use_sim', False)
        self.declare_parameter('connected_joints', ['part3', 'part5'])

        self.use_sim = self.get_parameter('use_sim').value
        self.dt = 1.0 / self.get_parameter('control_rate').value
        self._bus_period = 1.0 / self.get_parameter('bus_rate').value
        self._connected = set(
            self.get_parameter('connected_joints').value)

        # ── Motor map ───────────────────────────────────────────────
        # For 0xF5 (absolute motion by axis), the target is in the
        # SAME coordinate system as encoder ticks (0x31 reads).
        # So cmd_direction should match enc_direction — the target
        # tick value must equal what the encoder will read at that angle.
        #
        # Motor 6 (part5): CW→positive encoder ticks, CCW→negative
        #   enc_direction=1 → positive ticks = positive rad
        #   cmd_direction=1 → positive rad → positive axis ticks  (match encoder)
        self.motors = {
            'base':  MotorConfig(joint_name='base',  can_id=1),
            'part1': MotorConfig(joint_name='part1', can_id=2),
            'part2': MotorConfig(joint_name='part2', can_id=3),
            'part3': MotorConfig(joint_name='part3', can_id=4),
            'part4': MotorConfig(joint_name='part4', can_id=5),
            'part5': MotorConfig(joint_name='part5', can_id=6,
                                 cmd_direction=1, enc_direction=1),
        }

        # ── State ──────────────────────────────────────────────────
        self.joint_positions = {n: 0.0 for n in JOINT_NAMES}
        self.joint_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._ref_positions = {n: 0.0 for n in JOINT_NAMES}
        self._ref_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._last_ref_positions = {n: 0.0 for n in JOINT_NAMES}
        self._jtc_active = False
        self._encoder_read_index = 0
        self._last_bus_time = 0.0   # for 30 Hz down-sampling
        self._last_sent_pulses = {n: 0 for n in JOINT_NAMES}

        # Manual mode (direct velocity override for testing)
        self._manual_mode = False
        self._manual_vels = {n: 0.0 for n in JOINT_NAMES}

        # ── CAN bus ────────────────────────────────────────────────
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
            self.get_logger().info('SIMULATION mode — no CAN hardware')

        # ── Publishers / Subscribers ───────────────────────────────
        if self.use_sim:
            self.joint_state_pub = self.create_publisher(
                JointState, '/joint_states', 10)
        else:
            self.joint_state_pub = self.create_publisher(
                JointState, '/encoder_joint_states', 10)

        # JTC controller state (reference positions at 100 Hz)
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self._on_jtc_state,
            10)

        # Manual velocity override (for testing without MoveIt)
        self.vel_sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self._on_velocity_cmd,
            10)

        # ── Control timer ──────────────────────────────────────────
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.loop_count = 0

        mode_str = 'SIM' if self.use_sim else 'HARDWARE'
        rate = self.get_parameter('control_rate').value
        bus_hz = self.get_parameter('bus_rate').value
        js_topic = '/joint_states' if self.use_sim else '/encoder_joint_states'
        self.get_logger().info('=' * 56)
        self.get_logger().info('  Dexter Hardware Bridge v3.1')
        self.get_logger().info('  Mode     : {}'.format(mode_str))
        self.get_logger().info('  Timer    : {} Hz'.format(rate))
        self.get_logger().info('  Bus rate : {} Hz (CAN I/O)'.format(bus_hz))
        self.get_logger().info('  Cmd mode : 0xF5 absolute position')
        self.get_logger().info('  Joints   : {}'.format(JOINT_NAMES))
        self.get_logger().info('  Connected: {}'.format(
            sorted(self._connected)))
        self.get_logger().info('  Publishes: {}'.format(js_topic))
        self.get_logger().info('=' * 56)

    # ═══════════════════════════════════════════════════════════════
    #  Callbacks
    # ═══════════════════════════════════════════════════════════════

    def _on_jtc_state(self, msg):
        """Extract reference positions and velocities from JTC state."""
        names = list(msg.joint_names)
        ref_pos = msg.reference.positions
        ref_vel = msg.reference.velocities

        if len(ref_pos) == 0:
            return

        any_moving = False
        for i, name in enumerate(names):
            if name in self._ref_positions and i < len(ref_pos):
                self._ref_positions[name] = ref_pos[i]
                # Extract velocity if available
                if i < len(ref_vel):
                    self._ref_velocities[name] = ref_vel[i]
                else:
                    self._ref_velocities[name] = 0.0
                # Detect trajectory activity: reference differs from last
                if abs(ref_pos[i] - self._last_ref_positions.get(name, 0.0)) > 1e-6:
                    any_moving = True

        if any_moving and not self._jtc_active:
            self._jtc_active = True
            self._manual_mode = False
            self.get_logger().info('JTC trajectory ACTIVE')
        elif not any_moving and self._jtc_active:
            self._jtc_active = False
            # Log final position for diagnostics
            for name in sorted(self._connected):
                self.get_logger().info(
                    '  {} final ref={:.4f} rad  enc={:.4f} rad'.format(
                        name, self._ref_positions[name],
                        self.joint_positions[name]))
            self.get_logger().info('JTC trajectory IDLE')

        self._last_ref_positions = dict(self._ref_positions)

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

    # ═══════════════════════════════════════════════════════════════
    #  Control Loop  (runs at control_rate, e.g. 100 Hz)
    # ═══════════════════════════════════════════════════════════════

    def _control_loop(self):
        self.loop_count += 1

        now = time.monotonic()
        do_bus_io = (now - self._last_bus_time) >= self._bus_period

        if self.use_sim:
            # Sim mode: integrate manual velocities into positions
            if self._manual_mode:
                for name in JOINT_NAMES:
                    self.joint_positions[name] += (
                        self._manual_vels[name] * self.dt)
            else:
                # In sim, track the reference positions directly
                for name in JOINT_NAMES:
                    self.joint_positions[name] = self._ref_positions[name]
        elif do_bus_io:
            # ── 30 Hz synchronous CAN cycle ────────────────────────
            self._last_bus_time = now

            # 1. READ encoder(s) — always active, even during motion
            self._hw_read_encoders()

            # 2. WRITE position command
            if self._manual_mode:
                self._hw_write_manual_velocities()
            else:
                self._hw_write_positions(
                    self._ref_positions, self._ref_velocities)

        # PUBLISH joint states on every tick (100 Hz) for smooth RViz
        self._publish_joint_states()

        # Periodic log
        if self.loop_count % 500 == 0:
            mode = 'SIM' if self.use_sim else 'HW'
            state = ('MANUAL' if self._manual_mode
                     else ('ACTIVE' if self._jtc_active else 'IDLE'))
            pos = ', '.join(
                '{:+.3f}'.format(self.joint_positions[n])
                for n in JOINT_NAMES)
            ref = ', '.join(
                '{:+.3f}'.format(self._ref_positions[n])
                for n in JOINT_NAMES)
            self.get_logger().info(
                '[{}/{}] enc=[{}] ref=[{}]'.format(mode, state, pos, ref))

    # ═══════════════════════════════════════════════════════════════
    #  Hardware CAN — Encoder Read
    # ═══════════════════════════════════════════════════════════════

    def _hw_read_encoders(self):
        """Round-robin read of connected motors (one per 30 Hz tick).

        With 2 connected motors at 30 Hz, each motor is read at 15 Hz.
        This keeps the bus quiet enough to avoid collisions with writes.
        """
        connected = [n for n in JOINT_NAMES if n in self._connected]
        if not connected:
            return

        idx = self._encoder_read_index % len(connected)
        name = connected[idx]
        motor = self.motors[name]
        self._encoder_read_index += 1

        ticks = self._can_read_encoder(motor.can_id)
        if ticks is not None:
            revs = ticks / motor.encoder_ticks_per_rev / motor.gear_ratio
            self.joint_positions[name] = (
                revs * 2.0 * math.pi * motor.enc_direction)

    # ═══════════════════════════════════════════════════════════════
    #  Hardware CAN — Position Write  (0xF5 + Look-Ahead)
    # ═══════════════════════════════════════════════════════════════

    def _hw_write_positions(self, pos_cmds, vel_cmds):
        """Send 0xF5 absolute-position-by-axis commands."""
        for name, motor in self.motors.items():
            if name not in self._connected:
                continue

            target_rad = pos_cmds[name]
            
            # Default speed and acceleration
            speed = 100  # RPM
            accel = 100  # max

            # ── Convert radians → encoder ticks ────────────────────
            target_ticks = round(
                target_rad / (2.0 * math.pi)
                * motor.encoder_ticks_per_rev
                * motor.gear_ratio
                * motor.cmd_direction)

            # Clamp to 24-bit signed range
            target_ticks = max(-MAX_AXIS, min(MAX_AXIS, target_ticks))

            # Only send when tick target changes
            if target_ticks != self._last_sent_pulses[name]:
                self._can_send_absolute_axis(
                    motor.can_id, speed, accel, target_ticks)
                self.get_logger().info(
                    '  CAN TX M{}: {:.4f} rad -> {} ticks '
                    '(spd={} acc={})'.format(
                        motor.can_id, target_rad, target_ticks,
                        speed, accel))
                self._last_sent_pulses[name] = target_ticks

    def _hw_write_manual_velocities(self):
        """Fallback: send 0xF6 speed-mode commands for manual override."""
        for name, motor in self.motors.items():
            if name not in self._connected:
                continue
            vel_rad_s = self._manual_vels[name]
            rpm = (vel_rad_s * 60.0) / (2.0 * math.pi) * motor.cmd_direction
            self._can_send_speed(motor.can_id, rpm, accel=0)

    # ═══════════════════════════════════════════════════════════════
    #  CAN Primitives
    # ═══════════════════════════════════════════════════════════════

    def _can_msg(self, motor_id, data):
        """Build CAN message with MKS CRC."""
        crc = (motor_id + sum(data)) & 0xFF
        return can.Message(
            arbitration_id=motor_id,
            data=bytearray(data) + bytes([crc]),
            is_extended_id=False)

    def _can_send_absolute_axis(self, motor_id, speed, accel, abs_axis):
        """
        0xF5 — Absolute Motion by Axis (encoder ticks).

        Frame (8 bytes on wire):
            [0xF5] [SPD_H] [SPD_L] [ACC]
            [AXIS_H] [AXIS_M] [AXIS_L] [CRC]

        speed    : 0–3000 RPM (unsigned, top 4 bits only)
        accel    : 0–255
        abs_axis : signed 24-bit encoder ticks (−8 388 607 … +8 388 607)

        Unlike 0xFE (microsteps), 0xF5 targets are in the same
        coordinate system as 0x31 encoder reads.  Accepts mid-motion
        target updates — the motor re-targets on each command.
        """
        speed = min(max(speed, 0), 3000)
        accel = min(max(accel, 0), 255)

        # Convert signed int to 3-byte big-endian (two's complement)
        if abs_axis < 0:
            axis_bytes = (abs_axis + 0x1000000) & 0xFFFFFF
        else:
            axis_bytes = abs_axis & 0xFFFFFF

        data = [
            MksCommands.RUN_MOTOR_ABSOLUTE_MOTION_BY_AXIS_COMMAND.value,  # 0xF5
            (speed >> 8) & 0x0F,
            speed & 0xFF,
            accel & 0xFF,
            (axis_bytes >> 16) & 0xFF,
            (axis_bytes >> 8) & 0xFF,
            axis_bytes & 0xFF,
        ]
        msg = self._can_msg(motor_id, data)

        try:
            self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().error(
                'CAN TX fail motor {}: {}'.format(motor_id, e))

    def _can_send_speed(self, motor_id, rpm, accel=0):
        """0xF6 Speed Mode (kept for manual override / stop)."""
        speed = min(round(abs(rpm)), 3000)
        dir_bit = 0x80 if rpm < 0 else 0x00

        data = [
            MksCommands.RUN_MOTOR_SPEED_MODE_COMMAND.value,
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
        TX: [0x31] [CRC]           = 2 bytes
        RX: [0x31] [6-byte signed] [CRC] = 8 bytes
        """
        data = [MksCommands.READ_ENCODED_VALUE_ADDITION.value]
        msg = self._can_msg(motor_id, data)

        try:
            self.bus.send(msg)
            resp = self.bus.recv(timeout=0.005)

            if resp and resp.arbitration_id == motor_id:
                if len(resp.data) >= 7 and resp.data[0] == 0x31:
                    return int.from_bytes(
                        resp.data[1:7], 'big', signed=True)
            return None
        except can.CanError as e:
            self.get_logger().warn(
                'Encoder read fail motor {}: {}'.format(motor_id, e))
            return None

    # ═══════════════════════════════════════════════════════════════
    #  Publish
    # ═══════════════════════════════════════════════════════════════

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [self.joint_positions[n] for n in JOINT_NAMES]
        msg.velocity = [self.joint_velocities.get(n, 0.0)
                        for n in JOINT_NAMES]
        msg.effort = []
        self.joint_state_pub.publish(msg)

    # ═══════════════════════════════════════════════════════════════
    #  Shutdown
    # ═══════════════════════════════════════════════════════════════

    def destroy_node(self):
        if not self.use_sim and self.bus:
            self.get_logger().info('Stopping connected motors...')
            for name, motor in self.motors.items():
                if name in self._connected:
                    # Send zero-speed stop
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
