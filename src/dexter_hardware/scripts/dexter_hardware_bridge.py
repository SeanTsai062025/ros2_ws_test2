#!/usr/bin/env python3
"""
Dexter Hardware Bridge v4.0 — Single-rate 30 Hz pipeline.

Architecture
────────────
MoveIt → Commander → JTC (arm_controller, mock hardware @ 30 Hz)
                        │
   /arm_controller/controller_state (30 Hz, reference.positions)
                        │
             Hardware Bridge (this node, 30 Hz timer)
   ├─ READ  : 0x31 encoder (round-robin across connected motors)
   ├─ PUBLISH: /encoder_joint_states
   └─ WRITE : 0xF5 absolute-position by axis (encoder ticks)

Everything runs at 30 Hz: JTC update_rate, state_publish_rate, and
this node's timer.  No down-sampling or rate mismatch.

Conversions applied to JTC reference → motor command:
  1. 30:1 gear ratio  (radians ↔ encoder ticks)
  2. Per-motor direction sign  (cmd_direction / enc_direction)
  3. JTC reference velocity → 0xF5 motor-side RPM (× gear_ratio)
No additional smoothing or scaling.

0xF5 parameters:
  speed = derived from JTC reference.velocities (joint rad/s → motor RPM)
          Clamped to [F5_MIN_SPEED, 3000] RPM.  Falls back to F5_FALLBACK_SPEED
          when JTC velocity is zero or unavailable (idle / final waypoint).
  accel = F5_ACCEL (fixed)

Encoder read:  0x31  (addition mode, signed 48-bit ticks)
"""

import rclpy
from rclpy.node import Node
import can
import sys
import os
import math
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

# Fixed 0xF5 acceleration.
F5_ACCEL = 0

# Velocity parameters for 0xF5 speed field.
# The speed is derived from JTC reference.velocities each cycle.
# F5_MAX_SPEED   : hard ceiling — 0xF5 protocol limit.
# F5_MIN_SPEED   : floor so the motor still moves on tiny velocity commands.
# F5_FALLBACK_SPEED : used when JTC velocity is zero (idle hold / final waypoint).
F5_MAX_SPEED = 3000       # RPM (motor-shaft)
F5_MIN_SPEED = 10         # RPM — avoid stalling on very small motions
F5_FALLBACK_SPEED = 300   # RPM — moderate speed for hold / re-target


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

    LOOP_RATE = 30.0  # Hz — matches JTC update_rate exactly

    def __init__(self):
        super().__init__('dexter_hardware_bridge')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('use_sim', False)
        self.declare_parameter('connected_joints', ['part3', 'part5'])

        self.use_sim = self.get_parameter('use_sim').value
        self.dt = 1.0 / self.LOOP_RATE
        self._connected = set(
            self.get_parameter('connected_joints').value)

        # ── Motor map ───────────────────────────────────────────────
        # All motors have 30:1 gear reducer with output direction opposite to motor.
        #
        # cmd_direction: -1 → reducer output direction opposite to motor rotation
        # enc_direction: -1 for motors 2 & 6 (encoder feedback opposite to command)
        #                 1 for motors 1, 3, 4, 5 (normal behavior)
        self.motors = {
            'base':  MotorConfig(joint_name='base',  can_id=1,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=1),
            'part1': MotorConfig(joint_name='part1', can_id=2,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=-1),
            'part2': MotorConfig(joint_name='part2', can_id=3,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=1),
            'part3': MotorConfig(joint_name='part3', can_id=4,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=1),
            'part4': MotorConfig(joint_name='part4', can_id=5,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=1),
            'part5': MotorConfig(joint_name='part5', can_id=6,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=-1),
        }

        # ── State ──────────────────────────────────────────────────
        self.joint_positions = {n: 0.0 for n in JOINT_NAMES}
        self.joint_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._ref_positions = {n: 0.0 for n in JOINT_NAMES}
        self._ref_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._last_ref_positions = {n: 0.0 for n in JOINT_NAMES}
        self._jtc_active = False
        self._encoder_read_index = 0
        self._last_sent_ticks = {n: 0 for n in JOINT_NAMES}

        # Startup safety: don't send motor commands until we have read
        # every connected motor's encoder at least once.  This prevents
        # the bridge from forwarding a bogus JTC reference (e.g. from
        # mock-hardware initial_positions) that differs from the real
        # motor position, which would cause a violent snap on boot.
        self._encoders_initialized = set()  # names we've read at least once
        self._write_enabled = False         # flips True once all connected are read

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

        # JTC controller state (reference positions at 30 Hz)
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

        # ── Control timer (30 Hz — single rate) ───────────────────
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.loop_count = 0

        mode_str = 'SIM' if self.use_sim else 'HARDWARE'
        js_topic = '/joint_states' if self.use_sim else '/encoder_joint_states'
        self.get_logger().info('=' * 56)
        self.get_logger().info('  Dexter Hardware Bridge v4.0')
        self.get_logger().info('  Mode     : {}'.format(mode_str))
        self.get_logger().info('  Rate     : {} Hz (single-rate)'.format(
            self.LOOP_RATE))
        self.get_logger().info('  Cmd mode : 0xF5 absolute position')
        self.get_logger().info('  F5 speed : from JTC vel (fallback {} RPM)'.format(
            F5_FALLBACK_SPEED))
        self.get_logger().info('  F5 accel : {} (fixed)'.format(F5_ACCEL))
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
                # Extract reference velocity (joint-space rad/s)
                if i < len(ref_vel):
                    self._ref_velocities[name] = ref_vel[i]
                else:
                    self._ref_velocities[name] = 0.0
                # Detect trajectory activity: reference changed significantly.
                # Threshold of 0.001 rad (~0.06°) filters out floating-point
                # jitter from the JTC while still catching real motion.
                if abs(ref_pos[i] - self._last_ref_positions.get(name, 0.0)) > 0.001:
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
    #  Control Loop  (30 Hz — single rate)
    # ═══════════════════════════════════════════════════════════════

    def _control_loop(self):
        """Single-rate 30 Hz loop: read → publish → write."""
        self.loop_count += 1

        if self.use_sim:
            # Sim mode: integrate manual velocities or track reference
            if self._manual_mode:
                for name in JOINT_NAMES:
                    self.joint_positions[name] += (
                        self._manual_vels[name] * self.dt)
            else:
                for name in JOINT_NAMES:
                    self.joint_positions[name] = self._ref_positions[name]
        else:
            # ── READ encoder(s) ────────────────────────────────────
            self._hw_read_encoders()

            # ── WRITE position command (only after encoders init) ──
            if self._write_enabled:
                if self._manual_mode:
                    self._hw_write_manual_velocities()
                else:
                    self._hw_write_positions(self._ref_positions)

        # ── PUBLISH joint states ───────────────────────────────────
        self._publish_joint_states()

        # Periodic log
        if self.loop_count % 150 == 0:  # every 5 sec at 30 Hz
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

        On startup, reads ALL connected motors sequentially (once each)
        before enabling writes, so we never command a position we haven't
        verified the motor is actually near.
        """
        connected = [n for n in JOINT_NAMES if n in self._connected]
        if not connected:
            return

        if not self._write_enabled:
            # Startup phase: read every connected motor once
            for name in connected:
                if name in self._encoders_initialized:
                    continue
                motor = self.motors[name]
                ticks = self._can_read_encoder(motor.can_id)
                if ticks is not None:
                    revs = ticks / motor.encoder_ticks_per_rev / motor.gear_ratio
                    self.joint_positions[name] = (
                        revs * 2.0 * math.pi * motor.enc_direction)
                    self._encoders_initialized.add(name)
                    self.get_logger().info(
                        'Encoder init {}: {} ticks = {:.4f} rad'.format(
                            name, ticks, self.joint_positions[name]))
            # Check if all connected motors have been read
            if self._encoders_initialized >= self._connected:
                self._write_enabled = True
                self.get_logger().info(
                    'All encoders initialized — motor writes ENABLED')
            return

        # Normal operation: round-robin one motor per tick
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
    #  Hardware CAN — Position Write  (0xF5)
    # ═══════════════════════════════════════════════════════════════

    def _hw_write_positions(self, pos_cmds):
        """Send 0xF5 absolute-position-by-axis commands.

        Conversions:
          1. Position: 30:1 gear ratio (radians → encoder ticks) + direction sign
          2. Velocity: JTC reference velocity (joint rad/s) → motor-shaft RPM
             vel_motor = vel_joint × gear_ratio  (motor spins 30× faster)
             RPM = vel_motor × 60 / (2π)
        """
        for name, motor in self.motors.items():
            if name not in self._connected:
                continue

            target_rad = pos_cmds[name]

            # ── Convert radians → encoder ticks ────────────────────
            target_ticks = round(
                target_rad / (2.0 * math.pi)
                * motor.encoder_ticks_per_rev
                * motor.gear_ratio
                * motor.cmd_direction)

            # Clamp to 24-bit signed range
            if abs(target_ticks) > MAX_AXIS:
                self.get_logger().warn(
                    '0xF5 OVERFLOW CLAMP {}: {} ticks exceeds ±{} — '
                    'target clamped! Check encoder / zero position.'.format(
                        name, target_ticks, MAX_AXIS))
            target_ticks = max(-MAX_AXIS, min(MAX_AXIS, target_ticks))

            # ── Convert JTC velocity → motor-shaft RPM ─────────────
            # JTC reference.velocities is in joint-space rad/s.
            # Motor shaft spins gear_ratio× faster than the output.
            ref_vel = self._ref_velocities.get(name, 0.0)
            motor_rad_s = abs(ref_vel) * motor.gear_ratio
            motor_rpm = motor_rad_s * 60.0 / (2.0 * math.pi)

            if motor_rpm < 0.1:
                # Velocity is ~zero: trajectory endpoint or idle hold.
                # Use a moderate fallback so the motor can still re-target
                # if the position is slightly off.
                speed = F5_FALLBACK_SPEED
            else:
                speed = int(round(motor_rpm))
                speed = max(F5_MIN_SPEED, min(F5_MAX_SPEED, speed))

            # Only send when tick target changes
            if target_ticks != self._last_sent_ticks[name]:
                self._can_send_absolute_axis(
                    motor.can_id, speed, F5_ACCEL, target_ticks)
                self.get_logger().debug(
                    '  CAN TX M{}: {:.4f} rad -> {} ticks @ {} RPM'.format(
                        motor.can_id, target_rad, target_ticks, speed))
                self._last_sent_ticks[name] = target_ticks

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
