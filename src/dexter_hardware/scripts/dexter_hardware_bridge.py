#!/usr/bin/env python3
"""
Dexter Hardware Bridge v5.0 — Single-rate 100 Hz pipeline
                               + Unified Linear Blending.

Architecture
────────────
MoveIt → Commander → JTC (arm_controller, mock hardware @ 100 Hz)
                        │
   /arm_controller/controller_state (100 Hz, reference.positions)
                        │
             Hardware Bridge (this node, 100 Hz timer)
   ├─ READ  : 0x31 encoder — ALL connected motors every cycle
   ├─ PUBLISH: /encoder_joint_states
   └─ WRITE : 0xF5 absolute-position by axis (encoder ticks)
              — ALL connected motors every cycle

Part 5 is a direct 100 Hz pass-through: its target is exactly the JTC reference
position with no look-ahead or blending, and its MKS acceleration field is zero
(no motor-side ramp). CAN ID 6 is configured for SR_CLOSE at the Servo42D's
documented 1600 mA default current and 128 subdivisions. SR_CLOSE avoids the
large low-speed hunting measured with the self-adapting SR_vFOC loop. At 128
subdivisions one speed-field unit is nominally 0.125 RPM instead of 1 RPM, so
the Part 5 speed field is scaled by 8 to preserve the requested physical RPM.

Everything runs at 100 Hz: JTC update_rate, state_publish_rate, and
this node's timer.  No down-sampling or rate mismatch.
Every cycle reads ALL connected encoders and writes ALL connected motors.

CAN bus budget (1 Mbps):
  Standard CAN frame ≈ 0.13 ms on wire.
  Per cycle (6 motors): 6 read TX + 6 read RX + 6 write TX = 18 frames
  ≈ 2.3 ms  ≪  10 ms cycle budget.  Comfortable margin.

Unified Linear Blending Model (Parts 0–4 only)
──────────────────────────────────────────────
A velocity-dependent look-ahead that provides nonlinear damping:

  w      = clamp(|V_TJC| / 0.5, 0.0, 1.0)
  T_eff  = T_max × w                          (T_max = 30 ms)
  Lead   = clamp(V_TJC × T_eff, ±0.030 rad)
  Target = D_TJC + Lead

At rest the weight w → 0, so the target equals the JTC reference
(stiff hold, no overshoot).  During fast motion w → 1, giving a full
30 ms look-ahead that smooths tracking and reduces phase lag. The lead offset
is capped at 30 mrad so operation above the original 1 rad/s maximum cannot
increase the maximum extrapolated position.

T_max rationale: At 100 Hz the dt is 10 ms. A 30 ms look-ahead (3 cycles)
is proportionally equivalent to the 100 ms (3 cycles) used at 30 Hz,
providing the same phase-lag compensation without overshooting.

Conversions applied to blended target → motor command:
  1. 30:1 gear ratio  (radians ↔ encoder ticks)
  2. Per-motor direction sign  (cmd_direction / enc_direction)
  3. JTC reference velocity → 0xF5 motor-side RPM (× gear_ratio)
No additional smoothing or scaling.

0xF5 parameters:
  speed = derived from JTC reference.velocities (joint rad/s → motor RPM)
          The 30:1 axes use the shared floor/fallback. Part 5 uses 128
          subdivisions and an 8× speed-field scale (0.125 RPM/unit).
  accel = 0. Part 5 therefore has no motor-side acceleration smoothing.

Encoder read:  0x31  (addition mode, signed 48-bit ticks)
"""

import rclpy
from rclpy.node import Node
import can
import sys
import os
import math
import time
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

# Default velocity parameters for the 30:1 axes' 0xF5 speed field.
# The speed is derived from JTC reference.velocities each cycle.
# F5_MAX_SPEED   : hard ceiling — 0xF5 protocol limit.
# F5_MIN_SPEED   : floor so the motor still moves on tiny velocity commands.
# F5_FALLBACK_SPEED : used when JTC velocity is zero (idle hold / final waypoint).
F5_MAX_SPEED = 3000       # RPM (motor-shaft)
F5_MIN_SPEED = 10         # RPM — avoid stalling on very small motions
F5_FALLBACK_SPEED = 300   # RPM — moderate speed for hold / re-target

# Part 5 is direct drive, unlike the other 30:1 axes. Hardware capture showed
# that SR_vFOC's self-adapting current loop hunted badly at this axis's low
# direct-drive speeds, even with a smooth 100 Hz F6 command. SR_CLOSE at the
# Servo42D's documented 1600 mA default eliminated the reversals and pulses.
# At the motor's previous 16-subdivision setting, the integer speed field had
# 1 RPM resolution. The MKS manual specifies 1/8 physical speed at 128
# subdivisions, so multiplying desired RPM by 8 gives 0.125 RPM command
# resolution without changing the requested physical speed. Acceleration 0
# disables the motor-side ramp.
PART5_WORK_MODE = 4       # SR_CLOSE: serial interface, encoder closed loop
PART5_WORKING_CURRENT_MA = 1600
PART5_SUBDIVISIONS = 128
PART5_SPEED_FIELD_SCALE = 8.0
PART5_MIN_SPEED = 1
PART5_FALLBACK_SPEED = 1
PART5_ACCEL = 0

# ── Unified Linear Blending parameters ────────────────────────
# Dynamic look-ahead that scales with velocity for smoother motion.
#   w = clamp(|V_TJC| / BLEND_VEL_THRESHOLD, 0.0, 1.0)
#   T_eff = T_LOOKAHEAD_MAX * w
#   Lead = clamp(V_TJC * T_eff, ±MAX_LOOKAHEAD_OFFSET)
#   Target = D_TJC + Lead
#
# At 100 Hz (dt = 10 ms) a 30 ms look-ahead spans 3 control cycles,
# which is proportionally equivalent to 100 ms at 30 Hz (also 3 cycles).
# This preserves the same phase-lag compensation ratio while the higher
# update rate inherently provides smoother tracking.
BLEND_VEL_THRESHOLD = 0.5   # rad/s — velocity at which blending is fully active
T_LOOKAHEAD_MAX = 0.030      # seconds (30 ms) — maximum look-ahead time
MAX_LOOKAHEAD_OFFSET = 0.030  # radians — preserves the old 1 rad/s maximum lead


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
    min_speed_rpm: int = F5_MIN_SPEED
    fallback_speed_rpm: int = F5_FALLBACK_SPEED
    acceleration: int = F5_ACCEL
    speed_field_scale: float = 1.0


class DexterHardwareBridge(Node):

    LOOP_RATE = 100.0  # Hz — matches JTC update_rate exactly

    def __init__(self):
        super().__init__('dexter_hardware_bridge')

        # ── Parameters ──────────────────────────────────────────────
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('use_sim', False)
        self.declare_parameter('connected_joints',
                               ['base', 'part1', 'part2', 'part3', 'part4', 'part5'])

        self.use_sim = self.get_parameter('use_sim').value
        self.dt = 1.0 / self.LOOP_RATE
        self._connected = set(
            self.get_parameter('connected_joints').value)

        # ── Motor map ───────────────────────────────────────────────
        # Gear ratios and command directions are the existing calibrated values
        # and are intentionally unchanged here.
        #
        # cmd_direction:  1 → positive MoveIt rad → positive 0xF5 target ticks
        #                -1 → positive MoveIt rad → negative 0xF5 target ticks
        # enc_direction:  1 → positive raw 0x31 ticks = positive URDF rad
        #                -1 → positive raw 0x31 ticks = negative URDF rad
        #
        # Command and encoder directions are independent.  The command
        # directions below are the calibrated motor-control signs.  The
        # encoder directions map raw 0x31 counts into the URDF/MoveIt joint
        # convention used on /joint_states.
        self.motors = {
            'base':  MotorConfig(joint_name='base',  can_id=1,
                                 gear_ratio=30.0, cmd_direction=1, enc_direction=1),
            'part1': MotorConfig(joint_name='part1', can_id=2,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=1),
            'part2': MotorConfig(joint_name='part2', can_id=3,
                                 gear_ratio=30.0, cmd_direction=-1, enc_direction=-1),
            'part3': MotorConfig(joint_name='part3', can_id=4,
                                 gear_ratio=30.0, cmd_direction=1, enc_direction=1),
            'part4': MotorConfig(joint_name='part4', can_id=5,
                                 gear_ratio=30.0, cmd_direction=1, enc_direction=1),
            'part5': MotorConfig(joint_name='part5', can_id=6,
                                 gear_ratio=1.0, cmd_direction=-1, enc_direction=-1,
                                 min_speed_rpm=PART5_MIN_SPEED,
                                 fallback_speed_rpm=PART5_FALLBACK_SPEED,
                                 acceleration=PART5_ACCEL,
                                 speed_field_scale=PART5_SPEED_FIELD_SCALE),
        }

        # ── State ──────────────────────────────────────────────────
        self.joint_positions = {n: 0.0 for n in JOINT_NAMES}
        self.joint_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._ref_positions = {n: 0.0 for n in JOINT_NAMES}
        self._ref_velocities = {n: 0.0 for n in JOINT_NAMES}
        self._last_ref_positions = {n: 0.0 for n in JOINT_NAMES}
        self._jtc_active = False
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
                if 'part5' in self._connected:
                    self._configure_part5_driver()
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

        # ── Control timer (100 Hz — single rate) ──────────────────
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.loop_count = 0

        mode_str = 'SIM' if self.use_sim else 'HARDWARE'
        js_topic = '/joint_states' if self.use_sim else '/encoder_joint_states'
        self.get_logger().info('=' * 56)
        self.get_logger().info('  Dexter Hardware Bridge v5.0')
        self.get_logger().info('  Mode     : {}'.format(mode_str))
        self.get_logger().info('  Rate     : {} Hz (single-rate)'.format(
            self.LOOP_RATE))
        self.get_logger().info('  Cmd mode : 0xF5 absolute position')
        self.get_logger().info('  F5 speed : from JTC vel (fallback {} RPM)'.format(
            F5_FALLBACK_SPEED))
        self.get_logger().info('  F5 accel : {} (fixed)'.format(F5_ACCEL))
        self.get_logger().info(
            '  Part 5   : direct JTC target, SR_CLOSE, {} mA, '
            '{} subdivisions, 0.125 RPM/unit, accel={}'.format(
                PART5_WORKING_CURRENT_MA, PART5_SUBDIVISIONS,
                PART5_ACCEL))
        self.get_logger().info('  Blending : vel_thresh={} rad/s, '
                               'T_max={} ms, lead_max={} rad'.format(
            BLEND_VEL_THRESHOLD, T_LOOKAHEAD_MAX * 1000.0,
            MAX_LOOKAHEAD_OFFSET))
        self.get_logger().info('  Enc read : ALL connected motors every cycle')
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
    #  Control Loop  (100 Hz — single rate)
    # ═══════════════════════════════════════════════════════════════

    def _control_loop(self):
        """Single-rate 100 Hz loop: read ALL → publish → write ALL."""
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
            # ── READ ALL encoders ───────────────────────────────────
            self._hw_read_encoders()

            # ── WRITE ALL position commands (only after encoders init) ──
            if self._write_enabled:
                if self._manual_mode:
                    self._hw_write_manual_velocities()
                else:
                    self._hw_write_positions(self._ref_positions)

        # ── PUBLISH joint states ───────────────────────────────────
        self._publish_joint_states()

        # Periodic log
        if self.loop_count % 500 == 0:  # every 5 sec at 100 Hz
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
        """Read ALL connected motor encoders every 100 Hz cycle.

        At 1 Mbps CAN, each read is a 2-byte TX + 8-byte RX pair
        (~0.26 ms).  With 6 motors that's ~1.6 ms — well within the
        10 ms cycle budget even including the write phase (~0.8 ms).

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

        # Normal operation: read ALL connected motors every cycle
        for name in connected:
            motor = self.motors[name]
            ticks = self._can_read_encoder(motor.can_id)
            if ticks is not None:
                revs = ticks / motor.encoder_ticks_per_rev / motor.gear_ratio
                self.joint_positions[name] = (
                    revs * 2.0 * math.pi * motor.enc_direction)

    # ═══════════════════════════════════════════════════════════════
    #  Hardware CAN — Position Write  (0xF5)
    # ═══════════════════════════════════════════════════════════════

    def _hw_write_positions(self, pos_cmds):
        """Send 0xF5 absolute-position-by-axis commands to ALL connected motors.

        Unified Linear Blending Model (Parts 0–4 only):
          w      = clamp(|V_TJC| / BLEND_VEL_THRESHOLD, 0, 1)
          T_eff  = T_LOOKAHEAD_MAX × w
          lead   = clamp(V_TJC × T_eff, ±MAX_LOOKAHEAD_OFFSET)
          target = D_TJC + lead

        The blending weight *w* rises linearly with velocity so that:
          • At rest (vel ≈ 0) → w ≈ 0 → no look-ahead, target = reference.
          • At full speed      → w = 1 → full 30 ms look-ahead.
        This gives nonlinear damping: stiff holding at rest, smooth
        tracking during fast motions.

        At 100 Hz (dt = 10 ms) T_max = 30 ms spans 3 control cycles,
        matching the 3-cycle ratio from the original 30 Hz / 100 ms design.

        Part 5 bypasses this model: target = JTC reference position, with no
        look-ahead and no motor-side acceleration ramp. Its F5 frame is sent
        every bridge cycle.

        Conversions (after blending):
          1. Position: 30:1 gear ratio (radians → encoder ticks) + direction sign
          2. Velocity: JTC reference velocity (joint rad/s) → motor-shaft RPM
             vel_motor = vel_joint × gear_ratio  (motor spins 30× faster)
             RPM = vel_motor × 60 / (2π)
        """
        for name, motor in self.motors.items():
            if name not in self._connected:
                continue

            ref_pos = pos_cmds[name]
            ref_vel = self._ref_velocities.get(name, 0.0)

            if name == 'part5':
                # Part 5 is an exact 100 Hz JTC pass-through. Do not apply the
                # global velocity blend or any look-ahead to this joint.
                w = 0.0
                t_eff = 0.0
                target_rad = ref_pos
            else:
                # Existing unified blending remains unchanged on Parts 0–4.
                w = min(abs(ref_vel) / BLEND_VEL_THRESHOLD, 1.0)
                t_eff = T_LOOKAHEAD_MAX * w
                lookahead_offset = max(
                    -MAX_LOOKAHEAD_OFFSET,
                    min(MAX_LOOKAHEAD_OFFSET, ref_vel * t_eff))
                target_rad = ref_pos + lookahead_offset

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
            motor_rad_s = abs(ref_vel) * motor.gear_ratio
            motor_rpm = motor_rad_s * 60.0 / (2.0 * math.pi)

            if name == 'part5':
                # At 128 subdivisions each speed-field unit is nominally
                # 0.125 RPM. Scaling desired RPM by 8 preserves physical speed
                # while increasing command resolution eightfold.
                if motor_rpm < 1e-6:
                    speed = motor.fallback_speed_rpm
                else:
                    speed = int(round(
                        motor_rpm * motor.speed_field_scale))
                    speed = max(motor.min_speed_rpm,
                                min(F5_MAX_SPEED, speed))
            elif motor_rpm < 0.1:
                # Velocity is ~zero: trajectory endpoint or idle hold.
                # Use a moderate fallback so the motor can still re-target
                # if the position is slightly off.
                speed = motor.fallback_speed_rpm
            else:
                speed = int(round(motor_rpm))
                speed = max(motor.min_speed_rpm,
                            min(F5_MAX_SPEED, speed))

            # Forward Part 5 on every 100 Hz bridge cycle, even when adjacent
            # floating-point references quantize to the same encoder tick.
            # Parts 0–4 retain their existing change-only behavior.
            if name == 'part5' or target_ticks != self._last_sent_ticks[name]:
                self._can_send_absolute_axis(
                    motor.can_id, speed, motor.acceleration, target_ticks)
                self.get_logger().debug(
                    '  CAN TX M{}: ref={:.4f} target={:.4f} rad '
                    '(w={:.2f} T={:.1f}ms) -> {} ticks @ speed_field={}'.format(
                        motor.can_id, ref_pos, target_rad,
                        w, t_eff * 1000.0, target_ticks, speed))
                self._last_sent_ticks[name] = target_ticks

    def _hw_write_manual_velocities(self):
        """Fallback: send 0xF6 speed-mode commands for manual override."""
        for name, motor in self.motors.items():
            if name not in self._connected:
                continue
            vel_rad_s = self._manual_vels[name]
            rpm = (vel_rad_s * 60.0) / (2.0 * math.pi) * motor.cmd_direction
            self._can_send_speed(
                motor.can_id, rpm * motor.speed_field_scale, accel=0)

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

    def _can_wait_for(self, motor_id, command, timeout=0.2):
        """Wait for one checksum-valid response from a specific motor."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            response = self.bus.recv(timeout=deadline - time.monotonic())
            if response is None or response.arbitration_id != motor_id:
                continue
            data = bytes(response.data)
            if len(data) < 3 or data[0] != command:
                continue
            if ((motor_id + sum(data[:-1])) & 0xFF) == data[-1]:
                return data
        return None

    def _can_read_system_parameter(self, motor_id, parameter):
        """Read one MKS system parameter using command 0x00."""
        while self.bus.recv(timeout=0.0) is not None:
            pass
        self.bus.send(self._can_msg(motor_id, [0x00, parameter]))
        response = self._can_wait_for(motor_id, parameter)
        if response is None or response[1:-1] == b'\xff\xff':
            return None
        return int.from_bytes(response[1:-1], 'big', signed=False)

    def _can_set_system_parameter(self, motor_id, parameter, payload):
        """Set one MKS parameter and require a successful acknowledgement."""
        while self.bus.recv(timeout=0.0) is not None:
            pass
        self.bus.send(self._can_msg(motor_id, [parameter] + list(payload)))
        response = self._can_wait_for(motor_id, parameter)
        if response is None or response[1] != 1:
            raise RuntimeError(
                'Motor {} rejected parameter 0x{:02X}: {}'.format(
                    motor_id, parameter, response))
        time.sleep(0.05)

    def _configure_part5_driver(self):
        """Configure and verify only Part 5 / CAN ID 6.

        SR_vFOC caused a measured low-speed hunt on the direct-drive joint:
        the encoder repeatedly stopped, reversed, and surged despite smooth
        position and velocity commands. SR_CLOSE uses fixed current in the
        encoder closed loop and removed that oscillation in hardware tests.
        """
        motor_id = self.motors['part5'].can_id

        work_mode = self._can_read_system_parameter(motor_id, 0x82)
        if work_mode != PART5_WORK_MODE:
            self.get_logger().info(
                'Part 5 work mode: {} -> {} (SR_CLOSE)'.format(
                    work_mode, PART5_WORK_MODE))
            self._can_set_system_parameter(
                motor_id, 0x82, [PART5_WORK_MODE])
            work_mode = self._can_read_system_parameter(motor_id, 0x82)
        if work_mode != PART5_WORK_MODE:
            raise RuntimeError(
                'Part 5 work-mode verification failed: {}'.format(work_mode))

        working_current = self._can_read_system_parameter(motor_id, 0x83)
        if working_current != PART5_WORKING_CURRENT_MA:
            self.get_logger().info(
                'Part 5 working current: {} -> {} mA'.format(
                    working_current, PART5_WORKING_CURRENT_MA))
            self._can_set_system_parameter(
                motor_id, 0x83,
                [(PART5_WORKING_CURRENT_MA >> 8) & 0xFF,
                 PART5_WORKING_CURRENT_MA & 0xFF])
            working_current = self._can_read_system_parameter(motor_id, 0x83)
        if working_current != PART5_WORKING_CURRENT_MA:
            raise RuntimeError(
                'Part 5 current verification failed: {}'.format(
                    working_current))

        subdivisions = self._can_read_system_parameter(motor_id, 0x84)
        if subdivisions != PART5_SUBDIVISIONS:
            self.get_logger().info(
                'Part 5 subdivisions: {} -> {}'.format(
                    subdivisions, PART5_SUBDIVISIONS))
            self._can_set_system_parameter(
                motor_id, 0x84, [PART5_SUBDIVISIONS])
            subdivisions = self._can_read_system_parameter(motor_id, 0x84)

        if subdivisions != PART5_SUBDIVISIONS:
            raise RuntimeError(
                'Part 5 subdivision verification failed: {}'.format(
                    subdivisions))
        self.get_logger().info(
            'Part 5 driver verified: SR_CLOSE, {} mA, {} subdivisions '
            '(nominal 0.125 RPM/speed unit)'.format(
                working_current, subdivisions))

    def _can_send_absolute_axis(self, motor_id, speed, accel, abs_axis):
        """
        0xF5 — Absolute Motion by Axis (encoder ticks).

        Frame (8 bytes on wire):
            [0xF5] [SPD_H] [SPD_L] [ACC]
            [AXIS_H] [AXIS_M] [AXIS_L] [CRC]

        speed    : 0–3000 protocol units (RPM at 16/32/64 subdivisions;
                   nominally 0.125 RPM/unit for Part 5 at 128 subdivisions)
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
            if motor_id == 6:
                # F5 can publish both "starting" and "complete" status
                # frames. During 100 Hz Part 5 streaming those replies can
                # sit ahead of the requested 0x31 frame and starve feedback.
                # Drain only this direct-drive axis before making a fresh
                # request; no other motor's receive behavior is changed.
                while self.bus.recv(timeout=0.0) is not None:
                    pass

            self.bus.send(msg)
            deadline = time.monotonic() + 0.005
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                resp = self.bus.recv(timeout=remaining)
                if resp is None:
                    return None
                if (resp.arbitration_id == motor_id
                        and len(resp.data) >= 7
                        and resp.data[0] == 0x31):
                    return int.from_bytes(
                        resp.data[1:7], 'big', signed=True)
                if motor_id != 6:
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
