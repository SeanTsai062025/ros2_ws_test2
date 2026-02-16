#!/usr/bin/env python3
"""
Direct CAN test: simulate the JTC velocity profile and measure motor displacement.
This bypasses the ROS bridge entirely to isolate the motor behavior.

Profile: 1s ramp-up, 2.14s cruise at 9 RPM, 1s ramp-down
Expected displacement: ~3.14 rad (180 degrees)
"""
import can
import struct
import time
import math
import sys

sys.path.insert(0, '/home/sean/dexter_test_2/ros2_ws/src/dexter_hardware/mks-servo-can-main')
from mks_servo_can.mks_enums import MksCommands

MOTOR_ID = 6
TICKS_PER_REV = 16384


def can_msg(motor_id, data):
    """Build CAN message with MKS CRC."""
    crc = (motor_id + sum(data)) & 0xFF
    return can.Message(
        arbitration_id=motor_id,
        data=bytearray(data) + bytes([crc]),
        is_extended_id=False)


def send_speed(bus, motor_id, rpm, accel=0):
    """Send 0xF6 speed command."""
    speed = min(int(abs(rpm)), 3000)
    dir_bit = 0x80 if rpm < 0 else 0x00
    data = [
        MksCommands.RUN_MOTOR_SPEED_MODE_COMMAND.value,
        dir_bit | ((speed >> 8) & 0x0F),
        speed & 0xFF,
        accel & 0xFF,
    ]
    msg = can_msg(motor_id, data)
    bus.send(msg)


def read_encoder(bus, motor_id):
    """Read encoder value."""
    # Flush
    while bus.recv(timeout=0.005):
        pass
    data = [MksCommands.READ_ENCODED_VALUE_ADDITION.value]
    msg = can_msg(motor_id, data)
    bus.send(msg)
    resp = bus.recv(timeout=0.1)
    if resp and resp.arbitration_id == motor_id and len(resp.data) >= 7 and resp.data[0] == 0x31:
        return int.from_bytes(resp.data[1:7], 'big', signed=True)
    return None


def zero_encoder(bus, motor_id):
    """Zero the encoder using 0x92 command through MKS library."""
    notifier = can.Notifier(bus, [])
    sys.path.insert(0, '/home/sean/dexter_test_2/ros2_ws/src/dexter_hardware/mks-servo-can-main')
    from mks_servo_can import MksServo
    servo = MksServo(bus, notifier, motor_id)
    result = servo.set_current_axis_to_zero()
    notifier.stop()
    time.sleep(0.3)
    return result


def main():
    bus = can.interface.Bus(channel='can0', interface='socketcan')

    # Zero encoder first
    print("Zeroing encoder...")
    result = zero_encoder(bus, MOTOR_ID)
    print(f"  Zero result: {result}")
    time.sleep(0.5)

    enc_before = read_encoder(bus, MOTOR_ID)
    print(f"  Encoder before: {enc_before} ticks")

    # === Test 1: Constant speed (no ramp) ===
    # Send speed=9 for the exact duration needed for 3.14 rad
    # At 9 RPM: time = 3.14 / (9 * 2*pi/60) = 3.14 / 0.9425 = 3.332s
    target_rad = 3.14
    cruise_rpm = 9
    cruise_rads = cruise_rpm * 2 * math.pi / 60.0  # 0.9425 rad/s
    cruise_duration = target_rad / cruise_rads  # 3.332s

    print(f"\n=== Test 1: Constant {cruise_rpm} RPM for {cruise_duration:.3f}s ===")
    print(f"  Expected: {target_rad:.3f} rad ({target_rad*180/math.pi:.1f} deg)")

    enc_before = read_encoder(bus, MOTOR_ID)

    t0 = time.time()
    send_speed(bus, MOTOR_ID, cruise_rpm, accel=0)
    while time.time() - t0 < cruise_duration:
        time.sleep(0.01)
    send_speed(bus, MOTOR_ID, 0, accel=0)
    time.sleep(0.5)  # let motor coast to stop

    enc_after = read_encoder(bus, MOTOR_ID)
    delta = abs(enc_after - enc_before)
    actual_rad = delta / TICKS_PER_REV * 2 * math.pi
    print(f"  Encoder: {enc_before} -> {enc_after} (delta={delta} ticks)")
    print(f"  Actual: {actual_rad:.3f} rad ({actual_rad*180/math.pi:.1f} deg)")
    print(f"  Error: {(actual_rad/target_rad - 1)*100:+.1f}%")

    time.sleep(1)

    # === Test 2: Ramp profile (mimicking JTC at 100Hz) ===
    # Ramp up: 1s, 0->9 RPM in integer steps
    # Cruise: 2.14s at 9 RPM
    # Ramp down: 1s, 9->0 RPM in integer steps
    print(f"\n=== Test 2: Ramp profile (100Hz commands) ===")

    # Zero encoder again
    result = zero_encoder(bus, MOTOR_ID)
    time.sleep(0.5)
    enc_before = read_encoder(bus, MOTOR_ID)
    print(f"  Encoder before: {enc_before} ticks")

    dt = 0.01  # 100 Hz
    total_ticks_sent = 0

    # Build velocity profile matching JTC
    # Ramp up: 100 ticks, accel = 1.0 rad/s^2
    # Cruise: ~214 ticks at 1.0 rad/s
    # Ramp down: 100 ticks, decel = 1.0 rad/s^2
    profile = []
    # Ramp up
    for i in range(100):
        t = (i + 1) * dt
        vel = 1.0 * t  # 1.0 rad/s^2
        rpm = vel * 60.0 / (2.0 * math.pi)
        int_rpm = int(rpm)
        profile.append(int_rpm)
    # Cruise
    for i in range(214):
        profile.append(9)
    # Ramp down
    for i in range(100):
        t = (100 - i) * dt
        vel = 1.0 * t
        rpm = vel * 60.0 / (2.0 * math.pi)
        int_rpm = int(rpm)
        profile.append(int_rpm)

    # Compute expected integral
    expected_rad = sum(r * 2.0 * math.pi / 60.0 * dt for r in profile)
    print(f"  Profile: {len(profile)} ticks = {len(profile)*dt:.2f}s")
    print(f"  Int RPM integral: {expected_rad:.3f} rad ({expected_rad*180/math.pi:.1f} deg)")

    # Execute profile
    t0 = time.time()
    for i, rpm_val in enumerate(profile):
        target_time = t0 + (i + 1) * dt
        send_speed(bus, MOTOR_ID, rpm_val, accel=0)
        # Sleep until next tick
        now = time.time()
        if target_time > now:
            time.sleep(target_time - now)

    # Stop
    send_speed(bus, MOTOR_ID, 0, accel=0)
    elapsed = time.time() - t0
    print(f"  Actual execution time: {elapsed:.3f}s")

    time.sleep(0.5)

    enc_after = read_encoder(bus, MOTOR_ID)
    delta = abs(enc_after - enc_before)
    actual_rad = delta / TICKS_PER_REV * 2 * math.pi
    print(f"  Encoder: {enc_before} -> {enc_after} (delta={delta} ticks)")
    print(f"  Actual: {actual_rad:.3f} rad ({actual_rad*180/math.pi:.1f} deg)")
    print(f"  Error vs int integral: {(actual_rad/expected_rad - 1)*100:+.1f}%")
    print(f"  Error vs 3.14 target:  {(actual_rad/3.14 - 1)*100:+.1f}%")

    # === Test 3: Same profile but only send ONE speed command at transitions ===
    # Instead of 100Hz updates, only change speed when the integer RPM changes
    print(f"\n=== Test 3: Sparse commands (only on RPM change) ===")

    result = zero_encoder(bus, MOTOR_ID)
    time.sleep(0.5)
    enc_before = read_encoder(bus, MOTOR_ID)
    print(f"  Encoder before: {enc_before} ticks")

    # Execute profile but only send when RPM changes
    t0 = time.time()
    last_rpm = -1
    cmds_sent = 0
    for i, rpm_val in enumerate(profile):
        target_time = t0 + (i + 1) * dt
        if rpm_val != last_rpm:
            send_speed(bus, MOTOR_ID, rpm_val, accel=0)
            last_rpm = rpm_val
            cmds_sent += 1
        now = time.time()
        if target_time > now:
            time.sleep(target_time - now)

    send_speed(bus, MOTOR_ID, 0, accel=0)
    elapsed = time.time() - t0
    print(f"  Commands sent: {cmds_sent} (vs {len(profile)} in Test 2)")
    print(f"  Actual execution time: {elapsed:.3f}s")

    time.sleep(0.5)

    enc_after = read_encoder(bus, MOTOR_ID)
    delta = abs(enc_after - enc_before)
    actual_rad = delta / TICKS_PER_REV * 2 * math.pi
    print(f"  Encoder: {enc_before} -> {enc_after} (delta={delta} ticks)")
    print(f"  Actual: {actual_rad:.3f} rad ({actual_rad*180/math.pi:.1f} deg)")
    print(f"  Error vs 3.14 target: {(actual_rad/3.14 - 1)*100:+.1f}%")

    bus.shutdown()
    print("\nDone!")


if __name__ == '__main__':
    main()
