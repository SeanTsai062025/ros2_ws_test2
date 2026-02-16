#!/usr/bin/env python3
"""
Speed calibration test for Servo42D motor 6 (part5).

Sends a known speed for a known duration, reads encoder before/after,
and calculates the actual RPM vs commanded RPM.

Usage:
  python3 test_speed_calibration.py

Make sure can0 is up:
  sudo ip link set can0 txqueuelen 1000
  sudo ip link set can0 up type can bitrate 500000
"""

import can
import time
import sys
import os

# Add MKS library
workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_path = os.path.join(workspace_root, 'src/dexter_hardware/mks-servo-can-main')
if mks_path not in sys.path:
    sys.path.insert(0, mks_path)

from mks_servo_can.mks_enums import MksCommands

MOTOR_ID = 6  # part5
ENCODER_TICKS_PER_REV = 16384  # assumed — we'll verify this


def can_msg(motor_id, data):
    """Build CAN message with MKS CRC."""
    crc = (motor_id + sum(data)) & 0xFF
    return can.Message(
        arbitration_id=motor_id,
        data=bytearray(data) + bytes([crc]),
        is_extended_id=False)


def read_encoder(bus, motor_id):
    """Read encoder value addition (0x31)."""
    data = [0x31]
    msg = can_msg(motor_id, data)
    # Flush any stale messages
    while bus.recv(timeout=0.01):
        pass
    bus.send(msg)
    resp = bus.recv(timeout=0.1)
    if resp and resp.arbitration_id == motor_id:
        if len(resp.data) >= 7 and resp.data[0] == 0x31:
            ticks = int.from_bytes(resp.data[1:7], 'big', signed=True)
            return ticks
    return None


def send_speed(bus, motor_id, rpm_value, direction_cw=True):
    """Send 0xF6 speed command. rpm_value is the raw speed register value (0-3000)."""
    speed = min(int(abs(rpm_value)), 3000)
    dir_bit = 0x00 if direction_cw else 0x80
    data = [
        0xF6,
        dir_bit | ((speed >> 8) & 0x0F),
        speed & 0xFF,
        0,  # acceleration = 0 (instant)
    ]
    msg = can_msg(motor_id, data)
    bus.send(msg)


def stop_motor(bus, motor_id):
    """Send speed=0 to stop."""
    send_speed(bus, motor_id, 0)


def main():
    print("=" * 60)
    print("Servo42D Speed Calibration Test")
    print("Motor ID: {}".format(MOTOR_ID))
    print("=" * 60)

    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)

    try:
        # Step 1: Read initial encoder
        enc_start = read_encoder(bus, MOTOR_ID)
        if enc_start is None:
            print("ERROR: Cannot read encoder! Is motor {} powered on?".format(MOTOR_ID))
            return
        print("\nStep 1: Initial encoder = {} ticks".format(enc_start))

        # Step 2: Run at speed=10 for 5 seconds (very slow, safe)
        test_speed = 10  # raw speed register value
        test_duration = 5.0  # seconds

        print("\nStep 2: Sending speed={} CW for {:.1f} seconds...".format(
            test_speed, test_duration))
        send_speed(bus, MOTOR_ID, test_speed, direction_cw=True)

        time.sleep(test_duration)

        stop_motor(bus, MOTOR_ID)
        time.sleep(0.5)  # let motor settle

        # Step 3: Read final encoder
        enc_end = read_encoder(bus, MOTOR_ID)
        if enc_end is None:
            print("ERROR: Cannot read encoder after test!")
            return
        print("Step 3: Final encoder   = {} ticks".format(enc_end))

        # Step 4: Calculate
        delta_ticks = enc_end - enc_start
        print("\n" + "=" * 60)
        print("RESULTS:")
        print("  Delta ticks      = {}".format(delta_ticks))
        print("  Commanded speed  = {} (raw register)".format(test_speed))
        print("  Duration         = {:.1f} s".format(test_duration))

        # If 16384 ticks = 1 rev
        revolutions_16384 = delta_ticks / 16384.0
        actual_rpm_16384 = (revolutions_16384 / test_duration) * 60.0
        print("\n  Assuming 16384 ticks/rev:")
        print("    Revolutions    = {:.4f}".format(revolutions_16384))
        print("    Actual RPM     = {:.2f}".format(actual_rpm_16384))
        print("    Ratio (actual/cmd) = {:.4f}".format(actual_rpm_16384 / test_speed if test_speed else 0))

        # If 3200 ticks = 1 rev (200 steps * 16 microsteps)
        revolutions_3200 = delta_ticks / 3200.0
        actual_rpm_3200 = (revolutions_3200 / test_duration) * 60.0
        print("\n  Assuming 3200 ticks/rev (200*16 microsteps):")
        print("    Revolutions    = {:.4f}".format(revolutions_3200))
        print("    Actual RPM     = {:.2f}".format(actual_rpm_3200))
        print("    Ratio (actual/cmd) = {:.4f}".format(actual_rpm_3200 / test_speed if test_speed else 0))

        # Additional: test with higher speed
        print("\n" + "=" * 60)
        print("Step 5: Second test - speed=100 for 3 seconds")
        enc_start2 = read_encoder(bus, MOTOR_ID)
        send_speed(bus, MOTOR_ID, 100, direction_cw=True)
        time.sleep(3.0)
        stop_motor(bus, MOTOR_ID)
        time.sleep(0.5)
        enc_end2 = read_encoder(bus, MOTOR_ID)

        if enc_start2 is not None and enc_end2 is not None:
            delta2 = enc_end2 - enc_start2
            rev2_16384 = delta2 / 16384.0
            rpm2_16384 = (rev2_16384 / 3.0) * 60.0
            rev2_3200 = delta2 / 3200.0
            rpm2_3200 = (rev2_3200 / 3.0) * 60.0
            print("  Delta ticks      = {}".format(delta2))
            print("  Assuming 16384 ticks/rev: RPM = {:.2f}, ratio = {:.4f}".format(
                rpm2_16384, rpm2_16384 / 100.0))
            print("  Assuming 3200 ticks/rev:  RPM = {:.2f}, ratio = {:.4f}".format(
                rpm2_3200, rpm2_3200 / 100.0))

        # Return to original position
        print("\nStep 6: Returning to start position...")
        send_speed(bus, MOTOR_ID, 50, direction_cw=False)
        time.sleep(3.0)
        stop_motor(bus, MOTOR_ID)
        time.sleep(0.5)
        enc_final = read_encoder(bus, MOTOR_ID)
        print("  Final encoder = {} (started at {})".format(enc_final, enc_start))

    finally:
        stop_motor(bus, MOTOR_ID)
        bus.shutdown()
        print("\nDone. Motor stopped, bus closed.")


if __name__ == '__main__':
    main()
