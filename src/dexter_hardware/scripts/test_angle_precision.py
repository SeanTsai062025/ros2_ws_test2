#!/usr/bin/env python3
"""Quick test: read motor 6 config and do a precise angle test."""

import can
import time
import sys
import os

workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_path = os.path.join(workspace_root, 'src/dexter_hardware/mks-servo-can-main')
if mks_path not in sys.path:
    sys.path.insert(0, mks_path)

from mks_servo_can import MksServo

MOTOR_ID = 6

bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
notifier = can.Notifier(bus, [])
servo = MksServo(bus, notifier, MOTOR_ID)

print("=" * 50)
print("Motor {} diagnostics".format(MOTOR_ID))
print("=" * 50)

try:
    print("\n--- Encoder value (addition) ---")
    enc = servo.read_encoder_value_addition()
    print("  Raw: {}".format(enc))

    print("\n--- Motor speed ---")
    spd = servo.read_motor_speed()
    print("  Speed: {}".format(spd))

    print("\n--- Motor shaft angle error ---")
    try:
        err = servo.read_motor_shaft_angle_error()
        print("  Error: {}".format(err))
    except Exception as e:
        print("  Error reading: {}".format(e))

    # Now do a precise test: command motor to move exactly 16384 ticks
    # using absolute position mode, then check if it's exactly 1 revolution
    print("\n" + "=" * 50)
    print("PRECISE ANGLE TEST")
    print("=" * 50)

    # Read current position
    enc_before = servo.read_encoder_value_addition()
    print("\nEncoder before: {}".format(enc_before))

    # Use speed mode: send speed=50 CW, wait, stop, measure
    # Instead, let's use a precise approach:
    # Send speed 50 RPM for exactly 1.2 seconds
    # Expected: 50/60 * 1.2 = 1.0 revolution = 16384 ticks (if that's correct)
    print("\nSending speed=50 CW for 1.2 seconds (expecting 1 revolution)...")

    # Build raw CAN frames to be precise
    def can_msg(motor_id, data):
        crc = (motor_id + sum(data)) & 0xFF
        return can.Message(arbitration_id=motor_id,
                          data=bytearray(data) + bytes([crc]),
                          is_extended_id=False)

    # Speed = 50, CW (dir_bit=0)
    speed = 50
    data = [0xF6, (speed >> 8) & 0x0F, speed & 0xFF, 0]
    msg = can_msg(MOTOR_ID, data)
    bus.send(msg)

    time.sleep(1.2)

    # Stop
    data_stop = [0xF6, 0, 0, 0]
    msg_stop = can_msg(MOTOR_ID, data_stop)
    bus.send(msg_stop)
    time.sleep(0.5)

    enc_after = servo.read_encoder_value_addition()
    print("Encoder after: {}".format(enc_after))

    delta = enc_after - enc_before
    print("Delta ticks: {}".format(delta))
    print("If 16384/rev: {:.4f} revolutions = {:.1f} degrees".format(
        delta / 16384.0, delta / 16384.0 * 360.0))
    print("Expected: 1.0 rev = 360 degrees")
    print("Actual/Expected ratio: {:.4f}".format(abs(delta) / 16384.0))

    # Now return motor
    print("\nReturning motor...")
    data_rev = [0xF6, 0x80 | ((speed >> 8) & 0x0F), speed & 0xFF, 0]
    msg_rev = can_msg(MOTOR_ID, data_rev)
    bus.send(msg_rev)
    time.sleep(1.2)
    bus.send(msg_stop)

except Exception as e:
    print("Error: {}".format(e))
    import traceback
    traceback.print_exc()
finally:
    # Stop motor
    data_stop = [0xF6, 0, 0, 0]
    crc = (MOTOR_ID + sum(data_stop)) & 0xFF
    msg_stop = can.Message(arbitration_id=MOTOR_ID,
                          data=bytearray(data_stop) + bytes([crc]),
                          is_extended_id=False)
    bus.send(msg_stop)
    notifier.stop()
    bus.shutdown()
    print("\nDone.")
