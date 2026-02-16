#!/usr/bin/env python3
"""
Test the exact encoder delta when commanding 3.14 rad via the full pipeline.

1. Read encoder
2. Launch the system, send joint command via action
3. Wait for completion
4. Read encoder again
5. Compare delta vs expected

Run with NO other ROS nodes running.
Must have can0 up.
"""

import can
import time
import subprocess
import sys
import os

workspace_root = '/home/sean/dexter_test_2/ros2_ws'
mks_path = os.path.join(workspace_root, 'src/dexter_hardware/mks-servo-can-main')
if mks_path not in sys.path:
    sys.path.insert(0, mks_path)

MOTOR_ID = 6  # part5


def read_encoder_raw():
    """Read encoder using raw CAN, no ROS needed."""
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
    try:
        # Flush
        while bus.recv(timeout=0.01):
            pass

        data = [0x31]
        crc = (MOTOR_ID + sum(data)) & 0xFF
        msg = can.Message(arbitration_id=MOTOR_ID,
                         data=bytearray(data) + bytes([crc]),
                         is_extended_id=False)
        bus.send(msg)
        resp = bus.recv(timeout=0.2)
        if resp and resp.arbitration_id == MOTOR_ID:
            if len(resp.data) >= 7 and resp.data[0] == 0x31:
                return int.from_bytes(resp.data[1:7], 'big', signed=True)
        return None
    finally:
        bus.shutdown()


def main():
    print("=" * 60)
    print("Full Pipeline Angle Test")
    print("Commanding part5 to 3.14 rad via JTC")
    print("=" * 60)

    # Step 1: Read encoder BEFORE
    enc_before = read_encoder_raw()
    if enc_before is None:
        print("ERROR: Cannot read encoder!")
        return
    print("\nEncoder BEFORE: {} ticks".format(enc_before))
    print("(Make sure the motor is at rest at a known position)")
    print("\nNow launch the system and send the trajectory command.")
    print("After the trajectory completes and motor stops, run this again with --after {}".format(enc_before))


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--after':
        enc_before = int(sys.argv[2])
        # Read encoder AFTER
        bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000)
        try:
            while bus.recv(timeout=0.01):
                pass
            data = [0x31]
            crc = (MOTOR_ID + sum(data)) & 0xFF
            msg = can.Message(arbitration_id=MOTOR_ID,
                             data=bytearray(data) + bytes([crc]),
                             is_extended_id=False)
            bus.send(msg)
            resp = bus.recv(timeout=0.2)
            if resp and resp.arbitration_id == MOTOR_ID:
                if len(resp.data) >= 7 and resp.data[0] == 0x31:
                    enc_after = int.from_bytes(resp.data[1:7], 'big', signed=True)
                    delta = enc_after - enc_before
                    expected_rad = 3.14
                    expected_ticks = expected_rad / (2.0 * 3.14159265) * 16384
                    actual_rad = delta / 16384.0 * 2.0 * 3.14159265
                    print("\nEncoder AFTER:  {} ticks".format(enc_after))
                    print("Delta:          {} ticks".format(delta))
                    print("Expected:       {:.0f} ticks ({:.4f} rad)".format(expected_ticks, expected_rad))
                    print("Actual:         {:.4f} rad = {:.1f} degrees".format(actual_rad, actual_rad * 180 / 3.14159265))
                    print("Ratio (actual/expected): {:.4f}".format(actual_rad / expected_rad))
                else:
                    print("Bad response")
            else:
                print("No response")
        finally:
            bus.shutdown()
    else:
        main()
