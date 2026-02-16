#!/usr/bin/env python3
"""
Precise speed test: Send a known integer RPM for a known duration,
measure encoder delta to verify motor speed accuracy at low RPM.

This tests whether the motor actually runs at the integer RPM we command.
"""
import can
import struct
import time
import math

MOTOR_ID = 6
TICKS_PER_REV = 16384

def read_encoder(bus):
    """Read encoder with retry."""
    # Flush
    while bus.recv(timeout=0.01):
        pass
    
    crc = (MOTOR_ID + 0x31) & 0xFF
    msg = can.Message(arbitration_id=MOTOR_ID, data=[0x31, crc], is_extended_id=False)
    
    for attempt in range(5):
        bus.send(msg)
        time.sleep(0.02)
        resp = bus.recv(timeout=0.1)
        if resp and resp.arbitration_id == MOTOR_ID and len(resp.data) >= 7 and resp.data[0] == 0x31:
            return int.from_bytes(resp.data[1:7], 'big', signed=True)
    return None

def send_speed(bus, rpm_int, direction_cw=False):
    """Send 0xF6 speed command with integer RPM value."""
    speed = min(abs(rpm_int), 3000)
    dir_bit = 0x80 if direction_cw else 0x00
    data = [
        0xF6,
        dir_bit | ((speed >> 8) & 0x0F),
        speed & 0xFF,
        0  # accel=0 (use motor default)
    ]
    crc = (MOTOR_ID + sum(data)) & 0xFF
    msg = can.Message(arbitration_id=MOTOR_ID, data=bytearray(data) + bytes([crc]), is_extended_id=False)
    bus.send(msg)

def test_speed(bus, rpm_int, duration_s):
    """Run motor at exact integer RPM for exact duration, measure."""
    enc_before = read_encoder(bus)
    if enc_before is None:
        print("ERROR: Can't read encoder")
        return
    
    print(f"\n=== Test: speed={rpm_int} RPM for {duration_s}s ===")
    print(f"Encoder before: {enc_before}")
    
    t_start = time.time()
    send_speed(bus, rpm_int, direction_cw=False)  # CCW
    
    # Wait the exact duration
    time.sleep(duration_s)
    
    # Stop
    send_speed(bus, 0)
    t_elapsed = time.time() - t_start
    
    # Wait for motor to fully stop
    time.sleep(0.5)
    
    enc_after = read_encoder(bus)
    if enc_after is None:
        print("ERROR: Can't read encoder after")
        return
    
    delta = enc_after - enc_before
    actual_revs = abs(delta) / TICKS_PER_REV
    actual_rpm = actual_revs / (t_elapsed / 60.0)
    expected_revs = rpm_int * duration_s / 60.0
    expected_rad = expected_revs * 2 * math.pi
    actual_rad = actual_revs * 2 * math.pi
    
    print(f"Encoder after:  {enc_after}")
    print(f"Delta:          {delta} ticks")
    print(f"Elapsed:        {t_elapsed:.3f}s")
    print(f"Expected:       {expected_revs:.4f} rev = {expected_rad:.4f} rad ({expected_rad*180/math.pi:.1f} deg)")
    print(f"Actual:         {actual_revs:.4f} rev = {actual_rad:.4f} rad ({actual_rad*180/math.pi:.1f} deg)")
    print(f"Ratio actual/expected: {actual_rad/expected_rad:.4f} ({(actual_rad/expected_rad-1)*100:+.1f}%)")
    print(f"Implied actual RPM:    {actual_rpm:.2f} (commanded {rpm_int})")

def main():
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    
    try:
        # Test at speed=9 (what the bridge sends during cruise)
        test_speed(bus, 9, 4.0)
        time.sleep(1)
        
        # Test at speed=5 (ramp region)
        test_speed(bus, 5, 4.0)
        time.sleep(1)
        
        # Test at speed=1 (very low)
        test_speed(bus, 1, 5.0)
        time.sleep(1)
        
        # Test at speed=10 (just above what bridge sends)
        test_speed(bus, 10, 4.0)
        
    finally:
        send_speed(bus, 0)
        bus.shutdown()

if __name__ == '__main__':
    main()
