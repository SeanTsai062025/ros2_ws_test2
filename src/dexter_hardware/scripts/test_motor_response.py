#!/usr/bin/env python3
"""
Test: Does the MKS Servo42D respond to rapid speed changes at 100Hz?
Test: What does accel=0 do with speed=0 (stop vs keep running)?

This sends a series of speed commands at 100Hz to motor 6 and measures
the actual encoder displacement to verify the motor follows the commands.
"""
import can
import struct
import time
import math

MOTOR_ID = 6
TICKS_PER_REV = 16384

def read_encoder(bus):
    while bus.recv(timeout=0.01): pass
    crc = (MOTOR_ID + 0x31) & 0xFF
    msg = can.Message(arbitration_id=MOTOR_ID, data=[0x31, crc], is_extended_id=False)
    for attempt in range(5):
        bus.send(msg)
        time.sleep(0.02)
        resp = bus.recv(timeout=0.1)
        if resp and resp.arbitration_id == MOTOR_ID and len(resp.data) >= 7 and resp.data[0] == 0x31:
            return int.from_bytes(resp.data[1:7], 'big', signed=True)
    return None

def send_speed(bus, rpm_int, accel=0, direction_cw=False):
    """Send 0xF6 speed command."""
    speed = min(abs(rpm_int), 3000)
    dir_bit = 0x80 if direction_cw else 0x00
    data = [
        0xF6,
        dir_bit | ((speed >> 8) & 0x0F),
        speed & 0xFF,
        accel & 0xFF,
    ]
    crc = (MOTOR_ID + sum(data)) & 0xFF
    msg = can.Message(arbitration_id=MOTOR_ID, data=bytearray(data) + bytes([crc]), is_extended_id=False)
    bus.send(msg)

def test_accel_zero_stop(bus):
    """Test: does speed=0 with accel=0 stop the motor?"""
    print("\n=== TEST 1: speed=0 accel=0 stop behavior ===")
    enc_before = read_encoder(bus)
    
    # Start at speed=9, accel=0
    send_speed(bus, 9, accel=0)
    time.sleep(2.0)
    
    enc_mid = read_encoder(bus)
    print(f"After 2s at speed=9: moved {abs(enc_mid - enc_before)} ticks")
    
    # Send speed=0 with accel=0
    send_speed(bus, 0, accel=0)
    time.sleep(1.0)  # wait 1 second
    
    enc_after = read_encoder(bus)
    coast_ticks = abs(enc_after - enc_mid)
    print(f"After speed=0 accel=0 + 1s wait: moved {coast_ticks} more ticks")
    if coast_ticks < 100:
        print("→ Motor STOPPED with accel=0 speed=0 ✓")
    else:
        print(f"→ Motor STILL RUNNING! Moved {coast_ticks} ticks ({coast_ticks/16384*360:.1f} deg)")
    
    # Make sure motor is stopped
    send_speed(bus, 0, accel=10)
    time.sleep(0.5)

def test_rapid_commands(bus):
    """Test: simulate what the bridge does - send speed commands at 100Hz including ramp."""
    print("\n=== TEST 2: Simulated bridge ramp at 100Hz ===")
    enc_before = read_encoder(bus)
    
    dt = 0.01  # 100Hz
    # Simulate trapezoidal velocity profile: ramp up 1s, cruise 2.14s, ramp down 1s
    # Max vel = 1.0 rad/s = 9.549 RPM, accel = 1.0 rad/s^2
    velocities = []
    t = 0
    while t < 4.14:
        if t < 1.0:
            vel = t * 1.0  # ramp up
        elif t < 3.14:
            vel = 1.0  # cruise
        else:
            vel = max(0, 1.0 - (t - 3.14) * 1.0)  # ramp down
        velocities.append(vel)
        t += dt
    
    print(f"Profile: {len(velocities)} ticks, {len(velocities)*dt:.2f}s")
    float_integral = sum(v * dt for v in velocities)
    int_rpms = [int(v * 60 / (2*math.pi)) for v in velocities]
    int_integral = sum(r * 2*math.pi/60 * dt for r in int_rpms)
    print(f"Float velocity integral: {float_integral:.3f} rad ({float_integral*180/math.pi:.1f} deg)")
    print(f"Int RPM integral:        {int_integral:.3f} rad ({int_integral*180/math.pi:.1f} deg)")
    
    t_start = time.time()
    for i, vel in enumerate(velocities):
        rpm = int(vel * 60 / (2 * math.pi))
        send_speed(bus, rpm, accel=0)
        
        # Sleep to maintain 100Hz
        target = t_start + (i + 1) * dt
        sleep_time = target - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    # Stop
    send_speed(bus, 0, accel=0)
    elapsed = time.time() - t_start
    
    time.sleep(0.5)
    enc_after = read_encoder(bus)
    delta = abs(enc_after - enc_before)
    actual_rad = delta / TICKS_PER_REV * 2 * math.pi
    
    print(f"\nResults:")
    print(f"Elapsed:        {elapsed:.3f}s")
    print(f"Encoder delta:  {delta} ticks")
    print(f"Actual:         {actual_rad:.4f} rad ({actual_rad*180/math.pi:.1f} deg)")
    print(f"vs float plan:  {(actual_rad/float_integral - 1)*100:+.1f}%")
    print(f"vs int plan:    {(actual_rad/int_integral - 1)*100:+.1f}%")
    
    send_speed(bus, 0, accel=10)
    time.sleep(0.5)

def test_ramp_only(bus):
    """Test: just the ramp portion to see if motor follows low RPMs."""
    print("\n=== TEST 3: Ramp up only (1s, 0->9 RPM at 100Hz) ===")
    enc_before = read_encoder(bus)
    
    dt = 0.01
    t_start = time.time()
    for i in range(100):  # 1 second
        vel = (i + 1) * 0.01  # 0.01 to 1.0 rad/s
        rpm = int(vel * 60 / (2 * math.pi))
        send_speed(bus, rpm, accel=0)
        target = t_start + (i + 1) * dt
        sleep_time = target - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    # Stop immediately
    send_speed(bus, 0, accel=0)
    elapsed = time.time() - t_start
    
    time.sleep(0.5)
    enc_after = read_encoder(bus)
    delta = abs(enc_after - enc_before)
    actual_rad = delta / TICKS_PER_REV * 2 * math.pi
    
    # Expected from float integral
    float_int = sum((i+1)*0.01 * dt for i in range(100))
    int_int = sum(int((i+1)*0.01 * 60 / (2*math.pi)) * 2*math.pi/60 * dt for i in range(100))
    
    print(f"Elapsed:        {elapsed:.3f}s")
    print(f"Encoder delta:  {delta} ticks")
    print(f"Float integral: {float_int:.4f} rad ({float_int*180/math.pi:.1f} deg)")
    print(f"Int RPM integr: {int_int:.4f} rad ({int_int*180/math.pi:.1f} deg)")
    print(f"Actual:         {actual_rad:.4f} rad ({actual_rad*180/math.pi:.1f} deg)")
    
    send_speed(bus, 0, accel=10)
    time.sleep(0.5)

def main():
    bus = can.interface.Bus(channel='can0', interface='socketcan')
    
    try:
        test_accel_zero_stop(bus)
        time.sleep(1)
        test_ramp_only(bus)
        time.sleep(1)
        test_rapid_commands(bus)
    finally:
        send_speed(bus, 0, accel=10)
        bus.shutdown()

if __name__ == '__main__':
    main()
