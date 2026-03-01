"""Basic sanity tests for the servo_control package (gpiozero edition).

These tests verify the angle-to-pulse-width math that gpiozero's
AngularServo uses internally, without requiring GPIO hardware.
"""
import pytest

MIN_PULSE_WIDTH = 0.0005   # 0.5 ms → 0°
MAX_PULSE_WIDTH = 0.0025   # 2.5 ms → 180°
FRAME_WIDTH     = 0.020    # 20 ms (50 Hz)


def angle_to_duty_pct(
    angle: float,
    min_pw: float = MIN_PULSE_WIDTH,
    max_pw: float = MAX_PULSE_WIDTH,
    frame: float = FRAME_WIDTH,
) -> float:
    """Replicate gpiozero AngularServo math: angle → duty-cycle %."""
    clamped = max(0.0, min(180.0, angle))
    pulse = min_pw + (clamped / 180.0) * (max_pw - min_pw)
    return (pulse / frame) * 100.0


def test_angle_to_duty_mapping():
    """Verify key angle → duty-cycle conversions."""
    assert angle_to_duty_pct(0.0)   == pytest.approx(2.5,  abs=0.01)   # 0.5 ms
    assert angle_to_duty_pct(90.0)  == pytest.approx(7.5,  abs=0.01)   # 1.5 ms
    assert angle_to_duty_pct(180.0) == pytest.approx(12.5, abs=0.01)   # 2.5 ms
    assert angle_to_duty_pct(45.0)  == pytest.approx(5.0,  abs=0.01)   # 1.0 ms
    assert angle_to_duty_pct(135.0) == pytest.approx(10.0, abs=0.01)   # 2.0 ms


def test_angle_clamping():
    """Values outside 0–180 should be clamped."""
    assert angle_to_duty_pct(-10.0) == pytest.approx(2.5,  abs=0.01)   # → 0°
    assert angle_to_duty_pct(200.0) == pytest.approx(12.5, abs=0.01)   # → 180°
