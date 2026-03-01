"""
servo_node.py — MG996R Servo Controller for Raspberry Pi 5 + ROS 2 Jazzy

Subscribes to /servo_angle (std_msgs/Float64, 0–180°) and drives the
MG996R on GPIO 18 using gpiozero's AngularServo — the standard
Raspberry Pi servo library.

Jitter-free operation
---------------------
gpiozero uses lgpio (software-timed PWM) on Pi 5.  Continuous software
PWM causes visible servo jitter.  The conventional solution (used by
gpiozero itself) is the **detach** pattern:

    servo.angle = target      # start PWM → servo moves
    …servo reaches position…
    servo.detach()            # stop PWM  → servo holds mechanically

Once detached, the MG996R's internal gearbox holds position silently.
No signal → no timing jitter → no jitter.

This node uses a one-shot ROS timer to call detach() after a generous
settle window (default 1 s — enough for a full 0→180° sweep).

Hardware
--------
  MG996R         Raspberry Pi 5
  ──────         ──────────────
  VCC (red)  →   External 5 V supply
  GND (brown)→   Common GND with Pi
  PWM (orange)→  GPIO 18 (Pin 12)

Prerequisites
-------------
  pip install gpiozero          # uses lgpio pin factory on Pi 5
"""

from gpiozero import AngularServo

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float64


# ── Defaults ───────────────────────────────────────────────────────
GPIO_PIN        = 18       # BCM 18 (physical pin 12)
MIN_PULSE_WIDTH = 0.0005   # 0.5 ms → 0°
MAX_PULSE_WIDTH = 0.0025   # 2.5 ms → 180°
SETTLE_SEC      = 1.0      # PWM-on time before auto-detach


class ServoControlNode(Node):
    """ROS 2 node: /servo_angle → AngularServo → detach."""

    def __init__(self):
        super().__init__('servo_control_node')

        # ---------- ROS parameters --------------------------------------
        self.declare_parameter('gpio_pin',        GPIO_PIN)
        self.declare_parameter('min_pulse_width', MIN_PULSE_WIDTH)
        self.declare_parameter('max_pulse_width', MAX_PULSE_WIDTH)
        self.declare_parameter('settle_sec',      SETTLE_SEC)

        pin    = self.get_parameter('gpio_pin').value
        min_pw = self.get_parameter('min_pulse_width').value
        max_pw = self.get_parameter('max_pulse_width').value
        self._settle = self.get_parameter('settle_sec').value

        # ---------- gpiozero servo --------------------------------------
        # initial_angle=None → no PWM on startup (no jitter, no movement)
        self._servo = AngularServo(
            pin,
            initial_angle=None,
            min_angle=0,
            max_angle=180,
            min_pulse_width=min_pw,
            max_pulse_width=max_pw,
        )
        self._detach_timer: Timer | None = None

        self.get_logger().info(
            f'AngularServo on GPIO {pin}  '
            f'pulse {min_pw*1000:.1f}–{max_pw*1000:.1f} ms  '
            f'settle {self._settle:.1f} s  [IDLE]'
        )

        # ---------- ROS subscriber --------------------------------------
        self.create_subscription(
            Float64, '/servo_angle', self._on_angle, 10,
        )
        self.get_logger().info('Listening on /servo_angle  [0 – 180]')

    # ------------------------------------------------------------------ #
    def _on_detach(self):
        """One-shot callback: stop PWM, let servo hold mechanically."""
        self._servo.detach()
        self.get_logger().info('Detached — holding position (no PWM)')
        if self._detach_timer is not None:
            self._detach_timer.cancel()
            self._detach_timer = None

    # ------------------------------------------------------------------ #
    def _on_angle(self, msg: Float64):
        """Set servo angle, then schedule auto-detach."""
        angle = max(0.0, min(180.0, msg.data))

        # Reset any pending detach timer
        if self._detach_timer is not None:
            self._detach_timer.cancel()
            self._detach_timer = None

        self._servo.angle = angle                     # starts PWM
        self.get_logger().info(
            f'→ {angle:.1f}°  [detach in {self._settle:.1f} s]'
        )
        self._detach_timer = self.create_timer(       # schedule detach
            self._settle, self._on_detach,
        )

    # ------------------------------------------------------------------ #
    def destroy_node(self):
        self.get_logger().info('Shutting down…')
        try:
            if self._detach_timer is not None:
                self._detach_timer.cancel()
            self._servo.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
