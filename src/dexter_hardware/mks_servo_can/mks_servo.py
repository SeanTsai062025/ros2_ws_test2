"""MKS Servo Motor Control Class - Main Interface"""

import can
from typing import Optional, Dict, List
from .can_commands import CanCommands
from .can_motor import CanMotor
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MksServo:
    """
    Main class for controlling MKS Servo motors via CAN bus
    
    Supports multiple control modes:
    - Absolute position control
    - Relative position control
    - Velocity control
    - Homing
    - Status monitoring
    """

    def __init__(self, bus: can.BusABC, notifier: can.Notifier, motor_id: int):
        """
        Initialize MKS Servo instance
        
        Args:
            bus: python-can bus instance
            notifier: python-can notifier instance
            motor_id: Motor CAN ID (0-255)
        """
        self.bus = bus
        self.notifier = notifier
        self.motor_id = motor_id
        self.motor = CanMotor(motor_id)
        self.response_timeout = 1.0  # seconds
        
        logger.info(f"MksServo initialized for motor ID {motor_id}")

    def _send_command(self, can_id: int, data: List[int]) -> bool:
        """
        Send CAN command frame
        
        Args:
            can_id: CAN identifier
            data: Data bytes (0-8)
        
        Returns:
            True if successful, False otherwise
        """
        try:
            msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
            self.bus.send(msg)
            logger.debug(f"Sent CAN message to motor {self.motor_id}: ID=0x{can_id:03X}, Data={data}")
            return True
        except Exception as e:
            logger.error(f"Failed to send CAN command to motor {self.motor_id}: {e}")
            return False

    def run_motor_absolute_motion_by_pulses(self, speed: int, acceleration: int, 
                                           position: int) -> Optional[bool]:
        """
        Move motor to absolute position (in pulses)
        
        Args:
            speed: Motion speed in RPM (0-3000)
            acceleration: Acceleration rate (0-255, 0=max, 255=min)
            position: Target position in pulses (-8388607 to 8388607)
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.absolute_motion_by_pulses(
                self.motor_id, speed, acceleration, position
            )
            result = self._send_command(can_id, data)
            if result:
                logger.info(f"Motor {self.motor_id}: Absolute motion to {position} pulses at {speed} RPM")
            return result
        except Exception as e:
            logger.error(f"Error in absolute motion: {e}")
            return None

    def run_motor_relative_motion_by_pulses(self, speed: int, acceleration: int,
                                           position: int) -> Optional[bool]:
        """
        Move motor by relative position (in pulses)
        
        Args:
            speed: Motion speed in RPM (0-3000)
            acceleration: Acceleration rate (0-255, 0=max, 255=min)
            position: Relative movement in pulses (positive or negative)
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.relative_motion_by_pulses(
                self.motor_id, speed, acceleration, position
            )
            result = self._send_command(can_id, data)
            if result:
                logger.info(f"Motor {self.motor_id}: Relative motion by {position} pulses at {speed} RPM")
            return result
        except Exception as e:
            logger.error(f"Error in relative motion: {e}")
            return None

    def run_motor_velocity_motion(self, speed: int, acceleration: int,
                                 direction: int = 1) -> Optional[bool]:
        """
        Run motor at constant velocity
        
        Args:
            speed: Motion speed in RPM (0-3000)
            acceleration: Acceleration rate (0-255, 0=max, 255=min)
            direction: 1 for forward, -1 for reverse
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.velocity_motion(
                self.motor_id, speed, acceleration, direction
            )
            result = self._send_command(can_id, data)
            if result:
                dir_text = "forward" if direction == 1 else "reverse"
                logger.info(f"Motor {self.motor_id}: Velocity motion {dir_text} at {speed} RPM")
            return result
        except Exception as e:
            logger.error(f"Error in velocity motion: {e}")
            return None

    def stop_motor(self) -> Optional[bool]:
        """
        Stop motor immediately
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.motor_stop(self.motor_id)
            result = self._send_command(can_id, data)
            if result:
                logger.info(f"Motor {self.motor_id}: Stop command sent")
            return result
        except Exception as e:
            logger.error(f"Error stopping motor: {e}")
            return None

    def enable_motor(self) -> Optional[bool]:
        """
        Enable motor
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.motor_enable(self.motor_id)
            result = self._send_command(can_id, data)
            if result:
                logger.info(f"Motor {self.motor_id}: Enable command sent")
            return result
        except Exception as e:
            logger.error(f"Error enabling motor: {e}")
            return None

    def disable_motor(self) -> Optional[bool]:
        """
        Disable motor
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.motor_disable(self.motor_id)
            result = self._send_command(can_id, data)
            if result:
                logger.info(f"Motor {self.motor_id}: Disable command sent")
            return result
        except Exception as e:
            logger.error(f"Error disabling motor: {e}")
            return None

    def home_motor(self, speed: int = 100) -> Optional[bool]:
        """
        Home the motor (move to home/zero position)
        
        Args:
            speed: Homing speed in RPM
        
        Returns:
            True if command sent successfully, None on error
        """
        try:
            can_id, data = CanCommands.motor_home(self.motor_id, speed)
            result = self._send_command(can_id, data)
            if result:
                logger.info(f"Motor {self.motor_id}: Home command sent")
            return result
        except Exception as e:
            logger.error(f"Error homing motor: {e}")
            return None

    def get_motor_status(self) -> Optional[Dict]:
        """
        Request motor status
        
        Returns:
            Dictionary with status information, or None on error
        """
        try:
            can_id, data = CanCommands.get_motor_status(self.motor_id)
            self._send_command(can_id, data)
            logger.info(f"Motor {self.motor_id}: Status request sent")
            return self.motor.get_status_text()
        except Exception as e:
            logger.error(f"Error getting motor status: {e}")
            return None

    def get_motor_position(self) -> Optional[Dict]:
        """
        Request motor current position
        
        Returns:
            Dictionary with position information, or None on error
        """
        try:
            can_id, data = CanCommands.get_motor_position(self.motor_id)
            self._send_command(can_id, data)
            logger.info(f"Motor {self.motor_id}: Position request sent")
            return {
                'position_pulses': self.motor.current_position,
                'position_degrees': (self.motor.current_position / 10000) * 360,
            }
        except Exception as e:
            logger.error(f"Error getting motor position: {e}")
            return None

    def get_current_position_pulses(self) -> int:
        """Get last known motor position in pulses"""
        return self.motor.current_position

    def get_current_speed_rpm(self) -> int:
        """Get last known motor speed in RPM"""
        return self.motor.current_speed

    def get_temperature(self) -> int:
        """Get last known motor temperature in Celsius"""
        return self.motor.temperature

    def is_motor_moving(self) -> bool:
        """Check if motor is currently moving"""
        return self.motor.is_moving

    def get_last_error(self) -> int:
        """Get last error code from motor"""
        return self.motor.error_code

    def __repr__(self) -> str:
        """String representation"""
        return f"MksServo(motor_id={self.motor_id}, pos={self.motor.current_position}, speed={self.motor.current_speed})"
