"""CAN Command Frame Generation for MKS Servo Motors"""

import struct
from typing import List, Tuple
from .can_set import (
    CAN_CMD_MOTOR_ABSOLUTE_MOVE, CAN_CMD_MOTOR_RELATIVE_MOVE,
    CAN_CMD_MOTOR_VELOCITY_MOVE, CAN_CMD_MOTOR_ENABLE,
    CAN_CMD_MOTOR_DISABLE, CAN_CMD_MOTOR_STOP,
    CAN_CMD_MOTOR_GET_STATUS, CAN_CMD_MOTOR_GET_POSITION,
    CAN_CMD_MOTOR_HOME, POSITION_MIN, POSITION_MAX,
    SPEED_MIN, SPEED_MAX, ACCELERATION_MIN, ACCELERATION_MAX
)


class CanCommands:
    """Generate CAN command frames for MKS Servo motors"""

    @staticmethod
    def validate_parameters(motor_id: int, position: int = None, 
                           speed: int = None, acceleration: int = None) -> bool:
        """Validate motor control parameters"""
        if not (0 <= motor_id <= 255):
            raise ValueError(f"Motor ID must be 0-255, got {motor_id}")
        
        if position is not None and not (POSITION_MIN <= position <= POSITION_MAX):
            raise ValueError(f"Position must be {POSITION_MIN} to {POSITION_MAX}, got {position}")
        
        if speed is not None and not (SPEED_MIN <= speed <= SPEED_MAX):
            raise ValueError(f"Speed must be {SPEED_MIN} to {SPEED_MAX}, got {speed}")
        
        if acceleration is not None and not (ACCELERATION_MIN <= acceleration <= ACCELERATION_MAX):
            raise ValueError(f"Acceleration must be {ACCELERATION_MIN} to {ACCELERATION_MAX}, got {acceleration}")
        
        return True

    @staticmethod
    def absolute_motion_by_pulses(motor_id: int, speed: int, 
                                  acceleration: int, position: int) -> Tuple[int, List[int]]:
        """
        Generate CAN frame for absolute position motion
        
        Args:
            motor_id: Motor CAN ID (0-255)
            speed: Motion speed in RPM (0-3000)
            acceleration: Acceleration rate (0-255, 0=max, 255=min)
            position: Target position in pulses (-8388607 to 8388607)
        
        Returns:
            Tuple of (CAN_ID, [data_bytes])
        """
        CanCommands.validate_parameters(motor_id, position, speed, acceleration)
        
        # CAN ID format: 0x140 + motor_id (extended)
        can_id = 0x140 + motor_id
        
        # Pack data: [CMD, SPEED_H, SPEED_L, ACC, POS_3, POS_2, POS_1, POS_0]
        data = [
            CAN_CMD_MOTOR_ABSOLUTE_MOVE,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration & 0xFF,
            (position >> 24) & 0xFF,
            (position >> 16) & 0xFF,
            (position >> 8) & 0xFF,
            position & 0xFF
        ]
        
        return can_id, data

    @staticmethod
    def relative_motion_by_pulses(motor_id: int, speed: int,
                                  acceleration: int, position: int) -> Tuple[int, List[int]]:
        """
        Generate CAN frame for relative position motion
        
        Args:
            motor_id: Motor CAN ID (0-255)
            speed: Motion speed in RPM (0-3000)
            acceleration: Acceleration rate (0-255)
            position: Relative position in pulses (positive or negative)
        
        Returns:
            Tuple of (CAN_ID, [data_bytes])
        """
        CanCommands.validate_parameters(motor_id, position, speed, acceleration)
        
        can_id = 0x140 + motor_id
        
        # Handle negative positions (two's complement)
        if position < 0:
            position = position & 0xFFFFFFFF
        
        data = [
            CAN_CMD_MOTOR_RELATIVE_MOVE,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration & 0xFF,
            (position >> 24) & 0xFF,
            (position >> 16) & 0xFF,
            (position >> 8) & 0xFF,
            position & 0xFF
        ]
        
        return can_id, data

    @staticmethod
    def velocity_motion(motor_id: int, speed: int, 
                       acceleration: int, direction: int = 1) -> Tuple[int, List[int]]:
        """
        Generate CAN frame for velocity motion (continuous rotation)
        
        Args:
            motor_id: Motor CAN ID (0-255)
            speed: Motion speed in RPM (0-3000)
            acceleration: Acceleration rate (0-255)
            direction: 1 for forward, -1 for reverse
        
        Returns:
            Tuple of (CAN_ID, [data_bytes])
        """
        if direction not in [-1, 1]:
            raise ValueError("Direction must be 1 (forward) or -1 (reverse)")
        
        CanCommands.validate_parameters(motor_id, speed=speed, acceleration=acceleration)
        
        can_id = 0x140 + motor_id
        
        # Pack direction into speed (negative for reverse)
        if direction == -1:
            speed = speed | 0x8000  # Set sign bit for reverse
        
        data = [
            CAN_CMD_MOTOR_VELOCITY_MOVE,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            acceleration & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00
        ]
        
        return can_id, data

    @staticmethod
    def motor_enable(motor_id: int) -> Tuple[int, List[int]]:
        """Enable motor"""
        CanCommands.validate_parameters(motor_id)
        can_id = 0x140 + motor_id
        data = [CAN_CMD_MOTOR_ENABLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return can_id, data

    @staticmethod
    def motor_disable(motor_id: int) -> Tuple[int, List[int]]:
        """Disable motor"""
        CanCommands.validate_parameters(motor_id)
        can_id = 0x140 + motor_id
        data = [CAN_CMD_MOTOR_DISABLE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return can_id, data

    @staticmethod
    def motor_stop(motor_id: int) -> Tuple[int, List[int]]:
        """Stop motor immediately"""
        CanCommands.validate_parameters(motor_id)
        can_id = 0x140 + motor_id
        data = [CAN_CMD_MOTOR_STOP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return can_id, data

    @staticmethod
    def motor_home(motor_id: int, speed: int = 100) -> Tuple[int, List[int]]:
        """Home the motor (move to home position)"""
        CanCommands.validate_parameters(motor_id, speed=speed)
        can_id = 0x140 + motor_id
        data = [
            CAN_CMD_MOTOR_HOME,
            (speed >> 8) & 0xFF,
            speed & 0xFF,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00
        ]
        return can_id, data

    @staticmethod
    def get_motor_status(motor_id: int) -> Tuple[int, List[int]]:
        """Request motor status"""
        CanCommands.validate_parameters(motor_id)
        can_id = 0x140 + motor_id
        data = [CAN_CMD_MOTOR_GET_STATUS, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return can_id, data

    @staticmethod
    def get_motor_position(motor_id: int) -> Tuple[int, List[int]]:
        """Request motor current position"""
        CanCommands.validate_parameters(motor_id)
        can_id = 0x140 + motor_id
        data = [CAN_CMD_MOTOR_GET_POSITION, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        return can_id, data
