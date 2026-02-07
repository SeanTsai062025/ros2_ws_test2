"""CAN Motor Response Handler"""

from typing import Dict, List, Optional


class CanMotor:
    """Handle CAN responses from motors"""

    def __init__(self, motor_id: int):
        """Initialize motor handler"""
        self.motor_id = motor_id
        self.current_position = 0
        self.current_speed = 0
        self.temperature = 0
        self.status = 0
        self.is_moving = False
        self.error_code = 0

    def parse_status_response(self, data: List[int]) -> Dict:
        """
        Parse motor status response
        
        Args:
            data: CAN frame data bytes
        
        Returns:
            Dictionary with parsed status information
        """
        if len(data) < 8:
            return {}
        
        status_info = {
            'motor_id': self.motor_id,
            'error_code': data[1],
            'is_moving': bool(data[2] & 0x01),
            'temperature': data[3],
            'voltage': (data[4] << 8) | data[5],
        }
        
        self.error_code = status_info['error_code']
        self.is_moving = status_info['is_moving']
        self.temperature = status_info['temperature']
        
        return status_info

    def parse_position_response(self, data: List[int]) -> Dict:
        """
        Parse motor position response
        
        Args:
            data: CAN frame data bytes
        
        Returns:
            Dictionary with position information
        """
        if len(data) < 8:
            return {}
        
        # Extract 32-bit position (signed)
        position_raw = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]
        
        # Handle signed 32-bit
        if position_raw & 0x80000000:
            position = position_raw - 0x100000000
        else:
            position = position_raw
        
        position_info = {
            'motor_id': self.motor_id,
            'position_pulses': position,
            'position_degrees': (position / 10000) * 360,  # Approximate conversion
            'speed_rpm': (data[5] << 8) | data[6],
        }
        
        self.current_position = position
        self.current_speed = position_info['speed_rpm']
        
        return position_info

    def get_status_text(self) -> str:
        """Get human-readable status"""
        status = []
        
        if self.error_code != 0:
            error_messages = {
                0x01: "Over current",
                0x02: "Over voltage",
                0x04: "Under voltage",
                0x08: "Over temperature",
                0x10: "Position out of range",
                0x20: "Communication error",
                0x40: "Parameter error",
            }
            error_text = error_messages.get(self.error_code, f"Unknown error 0x{self.error_code:02X}")
            status.append(f"ERROR: {error_text}")
        
        if self.is_moving:
            status.append("Moving")
        else:
            status.append("Stopped")
        
        status.append(f"Temp: {self.temperature}°C")
        status.append(f"Pos: {self.current_position} pulses")
        status.append(f"Speed: {self.current_speed} RPM")
        
        return " | ".join(status)
