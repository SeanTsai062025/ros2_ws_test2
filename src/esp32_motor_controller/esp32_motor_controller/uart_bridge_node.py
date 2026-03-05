#!/usr/bin/env python3
"""
UART Bridge Node for ESP32 Motor Controller Communication.

This ROS2 node provides a bridge between ROS2 topics and an ESP32 
connected via UART (serial) on the Raspberry Pi 5.

Hardware Connection:
- Pi 5 TX (Pin 8 / GPIO 14) -> ESP32 RX (GPIO 16)
- Pi 5 RX (Pin 10 / GPIO 15) -> ESP32 TX (GPIO 17)
- Common GND connected
- Serial port: /dev/serial0 at 115200 baud
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import threading
import time


class UartBridgeNode(Node):
    """ROS2 Node that bridges UART communication with ESP32."""

    def __init__(self):
        super().__init__('uart_bridge_node')
        
        # Declare parameters with defaults
        # On Ubuntu Pi 5: /dev/ttyAMA0 (after enabling uart0-pi5 overlay)
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_timeout', 0.1)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.read_timeout = self.get_parameter('read_timeout').get_parameter_value().double_value
        
        # Initialize serial connection
        self.serial_conn = None
        self._init_serial()
        
        # Create subscriber for motor commands
        self.command_sub = self.create_subscription(
            String,
            '/motor_command',
            self.command_callback,
            10
        )
        
        # Create publisher for ESP32 responses
        self.response_pub = self.create_publisher(
            String,
            '/esp32_response',
            10
        )
        
        # Flag for controlling the read thread
        self._running = True
        
        # Start a separate thread for reading UART responses
        self.read_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info(
            f'UART Bridge Node started on {self.serial_port} at {self.baud_rate} baud'
        )

    def _init_serial(self):
        """Initialize the serial port connection."""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.read_timeout
            )
            self.get_logger().info(f'Serial port {self.serial_port} opened successfully')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.get_logger().warn(
                'Make sure the serial port is enabled and you have permission. '
                'Try: sudo usermod -a -G dialout $USER'
            )
            self.serial_conn = None

    def command_callback(self, msg: String):
        """
        Callback for receiving motor commands from ROS2 topic.
        
        Appends newline character and sends to ESP32 via UART.
        
        Args:
            msg: String message containing the command (e.g., "servo1 90 15")
        """
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.get_logger().error('Serial port not available, cannot send command')
            return
        
        try:
            # Append newline and encode to bytes
            command = msg.data.strip() + '\n'
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().info(f'Sent to ESP32: {msg.data.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {e}')

    def _read_serial_loop(self):
        """
        Continuously read incoming UART messages from ESP32.
        
        This runs in a separate thread to avoid blocking the main ROS2 loop.
        """
        while self._running:
            if self.serial_conn is None or not self.serial_conn.is_open:
                time.sleep(0.5)
                continue
            
            try:
                # Read a line from serial (blocks until timeout or newline)
                if self.serial_conn.in_waiting > 0:
                    raw_data = self.serial_conn.readline()
                    if raw_data:
                        # Decode and strip whitespace
                        response = raw_data.decode('utf-8', errors='replace').strip()
                        if response:
                            self.get_logger().info(f'ESP32 response: {response}')
                            
                            # Publish response to ROS2 topic
                            response_msg = String()
                            response_msg.data = response
                            self.response_pub.publish(response_msg)
                else:
                    # Small sleep to prevent CPU spinning when no data
                    time.sleep(0.01)
                    
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Unexpected error in read loop: {e}')
                time.sleep(0.1)

    def destroy_node(self):
        """
        Safely cleanup resources when node is shutting down.
        
        Overrides the parent method to ensure serial port is closed.
        """
        self.get_logger().info('Shutting down UART Bridge Node...')
        
        # Stop the read thread
        self._running = False
        
        # Wait for read thread to finish (with timeout)
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        # Close serial connection
        if self.serial_conn is not None and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                self.get_logger().info('Serial port closed successfully')
            except Exception as e:
                self.get_logger().error(f'Error closing serial port: {e}')
        
        # Call parent destroy_node
        super().destroy_node()


def main(args=None):
    """Main entry point for the UART Bridge Node."""
    rclpy.init(args=args)
    
    node = UartBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
