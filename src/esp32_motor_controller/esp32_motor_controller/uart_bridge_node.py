#!/usr/bin/env python3
"""Bidirectional ROS 2 to ESP32 UART bridge with correlated command results."""

from __future__ import annotations

import threading
import time
from typing import Dict, Optional, Tuple

from dexter_interfaces.msg import CommandResult, DeviceCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
import serial
from std_msgs.msg import Bool, String


TERMINAL_STATUSES = {
    'SUCCEEDED',
    'REJECTED',
    'FAILED',
    'CANCELLED',
    'TIMED_OUT',
    'UNKNOWN',
}
VALID_TARGETS = {'rail1', 'rail2', 'servo1', 'servo2', 'screen'}


def encode_command(command: DeviceCommand) -> str:
    """Encode one structured command for the ESP32 without changing its payload."""
    fields = (
        command.show_run_id,
        command.command_id,
        command.target,
        command.command,
    )
    if any(not field or '|' in field or '\n' in field or '\r' in field for field in fields):
        raise ValueError('command fields must be non-empty and cannot contain | or newlines')
    if command.target not in VALID_TARGETS:
        raise ValueError(f'unsupported target: {command.target}')
    return '|'.join(('CMD', *fields))


def encode_cancel(command: DeviceCommand) -> str:
    """Encode cancellation of the correlated command."""
    fields = (command.show_run_id, command.command_id, command.target)
    if any(not field or '|' in field or '\n' in field or '\r' in field for field in fields):
        raise ValueError('cancel fields must be non-empty and cannot contain | or newlines')
    if command.target not in VALID_TARGETS:
        raise ValueError(f'unsupported target: {command.target}')
    return '|'.join(('CANCEL', *fields))


def parse_result(line: str) -> Optional[Tuple[str, str, str, str, str]]:
    """Parse RESULT|run|command|target|status|detail, or return None."""
    if not line.startswith('RESULT|'):
        return None
    fields = line.split('|', 5)
    if len(fields) != 6:
        raise ValueError('malformed structured ESP32 result')
    _, show_run_id, command_id, target, status, detail = fields
    if not show_run_id or not command_id or target not in VALID_TARGETS:
        raise ValueError('structured ESP32 result has invalid correlation fields')
    if status not in TERMINAL_STATUSES:
        raise ValueError(f'unsupported ESP32 result status: {status}')
    return show_run_id, command_id, target, status, detail


class UartBridgeNode(Node):
    """Keep a UART connection open and bridge both legacy and structured traffic."""

    def __init__(self) -> None:
        super().__init__('uart_bridge_node')

        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_timeout', 0.1)
        self.declare_parameter('reconnect_interval_s', 1.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.read_timeout = self.get_parameter('read_timeout').value
        reconnect_interval = self.get_parameter('reconnect_interval_s').value

        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Existing topics remain available for manual commands and diagnostics.
        self.command_sub = self.create_subscription(
            String, '/motor_command', self.command_callback, realtime_qos
        )
        self.response_pub = self.create_publisher(
            String, '/esp32_response', realtime_qos
        )

        # Structured path used by the show orchestrator.
        self.device_command_sub = self.create_subscription(
            DeviceCommand,
            '/device_command',
            self.device_command_callback,
            reliable_qos,
        )
        self.device_cancel_sub = self.create_subscription(
            DeviceCommand,
            '/device_cancel',
            self.device_cancel_callback,
            reliable_qos,
        )
        self.device_result_pub = self.create_publisher(
            CommandResult, '/device_command_result', reliable_qos
        )
        self.connected_pub = self.create_publisher(
            Bool, '/esp32/connected', state_qos
        )

        self.serial_conn = None
        self._serial_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: Dict[Tuple[str, str], DeviceCommand] = {}
        self._connected_state: Optional[bool] = None
        self._last_pong_monotonic = 0.0
        self._running = True
        self._init_serial()

        self.read_thread = threading.Thread(
            target=self._read_serial_loop, daemon=True
        )
        self.read_thread.start()
        self.reconnect_timer = self.create_timer(
            float(reconnect_interval), self._maintain_connection
        )

        self.get_logger().info(
            f'UART bridge started on {self.serial_port} at {self.baud_rate} baud'
        )

    def _publish_connected(self, connected: bool) -> None:
        if self._connected_state == connected:
            return
        self._connected_state = connected
        message = Bool()
        message.data = connected
        self.connected_pub.publish(message)

    def _init_serial(self) -> None:
        with self._serial_lock:
            if self.serial_conn is not None and self.serial_conn.is_open:
                return
            try:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.read_timeout,
                )
            except serial.SerialException as error:
                self.serial_conn = None
                self.get_logger().error(
                    f'Failed to open serial port {self.serial_port}: {error}'
                )
                self._publish_connected(False)
                return
        self.get_logger().info(f'Serial port {self.serial_port} opened successfully')
        self._last_pong_monotonic = 0.0
        self._publish_connected(False)

    def _maintain_connection(self) -> None:
        """Reconnect the port and require a recent ESP32 PONG for readiness."""
        if self.serial_conn is None or not self.serial_conn.is_open:
            self._init_serial()
            return
        try:
            self._write_line('PING')
        except serial.SerialException as error:
            self._mark_disconnected(f'UART heartbeat write failed: {error}')
            return
        if time.monotonic() - self._last_pong_monotonic > 3.0:
            self._publish_connected(False)

    def _mark_disconnected(self, detail: str) -> None:
        with self._serial_lock:
            if self.serial_conn is not None:
                try:
                    self.serial_conn.close()
                except Exception:  # Serial is already unusable; preserve the cause.
                    pass
            self.serial_conn = None
        self._publish_connected(False)
        self._fail_all_pending('FAILED', detail)

    def _write_line(self, line: str) -> None:
        with self._serial_lock:
            if self.serial_conn is None or not self.serial_conn.is_open:
                raise serial.SerialException('serial port is not connected')
            self.serial_conn.write((line.strip() + '\n').encode('utf-8'))

    def command_callback(self, msg: String) -> None:
        """Forward a legacy uncorrelated command unchanged."""
        try:
            self._write_line(msg.data)
            self.get_logger().info(f'Sent legacy command to ESP32: {msg.data.strip()}')
        except serial.SerialException as error:
            self.get_logger().error(f'Failed to write legacy command: {error}')
            self._mark_disconnected(str(error))

    def device_command_callback(self, msg: DeviceCommand) -> None:
        """Send a correlated command and retain its ROS metadata until completion."""
        try:
            wire_command = encode_command(msg)
        except ValueError as error:
            self._publish_result(msg, 'REJECTED', str(error))
            return

        key = (msg.show_run_id, msg.command_id)
        with self._pending_lock:
            if key in self._pending:
                self._publish_result(msg, 'REJECTED', 'duplicate command_id')
                return
            self._pending[key] = msg

        try:
            self._write_line(wire_command)
            self.get_logger().info(
                f'Sent correlated command: run={msg.show_run_id} '
                f'node={msg.node_id} command={msg.command_id} target={msg.target}'
            )
        except serial.SerialException as error:
            with self._pending_lock:
                self._pending.pop(key, None)
            self._publish_result(msg, 'FAILED', f'UART write failed: {error}')
            self._mark_disconnected(str(error))

    def device_cancel_callback(self, msg: DeviceCommand) -> None:
        """Ask the ESP32 to cancel one exact in-flight command."""
        key = (msg.show_run_id, msg.command_id)
        with self._pending_lock:
            pending = self._pending.get(key)
        if pending is None or pending.target != msg.target:
            self.get_logger().warning(
                f'Ignoring stale device cancel: run={msg.show_run_id} '
                f'command={msg.command_id} target={msg.target}'
            )
            return
        try:
            self._write_line(encode_cancel(msg))
        except (ValueError, serial.SerialException) as error:
            with self._pending_lock:
                failed = self._pending.pop(key, None)
            if failed is not None:
                self._publish_result(
                    failed, 'UNKNOWN', f'cancel could not be confirmed: {error}'
                )
            if isinstance(error, serial.SerialException):
                self._mark_disconnected(str(error))

    def _publish_result(
        self, command: DeviceCommand, status: str, detail: str
    ) -> None:
        result = CommandResult()
        result.show_run_id = command.show_run_id
        result.node_id = command.node_id
        result.command_id = command.command_id
        result.attempt = command.attempt
        result.target = command.target
        result.status = status
        result.detail = detail
        self.device_result_pub.publish(result)

    def _fail_all_pending(self, status: str, detail: str) -> None:
        with self._pending_lock:
            pending = list(self._pending.values())
            self._pending.clear()
        for command in pending:
            self._publish_result(command, status, detail)

    def _handle_serial_line(self, response: str) -> None:
        legacy = String()
        legacy.data = f'ESP32 response: {response}'
        self.response_pub.publish(legacy)

        if response == 'PONG':
            self._last_pong_monotonic = time.monotonic()
            self._publish_connected(True)
            return

        try:
            parsed = parse_result(response)
        except ValueError as error:
            self.get_logger().error(f'Ignoring malformed ESP32 result: {error}')
            return
        if parsed is None:
            return

        show_run_id, command_id, target, status, detail = parsed
        key = (show_run_id, command_id)
        with self._pending_lock:
            command = self._pending.pop(key, None)
        if command is None:
            self.get_logger().warning(
                f'Ignoring stale ESP32 result: run={show_run_id} command={command_id}'
            )
            return
        if command.target != target:
            self._publish_result(
                command,
                'FAILED',
                f'ESP32 result target mismatch: expected {command.target}, got {target}',
            )
            return
        self._publish_result(command, status, detail)

    def _read_serial_loop(self) -> None:
        while self._running:
            connection = self.serial_conn
            if connection is None or not connection.is_open:
                time.sleep(0.1)
                continue
            try:
                raw_data = connection.readline()
                if not raw_data:
                    continue
                response = raw_data.decode('utf-8', errors='replace').strip()
                if response:
                    self.get_logger().info(f'ESP32 response: {response}')
                    self._handle_serial_line(response)
            except serial.SerialException as error:
                self.get_logger().error(f'Serial read error: {error}')
                self._mark_disconnected(f'UART read failed: {error}')
            except Exception as error:
                self.get_logger().error(f'Unexpected UART read error: {error}')
                time.sleep(0.1)

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down UART bridge')
        self._running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        self._mark_disconnected('UART bridge shut down')
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UartBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
