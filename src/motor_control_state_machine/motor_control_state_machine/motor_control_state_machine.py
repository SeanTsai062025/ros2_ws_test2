#!/usr/bin/env python3
"""
Motor Control State Machine Node for ROS 2.

This ROS2 node implements a state machine using the transitions library
to safely control motor commands with a specific sequence:
1. Send low command
2. Wait for ESP32 response (OK: rail1 completed message)
3. Wait 1 additional second
4. Send high command

The state machine ensures predictable, sequential execution without race conditions.
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

import subprocess
import sys
import re
from transitions import Machine
from typing import Optional


class MotorControlStateMachine(Node):
    """
    ROS 2 Node implementing a motor control state machine.
    
    State Flow:
        INIT -> SEND_LOW -> WAIT -> SEND_HIGH -> DONE
    
    The machine uses transitions to ensure predictable state changes
    and prevents concurrent execution of commands.
    """

    # Define all possible states
    states = ['INIT', 'SEND_LOW', 'WAIT', 'SEND_HIGH', 'DONE', 'ERROR']

    def __init__(self):
        """Initialize the ROS 2 node and state machine."""
        super().__init__('motor_control_state_machine')
        
        # Create logger for better debugging
        self.get_logger().info("Initializing Motor Control State Machine")
        
        # Configure QoS for real-time performance (match uart_bridge_node)
        # BEST_EFFORT: Low latency, no retransmission (best for real-time)
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Flag to track if execution has started
        self._execution_started = False
        
        # Wait timer reference
        self._wait_timer: Optional[Timer] = None
        
        # Flag to track if ESP32 response received
        self._esp32_response_received = False
        
        # Initialize the state machine
        self.machine = Machine(
            model=self,
            states=self.states,
            initial='INIT',
            auto_transitions=False  # Disable auto transitions for explicit control
        )
        
        # Define explicit transitions
        self.machine.add_transition(
            trigger='start_sequence',
            source='INIT',
            dest='SEND_LOW',
            before=self._validate_start
        )
        
        self.machine.add_transition(
            trigger='low_sent',
            source='SEND_LOW',
            dest='WAIT'
        )
        
        self.machine.add_transition(
            trigger='wait_complete',
            source='WAIT',
            dest='SEND_HIGH'
        )
        
        self.machine.add_transition(
            trigger='high_sent',
            source='SEND_HIGH',
            dest='DONE'
        )
        
        # Error transitions (from any state)
        for state in self.states:
            if state != 'ERROR':
                self.machine.add_transition(
                    trigger='error_occurred',
                    source=state,
                    dest='ERROR'
                )
        
        # Create a publisher for feedback (real-time QoS)
        self.status_pub = self.create_publisher(
            String,
            '/motor_control_status',
            realtime_qos
        )
        
        # Subscribe to uart_bridge_node responses (real-time QoS to match publisher)
        self.esp32_response_sub = self.create_subscription(
            String,
            '/esp32_response',
            self._esp32_response_callback,
            realtime_qos
        )
        
        # Create a timer to start the sequence after initialization
        self.create_timer(
            timer_period_sec=1.0,
            callback=self._start_sequence_callback
        )
        
        self.get_logger().info("State machine initialized successfully")

    def _validate_start(self) -> bool:
        """
        Callback before transitioning from INIT to SEND_LOW.
        Ensures the sequence only runs once.
        """
        if self._execution_started:
            self.get_logger().warn("Execution already started, ignoring request")
            return False
        
        self._execution_started = True
        self.get_logger().info("Starting motor control sequence")
        return True

    def _on_enter_state(self):
        """Callback when entering any state."""
        self.get_logger().info(f">>> Entered state: {self.state}")
        self._publish_status(f"State: {self.state}")

    def _on_exit_state(self):
        """Callback when exiting any state."""
        self.get_logger().info(f"<<< Exiting state: {self.state}")

    def _publish_status(self, message: str) -> None:
        """Publish status update to /motor_control_status topic."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def _start_sequence_callback(self) -> None:
        """Timer callback to initiate the sequence."""
        if self.state == 'INIT' and not self._execution_started:
            self.get_logger().info("Triggering sequence start...")
            try:
                self.start_sequence()
            except Exception as e:
                self.get_logger().error(f"Error triggering start: {e}")
                self.error_occurred()

    def _esp32_response_callback(self, msg: String) -> None:
        """
        Callback when ESP32 response is received.
        
        Expected message format:
        "ESP32 response: OK: rail1 completed 16100 steps"
        """
        self.get_logger().info(f"Received ESP32 response: {msg.data}")
        
        # Check if this is a completion message
        if "completed" in msg.data.lower():
            self._esp32_response_received = True
            self.get_logger().info("ESP32 completion confirmed")
            
            # If we're in WAIT state, trigger the next phase
            if self.state == 'WAIT':
                self.get_logger().info("Response received while in WAIT state, starting final wait...")
                self._start_final_wait()

    def on_enter_SEND_LOW(self) -> None:
        """Callback when entering SEND_LOW state."""
        self.get_logger().info("Executing LOW command...")
        try:
            self._execute_command("rail1 16100 low")
            self.get_logger().info("LOW command executed successfully")
            self._publish_status("LOW command sent")
            self.low_sent()
        except Exception as e:
            self.get_logger().error(f"Error executing LOW command: {e}")
            self._publish_status(f"Error: {e}")
            self.error_occurred()

    def on_enter_WAIT(self) -> None:
        """
        Callback when entering WAIT state.
        Wait for ESP32 response indicating motor completion.
        """
        self.get_logger().info("Waiting for ESP32 response...")
        self._esp32_response_received = False
        self._publish_status("Waiting for ESP32 response")
        
        # Set a timeout in case ESP32 doesn't respond
        # If response is received, _esp32_response_callback will trigger _start_final_wait()
        self._wait_timer = self.create_timer(
            timer_period_sec=30.0,  # 30 second timeout
            callback=self._wait_timeout_callback
        )

    def _wait_timeout_callback(self) -> None:
        """Callback when wait timer times out."""
        if not self._esp32_response_received:
            self.get_logger().error("ESP32 response timeout - no response received after 30 seconds")
            self._publish_status("Error: ESP32 response timeout")
            self.error_occurred()
        else:
            # Response was received before timeout, this shouldn't be called
            self.get_logger().warn("Wait timeout called but response already received")

    def _start_final_wait(self) -> None:
        """Start the final 1-second wait before sending HIGH command."""
        # Cancel the previous timer if it exists
        if self._wait_timer is not None:
            self._wait_timer.cancel()
        
        self.get_logger().info("Starting final 1-second wait before SEND_HIGH...")
        self._publish_status("Final wait: 1 second")
        
        # Create 1-second timer
        self._wait_timer = self.create_timer(
            timer_period_sec=1.0,
            callback=self._final_wait_complete_callback
        )

    def _final_wait_complete_callback(self) -> None:
        """Callback when final wait completes."""
        self.get_logger().info("Final wait complete, transitioning to SEND_HIGH")
        self.wait_complete()

    def on_enter_SEND_HIGH(self) -> None:
        """Callback when entering SEND_HIGH state."""
        self.get_logger().info("Executing HIGH command...")
        try:
            self._execute_command("rail1 16100 high")
            self.get_logger().info("HIGH command executed successfully")
            self._publish_status("HIGH command sent")
            self.high_sent()
        except Exception as e:
            self.get_logger().error(f"Error executing HIGH command: {e}")
            self._publish_status(f"Error: {e}")
            self.error_occurred()

    def on_enter_DONE(self) -> None:
        """Callback when entering DONE state."""
        # Cancel any pending timers
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self.get_logger().info("Cancelled pending timers")
        
        self.get_logger().info("Sequence completed successfully!")
        self._publish_status("Sequence completed")
        self.get_logger().info("Motor control state machine finished")

    def on_enter_ERROR(self) -> None:
        """Callback when entering ERROR state."""
        # Cancel any pending timers
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self.get_logger().info("Cancelled pending timers")
        
        self.get_logger().error("Error state reached - sequence halted")
        self._publish_status("Error occurred - sequence halted")

    def _execute_command(self, command_args: str) -> None:
        """
        Execute a ROS 2 command via subprocess.
        
        Args:
            command_args: The command arguments (e.g., "rail1 16100 low")
        
        Raises:
            Exception: If command execution fails
        """
        full_command = [
            'ros2', 'topic', 'pub', '--once',
            '/motor_command',
            'std_msgs/msg/String',
            f"data: '{command_args}'"
        ]
        
        self.get_logger().info(f"Executing: {' '.join(full_command)}")
        
        try:
            result = subprocess.run(
                full_command,
                capture_output=True,
                text=True,
                timeout=5.0,  # 5 second timeout
                check=True
            )
            
            if result.returncode != 0:
                raise Exception(f"Command failed with return code {result.returncode}")
            
            if result.stderr:
                self.get_logger().warn(f"Command stderr: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            raise Exception(f"Command timeout after 5 seconds")
        except subprocess.CalledProcessError as e:
            raise Exception(f"Command failed: {e.stderr}")
        except Exception as e:
            raise Exception(f"Command execution error: {str(e)}")


def main(args=None):
    """Main entry point for the ROS 2 node."""
    rclpy.init(args=args)
    
    node = MotorControlStateMachine()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
