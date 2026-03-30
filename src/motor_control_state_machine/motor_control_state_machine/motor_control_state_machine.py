#!/usr/bin/env python3
"""
Motor Control State Machine Node for ROS 2.

Configuration is loaded from a YAML file specified as a command-line argument.
Usage: python3 motor_control_state_machine.py [config_file.yaml]
       If no config file is specified, defaults to setup.yaml
"""

import os
import sys
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from transitions import Machine
from ament_index_python.packages import get_package_share_directory


class MotorControlStateMachine(Node):
    """
    Generic ROS 2 State Machine Node.
    Configuration loaded from YAML file.
    """

    def __init__(self, config_file='setup.yaml'):
        super().__init__('motor_control_state_machine')
        self.config_file = config_file
        self.get_logger().info(f'Initializing Motor Control State Machine with config: {config_file}')

        # Load configuration from YAML
        config = self._load_config()
        self._states = config['states']
        self._transitions = config['transitions']
        self._actions = config.get('actions', {})

        self.get_logger().info(f'Loaded {len(self._states)} states, {len(self._transitions)} transitions')

        # QoS profile matching uart_bridge_node
        realtime_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for motor commands
        self.command_pub = self.create_publisher(String, '/motor_command', realtime_qos)

        # Subscriber for ESP32 responses
        self.response_sub = self.create_subscription(
            String, '/esp32_response', self._on_esp32_response, realtime_qos
        )

        # Wait state management
        self._wait_match_keyword = None
        self._wait_then_seconds = 0.0
        self._wait_then_trigger = None
        self._wait_timer = None

        # Initialize transitions state machine
        self.machine = Machine(
            model=self,
            states=self._states,
            transitions=self._transitions,
            initial=self._states[0],
            auto_transitions=False,
            after_state_change=self._on_state_change
        )

        # Start after a short delay
        self.create_timer(0.5, self._start_once)
        self._started = False

        self.get_logger().info('State machine initialized')

    def _load_config(self) -> dict:
        """Load configuration from YAML file."""
        # Try package share directory first (installed), then local path (development)
        try:
            pkg_dir = get_package_share_directory('motor_control_state_machine')
            config_path = os.path.join(pkg_dir, 'config', self.config_file)
        except Exception:
            # Fallback for development: look relative to this file
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(__file__)),
                'config', self.config_file
            )

        self.get_logger().info(f'Loading config from: {config_path}')

        if not os.path.exists(config_path):
            self.get_logger().error(f'Config file not found: {config_path}')
            raise FileNotFoundError(f'Config file not found: {config_path}')

        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def _start_once(self):
        """Start the state machine (runs once)."""
        if self._started:
            return
        self._started = True
        self.get_logger().info(f'Starting from state: {self.state}')
        self.start()

    def _on_state_change(self):
        """Called after every state change. Executes the state's actions."""
        state = self.state
        self.get_logger().info(f'>>> Entered state: {state}')

        if state in self._actions:
            self._execute_actions(self._actions[state])

    def _execute_actions(self, actions: list):
        """Execute a list of actions defined in YAML."""
        for action in actions:
            if isinstance(action, dict):
                for action_type, params in action.items():
                    self._execute_single_action(action_type, params)
            else:
                # Simple string action like "trigger: command_sent"
                self.get_logger().warn(f'Unknown action format: {action}')

    def _execute_single_action(self, action_type: str, params):
        """Execute a single action by type."""
        if action_type == 'publish_command':
            self._action_publish_command(params)

        elif action_type == 'trigger':
            self._action_trigger(params)

        elif action_type == 'wait_for_response':
            self._action_wait_for_response(params)

        elif action_type == 'send_and_wait':
            self._action_send_and_wait(params)

        elif action_type == 'delay':
            self._action_delay(params)

        elif action_type == 'log':
            self.get_logger().info(params)

        else:
            self.get_logger().warn(f'Unknown action type: {action_type}')

    def _action_publish_command(self, command: str):
        """Publish a motor command."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f'Published: {command}')

    def _action_trigger(self, trigger_name: str):
        """Trigger a state transition."""
        self.get_logger().info(f'Triggering: {trigger_name}')
        getattr(self, trigger_name)()

    def _action_wait_for_response(self, params: dict):
        """Set up waiting for ESP32 response."""
        self._wait_match_keyword = params['match'].lower()
        self._wait_then_seconds = params.get('then_delay', 0.0)
        self._wait_then_trigger = params['then_trigger']
        self.get_logger().info(f'Waiting for response containing "{params["match"]}"...')

    def _action_send_and_wait(self, params: dict):
        """Combined action: publish command, wait for response, delay, then trigger."""
        # 1. Publish the command
        command = params['command']
        self._action_publish_command(command)

        # 2. Set up waiting for response (reuse existing wait logic)
        self._wait_match_keyword = params.get('match', 'completed').lower()
        self._wait_then_seconds = params.get('then_delay', 0.0)
        self._wait_then_trigger = params.get('then_trigger')
        self.get_logger().info(f'Waiting for response containing "{params.get("match", "completed")}"...')

    def _action_delay(self, params: dict):
        """Set up a pure time delay."""
        seconds = params['seconds']
        trigger = params['then_trigger']
        self.get_logger().info(f'Delaying {seconds}s...')
        self._wait_then_trigger = trigger
        self._wait_timer = self.create_timer(seconds, self._on_wait_complete)

    def _on_esp32_response(self, msg: String):
        """Handle incoming ESP32 responses."""
        self.get_logger().info(f'Received: {msg.data}')

        if self._wait_match_keyword and self._wait_match_keyword in msg.data.lower():
            self.get_logger().info(f'Match found! Waiting {self._wait_then_seconds}s...')
            self._wait_match_keyword = None

            self._wait_timer = self.create_timer(
                self._wait_then_seconds,
                self._on_wait_complete
            )

    def _on_wait_complete(self):
        """Called when wait/delay timer fires."""
        if self._wait_timer:
            self._wait_timer.cancel()
            self._wait_timer = None

        trigger = self._wait_then_trigger
        self._wait_then_trigger = None

        if trigger:
            self.get_logger().info(f'Wait complete, triggering: {trigger}')
            getattr(self, trigger)()


def main(args=None):
    """Main entry point."""
    # Parse command-line arguments for config file
    config_file = 'setup.yaml'  # default config file
    
    # If running as a script (not through entry point), get config from sys.argv
    if args is None and len(sys.argv) > 1:
        config_file = sys.argv[1]
        # Remove the config file from sys.argv before rclpy.init() processes args
        sys.argv = [sys.argv[0]] + sys.argv[2:]
    
    rclpy.init(args=args)
    
    try:
        node = MotorControlStateMachine(config_file=config_file)
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.get_default_context().try_shutdown()
    except Exception as e:
        print(f'Error: {e}', file=sys.stderr)
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
