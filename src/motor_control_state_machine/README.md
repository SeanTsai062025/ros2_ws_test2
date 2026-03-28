# Motor Control State Machine

A ROS 2 node that implements a state machine for safe sequential motor control using the `transitions` library.

## Features

- **State Machine Design**: Uses the `transitions` library for clear, predictable state management
- **Event-Driven**: Waits for ESP32 completion response instead of fixed timers
- **Safe Sequential Execution**: Ensures commands are executed in the correct order
- **Graceful Error Handling**: Includes error states and timeout protection
- **ROS 2 Integration**: Full rclpy integration with logging and status publishing

## State Flow

```
INIT → SEND_LOW → WAIT → SEND_HIGH → DONE
  ↓ (error from any state)
ERROR
```

### States:

- **INIT**: Initial state, waits for sequence to start
- **SEND_LOW**: Publishes low command to `/motor_command` topic
- **WAIT**: Waits for ESP32 completion response, then waits 1 additional second
- **SEND_HIGH**: Publishes high command to `/motor_command` topic
- **DONE**: Sequence completed successfully
- **ERROR**: Error handling state

## Dependencies

- `rclpy`: ROS 2 Python client library
- `transitions`: Python state machine library
- `std_msgs`: ROS 2 standard message types

Install with:
```bash
conda activate dexter_ros2
python -m pip install transitions
```

## Integration with UART Bridge Node

### Expected Message Flow:

1. **State Machine** publishes command to `/motor_command`:
   ```
   ros2 topic pub --once /motor_command std_msgs/msg/String "data: 'rail1 16100 low'"
   ```

2. **UART Bridge Node** receives command and sends to ESP32:
   ```
   [INFO] [timestamp] [uart_bridge_node]: Sent to ESP32: rail1 16100 low
   ```

3. **ESP32** executes command and responds back

4. **UART Bridge Node** receives response:
   ```
   [INFO] [timestamp] [uart_bridge_node]: ESP32 response: OK: rail1 completed 16100 steps
   ```

5. **State Machine** listens on `/esp32_response` topic for completion message

### Setup Requirements:

To enable full integration, the `uart_bridge_node` needs to publish ESP32 responses to the `/esp32_response` topic. This allows the state machine to:

- Wait for motor completion instead of using fixed timers
- Detect errors in command execution
- React to actual hardware state changes

### Modification Needed in uart_bridge_node:

✅ **ALREADY IMPLEMENTED!** 

The uart_bridge_node has been updated to publish ESP32 responses to the `/esp32_response` topic.

**How it works:**

1. When ESP32 sends a response (e.g., `OK: rail1 completed 16100 steps`)
2. uart_bridge_node logs it: `[INFO] [timestamp] [uart_bridge_node]: ESP32 response: OK: rail1 completed 16100 steps`
3. Publishes to `/esp32_response` with format: `ESP32 response: OK: rail1 completed 16100 steps`
4. State machine receives the message and checks for "completed" keyword
5. Automatically transitions to final 1-second wait, then sends next command

**Code in uart_bridge_node** (already present):

```python
# In uart_bridge_node.__init__():
self.response_pub = self.create_publisher(
    String,
    '/esp32_response',
    realtime_qos
)

# In _read_serial_loop() when receiving ESP32 response:
response_msg = String()
response_msg.data = f'ESP32 response: {response}'
self.response_pub.publish(response_msg)
```

**No additional changes needed!** The integration is complete and ready to use.

## Usage

### Build the package:

```bash
cd /home/sean/dexter_test_2/ros2_ws
colcon build --packages-select motor_control_state_machine
source install/setup.bash
```

### Run the state machine:

**Terminal 1** - Start the UART bridge node:
```bash
conda activate dexter_ros2
ros2 run esp32_motor_controller uart_bridge_node
```

**Terminal 2** - Start the state machine:
```bash
conda activate dexter_ros2
ros2 run motor_control_state_machine motor_control_state_machine
```

The state machine will automatically start 1 second after launch and execute the sequence.

### Monitor the state machine:

**Terminal 3** - Subscribe to status updates:
```bash
ros2 topic echo /motor_control_status
```

## Behavior

1. Waits 1 second for initialization
2. Sends LOW command
3. Waits for ESP32 completion response (max 30 seconds)
4. Waits 1 additional second
5. Sends HIGH command
6. Waits for ESP32 completion response (max 30 seconds)
7. Completes

## Error Handling

- If ESP32 doesn't respond within 30 seconds, transitions to ERROR state
- All state transitions are logged with timestamps
- Status updates are published to `/motor_control_status` for monitoring

## Future Enhancements

- Integrate with `dexter_moveit` for complex motion sequences
- Add configurable parameters for command strings and wait times
- Add service-based triggering instead of automatic start
- Support multiple command sequences
