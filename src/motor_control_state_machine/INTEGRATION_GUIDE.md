# Integration Guide: uart_bridge_node → motor_control_state_machine

## Overview

The `motor_control_state_machine` node now waits for actual ESP32 responses instead of using fixed timers. To make this work, the `uart_bridge_node` needs to publish ESP32 responses to a new ROS 2 topic.

## Changes Required in uart_bridge_node

### Location
File: `/home/sean/dexter_test_2/ros2_ws/src/esp32_motor_controller/esp32_motor_controller/uart_bridge_node.py`

### Change 1: Add Response Publisher in `__init__`

Find the section where publishers are created (around line 50-60) and add:

```python
# After existing publishers, add:
self.esp32_response_pub = self.create_publisher(
    String,
    '/esp32_response',
    10
)
```

### Change 2: Publish Response When Received

Find where ESP32 responses are logged. Look for lines like:
```
[INFO] [timestamp] [uart_bridge_node]: ESP32 response: OK: rail1 completed 16100 steps
```

This is typically in a method that handles serial responses. When you find this, add:

```python
# When you have the ESP32 response text, publish it:
response_msg = String()
response_msg.data = f"ESP32 response: {response_text}"
self.esp32_response_pub.publish(response_msg)
```

Example location (hypothetical - adjust to match actual code):

```python
def _handle_serial_response(self, response_text):
    """Handle response received from ESP32."""
    self.get_logger().info(f"ESP32 response: {response_text}")
    
    # NEW: Publish for state machine
    response_msg = String()
    response_msg.data = f"ESP32 response: {response_text}"
    self.esp32_response_pub.publish(response_msg)
```

## Testing the Integration

### Step 1: Verify Topics

After starting uart_bridge_node:
```bash
ros2 topic list
```

You should see `/esp32_response` in the list.

### Step 2: Monitor Responses

```bash
ros2 topic echo /esp32_response
```

You should see messages like:
```
data: 'ESP32 response: OK: rail1 completed 16100 steps'
```

### Step 3: Run Full System

**Terminal 1:**
```bash
conda activate dexter_ros2
cd /home/sean/dexter_test_2/ros2_ws
source install/setup.bash
ros2 run esp32_motor_controller uart_bridge_node
```

**Terminal 2:**
```bash
conda activate dexter_ros2
cd /home/sean/dexter_test_2/ros2_ws
source install/setup.bash
ros2 run motor_control_state_machine motor_control_state_machine
```

**Terminal 3 (Monitor Status):**
```bash
ros2 topic echo /motor_control_status
```

**Terminal 4 (Monitor Responses):**
```bash
ros2 topic echo /esp32_response
```

## Expected Behavior

1. State machine starts automatically after 1 second
2. Publishes LOW command to `/motor_command`
3. uart_bridge_node receives and sends to ESP32
4. ESP32 executes and responds
5. uart_bridge_node publishes response to `/esp32_response`
6. State machine receives response and waits 1 second
7. State machine publishes HIGH command to `/motor_command`
8. Process repeats and completes

## Troubleshooting

### State Machine Still in WAIT After 30 seconds?
- Check if uart_bridge_node is publishing to `/esp32_response`
- Verify response message contains "completed" keyword
- Check ESP32 is actually sending responses back

### No messages on `/esp32_response`?
- Verify publisher was added to uart_bridge_node
- Check uart_bridge_node is running
- Verify serial connection to ESP32 is working
- Look for ESP32 response logs in uart_bridge_node terminal

### State Machine errors?
- Check `/motor_control_status` topic for error messages
- Verify all required dependencies are installed: `transitions`, `rclpy`
- Run: `conda activate dexter_ros2 && python -m pip install transitions`

## Files Modified

- `uart_bridge_node.py` - Add response publisher and publishing logic

## Files Created/Updated

- `motor_control_state_machine.py` - Updated to listen for `/esp32_response`
- `README.md` - Updated documentation
- `UPDATE_SUMMARY.md` - Detailed change log
