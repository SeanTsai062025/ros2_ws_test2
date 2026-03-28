# State Machine Update Summary

## Changes Made

### Key Improvement: Event-Driven Response Waiting

**Before:**
- Fixed 3-second wait using `create_timer(3.0)`
- Didn't wait for actual motor completion
- Risk of timing issues

**After:**
- Waits for ESP32 completion response: `"ESP32 response: OK: rail1 completed 16100 steps"`
- Subscribes to `/esp32_response` topic
- 30-second timeout safety mechanism
- Then waits 1 additional second before next command
- More robust and reliable

## New Features

### 1. ESP32 Response Listener
```python
def _esp32_response_callback(self, msg: String) -> None:
    """Callback when ESP32 response is received."""
    if "completed" in msg.data.lower():
        self._esp32_response_received = True
        if self.state == 'WAIT':
            self._start_final_wait()
```

### 2. Smart WAIT State
```python
def on_enter_WAIT(self) -> None:
    """Wait for ESP32 response (max 30 seconds)"""
    # Set timeout in case no response
    self._wait_timer = self.create_timer(
        timer_period_sec=30.0,
        callback=self._wait_timeout_callback,
        oneshot=True
    )
```

### 3. Final 1-Second Wait
```python
def _start_final_wait(self) -> None:
    """After response received, wait 1 more second"""
    self._wait_timer = self.create_timer(
        timer_period_sec=1.0,
        callback=self._final_wait_complete_callback,
        oneshot=True
    )
```

### 4. Timeout Protection
- If ESP32 doesn't respond within 30 seconds, transitions to ERROR state
- Prevents infinite waiting

## Integration Steps

### Step 1: Update uart_bridge_node.py

Add a publisher to publish ESP32 responses:

```python
# In __init__:
self.esp32_response_pub = self.create_publisher(
    String,
    '/esp32_response',
    10
)

# When receiving ESP32 response (around line where you log "ESP32 response: ..."):
def _handle_esp32_response(self, response_text):
    self.get_logger().info(f"ESP32 response: {response_text}")
    
    # Publish to topic for state machine
    response_msg = String()
    response_msg.data = f"ESP32 response: {response_text}"
    self.esp32_response_pub.publish(response_msg)
```

### Step 2: Build and Test

```bash
cd /home/sean/dexter_test_2/ros2_ws
colcon build --packages-select motor_control_state_machine
source install/setup.bash
```

### Step 3: Run the System

**Terminal 1:**
```bash
conda activate dexter_ros2
ros2 run esp32_motor_controller uart_bridge_node
```

**Terminal 2:**
```bash
conda activate dexter_ros2
ros2 run motor_control_state_machine motor_control_state_machine
```

**Terminal 3 (Monitor):**
```bash
ros2 topic echo /motor_control_status
```

## State Sequence

```
1. INIT (1 second)
   ↓
2. SEND_LOW (publish command)
   ↓
3. WAIT (listen for "ESP32 response: OK: rail1 completed..." on /esp32_response)
   ↓ (response received)
4. FINAL_WAIT (1 second internal timer)
   ↓
5. SEND_HIGH (publish command)
   ↓
6. WAIT (listen for second completion)
   ↓ (response received)
7. FINAL_WAIT (1 second internal timer)
   ↓
8. DONE (complete)
```

## Log Example

```
[INFO] Initializing Motor Control State Machine
[INFO] State machine initialized successfully
[INFO] >>> Entered state: INIT
[INFO] Triggering sequence start...
[INFO] >>> Entered state: SEND_LOW
[INFO] Executing LOW command...
[INFO] Executing: ros2 topic pub --once /motor_command std_msgs/msg/String "data: 'rail1 16100 low'"
[INFO] LOW command executed successfully
[INFO] >>> Entered state: WAIT
[INFO] Waiting for ESP32 response...
[INFO] Received ESP32 response: ESP32 response: OK: rail1 completed 16100 steps
[INFO] ESP32 completion confirmed
[INFO] Response received while in WAIT state, starting final wait...
[INFO] Starting final 1-second wait before SEND_HIGH...
[INFO] Final wait complete, transitioning to SEND_HIGH
[INFO] >>> Entered state: SEND_HIGH
[INFO] Executing HIGH command...
[INFO] >>> Entered state: DONE
[INFO] Sequence completed successfully!
```

## Testing Checklist

- [ ] Build completes without errors
- [ ] State machine starts automatically after 1 second
- [ ] First command (LOW) is published
- [ ] uart_bridge_node receives and sends to ESP32
- [ ] ESP32 executes and sends response back
- [ ] State machine receives response on `/esp32_response`
- [ ] Waits 1 second after response
- [ ] Second command (HIGH) is published
- [ ] Process completes successfully
- [ ] No race conditions or timing issues
