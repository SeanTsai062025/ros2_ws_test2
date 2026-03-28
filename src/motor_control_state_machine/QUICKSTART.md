# Quick Start Guide

## ✅ Everything is Ready!

Both packages have been fully integrated and are ready to use.

## Build the Packages

```bash
cd /home/sean/dexter_test_2/ros2_ws
colcon build --packages-select esp32_motor_controller motor_control_state_machine
source install/setup.bash
```

## Run the System (4 Terminals)

### Terminal 1: Start UART Bridge Node
```bash
conda activate dexter_ros2
cd /home/sean/dexter_test_2/ros2_ws
source install/setup.bash
ros2 run esp32_motor_controller uart_bridge_node
```

Expected output:
```
[INFO] UART Bridge Node started on /dev/ttyAMA0 at 115200 baud
[INFO] Serial port /dev/ttyAMA0 opened successfully
```

### Terminal 2: Start Motor Control State Machine
```bash
conda activate dexter_ros2
cd /home/sean/dexter_test_2/ros2_ws
source install/setup.bash
ros2 run motor_control_state_machine motor_control_state_machine
```

Expected output:
```
[INFO] Initializing Motor Control State Machine
[INFO] State machine initialized successfully
[INFO] Triggering sequence start...
[INFO] >>> Entered state: SEND_LOW
[INFO] Executing LOW command...
[INFO] LOW command executed successfully
[INFO] >>> Entered state: WAIT
[INFO] Waiting for ESP32 response...
```

### Terminal 3: Monitor Status
```bash
ros2 topic echo /motor_control_status
```

Expected output:
```
data: 'State: INIT'
data: 'State: SEND_LOW'
data: 'LOW command sent'
data: 'State: WAIT'
data: 'Waiting for ESP32 response'
data: 'State: SEND_HIGH'
...
```

### Terminal 4: Monitor ESP32 Responses
```bash
ros2 topic echo /esp32_response
```

Expected output:
```
data: 'ESP32 response: OK: rail1 completed 16100 steps'
data: 'ESP32 response: OK: rail1 completed 16100 steps'
```

## Expected System Behavior

```
Time=0s:   [INIT] Waits for 1 second
Time=1s:   [SEND_LOW] Publishes command to /motor_command
           uart_bridge_node receives and sends to ESP32

Time=2-5s: ESP32 executes LOW command
           ESP32 sends response: "OK: rail1 completed 16100 steps"
           uart_bridge_node publishes to /esp32_response
           state_machine receives response in WAIT state

Time=5s:   [WAIT] Starts final 1-second timer

Time=6s:   [SEND_HIGH] Publishes command to /motor_command
           uart_bridge_node receives and sends to ESP32

Time=7-10s: ESP32 executes HIGH command
            ESP32 sends response: "OK: rail1 completed 16100 steps"
            uart_bridge_node publishes to /esp32_response
            state_machine receives response in WAIT state

Time=10s:  [WAIT] Starts final 1-second timer

Time=11s:  [DONE] Sequence completed successfully!
```

## Troubleshooting

### Issue: State machine stays in WAIT state longer than 30 seconds

**Solution:** Check if uart_bridge_node is working:
```bash
# In a separate terminal, check if responses are being published
ros2 topic echo /esp32_response

# Check uart_bridge_node logs for serial errors
# Look for: [ERROR] Serial read error or [ERROR] Failed to open serial port
```

### Issue: uart_bridge_node fails to open serial port

**Solution:** Check permissions:
```bash
# Grant user access to serial port
sudo usermod -a -G dialout $USER

# Verify serial port exists
ls -la /dev/ttyAMA0

# Test connection (optional)
ros2 topic pub --once /motor_command std_msgs/msg/String "data: 'test'"
```

### Issue: Commands are published but ESP32 doesn't respond

**Solution:** Check ESP32 hardware:
1. Verify UART connections (TX/RX, GND)
2. Check ESP32 firmware
3. Verify baud rate (115200)
4. Test with minicom or similar serial monitor

### Issue: state machine publishes but uart_bridge_node doesn't receive

**Solution:** Check topic name:
```bash
# Verify the topic exists and has messages
ros2 topic list
ros2 topic echo /motor_command
```

## Log Analysis

### Check all logs
```bash
# Watch logs from uart_bridge_node
ros2 run esp32_motor_controller uart_bridge_node 2>&1 | tee uart_bridge.log

# Watch logs from state_machine
ros2 run motor_control_state_machine motor_control_state_machine 2>&1 | tee state_machine.log
```

### Expected log sequence

**uart_bridge_node:**
```
[INFO] UART Bridge Node started on /dev/ttyAMA0 at 115200 baud
[INFO] Serial port /dev/ttyAMA0 opened successfully
[INFO] Sent to ESP32: rail1 16100 low
[INFO] ESP32 response: OK: rail1 completed 16100 steps
[DEBUG] Published to /esp32_response: ESP32 response: OK: rail1 completed 16100 steps
[INFO] Sent to ESP32: rail1 16100 high
[INFO] ESP32 response: OK: rail1 completed 16100 steps
[DEBUG] Published to /esp32_response: ESP32 response: OK: rail1 completed 16100 steps
```

**motor_control_state_machine:**
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

## System Architecture

```
┌─────────────────────────────────────────────┐
│   Motor Control State Machine                │
│   (motor_control_state_machine.py)          │
│                                              │
│  Publishes to: /motor_command               │
│  Subscribes to: /esp32_response            │
│  Publishes to: /motor_control_status       │
└──────────────────┬──────────────────────────┘
                   │
            /motor_command
                   │
                   ▼
┌──────────────────────────────────────────────┐
│   UART Bridge Node                           │
│   (uart_bridge_node.py)                     │
│                                              │
│  Subscribes to: /motor_command             │
│  Publishes to: /esp32_response             │
│  Serial UART ←→ ESP32 (115200 baud)       │
└──────────────────┬──────────────────────────┘
                   │
            /esp32_response
                   │
                   ▼
      ┌────────────────────────┐
      │  Raspberry Pi 5 UART   │
      │  (Dev: /dev/ttyAMA0)  │
      └────────────┬───────────┘
                   │
        (TX/RX, GND)
                   │
                   ▼
      ┌────────────────────────┐
      │  ESP32 Motor Controller│
      │  (Hardware)            │
      └────────────────────────┘
```

## Next Steps

1. ✅ Build both packages
2. ✅ Start uart_bridge_node
3. ✅ Start motor_control_state_machine
4. ✅ Monitor topics to verify integration
5. ✅ Verify motor executes LOW then HIGH commands
6. 🔄 Extend for additional commands or sequences
7. 🔄 Integrate with dexter_moveit for complex motion

## Files Modified

- `/home/sean/dexter_test_2/ros2_ws/src/esp32_motor_controller/esp32_motor_controller/uart_bridge_node.py`
  - Updated response publishing format to include "ESP32 response:" prefix
  - Added debug logging for published messages

- `/home/sean/dexter_test_2/ros2_ws/src/motor_control_state_machine/motor_control_state_machine/motor_control_state_machine.py`
  - Implemented event-driven waiting for ESP32 responses
  - Added 1-second final wait after response received
  - Added 30-second timeout protection

## Questions?

Check the following documentation files:
- `README.md` - Overview and features
- `INTEGRATION_GUIDE.md` - Detailed integration steps
- `UPDATE_SUMMARY.md` - Technical details of changes
