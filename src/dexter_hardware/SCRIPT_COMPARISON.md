# motor_controller_node.py vs dexter_hardware_bridge.py

## Summary Table

| Aspect | motor_controller_node.py | dexter_hardware_bridge.py |
|--------|--------------------------|--------------------------|
| **Purpose** | Manual absolute position control | ROS 2 integration for MoveIt/ros2_control |
| **Input** | Custom ROS message `/motor_absolute_command` | Standard ROS topic `/velocity_controller/commands` |
| **Control Mode** | Absolute motion (0xFE) - position-based | Velocity control (0xF6) - speed-based |
| **Update Rate** | Event-driven (on command received) | Continuous 100 Hz servo loop |
| **Use Case** | Direct motor testing, debugging | Production: MoveIt trajectory execution |
| **Feedback** | None - fire and forget | YES - reads encoders @ 100 Hz, publishes /joint_states |
| **PID Control** | None (motor handles its own) | Position tracking PID in JTC |
| **Startup** | `ros2 launch dexter_hardware motor_controller.launch.xml` | `ros2 launch dexter_hardware hardware_bridge.launch.xml` |

---

## 1. motor_controller_node.py

### What It Does
```
ROS 2 Message /motor_absolute_command
           ↓
   motor_controller_node
           ↓
    Builds CAN frame (0xFE)
    [0xFE][SPEED_H][SPEED_L][ACC][POS_H][POS_M][POS_L][CRC]
           ↓
       CAN bus (can0)
           ↓
    Servo42D motors
    (Motor moves to absolute position)
```

### Key Features
- **Command**: Absolute position (0xFE command)
- **Input Message**: `MotorAbsoluteCommand` from `dexter_hardware_interfaces`
  ```
  motor_id: int              # Which motor (1-6)
  absolute_position: int     # Target position in pulses (-8388607 to +8388607)
  speed: int                 # Speed in RPM (0-3000)
  acceleration: int          # Acceleration (0-255)
  ```
- **No feedback loop** - Just sends command and forgets
- **Use Case**: Quick testing, manual motor control, debugging individual motors

### Example Usage
```bash
# Publish one command to move motor 1 to position 10000 at 100 RPM
ros2 topic pub --once /motor_absolute_command \
  dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 10000, speed: 100, acceleration: 50}"
```

---

## 2. dexter_hardware_bridge.py

### What It Does
```
MoveIt/JTC → /follow_joint_trajectory action
              ↓
        (One-time handoff)
              ↓
JointTrajectoryController (100 Hz)
     ├─ Interpolates desired position @ t
     ├─ Reads /joint_states (actual position)
     ├─ Computes PID error
     └─ Outputs velocity command
              ↓
    /velocity_controller/commands (velocity array)
              ↓
   dexter_hardware_bridge.py (100 Hz loop)
              ├─ Receives velocity commands
              ├─ Sends CAN 0xF6 (speed mode) → motors
              └─ Reads encoders (0x31) → publishes /joint_states
              ↓
         Servo42D motors
    (Motor tracks velocity command)
```

### Key Features
- **Command**: Velocity (0xF6 speed mode command)
- **Input Topic**: `/velocity_controller/commands` - `Float64MultiArray`
  ```
  data: [joint0_vel_rad_s, joint1_vel_rad_s, ..., joint5_vel_rad_s]
  Example: [0.5, 0.3, 0.0, 0.0, 0.0, 0.0]
  ```
- **Feedback Loop @ 100 Hz**:
  1. READ: Send CAN 0x31 to get encoder position
  2. PUBLISH: `/joint_states` with current positions
  3. RECEIVE: Latest velocity commands
  4. WRITE: Send CAN 0xF6 speed mode commands
  5. Repeat

- **Use Case**: Integration with MoveIt, trajectory tracking, closed-loop control

### What It Publishes
- `/joint_states` - Current position, velocity, effort (100 Hz)

### What It Subscribes To
- `/velocity_controller/commands` - Velocity targets from JTC (100 Hz)

---

## Can dexter_hardware_bridge.py Do Absolute Control Mode?

### Current Answer: **NO** - it's designed for velocity-only control

### Why?
1. The bridge **only publishes velocity commands** (0xF6)
2. It **doesn't accept position targets** as input
3. The position tracking PID is in **JTC**, not in the bridge
4. ros2_control's JointTrajectoryController handles position→velocity conversion

### If You Want Absolute Control with the Bridge

You would need to **add absolute motion capability** to the bridge. Here's what would be needed:

#### Option A: Add Absolute Mode Subscriber (QUICK FIX)
Modify `dexter_hardware_bridge.py` to also subscribe to `/motor_absolute_command`:

```python
# In __init__:
self.absolute_cmd_sub = self.create_subscription(
    MotorAbsoluteCommand,
    '/motor_absolute_command',
    self.absolute_command_callback,
    10
)

# New callback:
def absolute_command_callback(self, msg):
    """Handle absolute position commands"""
    motor_id = msg.motor_id
    position = msg.absolute_position
    speed = msg.speed
    acceleration = msg.acceleration
    
    # Build 0xFE command
    data = [
        MksCommands.RUN_MOTOR_ABSOLUTE_MOTION_BY_PULSES_COMMAND.value,  # 0xFE
        (speed >> 8) & 0xFF,
        speed & 0xFF,
        acceleration & 0xFF,
        (position >> 16) & 0xFF,
        (position >> 8) & 0xFF,
        position & 0xFF,
    ]
    
    msg_can = self.create_can_message(motor_id, data)
    try:
        self.bus.send(msg_can)
    except can.CanError as e:
        self.get_logger().error(f'Failed to send absolute command: {e}')
```

#### Option B: Keep Them Separate (RECOMMENDED)
- Use `motor_controller_node.py` for **manual absolute positioning** (testing, setup)
- Use `dexter_hardware_bridge.py` for **MoveIt trajectory tracking** (production)

---

## Decision Matrix

**Use motor_controller_node.py if you want to:**
- Test individual motors
- Move to a specific absolute position
- Debug CAN communication
- Don't need feedback/closed-loop
- Send one-off commands

**Use dexter_hardware_bridge.py if you want to:**
- Execute trajectories from MoveIt
- Track positions over time with PID feedback
- Integrate with ros2_control ecosystem
- Run continuous servo loop @ 100 Hz
- Publish joint state for other ROS nodes

**Use both if you want to:**
- Motor testing + MoveIt integration
- Switch between modes (manual vs auto)
  ```bash
  # Manual mode:
  ros2 launch dexter_hardware motor_controller.launch.xml
  
  # Production mode:
  ros2 launch dexter_hardware hardware_bridge.launch.xml use_sim:=false
  ```
  (Don't run both at the same time on the same motors - CAN conflict!)

---

## Command Comparison

### Test Motor 1 with motor_controller_node.py (ABSOLUTE)
```bash
# Move motor 1 to position 5000 at 150 RPM
ros2 topic pub --once /motor_absolute_command \
  dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 5000, speed: 150, acceleration: 100}"
```

### Test Motor 1 with dexter_hardware_bridge.py (VELOCITY)
```bash
# Spin motor 1 at 0.5 rad/s (feedforward to hardware)
ros2 topic pub -r 10 /velocity_controller/commands \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

---

## Summary Answer

| Question | Answer |
|----------|--------|
| What does motor_controller_node.py do? | Sends **absolute position commands** (0xFE) to individual motors via ROS messages |
| What does dexter_hardware_bridge.py do? | Implements a **100 Hz servo loop** that reads encoders and sends **velocity commands** (0xF6) from ros2_control (JTC) |
| Can bridge do absolute control? | **Not currently** - but you could add a subscriber to handle it. Better to keep them separate. |
| Which one for MoveIt? | **hardware_bridge.py** - it provides feedback and integrates with JTC's PID loop |
| Which one for testing? | **motor_controller_node.py** - simpler, more direct, no feedback needed |
