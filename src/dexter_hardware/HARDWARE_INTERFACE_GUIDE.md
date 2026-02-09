# Dexter Hardware Interface Implementation Guide

## Architecture Overview

```
┌─────────────────┐
│     MoveIt      │
│  (Motion Plan)  │
└────────┬────────┘
         │ FollowJointTrajectory action
         ▼
┌─────────────────────────────────────────────────────┐
│           JointTrajectoryController (JTC)           │
│  ┌─────────────────────────────────────────────┐   │
│  │ For each joint at 100Hz:                     │   │
│  │   pos_error = desired_pos - actual_pos       │   │
│  │   vel_cmd = Kp*pos_error + Kd*vel_error     │   │
│  │           + velocity_feedforward             │   │
│  └─────────────────────────────────────────────┘   │
└────────┬───────────────────────────┬───────────────┘
         │ velocity commands         │ position feedback
         ▼                           ▲
┌─────────────────────────────────────────────────────┐
│         DexterHardwareBridge (Python Node)          │
│  ┌───────────────┐         ┌───────────────────┐   │
│  │ write():      │         │ read():           │   │
│  │ rad/s → RPM   │         │ ticks → radians   │   │
│  │ Send 0xF6     │         │ Query 0x31        │   │
│  └───────┬───────┘         └─────────▲─────────┘   │
│          │                           │              │
└──────────┼───────────────────────────┼──────────────┘
           │ CAN bus (500kbps)         │
           ▼                           │
┌─────────────────────────────────────────────────────┐
│              Servo42D Motors (x6)                   │
│  ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐  │
│  │ ID1 │ │ ID2 │ │ ID3 │ │ ID4 │ │ ID5 │ │ ID6 │  │
│  │base │ │part1│ │part2│ │part3│ │part4│ │part5│  │
│  └─────┘ └─────┘ └─────┘ └─────┘ └─────┘ └─────┘  │
└─────────────────────────────────────────────────────┘
```

## Answers to Your Questions

### 1. Which ROS2 topic carries the 100Hz trajectory from MoveIt?

**It's NOT a topic - it's an ACTION!**

```
Action Server: /arm_controller/follow_joint_trajectory
Action Type:   control_msgs/action/FollowJointTrajectory
```

MoveIt sends a trajectory (list of waypoints with timestamps) via this action.
The JTC then interpolates between waypoints at 100Hz internally.

### 2. How does it connect to joint_trajectory_controller?

**JTC does the heavy lifting:**
- Receives trajectory via action
- Interpolates at 100Hz (cubic spline)
- Computes velocity commands using PID
- Sends velocity to hardware interface
- Reads position from hardware interface

**PID in JTC:**
```
velocity_cmd = Kp * position_error + Kd * velocity_error + velocity_feedforward
```

Configure gains in `ros2_controllers.yaml`:
```yaml
arm_controller:
  ros__parameters:
    gains:
      base: {p: 10.0, i: 0.0, d: 0.5}
      part1: {p: 10.0, i: 0.0, d: 0.5}
      ...
```

### 3. How to switch Servo42D from Position Mode to Speed Mode?

**Speed Mode is command 0xF6:**
```
CAN Frame:
  ID: motor_id (1-6)
  Data: [0xF6] [DIR+SPEED_H] [SPEED_L] [ACC] [0x00] [0x00] [0x00] [CRC]

Where:
  - DIR: 0x00=CCW, 0x80=CW (in high nibble)
  - SPEED: 0-3000 RPM (12-bit)
  - ACC: 0-255 (0=fastest)
  - CRC: (CAN_ID + sum(bytes[0:7])) & 0xFF
```

**To STOP the motor:**
```
Send 0xF6 with SPEED=0
```

### 4. How should read()/write() functions be structured?

**read() - Query Encoder (0x31):**
```python
def read_encoder_position(self, motor_id: int) -> int:
    # Send: [0x31] [CRC]
    # Receive: [0x31] [POS_5] [POS_4] [POS_3] [POS_2] [POS_1] [POS_0] [CRC]
    # Return: 48-bit signed integer (encoder ticks)
```

**write() - Send Velocity (0xF6):**
```python
def send_velocity_command(self, motor_id: int, velocity_rpm: float):
    # Convert rad/s to RPM
    # Determine direction (+ = CCW, - = CW)
    # Send: [0xF6] [DIR+SPEED_H] [SPEED_L] [ACC] [0x00] [0x00] [0x00] [CRC]
```

## Unit Conversions

**Position: Encoder Ticks → Radians**
```python
revolutions = encoder_ticks / ticks_per_rev / gear_ratio
position_rad = revolutions * 2 * math.pi
```

**Velocity: Radians/sec → RPM**
```python
rpm = (rad_per_sec * 60) / (2 * math.pi)
```

## Files Created

| File | Purpose |
|------|---------|
| `scripts/dexter_hardware_bridge.py` | Main hardware bridge node |
| `config/hardware_params.yaml` | Motor configuration (CAN IDs, etc.) |
| `launch/hardware_bridge.launch.xml` | Launch file for hardware bridge |
| `ros2_controllers.yaml` (updated) | JTC with velocity commands + PID gains |

## How to Run

### Step 1: Setup CAN interface
```bash
sudo ip link set can0 up type can bitrate 500000
```

### Step 2: Launch hardware bridge
```bash
ros2 launch dexter_hardware hardware_bridge.launch.xml
```

### Step 3: Launch MoveIt + controllers
```bash
ros2 launch dexter_moveit_config demo.launch.py
```

### Step 4: Plan and execute in RViz
- Use MoveIt RViz plugin to plan trajectory
- Click "Plan & Execute"
- Motors should move!

## Tuning Tips

### PID Gains
Start with low gains and increase:
```yaml
gains:
  base:
    p: 5.0    # Proportional (increase for faster response)
    i: 0.0    # Integral (increase to eliminate steady-state error)
    d: 0.5    # Derivative (increase to reduce overshoot)
```

### Motor Safety
- Set `max_rpm` in config to limit velocity
- Start with low values (100 RPM) for testing
- The hardware bridge stops all motors on shutdown

## Troubleshooting

### No motor movement
1. Check CAN connection: `candump can0`
2. Verify motor IDs match config
3. Check motor power supply
4. Try manual command: `cansend can0 001#F6000A0000000017`

### Jerky motion
1. Increase control rate (but not above 100Hz for CAN bus)
2. Tune PID gains (lower P, higher D)
3. Check for encoder noise

### Position drift
1. Check encoder readings are stable
2. Add integral gain (Ki)
3. Verify gear ratio is correct

