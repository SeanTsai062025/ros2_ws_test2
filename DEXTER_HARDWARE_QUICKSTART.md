# Dexter Hardware Package - Quick Start

## What Was Created

Two new ROS 2 packages:

1. **dexter_hardware_interfaces** - Custom message types
   - `MotorAbsoluteCommand.msg` - Message for absolute position commands

2. **dexter_hardware** - Motor controller node
   - `motor_controller_node.py` - Python node that:
     - Initializes CAN bus (`can0`)
     - Subscribes to `/motor_absolute_command` topic
     - Translates commands into CAN frames using `mks-servo-can` library
     - Sends frames to motors via SocketCAN

## Quick Test (Software-Only)

### 1. Prerequisites
```bash
pip install python-can
sudo apt-get install can-utils
```

### 2. Build
```bash
cd ~/dexter_test_2/ros2_ws
colcon build --packages-select dexter_hardware_interfaces dexter_hardware
source install/setup.bash
```

### 3. Set Up Virtual CAN (for testing without hardware)
```bash
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

### 4. Start Node (Terminal 1)
```bash
ros2 launch dexter_hardware motor_controller.launch.xml can_interface:=vcan0
```

### 5. Monitor CAN (Terminal 2)
```bash
candump vcan0
```

### 6. Send Command (Terminal 3)
```bash
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}"
```

**You should see CAN frames in the candump output!**

## Real Hardware Setup

When you have the USB-CAN adapter connected to Pi:

```bash
# Set up real CAN interface
sudo slcand -o -c -f -s8 /dev/ttyUSB0 can0
sudo ip link set can0 up

# Launch with real interface (default is can0)
ros2 launch dexter_hardware motor_controller.launch.xml
```

## Command Format

```bash
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: MOTOR_ID, absolute_position: POSITION, speed: SPEED, acceleration: ACCEL}"
```

Example (move motor 1 to position 0):
```bash
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}"
```

## Verify CAN Communication

```bash
# Check CAN interface
ip link show can0

# Monitor CAN traffic
candump can0

# Expected frame format:
# can0  001   [6]  81 2C 0A 00 00 00
#       ^^^   ^^^  ^^^^^^^^^^^^^^^^^^
#       ID    DLC  Command data

# Send raw CAN frame (testing)
cansend can0 001#812C0A000000
```

## Files Created

```
dexter_hardware_node/
├── CMakeLists.txt
├── package.xml
├── README.md
├── dexter_hardware/
│   └── __init__.py
├── scripts/
│   └── motor_controller_node.py
└── launch/
    └── motor_controller.launch.xml

dexter_hardware_interfaces/
├── CMakeLists.txt
├── package.xml
└── msg/
    └── MotorAbsoluteCommand.msg
```

## Node Behavior

- Initializes CAN bus at 1Mbps (standard for MKS Servo)
- Waits for messages on `/motor_absolute_command`
- Creates servo instances on-demand per motor ID
- Sends absolute position commands via CAN
- Logs all activity to console

## Supported Control Modes (Currently)

- ✅ Absolute position mode (`run_motor_absolute_motion_by_pulses`)

Can easily extend to support:
- Speed mode
- Relative motion
- Calibration
- Home positioning
- etc.

See README.md for full documentation and troubleshooting.
