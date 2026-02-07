# Dexter Robot ROS 2 Setup - Complete ✅

## Summary

All Dexter robot packages have been successfully created, built, and verified. Your ROS 2 workspace is now ready for:
- **Motion Planning**: MoveIt 2 planning and execution
- **Hardware Control**: CAN-based motor control via mks-servo-can library
- **Visualization**: RViz display and monitoring

## Package Status

All 7 packages built successfully and are discoverable:

| Package | Type | Status | Purpose |
|---------|------|--------|---------|
| `dexter_description` | CMake/URDF | ✅ Built | Robot URDF model and visualization assets |
| `dexter_moveit_config` | CMake | ✅ Built | MoveIt 2 configuration, SRDF, controllers |
| `dexter_bringup` | CMake | ✅ Built | Launch file to start entire robot stack |
| `dexter_interfaces` | CMake | ✅ Built | Custom ROS 2 messages (PoseCommand) |
| `dexter_commander_cpp` | C++/CMake | ✅ Built | Motion command subscriber and executor |
| `dexter_hardware_interfaces` | CMake | ✅ Built | Hardware messages (MotorAbsoluteCommand) |
| `dexter_hardware` | Python | ✅ Built | Motor controller node for CAN communication |

## Quick Start

### 1. Set up ROS 2 environment
```bash
source /opt/ros/jazzy/setup.bash
source ~/dexter_test_2/ros2_ws/install/setup.bash
```

### 2. Launch robot with MoveIt
```bash
ros2 launch dexter_bringup dexter.launch.xml
```

This will start:
- Robot state publisher
- ROS 2 control manager
- MoveIt planning scene
- RViz with motion planning interface

### 3. Launch motor controller (for hardware)
```bash
ros2 launch dexter_hardware motor_controller.launch.xml
```

Optional parameters:
```bash
ros2 launch dexter_hardware motor_controller.launch.xml \
  can_interface:=can0 \
  bitrate:=1000000
```

### 4. Send motor commands
```bash
ros2 topic pub -1 /motor_absolute_command \
  dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}"
```

## Testing Without Hardware

You can test the entire software stack without physical hardware using virtual CAN:

```bash
# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

# Launch motor controller with virtual interface
ros2 launch dexter_hardware motor_controller.launch.xml can_interface:=vcan0

# In another terminal, monitor CAN frames
candump vcan0

# In a third terminal, send test commands
ros2 topic pub -1 /motor_absolute_command \
  dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}"
```

You should see CAN frames appearing in the candump output.

## Hardware Setup (When Ready)

### Prerequisites
- Raspberry Pi 5 with Ubuntu 24.04
- USB-CAN adapter (e.g., PEAK-System PCAN-USB)
- MKS Servo42D motors with CAN IDs 1-6
- python-can and can-utils packages

### Setup CAN Interface
```bash
# Install can-utils
sudo apt install can-utils

# Set up SocketCAN for USB adapter
sudo slcand -o -c -f -s8 /dev/ttyUSB0 can0
sudo ip link set can0 up

# Verify CAN interface
ip link show can0
```

### Verify CAN Communication
```bash
# Monitor CAN traffic
candump can0

# Send test frame (motor 1 to position 0)
cansend can0 001#3001080000000000
```

### Run Motor Controller
```bash
ros2 launch dexter_hardware motor_controller.launch.xml
```

## File Structure

```
dexter_hardware/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
├── README.md                   # Detailed documentation
├── dexter_hardware/
│   └── __init__.py            # Python package marker
├── launch/
│   └── motor_controller.launch.xml  # Node launcher
└── scripts/
    └── motor_controller_node.py     # Main Python node
```

## Verification Commands

Check if all packages are installed:
```bash
ros2 pkg list | grep dexter
```

Expected output:
```
dexter_bringup
dexter_commander_cpp
dexter_description
dexter_hardware
dexter_hardware_interfaces
dexter_interfaces
dexter_moveit_config
```

Check message definition:
```bash
ros2 interface show dexter_hardware_interfaces/msg/MotorAbsoluteCommand
```

Expected output:
```
uint8 motor_id
int32 absolute_position
uint16 speed
uint8 acceleration
```

Check available interfaces:
```bash
ip link show
# Look for can0 (real) or vcan0 (virtual)
```

## Motor Command Format

Send motor commands via `/motor_absolute_command` topic:

```python
MotorAbsoluteCommand:
  motor_id: 0-255              # MKS Servo CAN ID
  absolute_position: -8388607 to +8388607  # Pulses
  speed: 0-3000               # RPM
  acceleration: 0-255         # Acceleration rate (0=max, 255=min)
```

Example:
```bash
ros2 topic pub /motor_absolute_command \
  dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  '{motor_id: 1, absolute_position: 1000, speed: 500, acceleration: 50}'
```

## Troubleshooting

### CAN interface not found
```bash
# Check connected USB devices
lsusb

# Check if CAN adapter is recognized
ls -la /dev/ttyUSB*

# If using slcand, verify it's running
ps aux | grep slcand
```

### Motor controller node crashes
Check the logs:
```bash
ros2 launch dexter_hardware motor_controller.launch.xml --log-level debug
```

### No CAN frames being sent
1. Verify CAN interface is up: `ip link show can0`
2. Check node is running: `ros2 node list`
3. Monitor CAN traffic: `candump can0`
4. Send test command and watch for frames

### Motor not responding
1. Verify motor CAN ID matches command
2. Check motor power supply
3. Test with MKS Servo debug tools
4. Verify CAN bitrate (default 1000000 = 1Mbps)

## Next Steps

### Motion Planning
Use the MoveIt interactive marker in RViz to plan and execute trajectories:
1. Launch dexter_bringup
2. In RViz, open the MotionPlanning panel
3. Use the interactive marker to set target pose
4. Click "Plan" then "Execute"

### Commander Node
Use the dexter_commander_cpp node to subscribe to motion commands:
```bash
ros2 run dexter_commander_cpp commander
```

Then publish commands:
```bash
# Joint space command
ros2 topic pub /joint_command std_msgs/Float64MultiArray \
  "{data: [0, 0, 0, 0, 0, 0]}"

# Cartesian space command
ros2 topic pub /pose_command dexter_interfaces/msg/PoseCommand \
  "{x: 0.3, y: 0.0, z: 0.3, roll: 0, pitch: 0, yaw: 0, cartesian_path: false}"
```

### Hardware Integration
Connect actual motors and test the hardware layer:
1. Set up USB-CAN adapter with slcand
2. Verify CAN interface with candump
3. Launch motor_controller node
4. Send motor commands via ROS 2 topic pub

## Documentation

For more detailed information, see:
- `dexter_hardware/README.md` - Complete hardware controller documentation
- `DEXTER_HARDWARE_QUICKSTART.md` - Quick start guide for motor controller
- Individual package README files in each package directory

## Workspace Location

All files are located in:
```
~/dexter_test_2/ros2_ws/src/
├── dexter_description/
├── dexter_moveit_config/
├── dexter_bringup/
├── dexter_interfaces/
├── dexter_commander_cpp/
├── dexter_hardware_interfaces/
└── dexter_hardware/
```

Build artifacts are in:
```
~/dexter_test_2/ros2_ws/build/
~/dexter_test_2/ros2_ws/install/
```

## Build Information

- **Build Time**: ~55 seconds (all 7 packages)
- **Build Status**: ✅ All successful
- **Build System**: Colcon with Ament CMake and Python

Last built: **2025-02-06**

## Support

For issues or questions:
1. Check the Troubleshooting section in `dexter_hardware/README.md`
2. Review the QUICKSTART guide
3. Check ROS 2 logs: `ros2 launch ... --log-level debug`
4. Monitor CAN traffic: `candump can0`

---

**Setup complete! Your Dexter robot is ready to use.** ✅
