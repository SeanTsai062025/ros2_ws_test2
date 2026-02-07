# Dexter Hardware Control - ROS 2 Package

This package provides a ROS 2 node for controlling MKS Servo stepper motors (Servo42D/Servo57D) via CAN bus on a Raspberry Pi 5 using a USB-CAN adapter.

## Components

- **dexter_hardware_interfaces**: Custom ROS 2 message definitions
- **dexter_hardware**: Motor controller node that communicates with servos via CAN

## System Setup

### Prerequisites

1. **Python CAN library** (required by mks-servo-can):
   ```bash
   pip install python-can
   ```

2. **CAN tools** (for testing and verification):
   ```bash
   sudo apt-get install can-utils
   ```

3. **SocketCAN interface** on Raspberry Pi 5:
   - Connect your USB-CAN adapter to the Pi
   - Check if it's recognized:
     ```bash
     ls /dev/ttyUSB*
     ```

### Configure CAN Interface

If your CAN adapter shows up as `/dev/ttyUSB0`, create a virtual SocketCAN interface:

```bash
# Install slcan utilities
sudo apt-get install can-utils

# Bring up the interface at 1Mbps (default for MKS Servo)
sudo slcand -o -c -f -s8 /dev/ttyUSB0 can0
sudo ip link set can0 up

# Verify it's up
ip link show can0
```

To make this persistent across reboots, add to `/etc/network/interfaces` or create a systemd service.

## Building the Package

```bash
cd ~/dexter_test_2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dexter_hardware_interfaces dexter_hardware
source install/setup.bash
```

## Running the Motor Controller

### Terminal 1: Start the motor controller node

```bash
ros2 launch dexter_hardware motor_controller.launch.xml
```

You should see:
```
[INFO] [motor_controller-1]: Initializing CAN interface: can0 at 1000000 bps
[INFO] [motor_controller-1]: CAN bus initialized successfully
[INFO] [motor_controller-1]: Motor controller node initialized. Waiting for commands on /motor_absolute_command
```

### Terminal 2: Send a motor command

Example: Move motor ID 1 to absolute position 0 with speed 300 and acceleration 10

```bash
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}"
```

You should see in Terminal 1:
```
[INFO] [motor_controller-1]: Received motor command: ID=1, position=0, speed=300, acc=10
[INFO] [motor_controller-1]: Sending absolute motion command to motor 1
[INFO] [motor_controller-1]: Created servo instance for motor ID 1
[INFO] [motor_controller-1]: Motor 1 command sent successfully. Result: RunComplete
```

## Message Format

The node subscribes to the topic `/motor_absolute_command` with the following message structure:

```
uint8 motor_id           # Motor CAN ID (typically 1-8)
int32 absolute_position  # Target position in pulses (-8388607 to +8388607)
uint16 speed             # Motor speed in RPM (0-3000)
uint8 acceleration       # Acceleration level (0-255)
```

## Verifying CAN Communication

### Check CAN Interface Status

```bash
# Show CAN interface details
ip -details link show can0

# Expected output should show something like:
# can0: <NOTRAILERS,UP,LOWER_UP> mtu 72 qdisc pfifo_fast
#     link/can  promiscuity 0 allmulti 0
```

### Monitor CAN Traffic in Real-Time

In a terminal, run `candump` to see all CAN frames:

```bash
# Monitor can0 interface
candump can0

# You should see frames like:
# can0  001   [6]  81 2C 0A 00 00 00
# can0  001   [3]  81 02 00
```

The format is: `[CAN_ID] [DLC] [DATA_BYTES]`

- `CAN_ID`: Motor ID in hex (e.g., `001` = motor 1)
- `DLC`: Data length (number of bytes)
- `DATA_BYTES`: The actual CAN command frame

### Send a Raw CAN Frame (for manual testing)

You can also send raw CAN frames to test the motor:

```bash
# Example: Send an enable command to motor 1
cansend can0 001#0100

# Example: Send an absolute position command
cansend can0 001#812C0A000000
```

### Capture CAN Traffic to File

```bash
# Capture traffic for analysis
candump -l can0

# This creates a log file (candump-YYYY-MM-DD_HH-MM-SS.log)
# You can replay it later:
canplayer -I candump-2025-02-05_10-30-00.log

```

## Troubleshooting

### CAN Interface Not Found

If you get `Failed to initialize CAN bus`:

1. Check the interface exists:
   ```bash
   ifconfig can0  # or: ip link show can0
   ```

2. If `can0` doesn't exist, set it up again:
   ```bash
   sudo slcand -o -c -f -s8 /dev/ttyUSB0 can0
   sudo ip link set can0 up
   ```

3. Verify the USB-CAN adapter is connected:
   ```bash
   lsusb
   ```

### Motor Not Responding

1. Check that the motor ID is correct (should match the motor's CAN ID)
2. Monitor the CAN bus to see if frames are being sent:
   ```bash
   candump can0
   ```
3. Check motor status with the library's query commands (implement if needed)

### No CAN Frames Observed

1. Ensure `candump` is running BEFORE publishing the command
2. Verify the CAN interface is up:
   ```bash
   ip link show can0
   ```
3. Check that the ROS2 node started without errors

## Command Examples

### Example 1: Home Position

Move to position 0 (typically home) with moderate speed:

```bash
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 0, speed: 500, acceleration: 50}"
```

### Example 2: Full Rotation

Rotate the motor one full rotation (~51200 pulses for a typical 200-step motor):

```bash
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 51200, speed: 1000, acceleration: 100}"
```

### Example 3: Multi-Motor Command

Send commands to multiple motors (in separate commands):

```bash
# Motor 1 to position 10000
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 1, absolute_position: 10000, speed: 300, acceleration: 10}"

# Motor 2 to position 5000
ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  "{motor_id: 2, absolute_position: 5000, speed: 300, acceleration: 10}"
```

## Testing Without Hardware

You can test the node and verify CAN frames are being sent without actual hardware:

1. Set up a virtual CAN interface:
   ```bash
   sudo ip link add dev vcan0 type vcan
   sudo ip link set vcan0 up
   ```

2. Modify the launch file to use `vcan0` instead of `can0`, or pass as parameter:
   ```bash
   ros2 launch dexter_hardware motor_controller.launch.xml can_interface:=vcan0
   ```

3. Monitor with candump:
   ```bash
   candump vcan0
   ```

4. Send a command and verify frames appear in candump output

## Performance Notes

- Default CAN bitrate: **1Mbps** (standard for MKS Servo)
- Command response timeout: **1 second** (configurable in the library)
- CAN message format follows MKS Servo specification

## For More Information

- MKS Servo documentation: Check the MKS SERVO42&57D_CAN User Manual
- mks-servo-can library: `/home/sean/dexter_test_2/ros2_ws/src/dexter_hardware/mks-servo-can/`
- python-can documentation: https://python-can.readthedocs.io/

