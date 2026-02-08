#!/bin/bash
# Test script for Dexter Hardware Motor Controller

echo "======================================"
echo "Dexter Hardware Test Script"
echo "======================================"
echo ""

# Check if ros2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS 2 not found. Please source your ROS 2 setup:"
    echo "   source /opt/ros/jazzy/setup.bash"
    echo "   source ~/dexter_test_2/ros2_ws/install/setup.bash"
    exit 1
fi

echo "✓ ROS 2 found"
echo ""

# Check if packages are built
echo "Checking packages..."
ros2 pkg list | grep -E "dexter_hardware_interfaces|dexter_hardware" > /dev/null
if [ $? -ne 0 ]; then
    echo "❌ Packages not found. Please build first:"
    echo "   cd ~/dexter_test_2/ros2_ws"
    echo "   colcon build --packages-select dexter_hardware_interfaces dexter_hardware"
    exit 1
fi

echo "✓ dexter_hardware packages found"
echo ""

# Check message interface
echo "Checking message definition..."
ros2 interface show dexter_hardware_interfaces/msg/MotorAbsoluteCommand > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo "❌ Message interface not found"
    exit 1
fi

echo "✓ MotorAbsoluteCommand message found"
echo ""

# Check if CAN interface is available
echo "Checking CAN interfaces..."
if ip link show can0 > /dev/null 2>&1; then
    echo "✓ can0 interface found"
    can_interface="can0"
elif ip link show vcan0 > /dev/null 2>&1; then
    echo "⚠ can0 not found, but vcan0 available (virtual/test mode)"
    can_interface="vcan0"
else
    echo "⚠ No CAN interfaces found (can0 or vcan0)"
    echo "   For real hardware: Set up USB-CAN adapter"
    echo "   For testing: Create virtual interface:"
    echo "     sudo ip link add dev vcan0 type vcan"
    echo "     sudo ip link set vcan0 up"
    can_interface="(none)"
fi

echo ""
echo "======================================"
echo "Setup Summary"
echo "======================================"
echo "✓ ROS 2 environment ready"
echo "✓ Packages built successfully"
echo "✓ Message interface available"
if [ "$can_interface" != "(none)" ]; then
    echo "✓ CAN interface: $can_interface"
else
    echo "⚠ No CAN interface available"
fi

echo ""
echo "======================================"
echo "Next Steps"
echo "======================================"
echo ""
echo "1. Start the motor controller node:"
echo "   ros2 launch dexter_hardware motor_controller.launch.xml"
echo ""
echo "2. In another terminal, monitor CAN traffic:"
echo "   candump $can_interface"
echo ""
echo "3. In a third terminal, send a test command:"
echo "   ros2 topic pub -1 /motor_absolute_command dexter_hardware_interfaces/msg/MotorAbsoluteCommand \\"
echo "     \"{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}\""
echo ""
echo "4. Check the output in all terminals to verify CAN frames are being sent"
echo ""
