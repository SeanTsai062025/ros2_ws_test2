#!/bin/bash
# Servo Control Launcher Script
# This ensures the servo node runs with the correct conda environment

# Make sure we're in the dexter_ros2 conda environment
if [[ "$CONDA_DEFAULT_ENV" != "dexter_ros2" ]]; then
    echo "Error: Please activate the dexter_ros2 conda environment first:"
    echo "  conda activate dexter_ros2"
    exit 1
fi

# Make sure the workspace is sourced
if [[ -z "$ROS_DISTRO" ]] || [[ -z "$AMENT_PREFIX_PATH" ]]; then
    echo "Error: Please source the ROS 2 workspace first:"
    echo "  source ~/dexter_test_2/ros2_ws/install/setup.bash"
    exit 1
fi

echo "Starting servo control node..."
echo "Hardware: MG996R servo on GPIO 18 (Pin 12)"
echo "Topic: /servo_angle (std_msgs/msg/Float64)"
echo "Range: 0.0 - 180.0 degrees"
echo ""

# Run the node using Python module instead of ros2 run
python -m servo_control.servo_node "$@"