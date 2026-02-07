#!/bin/bash
# START HERE - Dexter Robot Setup Guide Index

cat << 'EOF'

╔════════════════════════════════════════════════════════════════════════╗
║                                                                        ║
║              DEXTER ROBOT ROS 2 SETUP - START HERE ✅                ║
║                                                                        ║
║                 Everything is ready to use!                           ║
║                                                                        ║
╚════════════════════════════════════════════════════════════════════════╝

📚 DOCUMENTATION FILES (READ THESE)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. README_FINAL.md (9.1K) ⭐ START HERE
   └─ Complete setup summary, quick start, all features explained
   └─ Command examples and next steps
   └─ Location: ~/dexter_test_2/ros2_ws/

2. DEXTER_SETUP_COMPLETE.md (7.6K)
   └─ Detailed setup guide with troubleshooting
   └─ Hardware integration instructions
   └─ Testing methods (virtual CAN, real hardware)
   └─ Location: ~/dexter_test_2/ros2_ws/

3. DEXTER_HARDWARE_QUICKSTART.md (3.3K)
   └─ Motor controller quick reference
   └─ Direct testing instructions
   └─ Virtual CAN setup guide
   └─ Location: ~/dexter_test_2/ros2_ws/

4. dexter_hardware/README.md (7.0K)
   └─ Motor controller node documentation
   └─ CAN configuration details
   └─ Advanced examples and troubleshooting
   └─ Location: ~/dexter_test_2/ros2_ws/src/dexter_hardware/

🔧 VERIFICATION & TESTING SCRIPTS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. verify_dexter_setup.sh ✅ RUN THIS FIRST
   └─ Checks all packages are installed
   └─ Verifies message interfaces
   └─ Checks Python dependencies
   └─ Usage: ~/dexter_test_2/ros2_ws/verify_dexter_setup.sh

2. test_dexter_hardware.sh
   └─ Quick test of motor controller setup
   └─ Checks CAN interfaces
   └─ Provides next steps for testing
   └─ Usage: ~/dexter_test_2/ros2_ws/test_dexter_hardware.sh

📦 PACKAGES CREATED (7 TOTAL)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Core Packages:
✅ dexter_description       - Robot URDF model and assets
✅ dexter_moveit_config     - MoveIt 2 planning configuration
✅ dexter_bringup           - Main robot launch file
✅ dexter_interfaces        - Custom message types (PoseCommand)
✅ dexter_commander_cpp     - Motion command executor

Hardware Packages:
✅ dexter_hardware_interfaces   - Hardware message types
✅ dexter_hardware              - CAN motor controller node

📍 LOCATION: ~/dexter_test_2/ros2_ws/src/

🚀 QUICK START (3 STEPS)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Step 1: Setup environment
  source /opt/ros/jazzy/setup.bash
  source ~/dexter_test_2/ros2_ws/install/setup.bash

Step 2: Verify setup
  ~/dexter_test_2/ros2_ws/verify_dexter_setup.sh

Step 3: Launch robot or motor controller

  Option A - Full robot with MoveIt (RECOMMENDED):
    ros2 launch dexter_bringup dexter.launch.xml

  Option B - Motor controller only:
    ros2 launch dexter_hardware motor_controller.launch.xml

  Option C - Motor controller with virtual CAN (testing):
    sudo ip link add dev vcan0 type vcan
    sudo ip link set vcan0 up
    ros2 launch dexter_hardware motor_controller.launch.xml can_interface:=vcan0

🎯 WHAT YOU CAN DO NOW
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✓ Visualize robot in RViz
✓ Plan and execute trajectories with MoveIt 2
✓ Send motion commands via ROS 2 topics
✓ Control motors via CAN bus (with hardware connected)
✓ Test motor controller with virtual CAN (no hardware needed)
✓ Monitor CAN traffic with candump
✓ Extend with custom control modes

📖 RECOMMENDED READING ORDER
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

For Quick Start:
  1. This file (you are here!)
  2. README_FINAL.md - Overview and commands
  3. DEXTER_HARDWARE_QUICKSTART.md - Motor control

For Complete Understanding:
  1. README_FINAL.md - Features and architecture
  2. DEXTER_SETUP_COMPLETE.md - Detailed setup
  3. dexter_hardware/README.md - Technical details
  4. Individual package CMakeLists.txt and package.xml files

For Hardware Integration:
  1. DEXTER_SETUP_COMPLETE.md - Hardware section
  2. dexter_hardware/README.md - Complete guide
  3. Section "Hardware Setup (When Ready)" in README_FINAL.md

🔐 FIRST TIME SETUP
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Source ROS 2:
   source /opt/ros/jazzy/setup.bash
   source ~/dexter_test_2/ros2_ws/install/setup.bash

2. Verify everything works:
   ~/dexter_test_2/ros2_ws/verify_dexter_setup.sh

3. Test with virtual CAN (no hardware):
   sudo ip link add dev vcan0 type vcan
   sudo ip link set vcan0 up
   ros2 launch dexter_hardware motor_controller.launch.xml can_interface:=vcan0

4. In another terminal, send test command:
   ros2 topic pub -1 /motor_absolute_command \
     dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
     '{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}'

5. Monitor CAN traffic:
   candump vcan0

✨ KEY FEATURES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

✅ 7 integrated ROS 2 packages
✅ MoveIt 2 motion planning support
✅ CAN bus motor control via mks-servo-can library
✅ Hardware simulation with ros2_control
✅ Complete RViz visualization
✅ Comprehensive documentation
✅ Software testing without hardware (virtual CAN)
✅ Real-time motor control when hardware connected
✅ Extensible architecture for custom features

⚡ COMMANDS YOU'LL USE OFTEN
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# Launch robot with MoveIt
ros2 launch dexter_bringup dexter.launch.xml

# Launch motor controller
ros2 launch dexter_hardware motor_controller.launch.xml

# Send motor command
ros2 topic pub /motor_absolute_command \
  dexter_hardware_interfaces/msg/MotorAbsoluteCommand \
  '{motor_id: 1, absolute_position: 0, speed: 300, acceleration: 10}'

# Monitor motor commands
ros2 topic echo /motor_absolute_command

# Monitor CAN traffic
candump can0

# List all dexter packages
ros2 pkg list | grep dexter

# Show message definition
ros2 interface show dexter_hardware_interfaces/msg/MotorAbsoluteCommand

🎓 NEXT STEPS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Read README_FINAL.md for complete overview
2. Run verification script to confirm setup
3. Test with virtual CAN (software-only)
4. When hardware is ready, follow hardware setup guide
5. Experiment with motion planning in RViz
6. Extend with custom features as needed

🆘 SOMETHING NOT WORKING?
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1. Run verification: ~/dexter_test_2/ros2_ws/verify_dexter_setup.sh
2. Check logs: ros2 launch ... --log-level debug
3. Monitor CAN: candump can0 (or vcan0 for virtual)
4. Read troubleshooting in dexter_hardware/README.md
5. Check that ROS 2 and workspace are properly sourced

📞 HELPFUL RESOURCES
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

ROS 2 Documentation:    https://docs.ros.org/en/jazzy/
MoveIt 2 Documentation: https://moveit.ros.org/
CAN Utils:              https://github.com/linux-can/can-utils
python-can:             https://python-can.readthedocs.io/

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

                    🎉 SETUP COMPLETE AND READY TO USE! 🎉

                      Ready to get started? Let's go! 🚀

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

EOF
