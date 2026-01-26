----------------------------------------------
!!launch below two commands in two terminal!!
----------------------------------------------
ros2 launch my_robot_bringup my_robot.launch.xml

ros2 run my_robot_commander_cpp commander
----------------------------------------------------------
!!below three command are three type of control method !!
----------------------------------------------------------
ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "data: false"

ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]}"

ros2 topic pub -1 /pose_command my_robot_interfaces/msg/PoseCommand "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
