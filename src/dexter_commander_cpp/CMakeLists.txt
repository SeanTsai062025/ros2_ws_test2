cmake_minimum_required(VERSION 3.8)
project(dexter_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dexter_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

add_executable(commander src/commander.cpp)
ament_target_dependencies(
  commander
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  std_msgs
  dexter_interfaces
)

add_library(robot_pose_panel SHARED
  src/robot_pose_panel.cpp
  include/dexter_commander_cpp/robot_pose_panel.hpp
)
target_include_directories(robot_pose_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_pose_panel Qt5::Widgets)
ament_target_dependencies(robot_pose_panel
  pluginlib
  rclcpp
  rviz_common
  sensor_msgs
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(rviz_common rviz_plugins.xml)

install(TARGETS commander robot_pose_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
