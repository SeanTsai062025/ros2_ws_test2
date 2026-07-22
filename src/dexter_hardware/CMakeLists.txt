cmake_minimum_required(VERSION 3.8)
project(dexter_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(dexter_hardware_interfaces REQUIRED)

ament_python_install_package(dexter_hardware)

add_library(dexter_can SHARED
  src/mks_protocol.cpp
  src/mks_can_client.cpp
  src/socketcan_transport.cpp
)
target_compile_features(dexter_can PUBLIC cxx_std_17)
target_include_directories(dexter_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(dexter_system SHARED
  src/dexter_system.cpp
)
target_compile_features(dexter_system PUBLIC cxx_std_17)
target_include_directories(dexter_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(dexter_system dexter_can)
ament_target_dependencies(dexter_system
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  hardware_interface dexter_hardware_plugins.xml)

install(PROGRAMS
  scripts/motor_controller_node.py
  scripts/dexter_hardware_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(
  FILES README.md REAL_HARDWARE_ROS2_CONTROL.md
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS dexter_can dexter_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_mks_protocol test/test_mks_protocol.cpp)
  target_link_libraries(test_mks_protocol dexter_can)
  target_include_directories(test_mks_protocol PRIVATE include)

  ament_add_gtest(test_mks_can_client test/test_mks_can_client.cpp)
  target_link_libraries(test_mks_can_client dexter_can)
  target_include_directories(test_mks_can_client PRIVATE include)

  ament_add_gtest(test_control_cycle_guard test/test_control_cycle_guard.cpp)
  target_include_directories(test_control_cycle_guard PRIVATE include)

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  ament_target_dependencies(test_plugin_loading hardware_interface pluginlib)

  # ROS Jazzy's launch_testing/pytest modules are installed for the system
  # interpreter. The development shell may have an unrelated Conda Python first.
  find_program(DEXTER_ROS_PYTHON python3 PATHS /usr/bin NO_DEFAULT_PATH REQUIRED)
  add_launch_test(
    test/test_jtc_physical_feedback.py
    TIMEOUT 30
    PYTHON_EXECUTABLE ${DEXTER_ROS_PYTHON}
    ENV ROS_HOME=${CMAKE_CURRENT_BINARY_DIR}/test_ros_home ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/test_ros_logs
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(dexter_can dexter_system)
ament_export_dependencies(hardware_interface pluginlib rclcpp diagnostic_msgs)

ament_package()
