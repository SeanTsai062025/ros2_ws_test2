cmake_minimum_required(VERSION 3.8)
project(servo_hw_pwm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---------- main node ----------
add_executable(servo_hw_pwm_node src/servo_hw_pwm_node.cpp)
ament_target_dependencies(servo_hw_pwm_node rclcpp std_msgs)

# ---------- clock-fix helper (no ROS dependency) ----------
add_executable(fix_pwm0_clock src/fix_pwm0_clock.c)

install(TARGETS servo_hw_pwm_node fix_pwm0_clock
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_angle_conversion test/test_angle_conversion.cpp)
  ament_target_dependencies(test_angle_conversion rclcpp std_msgs)
endif()

ament_package()
