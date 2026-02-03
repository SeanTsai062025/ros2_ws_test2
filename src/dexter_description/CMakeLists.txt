cmake_minimum_required(VERSION 3.8)
project(dexter_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf_dexter
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
