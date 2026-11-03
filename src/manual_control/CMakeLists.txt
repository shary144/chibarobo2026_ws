cmake_minimum_required(VERSION 3.8)
project(manual_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robomas_interfaces REQUIRED)

add_executable(manual_control_node src/manual_control.cpp)
ament_target_dependencies(manual_control_node rclcpp sensor_msgs robomas_interfaces)

install(TARGETS
  manual_control_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
