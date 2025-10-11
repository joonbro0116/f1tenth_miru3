cmake_minimum_required(VERSION 3.8)
project(key_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop rclcpp ackermann_msgs)

install(TARGETS keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})

ament_package()
