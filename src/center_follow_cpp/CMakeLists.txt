cmake_minimum_required(VERSION 3.8)
project(center_follow_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(center_follower src/center_follow_node.cpp)
ament_target_dependencies(center_follower
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  visualization_msgs
)

install(TARGETS
  center_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

