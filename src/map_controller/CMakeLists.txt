cmake_minimum_required(VERSION 3.12)
project(map_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_library(map_controller_core
  src/map_controller.cpp
  src/steering_lookup.cpp
)

ament_target_dependencies(map_controller_core
  rclcpp
  ackermann_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

add_executable(map_controller_node
  src/controller_manager_node.cpp
)

ament_target_dependencies(map_controller_node
  rclcpp
  ackermann_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(map_controller_node
  map_controller_core
)

install(TARGETS
  map_controller_core
  map_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_to_amcl_bridge.py
  scripts/record_waypoints.py
  scripts/ackermann_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
