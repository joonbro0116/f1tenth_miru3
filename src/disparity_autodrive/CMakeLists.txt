cmake_minimum_required(VERSION 3.8)
project(disparity_autodrive)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)        # ✅ 추가

add_executable(disparity_autodrive_node src/disparity_autodrive_node.cpp)

ament_target_dependencies(disparity_autodrive_node
  rclcpp
  sensor_msgs
  ackermann_msgs
  nav_msgs
  visualization_msgs
  geometry_msgs                                  # ✅ 추가
)

install(TARGETS disparity_autodrive_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
