cmake_minimum_required(VERSION 3.5)
project(lidar_processing)

# C++17 사용
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 컴파일 경고 옵션
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 헤더 검색 경로
include_directories(
  include
)

# 실행 파일 선언
add_executable(lidar_node
  src/lidar_node.cpp
)

add_executable(wall_follow_node
  src/wall_follow_node.cpp
)

add_executable(gap_follow_node
  src/gap_follow_node.cpp
)

# 의존성 링크
ament_target_dependencies(lidar_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(wall_follow_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
)

ament_target_dependencies(gap_follow_node
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  geometry_msgs
)

# 설치 설정
install(TARGETS lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS wall_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gap_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
