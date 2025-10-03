cmake_minimum_required(VERSION 3.8)
project(map_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(map_controller_core
  src/map_controller.cpp
  src/steering_lookup.cpp
)

target_include_directories(map_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(map_controller_core PUBLIC cxx_std_17)

ament_target_dependencies(map_controller_core
  ament_index_cpp
)

set_target_properties(map_controller_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(map_controller_component SHARED
  src/controller_manager_node.cpp
)

target_include_directories(map_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(map_controller_component
  map_controller_core
)

ament_target_dependencies(map_controller_component
  rclcpp
  rclcpp_components
  ackermann_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rclcpp_components_register_nodes(map_controller_component "map_controller::ControllerManagerNode")

add_executable(map_controller_main
  src/map_controller_main.cpp
)

target_link_libraries(map_controller_main
  map_controller_component
)

ament_target_dependencies(map_controller_main
  rclcpp
)

install(TARGETS
  map_controller_core
  map_controller_component
  map_controller_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  config
  launch
  resources
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
