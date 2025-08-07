cmake_minimum_required(VERSION 3.8)
project(f1tenth_slam_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files  
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables (note: use the module path)
install(PROGRAMS
  src/slam_nav_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

# Copy Python manager to package directory for proper module import
install(FILES
  src/slam_nav_manager.py
  DESTINATION lib/python3.8/site-packages/${PROJECT_NAME}/
  RENAME slam_nav_manager.py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()