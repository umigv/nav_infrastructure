cmake_minimum_required(VERSION 3.8)
project(planner_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all required dependencies. The order here doesn't matter technically,
# but organizing related packages together makes the file more maintainable.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(infra_common REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(infra_interfaces REQUIRED)

# Create the interface library for our abstract path planner.
# We use INTERFACE because this is a header-only library with no source files.
add_library(path_planner INTERFACE)

# Set up include directories for the path_planner interface.
# BUILD_INTERFACE is used during compilation of our package
# INSTALL_INTERFACE is used by other packages that depend on us
# We also include infra_common's headers so we can use costmap.hpp
target_include_directories(path_planner INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${infra_common_INCLUDE_DIRS}
)

# Link our interface library with infra_common's libraries.
# This ensures that any symbols needed from infra_common are available.
target_link_libraries(path_planner INTERFACE
  infra_common::infra_common  # Use target instead of ${infra_common_LIBRARIES}
)

# Create the executable for the actual ROS 2 node
add_executable(planner_server 
  src/planner_server.cpp)

# Link the executable with our path_planner interface
target_link_libraries(planner_server
  path_planner
)

# Add ROS 2 dependencies for the executable.
# These are needed for the node implementation but not for the interface.
ament_target_dependencies(planner_server 
  rclcpp 
  pluginlib
  nav_msgs
  infra_common
  rclcpp_action
  infra_interfaces
)

# Install our header files so other packages can use them
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Only target should be path_planner, not planner_server
install(
  TARGETS path_planner
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

# Export our targets so other packages can find and use them
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(infra_common)

# Export our include directories
ament_export_include_directories(
  include
)

ament_package()