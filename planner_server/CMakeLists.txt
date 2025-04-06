cmake_minimum_required(VERSION 3.8)
project(planner_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(infra_common REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(infra_interfaces REQUIRED)
find_package(plugin_base_classes REQUIRED)

include_directories(include)

# Create the executable for the actual ROS 2 node
add_executable(planner_server 
  src/planner_server.cpp
)

ament_target_dependencies(planner_server 
  rclcpp 
  pluginlib
  nav_msgs
  infra_common
  rclcpp_action
  infra_interfaces
  plugin_base_classes
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install both our interface library and executable.
# The export tag ensures our interface can be found by dependent packages.
install(TARGETS planner_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()