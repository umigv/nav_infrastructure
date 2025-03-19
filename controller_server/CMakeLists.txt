cmake_minimum_required(VERSION 3.8)
project(controller_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(infra_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plugin_base_classes REQUIRED)
find_package(infra_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(controller_server 
  src/controller_server.cpp
  src/pose_manager.cpp
)

ament_target_dependencies(controller_server
  rclcpp
  infra_interfaces
  pluginlib
  rclcpp_action
  geometry_msgs
  plugin_base_classes
  infra_common
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS controller_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()