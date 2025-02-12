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

add_library(controller INTERFACE)

target_include_directories(controller INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${infra_common_INCLUDE_DIRS}
)

add_executable(controller_server 
  src/controller_server.cpp
)

target_link_libraries(controller_server
  controller
)

ament_target_dependencies(controller_server
  rclcpp
  infra_interfaces
  pluginlib
  rclcpp_action
  geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  controller
  controller_server
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_include_directories(
  include
)

ament_package()