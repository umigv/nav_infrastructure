cmake_minimum_required(VERSION 3.20)
project(infra_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToGoal.action"
  "msg/CellCoordinateMsg.msg"
  DEPENDENCIES 
  nav_msgs 
  geometry_msgs
)

# Critical exports for downstream packages
ament_export_dependencies(
  nav_msgs
  geometry_msgs
  rosidl_default_runtime  # Required for generated headers
)
ament_export_include_directories(include)

# Required to expose the interface targets
ament_package()