cmake_minimum_required(VERSION 3.8)
project(example_path_planner_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(planner_server REQUIRED)
find_package(pluginlib REQUIRED)
find_package(infra_common REQUIRED)
# infra_interfaces isn't used in this plugin package, but it's necessary
# to make sure the PathPlanner base class builds correctly
find_package(infra_interfaces REQUIRED) 

pluginlib_export_plugin_description_file(planner_server plugins.xml)

add_library(${PROJECT_NAME} src/example_path_planner_plugin.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  pluginlib
  infra_common
  nav_msgs
  infra_interfaces
)

target_link_libraries(${PROJECT_NAME} planner_server::path_planner)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "EXAMPLE_PATH_PLANNER_PLUGIN_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
