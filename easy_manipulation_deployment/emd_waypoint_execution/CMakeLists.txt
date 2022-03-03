cmake_minimum_required(VERSION 3.5)
project(emd_waypoint_execution)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jsoncpp CONFIG REQUIRED)
find_package(emd_grasp_execution REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  PUBLIC
  include
)

add_library(waypoint_execution_interface
  SHARED
  src/waypoint_parser.cpp
)

ament_target_dependencies(waypoint_execution_interface
  emd_grasp_execution
  rclcpp
  geometry_msgs)

target_link_libraries(waypoint_execution_interface
  jsoncpp_lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS waypoint_execution_interface
  EXPORT export_waypoint_execution_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

ament_export_include_directories(
  include
)

ament_export_libraries(waypoint_execution_interface)
ament_export_targets(export_waypoint_execution_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(jsoncpp)
ament_package()
