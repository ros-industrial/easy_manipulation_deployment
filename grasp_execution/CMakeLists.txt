cmake_minimum_required(VERSION 3.5)
project(grasp_execution)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  PUBLIC
  include
)

add_library(grasp_execution_interface
  SHARED
  src/default_planning_action.cpp
  src/grasp_execution.cpp
  src/scheduler.cpp
)

ament_target_dependencies(grasp_execution_interface
  emd_msgs
  rclcpp
  tf2
  tf2_ros
)

add_library(moveit_cpp_grasp_execution_interface
  SHARED
  src/moveit_cpp_if.cpp
)

target_link_libraries(moveit_cpp_grasp_execution_interface
  grasp_execution_interface
)

ament_target_dependencies(moveit_cpp_grasp_execution_interface
  moveit_ros_planning_interface
)

# Default Executor plugin
add_library(moveit_cpp_default_executor_plugin
  SHARED
  src/moveit2/default_executor.cpp
)
ament_target_dependencies(moveit_cpp_default_executor_plugin
  moveit_ros_planning_interface
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  grasp_execution_interface
  moveit_cpp_grasp_execution_interface
  moveit_cpp_default_executor_plugin
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# TODO(Briancbn): convert to tests instead
add_subdirectory(
  example
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  grasp_execution_interface
  moveit_cpp_grasp_execution_interface
  moveit_cpp_default_executor_plugin
)
ament_export_dependencies(
  moveit_ros_planning_interface
  grasp_planning
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(grasp_execution
  moveit_cpp_default_executor_plugin.xml)

ament_package()
