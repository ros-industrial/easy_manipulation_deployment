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
find_package(grasp_planning REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_library(grasp_execution src/grasp_execution.cpp)
ament_target_dependencies(grasp_execution
  moveit_ros_planning_interface
  grasp_planning
  rclcpp
  tf2
  tf2_ros
)

add_executable(grasp_execution_node src/grasp_execution_node.cpp)
target_link_libraries(grasp_execution_node
  grasp_execution
)
ament_target_dependencies(grasp_execution_node
  rclcpp
)

add_executable(fake_grasp_pose_publisher src/fake_grasp_pose_publisher.cpp)
ament_target_dependencies(fake_grasp_pose_publisher
  grasp_planning
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  grasp_execution_node
  fake_grasp_pose_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
