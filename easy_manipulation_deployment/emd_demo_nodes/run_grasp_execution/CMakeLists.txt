cmake_minimum_required(VERSION 3.5)
project(run_grasp_execution)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(emd_grasp_execution REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(demo_node src/demo_node.cpp)

ament_target_dependencies(demo_node
  emd_grasp_execution
  emd_msgs
  rclcpp
)

add_executable(fake_grasp_pose_publisher src/fake_grasp_pose_publisher.cpp)
ament_target_dependencies(fake_grasp_pose_publisher
  emd_grasp_execution
  emd_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  demo_node
  fake_grasp_pose_publisher
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


