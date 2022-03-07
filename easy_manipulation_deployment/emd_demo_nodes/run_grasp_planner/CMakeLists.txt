cmake_minimum_required(VERSION 3.5)
project(run_grasp_planner)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(emd_grasp_planner REQUIRED)
find_package(emd_msgs REQUIRED)

add_executable(demo_node src/demo_node.cpp)

target_link_libraries(demo_node
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${FCL_LIBRARIES}
  ccd
)

ament_target_dependencies(demo_node
  rclcpp
  emd_grasp_planner
  emd_msgs
)

install(TARGETS
  demo_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

