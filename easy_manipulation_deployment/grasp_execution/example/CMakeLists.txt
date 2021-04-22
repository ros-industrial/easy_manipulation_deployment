find_package(tf2_geometry_msgs REQUIRED)

add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node
  moveit_cpp_grasp_execution_interface
)
ament_target_dependencies(demo_node
  rclcpp
)

add_executable(fake_grasp_pose_publisher src/fake_grasp_pose_publisher.cpp)
ament_target_dependencies(fake_grasp_pose_publisher
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
