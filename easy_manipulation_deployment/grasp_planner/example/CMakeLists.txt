add_executable(demo_node src/demo_node.cpp)

target_link_libraries(demo_node
  grasp_planning_interface
)

ament_target_dependencies(demo_node
  rclcpp
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
