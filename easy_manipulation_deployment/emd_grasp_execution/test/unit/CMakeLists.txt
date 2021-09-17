# Scheduler Unit Test
ament_add_gtest(test_scheduler
  test_scheduler.cpp
)
target_link_libraries(test_scheduler
  grasp_execution_interface
)
