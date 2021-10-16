set(TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions( -DTEST_DIRECTORY=\"${CMAKE_CURRENT_SOURCE_DIR}\")

add_library(test_hardware
  test_hardware.cpp
)

add_definitions(${BULLET_DEFINITIONS})
target_link_libraries(test_hardware
  ${PROJECT_NAME}
  stdc++fs
)

ament_add_gtest(test_collision_checking
  test_collision_checking.cpp
)
target_link_libraries(test_collision_checking
  test_hardware
  ${PROJECT_NAME}
  ${PLANNING_FRAMEWORK_LIBRARY}
)
