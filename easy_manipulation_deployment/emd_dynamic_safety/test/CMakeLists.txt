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
)

# Flags are set internally
ament_add_gtest(test_replanner_moveit
  test_replanner_moveit.cpp
)
target_link_libraries(test_replanner_moveit
  test_hardware
  ${PROJECT_NAME}
)

# Flags are set internally
ament_add_gtest(test_replanner_tesseract
  test_replanner_tesseract.cpp
)
target_link_libraries(test_replanner_tesseract
  test_hardware
  ${PROJECT_NAME}
)
