cmake_minimum_required(VERSION 3.5)
project(emd_dynamic_safety)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(joint_trajectory_controller REQUIRED)

find_package(moveit_core QUIET)

find_package(tesseract_environment QUIET)

set(PLANNING_FRAMEWORK_LIBRARY)

include_directories(
  PUBLIC
  include
)
if(tesseract_environment_FOUND)
  add_definitions(-DEMD_DYNAMIC_SAFETY_TESSERACT)
endif()
if(moveit_core_FOUND)
  add_definitions(-DEMD_DYNAMIC_SAFETY_MOVEIT)
endif()

# MoveIt
if(moveit_core_FOUND)
  message(STATUS MoveIt Found!!)

  add_library(${PROJECT_NAME}_moveit
    SHARED
    src/moveit/collision_checker_moveit.cpp
    src/moveit/replanner_moveit.cpp
  )

  ament_target_dependencies(${PROJECT_NAME}_moveit
    rclcpp
    moveit_core
    pluginlib
  )

  ament_export_libraries(
    ${PROJECT_NAME}_moveit
  )

  list(APPEND PLANNING_FRAMEWORK_LIBRARY ${PROJECT_NAME}_moveit)

  install(
    TARGETS
    ${PROJECT_NAME}_moveit
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
else()
  message(WARNING MoveIt Not Found!!)
endif()

# Tesseract
if(tesseract_environment_FOUND)
  message(STATUS Tesseract Found!!)

  # Force -Wno-dev
  if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
    set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
  endif()

  find_package(tesseract_rosutils REQUIRED)
  find_package(tesseract_msgs REQUIRED)
  find_package(tesseract_motion_planners REQUIRED)
  find_package(tesseract_process_managers REQUIRED)

  add_library(${PROJECT_NAME}_tesseract
    SHARED
    src/tesseract/collision_checker_tesseract.cpp
    src/tesseract/replanner_tesseract.cpp
  )

  target_link_libraries(${PROJECT_NAME}_tesseract
    tesseract::tesseract_environment_core
    tesseract::tesseract_environment_kdl
    tesseract::tesseract_environment_ofkt
    tesseract::tesseract_collision_core
    tesseract::tesseract_collision_bullet
    tesseract::tesseract_collision_fcl
    tesseract::tesseract_kinematics_core
    tesseract::tesseract_kinematics_opw
    tesseract::tesseract_kinematics_kdl
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_motion_planners_ompl
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_trajopt_ifopt
    tesseract::tesseract_time_parameterization
    tesseract::tesseract_process_managers
    tesseract::tesseract_rosutils
  )

  ament_target_dependencies(${PROJECT_NAME}_tesseract
    rclcpp
    tesseract_rosutils
    tesseract_msgs
    moveit_msgs
  )

  ament_export_libraries(
    ${PROJECT_NAME}_tesseract
  )

  list(APPEND PLANNING_FRAMEWORK_LIBRARY
    ${PROJECT_NAME}_tesseract
    fcl ${CCD_LIBRARIES}
    ${BULLET_LIBRARIES}
  )

  install(
    TARGETS
    ${PROJECT_NAME}_tesseract
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib)
else()
  message(WARNING Tesseract Not Found!!)
endif()


add_library(${PROJECT_NAME}
  SHARED
  #src/next_point_publisher.cpp
  src/replanner.cpp
  src/safety_zone.cpp
  src/collision_checker.cpp
  src/interpolate.cpp
  src/visualizer.cpp
  src/dynamic_safety.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PLANNING_FRAMEWORK_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

add_library(${PROJECT_NAME}_controller
  SHARED
  src/dynamic_safety_trajectory_controller.cpp
)

# Remove pedantic
get_target_property(target_options ${PROJECT_NAME}_controller COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-Wpedantic")
list(REMOVE_ITEM target_options "-Wconversion")
list(REMOVE_ITEM target_options "-Wsign-conversion")
set_property(TARGET ${PROJECT_NAME}_controller PROPERTY COMPILE_OPTIONS ${target_options})
target_link_libraries(${PROJECT_NAME}_controller
  ${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}_controller
  rclcpp
  rclcpp_lifecycle
  joint_trajectory_controller
  realtime_tools
  pluginlib
)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_controller
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

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)

endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
  ${PROJECT_NAME}_controller
)

ament_package()
