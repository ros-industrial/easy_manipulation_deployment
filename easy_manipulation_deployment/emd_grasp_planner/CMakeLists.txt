cmake_minimum_required(VERSION 3.5)
project(emd_grasp_planner)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# add_definitions(-DEIGEN_MAX_ALIGN_BYTES=16)
# add_definitions(-DEIGEN_UNALIGNED_VECTORIZE=0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io visualization filters sample_consensus segmentation features)

find_package(fcl QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  set(using_new_fcl true)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
else()
  include(FindPkgConfig)
  pkg_check_modules(FCL REQUIRED fcl)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
endif()
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(epd_msgs QUIET)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED) # Delete later
find_package(visualization_msgs REQUIRED)

include_directories(
  PUBLIC
  include
  msg
  # ${PCL_INCLUDE_DIRS}
  # ${OCTOMAP_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_library(grasp_planning_interface
  SHARED
  src/grasp_scene.cpp
  src/grasp_object.cpp
  src/end_effectors/finger_gripper.cpp
  src/end_effectors/suction_gripper.cpp
  src/common/pcl_functions.cpp
  src/common/pcl_visualizer.cpp
  src/common/fcl_functions.cpp
  src/common/math_functions.cpp
)

if(${FCL_VERSION} VERSION_GREATER_EQUAL 0.6.0)
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    FCL_VERSION_0_6_OR_HIGHER=1
  )
else()
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    FCL_VERSION_0_6_OR_HIGHER=0
  )
endif()

get_target_property(target_options grasp_planning_interface COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-Wpedantic")
set_property(TARGET grasp_planning_interface PROPERTY COMPILE_OPTIONS ${target_options})

if(epd_msgs_FOUND)
  ament_target_dependencies(grasp_planning_interface
        tf2
        rclcpp
        emd_msgs
        epd_msgs
        sensor_msgs
        geometry_msgs
        shape_msgs
        tf2_ros
        message_filters
        cv_bridge #temp
        visualization_msgs)
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    EPD_ENABLED=1
  )
else()
  ament_target_dependencies(grasp_planning_interface
        tf2
        rclcpp
        emd_msgs
        sensor_msgs
        geometry_msgs
        shape_msgs
        tf2_ros
        message_filters
        cv_bridge #temp
        visualization_msgs)
  target_compile_definitions(grasp_planning_interface
    PUBLIC
    EPD_ENABLED=0
  )
endif()

target_link_libraries(grasp_planning_interface
  ${PCL_LIBRARIES}
  ${FCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# install(
#   TARGETS
#   grasp_planning_interface
#   RUNTIME DESTINATION bin
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib)

# ament_export_include_directories(
#   include
# )

# ament_export_libraries(
#   grasp_planning_interface
# )

# add_subdirectory(
#   example
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(grasptest test/init_tests.cpp)
  ament_target_dependencies(grasptest ament_index_cpp)

  if(${FCL_VERSION} VERSION_GREATER_EQUAL 0.6.0)
  target_compile_definitions(grasptest
    PUBLIC
    FCL_VERSION_0_6_OR_HIGHER=1
  )
  else()
    target_compile_definitions(grasptest
      PUBLIC
      FCL_VERSION_0_6_OR_HIGHER=0
    )
  endif()

  target_link_libraries(grasptest
  grasp_planning_interface
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${FCL_LIBRARIES}
  ${GTEST_LIBRARIES}
  ${Boost_LIBRARIES}
  ccd)

  ament_export_dependencies(
    rosidl_default_runtime)

endif()


install(
  TARGETS grasp_planning_interface
  EXPORT export_grasp_planning_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

ament_export_include_directories(
  include
)

ament_export_libraries(grasp_planning_interface)
ament_export_targets(export_grasp_planning_interface HAS_LIBRARY_TARGET)

if(epd_msgs_FOUND)
  ament_export_dependencies(
    emd_msgs
    epd_msgs
    shape_msgs
    visualization_msgs
    cv_bridge
    message_filters
    tf2_ros
    PCL
    octomap
  )
else()
  ament_export_dependencies(
    emd_msgs
    shape_msgs
    visualization_msgs
    cv_bridge
    message_filters
    tf2_ros
    PCL
    octomap
  )
endif()

ament_package()
