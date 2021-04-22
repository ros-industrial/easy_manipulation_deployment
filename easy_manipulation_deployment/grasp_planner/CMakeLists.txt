cmake_minimum_required(VERSION 3.5)
project(grasp_planner)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -mavx)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL 1.11 REQUIRED COMPONENTS common io visualization filters sample_consensus segmentation features)

find_package(fcl QUIET)
if(fcl_FOUND)
  set(FCL_LIBRARIES fcl)
  set(using_new_fcl true)
  message(STATUS "Using FCL version: ${FCL_VERSION}")
else()
  include(FindPkgConfig)
  pkg_check_modules(PC_FCL REQUIRED fcl)
  pkg_check_modules(PC_CCD REQUIRED ccd)
  set(FCL_LIBRARIES ${PC_FCL_LIBRARIES} ${PC_CCD_LIBRARIES})
  set(using_new_fcl)
  message(STATUS "Using FCL version: ${PC_FCL_VERSION}")
endif()
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(octomap REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(emd_msgs REQUIRED)
find_package(epd_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(cv_bridge REQUIRED) # Delete later

#add_library(PclFunctions
#  include/pcl_functions.h
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspPose.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs shape_msgs sensor_msgs
)

add_executable(pcl_test_node src/pcl_test.cpp)
include_directories(include msg ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${LIBFCL_INCLUDE_DIRS})
add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} ${FCL_LIBRARY_DIRS})
target_link_libraries(pcl_test_node ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${FCL_LIBRARIES} ccd)
rosidl_target_interfaces(pcl_test_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(
        pcl_test_node
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
)

install(TARGETS
  pcl_test_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  data
  include
  msg
  launch
  config
  test
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(grasptest test/init_tests.cpp)
  ament_target_dependencies(grasptest ament_index_cpp)
  target_link_libraries(grasptest ${Boost_LIBRARIES})
  rosidl_target_interfaces(grasptest ${PROJECT_NAME} "rosidl_typesupport_cpp")

  add_custom_target(gcov
    COMMAND mkdir -p coverage
    COMMAND ${CMAKE_MAKE_PROGRAM} test
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  add_custom_command(TARGET gcov
    COMMAND echo "=================== GCOV ===================="
    COMMAND gcov -b ${CMAKE_SOURCE_DIR}/*.cpp -o ${OBJECT_DIR}
        | grep -A 5 "src/grasp_planning.cpp" > CoverageSummary.tmp
    COMMAND cat CoverageSummary.tmp
    COMMAND echo "-- Coverage files have been output to ${CMAKE_BINARY_DIR}/coverage"
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/coverage)

  target_link_libraries(grasptest ${GTEST_LIBRARIES} ${Boost_LIBRARIES})
  ament_export_dependencies(rosidl_default_runtime)
endif()


ament_package()
