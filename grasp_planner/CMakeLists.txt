cmake_minimum_required(VERSION 3.5)
project(grasp_planning)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PKG_CONFIG_EXECUTABLE "/usr/bin/pkg-config")
find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)
#find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(epd_msgs REQUIRED)
set(BOOST_ROOT "/usr/include/boost")
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

#find_package(Boost)
#IF (Boost_FOUND)
#    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
#    ADD_DEFINITIONS( "-DHAS_BOOST" )
#ENDIF()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraspPose.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs shape_msgs sensor_msgs epd_msgs
)


add_executable(grasp_planning_node src/grasp_planning.cpp)
target_link_libraries(grasp_planning_node ${Boost_LIBRARIES} yaml-cpp)
rosidl_target_interfaces(grasp_planning_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(perception_saver src/perception_saver.cpp)
target_link_libraries(perception_saver ${Boost_LIBRARIES} yaml-cpp)
rosidl_target_interfaces(perception_saver ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include test msg yaml-cpp config ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


ament_target_dependencies(
        grasp_planning_node
        rclcpp
        kdl_parser
        sensor_msgs
        std_msgs
        shape_msgs
        geometry_msgs
        cv_bridge
        tf2
        tf2_ros
        ament_index_cpp
        urdf
        epd_msgs
)

ament_target_dependencies(
        perception_saver
        rclcpp
        kdl_parser
        sensor_msgs
        std_msgs
        shape_msgs
        geometry_msgs
        cv_bridge
        tf2
        tf2_ros
        ament_index_cpp
        urdf
        epd_msgs
)


install(TARGETS

  grasp_planning_node
  perception_saver

  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

#install(TARGETS perception_saver
#  EXPORT export_${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME}
#)

install(DIRECTORY

  launch
  config
  include
  msg
  test
  DESTINATION share/${PROJECT_NAME}
)
#install(DIRECTORY config
#  DESTINATION share/${PROJECT_NAME}
#)
#install(DIRECTORY include
#  DESTINATION share/${PROJECT_NAME}
#)

#install(DIRECTORY msg
#  DESTINATION share/${PROJECT_NAME}
#)

#install(DIRECTORY test
#  DESTINATION share/${PROJECT_NAME}
#)







#add_executable(runTests test/init_tests.cpp)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(grasptest test/init_tests.cpp)
  ament_target_dependencies(grasptest cv_bridge ament_index_cpp)
  target_link_libraries(grasptest ${Boost_LIBRARIES} yaml-cpp)
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
