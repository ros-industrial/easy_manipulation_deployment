cmake_minimum_required(VERSION 3.5)
project(realsense2_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
# Install files
install(DIRECTORY launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME})



ament_package()
