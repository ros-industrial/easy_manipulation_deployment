cmake_minimum_required(VERSION 3.10.2)
project(robotiq_3f_moveit_config)
find_package(ament_cmake REQUIRED)


install(DIRECTORY config DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY launch DESTINATION "share/${PROJECT_NAME}")
ament_package()

