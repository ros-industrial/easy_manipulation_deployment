cmake_minimum_required(VERSION 3.5)

project(workcell_builder LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(ament_cmake REQUIRED)
set(BOOST_ROOT "/usr/include/boost")
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
include_directories(include)
include_directories(include/attributes)
include_directories(include/yaml_parser)

set(PKG_CONFIG_EXECUTABLE "/usr/bin/pkg-config")
find_package(PkgConfig REQUIRED)
pkg_check_modules(yaml-cpp REQUIRED yaml-cpp)

include_directories(gui)


add_executable(workcell_builder
    gui/main.cpp
    gui/mainwindow.cpp
    gui/mainwindow.h
    gui/mainwindow.ui

    gui/replacewarning.cpp
    gui/replacewarning.h
    gui/replacewarning.ui

    gui/scene_select.cpp
    gui/scene_select.h
    gui/scene_select.ui

    gui/addcollision.cpp
    gui/addcollision.h
    gui/addcollision.ui

    gui/addexternaljoint.cpp
    gui/addexternaljoint.h
    gui/addexternaljoint.ui

    gui/addinertial.cpp
    gui/addinertial.h
    gui/addinertial.ui

    gui/addjoint.cpp
    gui/addjoint.h
    gui/addjoint.ui

    gui/addlink.cpp
    gui/addlink.h
    gui/addlink.ui

    gui/addobject.cpp
    gui/addobject.h
    gui/addobject.ui

    gui/addvisual.cpp
    gui/addvisual.h
    gui/addvisual.ui

    gui/addscene.cpp
    gui/addscene.h
    gui/addscene.ui

    gui/addrobot.cpp
    gui/addrobot.h
    gui/addrobot.ui

    gui/addendeffector.cpp
    gui/addendeffector.h
    gui/addendeffector.ui

    gui/replacewarning.cpp
    gui/replacewarning.h
    gui/replacewarning.ui

    gui/loadobjects.cpp
    gui/loadobjects.h
    gui/loadobjects.ui)

target_link_libraries(workcell_builder PRIVATE Qt5::Widgets ${Boost_LIBRARIES} yaml-cpp)
target_include_directories(workcell_builder PUBLIC ${Boost_INCLUDE_DIRS})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS workcell_builder
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

ament_package()
