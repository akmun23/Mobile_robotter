cmake_minimum_required(VERSION 3.16)

project(MobileRobotterMapping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)

qt_standard_project_setup()

qt_add_executable(MobileRobotterMapping
    main.cpp
    wallClass.h
    helperClass.h helperClass.cpp
    windowClass.h windowClass.cpp
    widgetClass.h widgetClass.cpp
    functions.h functions.cpp
    lidarData.h lidarData.lidarData.cpp
)

set_target_properties(MobileRobotterMapping PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

target_link_libraries(MobileRobotterMapping PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

# Specify the dependencies for the target
ament_target_dependencies(lidar_data
  rclcpp
  geometry_msgs
  sensor_msgs
  rclcpp_components
  nav_msgs
)

include(GNUInstallDirs)
install(TARGETS MobileRobotterMapping
    BUNDLE  DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

ament_package()
