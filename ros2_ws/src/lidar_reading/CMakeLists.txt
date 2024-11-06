cmake_minimum_required(VERSION 3.16)
project(lidar_reading LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find nødvendige ROS 2- og Qt6-pakker
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)

# Definer Qt GUI-eksekverbar med alle nødvendige filer
qt_add_executable(lidar_reading
    src/main.cpp
    src/wallClass.h
    src/helperClass.h src/helperClass.cpp
    src/windowClass.h src/windowClass.cpp
    src/widgetClass.h src/widgetClass.cpp
    src/functions.h src/functions.cpp
    src/lidarData.h src/lidarData.cpp
)

# Sæt egenskaber for eksekverbar
set_target_properties(lidar_reading PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

# Link Qt- og ROS 2-biblioteker til eksekverbar med nøgleord
target_link_libraries(lidar_reading 
    PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

# Tilføj ROS 2-afhængigheder med nøgleord
ament_target_dependencies(lidar_reading
    PUBLIC
    rclcpp
    geometry_msgs
    sensor_msgs
    rclcpp_components
    nav_msgs
)

# Installeringsdirektiver
include(GNUInstallDirs)

install(TARGETS lidar_reading
    BUNDLE  DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Installer launch-mappe hvis eksisterende
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

