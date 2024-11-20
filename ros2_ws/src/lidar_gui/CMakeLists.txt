cmake_minimum_required(VERSION 3.5)
project(lidar_gui LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Update the module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(X11 REQUIRED)

include_directories(include)

# Add executable
add_executable(lidar_gui 
  src/main.cpp
  src/lidarnode.cpp include/lidarnode.h
  src/dataTypes.cpp include/dataTypes.h
  src/GUIWindow.cpp include/GUIWindow.h
)

# Specify the dependencies for the target
ament_target_dependencies(lidar_gui
  rclcpp
  geometry_msgs
  sensor_msgs
  rclcpp_components
  nav_msgs
)

target_link_libraries(lidar_gui X11::X11)

# Install the executable to the appropriate directory
install(TARGETS
  lidar_gui
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

# Call ament package function to finalize the setup
ament_package()

