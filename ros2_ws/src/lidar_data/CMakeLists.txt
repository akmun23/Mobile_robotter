cmake_minimum_required(VERSION 3.5)
project(lidar_data LANGUAGES CXX)

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
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Sql)

include_directories(include)

# Add executable
add_executable(lidar_data src/main.cpp
                          src/lidarnode.cpp include/lidarnode.h)

# Specify the dependencies for the target
ament_target_dependencies(lidar_data
  rclcpp
  geometry_msgs
  sensor_msgs
  rclcpp_components
  nav_msgs
)

# Link Qt6 libraries
target_link_libraries(lidar_data Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Sql)

# Install the executable to the appropriate directory
install(TARGETS
  lidar_data
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

# Call ament package function to finalize the setup
ament_package()

