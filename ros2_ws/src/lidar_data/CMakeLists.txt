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
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

include_directories(
    /home/aksel/matplotlib-cpp
    ${Python3_INCLUDE_DIRS}
)

include_directories(include)

# Add executable
add_executable(lidar_data src/main.cpp)


target_link_libraries(${PROJECT_NAME}
    ${Python3_LIBRARIES}
)

# Specify the dependencies for the target
ament_target_dependencies(lidar_data
  rclcpp
  geometry_msgs
  sensor_msgs
  rclcpp_components
  nav_msgs
)

# Install the executable
install(TARGETS
  lidar_data
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

