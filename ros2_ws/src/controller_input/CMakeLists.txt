cmake_minimum_required(VERSION 3.5)
project(controller_input LANGUAGES CXX)

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
find_package(teleop_twist_joy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(SFML 2.5 COMPONENTS system window graphics network audio REQUIRED)

include_directories(include)

# Add executable
add_executable(controller_input src/main.cpp)

target_link_libraries(controller_input sfml-system sfml-window sfml-graphics sfml-network sfml-audio)

# Specify the dependencies for the target
ament_target_dependencies(controller_input
  rclcpp
  geometry_msgs
  sensor_msgs
  teleop_twist_joy
  rclcpp_components
)

# Install the executable
install(TARGETS
  controller_input
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_package()

