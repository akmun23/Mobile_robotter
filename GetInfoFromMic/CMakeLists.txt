cmake_minimum_required(VERSION 3.10)

# Project name
project(GetInfoFromMic)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories

# Add the executable
add_executable(${PROJECT_NAME} main.cpp
    audio.h audio.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
        rt
        asound
        jack
        pthread
        portaudio
)

# Custom target for installing dependencies
add_custom_target(install-deps
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/install_portaudio.cmake

)

# Custom target for uninstalling dependencies
add_custom_target(uninstall-deps
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/uninstall_portaudio.cmake
)


