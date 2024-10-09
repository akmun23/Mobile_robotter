cmake_minimum_required(VERSION 3.10)

# Project name
project(GetInfoFromMic)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
# Can maybe be deleted
include_directories(/home/pascal/Dokumenter/3.Semester/SemesterProjekt/Test/GetInfoFromMic/fftw-3.3.10/api)

# Add the executable
add_executable(${PROJECT_NAME} main.cpp
    audio.h audio.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME}
        #${CMAKE_SOURCE_DIR}/home/pascal/Dokumenter/3.Semester/SemesterProjekt/Test/GetInfoFromMic/portaudio/lib/libportaudio.la
        fftw3
        rt
        asound
        jack
        pthread
        portaudio
)

# Custom target for installing dependencies
add_custom_target(install-deps
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/install_portaudio.cmake
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/install_fftw.cmake
)

# Custom target for uninstalling dependencies
add_custom_target(uninstall-deps
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/uninstall_portaudio.cmake
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_SOURCE_DIR}/cmake/uninstall_fftw.cmake
)


