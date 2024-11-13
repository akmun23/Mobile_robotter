cmake_minimum_required(VERSION 3.5)
project(CMakeSFMLProject LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

project(oioioi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.5 COMPONENTS system window graphics network audio REQUIRED)

add_executable(oioioi main.cpp
        sound.h
)

target_link_libraries(oioioi sfml-system sfml-window sfml-graphics sfml-network sfml-audio)
