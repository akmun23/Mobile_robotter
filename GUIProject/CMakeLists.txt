cmake_minimum_required(VERSION 3.16)

project(GUIProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(X11 REQUIRED)

add_executable(GUIProject main.cpp
    GUIWindow.h GUIWindow.cpp
    dataTypes.h
)

target_link_libraries(GUIProject X11::X11)

include(GNUInstallDirs)
install(TARGETS GUIProject
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
