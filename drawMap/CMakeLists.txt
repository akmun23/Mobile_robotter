cmake_minimum_required(VERSION 3.16)

project(drawMap LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Sql)
find_package(X11 REQUIRED)

add_executable(drawMap main.cpp
    GUIWindow.h GUIWindow.cpp
    dataTypes.h dataTypes.cpp
)

target_link_libraries(drawMap Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Sql X11::X11)

include(GNUInstallDirs)
install(TARGETS drawMap
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
