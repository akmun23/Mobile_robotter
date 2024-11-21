cmake_minimum_required(VERSION 3.16)

project(GUIProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(X11 REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Sql)
qt_standard_project_setup()

add_executable(GUIProject main.cpp
    GUIWindow.h GUIWindow.cpp
    dataTypes.h dataTypes.cpp
)

target_link_libraries(GUIProject X11::X11
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::Sql
)

include(GNUInstallDirs)
install(TARGETS GUIProject
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
