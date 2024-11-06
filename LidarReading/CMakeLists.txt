cmake_minimum_required(VERSION 3.16)

project(MobileRobotterMapping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)

qt_standard_project_setup()

qt_add_executable(MobileRobotterMapping
    main.cpp
    wallClass.h
    helperClass.h helperClass.cpp
    windowClass.h windowClass.cpp
    widgetClass.h widgetClass.cpp
    functions.h
    functions.cpp
)

set_target_properties(MobileRobotterMapping PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

target_link_libraries(MobileRobotterMapping PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)

include(GNUInstallDirs)
install(TARGETS MobileRobotterMapping
    BUNDLE  DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
