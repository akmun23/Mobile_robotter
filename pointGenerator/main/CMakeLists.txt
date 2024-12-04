cmake_minimum_required(VERSION 3.16)

project(main LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Sql)

add_executable(pointGenerator main.cpp)

target_link_libraries(pointGenerator PRIVATE Qt::Core Qt::Gui Qt::Widgets Qt::Sql)

include(GNUInstallDirs)
install(TARGETS pointGenerator
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
