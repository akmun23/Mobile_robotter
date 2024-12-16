cmake_minimum_required(VERSION 3.16)

project(drawMap LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Sql)
find_package(X11 REQUIRED)

# Enable automatic MOC processing for Qt
set(CMAKE_AUTOMOC ON)

# Source files
set(SOURCES
    main.cpp
    GUIWindow.cpp
    dataTypes.cpp
)

# Header files
set(HEADERS
    GUIWindow.h
    dataTypes.h
)

# Add executable
add_executable(drawMap ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(drawMap PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Link libraries
target_link_libraries(drawMap PRIVATE Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Sql X11::X11)

# Install targets
include(GNUInstallDirs)
install(TARGETS drawMap
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(FILES ${HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Enable warnings for Debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-Wall -Wextra -pedantic -g)
endif()
