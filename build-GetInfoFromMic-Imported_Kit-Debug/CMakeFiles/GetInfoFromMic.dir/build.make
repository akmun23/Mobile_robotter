# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug

# Include any dependencies generated for this target.
include CMakeFiles/GetInfoFromMic.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/GetInfoFromMic.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/GetInfoFromMic.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/GetInfoFromMic.dir/flags.make

CMakeFiles/GetInfoFromMic.dir/main.cpp.o: CMakeFiles/GetInfoFromMic.dir/flags.make
CMakeFiles/GetInfoFromMic.dir/main.cpp.o: /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/main.cpp
CMakeFiles/GetInfoFromMic.dir/main.cpp.o: CMakeFiles/GetInfoFromMic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/GetInfoFromMic.dir/main.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/GetInfoFromMic.dir/main.cpp.o -MF CMakeFiles/GetInfoFromMic.dir/main.cpp.o.d -o CMakeFiles/GetInfoFromMic.dir/main.cpp.o -c /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/main.cpp

CMakeFiles/GetInfoFromMic.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GetInfoFromMic.dir/main.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/main.cpp > CMakeFiles/GetInfoFromMic.dir/main.cpp.i

CMakeFiles/GetInfoFromMic.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GetInfoFromMic.dir/main.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/main.cpp -o CMakeFiles/GetInfoFromMic.dir/main.cpp.s

CMakeFiles/GetInfoFromMic.dir/audio.cpp.o: CMakeFiles/GetInfoFromMic.dir/flags.make
CMakeFiles/GetInfoFromMic.dir/audio.cpp.o: /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/audio.cpp
CMakeFiles/GetInfoFromMic.dir/audio.cpp.o: CMakeFiles/GetInfoFromMic.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/GetInfoFromMic.dir/audio.cpp.o"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/GetInfoFromMic.dir/audio.cpp.o -MF CMakeFiles/GetInfoFromMic.dir/audio.cpp.o.d -o CMakeFiles/GetInfoFromMic.dir/audio.cpp.o -c /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/audio.cpp

CMakeFiles/GetInfoFromMic.dir/audio.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/GetInfoFromMic.dir/audio.cpp.i"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/audio.cpp > CMakeFiles/GetInfoFromMic.dir/audio.cpp.i

CMakeFiles/GetInfoFromMic.dir/audio.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/GetInfoFromMic.dir/audio.cpp.s"
	/usr/bin/g++-11 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic/audio.cpp -o CMakeFiles/GetInfoFromMic.dir/audio.cpp.s

# Object files for target GetInfoFromMic
GetInfoFromMic_OBJECTS = \
"CMakeFiles/GetInfoFromMic.dir/main.cpp.o" \
"CMakeFiles/GetInfoFromMic.dir/audio.cpp.o"

# External object files for target GetInfoFromMic
GetInfoFromMic_EXTERNAL_OBJECTS =

GetInfoFromMic: CMakeFiles/GetInfoFromMic.dir/main.cpp.o
GetInfoFromMic: CMakeFiles/GetInfoFromMic.dir/audio.cpp.o
GetInfoFromMic: CMakeFiles/GetInfoFromMic.dir/build.make
GetInfoFromMic: CMakeFiles/GetInfoFromMic.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable GetInfoFromMic"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/GetInfoFromMic.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/GetInfoFromMic.dir/build: GetInfoFromMic
.PHONY : CMakeFiles/GetInfoFromMic.dir/build

CMakeFiles/GetInfoFromMic.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/GetInfoFromMic.dir/cmake_clean.cmake
.PHONY : CMakeFiles/GetInfoFromMic.dir/clean

CMakeFiles/GetInfoFromMic.dir/depend:
	cd /home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic /home/aksel/Documents/GitHub/Mobile_robotter/GetInfoFromMic /home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug /home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug /home/aksel/Documents/GitHub/Mobile_robotter/build-GetInfoFromMic-Imported_Kit-Debug/CMakeFiles/GetInfoFromMic.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/GetInfoFromMic.dir/depend

