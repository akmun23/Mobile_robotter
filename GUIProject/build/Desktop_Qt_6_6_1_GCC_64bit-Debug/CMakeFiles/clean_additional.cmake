# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/GUIProject_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/GUIProject_autogen.dir/ParseCache.txt"
  "GUIProject_autogen"
  )
endif()
