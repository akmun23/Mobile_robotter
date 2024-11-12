# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles/LidarReading_autogen.dir/AutogenUsed.txt"
  "CMakeFiles/LidarReading_autogen.dir/ParseCache.txt"
  "LidarReading_autogen"
  )
endif()
