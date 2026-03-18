#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robot_math::robot_math" for configuration "Release"
set_property(TARGET robot_math::robot_math APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robot_math::robot_math PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobot_math.so"
  IMPORTED_SONAME_RELEASE "librobot_math.so"
  )

list(APPEND _cmake_import_check_targets robot_math::robot_math )
list(APPEND _cmake_import_check_files_for_robot_math::robot_math "${_IMPORT_PREFIX}/lib/librobot_math.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
