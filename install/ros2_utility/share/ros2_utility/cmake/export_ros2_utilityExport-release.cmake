#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ros2_utility::ros2_utility" for configuration "Release"
set_property(TARGET ros2_utility::ros2_utility APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ros2_utility::ros2_utility PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libros2_utility.so"
  IMPORTED_SONAME_RELEASE "libros2_utility.so"
  )

list(APPEND _cmake_import_check_targets ros2_utility::ros2_utility )
list(APPEND _cmake_import_check_files_for_ros2_utility::ros2_utility "${_IMPORT_PREFIX}/lib/libros2_utility.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
