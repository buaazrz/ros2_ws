#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "robot_hardware_interface::robot_hardware_interface" for configuration "Release"
set_property(TARGET robot_hardware_interface::robot_hardware_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(robot_hardware_interface::robot_hardware_interface PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librobot_hardware_interface.so"
  IMPORTED_SONAME_RELEASE "librobot_hardware_interface.so"
  )

list(APPEND _cmake_import_check_targets robot_hardware_interface::robot_hardware_interface )
list(APPEND _cmake_import_check_files_for_robot_hardware_interface::robot_hardware_interface "${_IMPORT_PREFIX}/lib/librobot_hardware_interface.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
