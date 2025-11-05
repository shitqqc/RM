#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rm_tools::rm_tools" for configuration ""
set_property(TARGET rm_tools::rm_tools APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rm_tools::rm_tools PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librm_tools.so"
  IMPORTED_SONAME_NOCONFIG "librm_tools.so"
  )

list(APPEND _cmake_import_check_targets rm_tools::rm_tools )
list(APPEND _cmake_import_check_files_for_rm_tools::rm_tools "${_IMPORT_PREFIX}/lib/librm_tools.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
