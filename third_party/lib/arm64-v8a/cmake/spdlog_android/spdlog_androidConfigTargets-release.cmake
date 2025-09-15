#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "spdlog::spdlog_android" for configuration "Release"
set_property(TARGET spdlog::spdlog_android APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spdlog::spdlog_android PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/arm64-v8a/libspdlog_android.so"
  IMPORTED_SONAME_RELEASE "libspdlog_android.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS spdlog::spdlog_android )
list(APPEND _IMPORT_CHECK_FILES_FOR_spdlog::spdlog_android "${_IMPORT_PREFIX}/arm64-v8a/libspdlog_android.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
