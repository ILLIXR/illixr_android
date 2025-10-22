#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "protobuf::libprotobuf-lite_android" for configuration "Release"
set_property(TARGET protobuf::libprotobuf-lite_android APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(protobuf::libprotobuf-lite_android PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/arm64-v8a/libprotobuf-lite_android.a"
  )

list(APPEND _cmake_import_check_targets protobuf::libprotobuf-lite_android )
list(APPEND _cmake_import_check_files_for_protobuf::libprotobuf-lite_android "${_IMPORT_PREFIX}/lib/arm64-v8a/libprotobuf-lite_android.a" )

# Import target "protobuf::libprotobuf_android" for configuration "Release"
set_property(TARGET protobuf::libprotobuf_android APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(protobuf::libprotobuf_android PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/arm64-v8a/libprotobuf_android.a"
  )

list(APPEND _cmake_import_check_targets protobuf::libprotobuf_android )
list(APPEND _cmake_import_check_files_for_protobuf::libprotobuf_android "${_IMPORT_PREFIX}/lib/arm64-v8a/libprotobuf_android.a" )

# Import target "protobuf::libprotoc_android" for configuration "Release"
set_property(TARGET protobuf::libprotoc_android APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(protobuf::libprotoc_android PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/arm64-v8a/libprotoc_android.a"
  )

list(APPEND _cmake_import_check_targets protobuf::libprotoc_android )
list(APPEND _cmake_import_check_files_for_protobuf::libprotoc_android "${_IMPORT_PREFIX}/lib/arm64-v8a/libprotoc_android.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
