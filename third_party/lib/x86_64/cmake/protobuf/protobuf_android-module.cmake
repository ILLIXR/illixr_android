# Environment

# Backwards compatibility
# Define camel case versions of input variables
foreach(UPPER
    PROTOBUF_ANDROID_SRC_ROOT_FOLDER
    PROTOBUF_ANDROID_IMPORT_DIRS
    PROTOBUF_ANDROID_DEBUG
    PROTOBUF_ANDROID_LIBRARY
    PROTOBUF_ANDROID_PROTOC_LIBRARY
    PROTOBUF_ANDROID_INCLUDE_DIR
    PROTOBUF_ANDROID_PROTOC_EXECUTABLE
    PROTOBUF_ANDROID_LIBRARY_DEBUG
    PROTOBUF_ANDROID_PROTOC_LIBRARY_DEBUG
    PROTOBUF_ANDROID_LITE_LIBRARY
    PROTOBUF_ANDROID_LITE_LIBRARY_DEBUG
    )
    if (DEFINED ${UPPER})
        string(REPLACE "PROTOBUF_" "Protobuf_" Camel ${UPPER})
        if (NOT DEFINED ${Camel})
            set(${Camel} ${${UPPER}})
        endif()
    endif()
endforeach()

if(DEFINED Protobuf_SRC_ROOT_FOLDER)
  message(AUTHOR_WARNING "Variable Protobuf_SRC_ROOT_FOLDER defined, but not"
    " used in CONFIG mode")
endif()

include(SelectLibraryConfigurations)

# Internal function: search for normal library as well as a debug one
#    if the debug one is specified also include debug/optimized keywords
#    in *_LIBRARIES variable
function(_protobuf_find_libraries name filename)
  if(${name}_LIBRARIES)
    # Use result recorded by a previous call.
  elseif(${name}_LIBRARY)
    # Honor cache entry used by CMake 3.5 and lower.
    set(${name}_LIBRARIES "${${name}_LIBRARY}" PARENT_SCOPE)
  elseif(TARGET protobuf::lib${filename})
    get_target_property(${name}_LIBRARY_RELEASE protobuf::lib${filename}
      LOCATION_RELEASE)
    get_target_property(${name}_LIBRARY_RELWITHDEBINFO protobuf::lib${filename}
      LOCATION_RELWITHDEBINFO)
    get_target_property(${name}_LIBRARY_MINSIZEREL protobuf::lib${filename}
      LOCATION_MINSIZEREL)
    get_target_property(${name}_LIBRARY_DEBUG protobuf::lib${filename}
      LOCATION_DEBUG)

    select_library_configurations(${name})
    set(${name}_LIBRARY ${${name}_LIBRARY} PARENT_SCOPE)
    set(${name}_LIBRARIES ${${name}_LIBRARIES} PARENT_SCOPE)
  endif()
endfunction()

#
# Main.
#

# By default have PROTOBUF_GENERATE_CPP macro pass -I to protoc
# for each directory where a proto file is referenced.
if(NOT DEFINED PROTOBUF_GENERATE_CPP_APPEND_PATH)
  set(PROTOBUF_GENERATE_CPP_APPEND_PATH TRUE)
endif()

# The Protobuf library
_protobuf_find_libraries(Protobuf_android protobuf_android)

# The Protobuf Lite library
_protobuf_find_libraries(Protobuf_android_LITE protobuf-lite_android)

# The Protobuf Protoc Library
_protobuf_find_libraries(Protobuf_android_PROTOC protoc_android)

# Set the include directory
get_target_property(Protobuf_INCLUDE_DIRS protobuf::libprotobuf_android
  INTERFACE_INCLUDE_DIRECTORIES)


# Version info variable
set(Protobuf_VERSION "3.21.12.0")

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(Protobuf
    REQUIRED_VARS Protobuf_android_LIBRARIES Protobuf_android_INCLUDE_DIRS
    VERSION_VAR Protobuf_android_VERSION
)

# Backwards compatibility
# Define upper case versions of output variables
foreach(Camel
    Protobuf_android_VERSION
    Protobuf_android_SRC_ROOT_FOLDER
    Protobuf_android_IMPORT_DIRS
    Protobuf_android_DEBUG
    Protobuf_android_INCLUDE_DIRS
    Protobuf_android_LIBRARIES
    Protobuf_android_PROTOC_LIBRARIES
    Protobuf_android_LITE_LIBRARIES
    Protobuf_android_LIBRARY
    Protobuf_android_PROTOC_LIBRARY
    Protobuf_android_INCLUDE_DIR
    Protobuf_android_PROTOC_EXECUTABLE
    Protobuf_android_LIBRARY_DEBUG
    Protobuf_android_PROTOC_LIBRARY_DEBUG
    Protobuf_android_LITE_LIBRARY
    Protobuf_android_LITE_LIBRARY_DEBUG
    )
    string(TOUPPER ${Camel} UPPER)
    set(${UPPER} ${${Camel}})
endforeach()
