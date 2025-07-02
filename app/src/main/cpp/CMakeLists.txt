cmake_minimum_required(VERSION 3.22.1)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
include(ExternalProject)

set(Boost_USE_STATIC_LIBS ON)
project(illixr_android VERSION 4.0.1)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-DGLSL_VERSION="330")
# build native_app_glue as a static lib

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH})

set(ILLIXR_INTEGRATION 1)
find_package(OpenCV 4.5.5 REQUIRED CONFIG COMPONENTS opencv_java)
if (CMAKE_VERSION VERSION_LESS "3.30.0")
    find_package(Boost 1.72 REQUIRED COMPONENTS chrono thread filesystem atomic iostreams serialization)
else()
    find_package(Boost 1.72 REQUIRED CONFIG COMPONENTS chrono thread filesystem atomic iostreams serialization)
endif()
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
set(ILLIXR_SOURCE_DIR ${CMAKE_SOURCE_DIR})
#set(CMAKE_BUILD_TYPE "Release")
find_package(Eigen3 REQUIRED)

include_directories(include)
set(${CMAKE_C_FLAGS} "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
        ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)

add_compile_definitions(ILLIXR_ANDROID_BUILD)
find_package(PkgConfig REQUIRED)
# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS}  -latomic -u ANativeActivity_onCreate -v -fvisibility=hidden")
include(cmake/Getspdlog.cmake)
link_libraries(Boost::serialization)
add_subdirectory(services/extended_window)
add_subdirectory(services/common_lock)
add_subdirectory(services/pose_prediction)
add_subdirectory(services/pose_lookup)
add_subdirectory(services/fauxpose)

add_subdirectory(utils)


if (ILLIXR_CAM)
    add_subdirectory(plugins/offline_cam)
    add_subdirectory(plugins/offline_imu)
endif ()

add_subdirectory(plugins/openvins)
add_subdirectory(plugins/android_data)
add_subdirectory(plugins/gldemo)
#add_subdirectory(plugins/gtsam_integrator)
add_subdirectory(plugins/rk4_integrator)
add_subdirectory(plugins/timewarp_gl)


add_subdirectory(src)
