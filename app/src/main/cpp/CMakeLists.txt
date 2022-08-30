#
# Copyright (C) The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.18.0)

# build native_app_glue as a static lib
set(OpenCV_STATIC ON)
set(OpenCV_DIR $ENV{OPENCV_ANDROID}/sdk/native/jni)
#find_package(OpenCV REQUIRED)
#set(CMAKE_BUILD_TYPE "Release")
# Declares and names the project.
include_directories(~/Downloads/opencv-4.5.5-android-sdk/OpenCV-android-sdk/sdk/native/jni/include)
include_directories(GLFW)
include_directories(Eigen)
include_directories(boost)
include_directories(common)
include_directories(runtime)
include_directories(timewarp_gl)
include_directories(timewarp_gl/utils)
include_directories(gldemo)
include_directories(pose_lookup)
include_directories(ILLIXR_DATA)

set(CMAKE_C_COMPILER "clang-10")

set(${CMAKE_C_FLAGS}, "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
    ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS}  -latomic -u ANativeActivity_onCreate")

add_library(native-activity
            SHARED
            main.cpp
            runtime/plugin.cpp
        )

add_library(gldemo
        SHARED
        gldemo/plugin.cpp
        )

add_library(timewarp_gl
        SHARED
        timewarp_gl/plugin.cpp
        timewarp_gl/utils/hmd.cpp
        )

add_library(pose_lookup
        SHARED
        pose_lookup/plugin.cpp
        )

add_library(sample
        SHARED
        sample/sample.cpp
        )

target_include_directories(native-activity PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(gldemo PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(timewarp_gl PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(pose_lookup PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)


target_include_directories(sample PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

# add lib dependencies
target_link_libraries(native-activity
    android
    native_app_glue
    EGL
    GLESv3
    log)

target_link_libraries(gldemo
    EGL
    GLESv3
    log
    )

target_link_libraries(timewarp_gl
    EGL
    GLESv3
    log
    )

target_link_libraries(pose_lookup
    EGL
    GLESv3
    log
    )

target_link_libraries(sample
    EGL
    GLESv3
    log
    )