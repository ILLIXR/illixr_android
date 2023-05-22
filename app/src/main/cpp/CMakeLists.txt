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

# OpenCV
set(OpenCV_STATIC ON)
set(OpenCV_DIR ~/Downloads/opencv-4.5.5-android-sdk/OpenCV-android-sdk/sdk/native/jni)
include_directories(~/Downloads/opencv-4.5.5-android-sdk/OpenCV-android-sdk/sdk/native/jni/include)

#Boost
set(BOOST_ROOT /home/madhuparna/boost-android/arm64/include)

#Eigen
set(EIGEN_LIB /home/madhuparna/Downloads/eigen-3.4.0)

set(${CMAKE_C_FLAGS}, "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
        ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}  -latomic -u ANativeActivity_onCreate -v -fvisibility=hidden")

#Native Activity
add_library(native-activity
        SHARED
        main.cpp
        runtime/plugin.cpp)

target_include_directories(native-activity PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${BOOST_ROOT}
        ${EIGEN_LIB}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(native-activity
        android
        native_app_glue
        EGL
        GLESv3
        log)

#For Monado
#add_library(runtime
#        SHARED
#        runtime/plugin.cpp)
#
#target_link_libraries(runtime
#        EGL
#        GLESv3
#        log)

#GlDemo
add_library(gldemo
        SHARED
        gldemo/plugin.cpp)

target_include_directories(gldemo PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(gldemo
        EGL
        GLESv3
        log)

#Common Lock
add_library(common_lock
        SHARED
        common_lock/plugin.cpp)

target_include_directories(common_lock PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(common_lock
        log)

#Log Service
add_library(log_service
        SHARED
        log_service/plugin.cpp)

target_include_directories(log_service PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(log_service
        log)

#Timewarp GL
add_library(timewarp_gl
        SHARED
        timewarp_gl/plugin.cpp
        timewarp_gl/utils/hmd.cpp)

target_include_directories(timewarp_gl PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(timewarp_gl
        EGL
        GLESv3
        log
        android)

#Pose Lookup
add_library(pose_lookup
        SHARED
        pose_lookup/plugin.cpp)

target_include_directories(pose_lookup PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(pose_lookup
        EGL
        GLESv3
        log)