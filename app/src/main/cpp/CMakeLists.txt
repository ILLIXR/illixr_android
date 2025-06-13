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

cmake_minimum_required(VERSION 3.22.1)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(Boost_USE_STATIC_LIBS ON)
project(illixr_android)
# build native_app_glue as a static lib
#set(BOOST_ROOT_DIR /home/friedel/Downloads/ndk_21_boost_1.72.0/libs/arm64-v8a/cmake)
#set(OpenCV_LIBRARY_DIR /home/friedel/Downloads/OpenCV-android-sdk/sdk/native/staticlibs/arm64-v8a)
set(ILLIXR_INTEGRATION 1)
#set(OpenCV_CONFIG_SUBDIR arm64-v8a)
#set(CMAKE_MODULE_PATH "${BOOST_ROOT_DIR};${CMAKE_MODULE_PATH}")
#set(Boost_VERBOSE ON)
#set(Boost_DEBUG ON)
find_package(OpenCV 4.5.5 REQUIRED CONFIG COMPONENTS opencv_java)
#set(CMAKE_FIND_DEBUG_MODE TRUE)
find_package(Boost 1.72 REQUIRED CONFIG COMPONENTS chrono thread filesystem atomic)
include_directories(${OpenCV_INCLUDE_DIRS})
#set(CMAKE_BUILD_TYPE "Release")
#include_directories(/Users/madhuparnabhowmik/out/sdk/native/jni/include)
find_package(Eigen3 REQUIRED)

include_directories(GLFW)
#include_directories(/home/madhuparna/AndroidStudioProjects/illixr-native-activity/app/src/main/cpp/boost)
#include_directories(/home/madhuparna/Android/Sdk/ndk/21.4.7075529/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/include/boost/thread/detail/thread.hpp)
include_directories(common)
include_directories(runtime)
include_directories(timewarp_gl)
include_directories(GLFW/utils)
include_directories(gldemo)
include_directories(open_vins/ov_core)
#include_directories(open_vins/ov_msckf)
include_directories(rk4_integrator)
include_directories(android_cam)
include_directories(android_imu)
include_directories(pose_prediction)
include_directories(log_service)
#include_directories(pose_lookup)
#include_directories(ILLIXR_DATA)
#set(CMAKE_C_COMPILER "clang-10")

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
        "${CMAKE_SHARED_LINKER_FLAGS}  -latomic -u ANativeActivity_onCreate -v -fvisibility=hidden")

add_library(native-activity
        SHARED
        main.cpp
        runtime/plugin.cpp
)

add_library(runtime
        SHARED
        runtime/plugin.cpp
)

add_library(gldemo
        SHARED
        gldemo/plugin.cpp
)

add_library(common_lock
        SHARED
        common_lock/plugin.cpp
)

add_library(log_service
        SHARED
        log_service/plugin.cpp
)

add_library(timewarp_gl
        SHARED
        timewarp_gl/plugin.cpp
        GLFW/utils/hmd.cpp
)

add_library(pose_lookup
        SHARED
        pose_lookup/plugin.cpp
)

add_library(pose_prediction
        SHARED
        pose_prediction/plugin.cpp
)

add_library(rk4_integrator
        SHARED
        rk4_integrator/plugin.cpp
)

#add_library(android_cam
#        SHARED
#        android_cam/plugin.cpp
#        )
#
#add_library(android_imu
#        SHARED
#        android_imu/plugin.cpp
#        )

#add_library(offline_cam
#        SHARED
#        offline_cam/plugin.cpp
#        )

#add_library(offline_imu
#        SHARED
#        offline_imu/plugin.cpp
#        )

add_library(offline_imu_cam
        SHARED
        offline_imu_cam/plugin.cpp
)

add_library(android_imu_cam
        SHARED
        android_imu_cam/plugin.cpp
)

add_library(ov_core_lib
        SHARED
        open_vins/ov_core/src/dummy.cpp
        open_vins/ov_core/src/init/InertialInitializer.cpp
        open_vins/ov_core/src/sim/BsplineSE3.cpp
        open_vins/ov_core/src/track/TrackBase.cpp
        #open_vins/ov_core/src/track/TrackAruco.cpp
        open_vins/ov_core/src/track/TrackDescriptor.cpp
        open_vins/ov_core/src/track/TrackKLT.cpp
        open_vins/ov_core/src/track/TrackSIM.cpp
        open_vins/ov_core/src/types/Landmark.cpp
        open_vins/ov_core/src/feat/Feature.cpp
        open_vins/ov_core/src/feat/FeatureInitializer.cpp
)

add_library(ov_msckf_lib
        SHARED
        #open_vins/ov_msckf/src/sim/Simulator.cpp
        open_vins/ov_msckf/src/state/State.cpp
        open_vins/ov_msckf/src/state/StateHelper.cpp
        open_vins/ov_msckf/src/state/Propagator.cpp
        open_vins/ov_msckf/src/core/VioManager.cpp
        open_vins/ov_msckf/src/update/UpdaterHelper.cpp
        open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp
        open_vins/ov_msckf/src/update/UpdaterSLAM.cpp
)

add_library(slam
        SHARED
        open_vins/ov_msckf/src/slam2.cpp
)

#add_library(faux_pose
#        SHARED
#        faux_pose/plugin.cpp
#        )

target_include_directories(native-activity PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(runtime PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(gldemo PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(common_lock PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(log_service PRIVATE
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

target_include_directories(pose_prediction PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(rk4_integrator PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

#target_include_directories(android_cam PRIVATE
#        ${OpenCV_LIBS}
#        ${jnigraphics-lib}
#        ${ANDROID_NDK}/sources/android/native_app_glue)
#
#target_include_directories(android_imu PRIVATE
#        ${OpenCV_LIBS}
#        ${jnigraphics-lib}
#        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(offline_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(android_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

#target_include_directories(offline_imu PRIVATE
#        ${OpenCV_LIBS}
#        ${jnigraphics-lib}
#        ${ANDROID_NDK}/sources/android/native_app_glue)

#target_include_directories(offline_cam PRIVATE
#        ${OpenCV_LIBS}
#        ${jnigraphics-lib}
#        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(ov_core_lib PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

target_include_directories(ov_msckf_lib PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        open_vins/ov_core
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(slam PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

#target_include_directories(faux_pose PRIVATE
#        ${OpenCV_LIBS}
#        ${jnigraphics-lib}
#        ${ANDROID_NDK}/sources/android/native_app_glue)

# add lib dependencies


target_link_libraries(native-activity
        android
        native_app_glue
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(runtime
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(gldemo
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(common_lock
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(log_service
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(timewarp_gl
        EGL
        GLESv3
        log
        android
        Eigen3::Eigen
)

target_link_libraries(pose_lookup
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(pose_prediction
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_link_libraries(rk4_integrator
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

#target_link_libraries(offline_imu
#        EGL
#        GLESv3
#        log
#        )

target_link_libraries(offline_imu_cam
        EGL
        GLESv3
        log
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_imgcodecs
        Eigen3::Eigen
)

target_link_libraries(android_imu_cam
        EGL
        GLESv3
        log
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_imgcodecs
        camera2ndk
        mediandk
        android
        Eigen3::Eigen
)

#target_link_libraries(offline_cam
#        EGL
#        GLESv3
#        log
#        opencv_core
#        opencv_imgproc
#        opencv_highgui
#        opencv_imgcodecs
#        )

#target_link_libraries(android_cam
#        EGL
#        GLESv3
#        log
#        opencv_core
#        opencv_imgproc
#        opencv_highgui
#        opencv_imgcodecs
#        camera2ndk
#        mediandk
#        android
#        )
#
#target_link_libraries(android_imu
#        EGL
#        GLESv3
#        log
#        android
#        )

target_link_libraries(ov_core_lib
        EGL
        GLESv3
        opencv_core
        opencv_imgproc
        opencv_tracking
        opencv_imgcodecs
        opencv_video
        opencv_calib3d
        opencv_highgui
        opencv_features2d
        opencv_aruco
        opencv_dnn
        opencv_flann
        opencv_objdetect
        Boost::chrono
        Boost::thread
        Boost::filesystem
        Boost::atomic
        android
        Eigen3::Eigen
        log)

target_link_libraries(ov_msckf_lib
        EGL
        GLESv3
        log
        ov_core_lib
        opencv_core
        opencv_imgproc
        opencv_tracking
        opencv_calib3d
        Boost::chrono
        Boost::thread
        Boost::filesystem
        Boost::atomic
        android
        Eigen3::Eigen
)

target_link_libraries(slam
        EGL
        GLESv3
        log
        ov_core_lib
        ov_msckf_lib
        opencv_core
        opencv_imgproc
        opencv_tracking
        Boost::chrono
        Boost::thread
        Boost::filesystem
        Boost::atomic
        Eigen3::Eigen
)

#target_link_libraries(faux_pose
#    EGL
#    GLESv3
#    log
#    )


