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

set(Boost_USE_STATIC_LIBS ON)
# build native_app_glue as a static lib
set(OpenCV_STATIC ON)
set(OpenCV_DIR $ENV{OPENCV_ANDROID}/sdk/native/jni)
set(ILLIXR_INTEGRATION 1)
#find_package(OpenCV REQUIRED)
find_package(Boost 1.72 REQUIRED CONFIG COMPONENTS chrono thread filesystem atomic)
#set(CMAKE_BUILD_TYPE "Release")
# eclares and names the project.
include_directories(/Users/madhuparnabhowmik/out/sdk/native/jni/include)
include_directories(/Users/madhuparnabhowmik/OpenCV-android-sdk/sdk/native/jni/include)
#include_directories(SYSTEM /home/madhuparna/opencv/include)
set(EIGEN_DIR /Users/madhuparnabhowmik/eigen-3.4.0)
#find_package(Boost)
#find_package(Boost)
#include_directories(${Boost_INCLUDE_DIRS})

include_directories(GLFW)
include_directories(Eigen)
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

#opencv
add_library( cv_core-lib SHARED IMPORTED)
set_target_properties( cv_core-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_core.so )

add_library( cv_imgproc-lib SHARED IMPORTED)
set_target_properties( cv_imgproc-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_imgproc.so )

add_library( cv_imgcodecs-lib SHARED IMPORTED)
set_target_properties( cv_imgcodecs-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_imgcodecs.so )

add_library( cv_tracking-lib SHARED IMPORTED)
set_target_properties( cv_tracking-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_tracking.so )

add_library( cv_video-lib SHARED IMPORTED)
set_target_properties( cv_video-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_video.so)

add_library( cv_calib3d-lib SHARED IMPORTED)
set_target_properties( cv_calib3d-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_calib3d.so)


add_library( cv_highgui-lib SHARED IMPORTED)
set_target_properties( cv_highgui-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_highgui.so)

add_library( cv_features2d-lib SHARED IMPORTED)
set_target_properties( cv_features2d-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_features2d.so)

add_library( cv_aruco-lib SHARED IMPORTED)
set_target_properties( cv_aruco-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_aruco.so)

add_library( cv_dnn-lib SHARED IMPORTED)
set_target_properties( cv_dnn-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_dnn.so)

add_library( cv_flann-lib SHARED IMPORTED)
set_target_properties( cv_flann-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_flann.so)

add_library( cv_objdetect-lib SHARED IMPORTED)
set_target_properties( cv_objdetect-lib
        PROPERTIES IMPORTED_LOCATION
        /Users/madhuparnabhowmik/arm64-v8a/libopencv_objdetect.so)
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
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(runtime PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(gldemo PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(common_lock PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(log_service PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(timewarp_gl PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(pose_lookup PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(pose_prediction PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(rk4_integrator PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
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
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(android_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
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
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
        )

target_include_directories(ov_msckf_lib PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        open_vins/ov_core
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(slam PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
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
        )

target_link_libraries(runtime
        EGL
        GLESv3
        log
        )

target_link_libraries(gldemo
        EGL
        GLESv3
        log
        )

target_link_libraries(common_lock
        EGL
        GLESv3
        log
        )

target_link_libraries(log_service
        EGL
        GLESv3
        log
        )

target_link_libraries(timewarp_gl
        EGL
        GLESv3
        log
        android
        )

target_link_libraries(pose_lookup
        EGL
        GLESv3
        log
        )

target_link_libraries(pose_prediction
        EGL
        GLESv3
        log
        )

target_link_libraries(rk4_integrator
        EGL
        GLESv3
        log
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
        cv_core-lib
        cv_imgproc-lib
        cv_highgui-lib
        cv_imgcodecs-lib
        )

target_link_libraries(android_imu_cam
        EGL
        GLESv3
        log
        cv_core-lib
        cv_imgproc-lib
        cv_highgui-lib
        cv_imgcodecs-lib
        camera2ndk
        mediandk
        android
        )

#target_link_libraries(offline_cam
#        EGL
#        GLESv3
#        log
#        cv_core-lib
#        cv_imgproc-lib
#        cv_highgui-lib
#        cv_imgcodecs-lib
#        )

#target_link_libraries(android_cam
#        EGL
#        GLESv3
#        log
#        cv_core-lib
#        cv_imgproc-lib
#        cv_highgui-lib
#        cv_imgcodecs-lib
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
        cv_core-lib
        cv_imgproc-lib
        cv_tracking-lib
        cv_imgcodecs-lib
        cv_video-lib
        cv_calib3d-lib
        cv_highgui-lib
        cv_features2d-lib
        cv_aruco-lib
        cv_dnn-lib
        cv_flann-lib
        cv_objdetect-lib
        Boost::chrono
        Boost::thread
        Boost::filesystem
        Boost::atomic
        android
        log)

target_link_libraries(ov_msckf_lib
        EGL
        GLESv3
        log
        ${BOOST_ROOT}
        ov_core_lib
        cv_core-lib
        cv_imgproc-lib
        cv_tracking-lib
        cv_calib3d-lib
        Boost::chrono
        Boost::thread
        Boost::filesystem
        Boost::atomic
        android
        )

target_link_libraries(slam
        EGL
        GLESv3
        log
        ov_core_lib
        ov_msckf_lib
        cv_core-lib
        cv_imgproc-lib
        cv_tracking-lib
        Boost::chrono
        Boost::thread
        Boost::filesystem
        Boost::atomic
        boost_atomic-lib
        )

#target_link_libraries(faux_pose
#    EGL
#    GLESv3
#    log
#    )


