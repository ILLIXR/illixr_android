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
project("illixr_android")

# OpenCV
set(OpenCV_STATIC ON)
set(OPENCV_LIBS /Users/madhuparnabhowmik/out/sdk/native/libs/arm64-v8a)
include_directories(/Users/madhuparnabhowmik/out/sdk/native/jni/include)

# Boost
set(BOOST_ROOT /Users/madhuparnabhowmik/boost-libs/arm64/include)

#Eigen
set(EIGEN_DIR /Users/madhuparnabhowmik/eigen-3.4.0)

include_directories(GLFW)
include_directories(Eigen)
include_directories(common)
include_directories(runtime)
include_directories(timewarp_gl)
include_directories(gldemo)
include_directories(open_vins/ov_core)
include_directories(log_service)

set(${CMAKE_C_FLAGS}, "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
        ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)

# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS}  -latomic -u ANativeActivity_onCreate -v -fvisibility=hidden")

# OpenCV Libs
add_library( cv_core-lib SHARED IMPORTED)
set_target_properties( cv_core-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_core.so )

add_library( cv_imgproc-lib SHARED IMPORTED)
set_target_properties( cv_imgproc-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_imgproc.so )

add_library( cv_imgcodecs-lib SHARED IMPORTED)
set_target_properties( cv_imgcodecs-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_imgcodecs.so )

add_library( cv_tracking-lib SHARED IMPORTED)
set_target_properties( cv_tracking-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_tracking.so )

add_library( cv_video-lib SHARED IMPORTED)
set_target_properties( cv_video-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_video.so)

add_library( cv_calib3d-lib SHARED IMPORTED)
set_target_properties( cv_calib3d-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_calib3d.so)


add_library( cv_highgui-lib SHARED IMPORTED)
set_target_properties( cv_highgui-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_highgui.so)

add_library( cv_features2d-lib SHARED IMPORTED)
set_target_properties( cv_features2d-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_features2d.so)

add_library( cv_aruco-lib SHARED IMPORTED)
set_target_properties( cv_aruco-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_aruco.so)

add_library( cv_dnn-lib SHARED IMPORTED)
set_target_properties( cv_dnn-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_dnn.so)

add_library( cv_flann-lib SHARED IMPORTED)
set_target_properties( cv_flann-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_flann.so)

add_library( cv_objdetect-lib SHARED IMPORTED)
set_target_properties( cv_objdetect-lib
        PROPERTIES IMPORTED_LOCATION
        ${OPENCV_LIBS}/libopencv_objdetect.so)

# Boost Libs
add_library( boost_chrono-lib SHARED IMPORTED)
set_target_properties( boost_chrono-lib
        PROPERTIES IMPORTED_LOCATION
        ${BOOST_ROOT}/libboost_chrono.so)

add_library( boost_thread-lib SHARED IMPORTED)
set_target_properties( boost_thread-lib
        PROPERTIES IMPORTED_LOCATION
        ${BOOST_ROOT}/libboost_thread.so)

add_library( boost_filesystem-lib SHARED IMPORTED)
set_target_properties( boost_filesystem-lib
        PROPERTIES IMPORTED_LOCATION
        ${BOOST_ROOT}/libboost_filesystem.so)

add_library( boost_atomic-lib SHARED IMPORTED)
set_target_properties( boost_atomic-lib
        PROPERTIES IMPORTED_LOCATION
        ${BOOST_ROOT}/libboost_atomic.so)

# Runtime plugin
add_library(native-activity
        SHARED
        main.cpp
        runtime/plugin.cpp)

target_include_directories(native-activity PRIVATE
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

# For Monado
#add_library(runtime
#        SHARED
#        runtime/plugin.cpp)
#
#target_link_libraries(runtime
#        EGL
#        GLESv3
#        log)


# GlDemo
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

# Common Lock
add_library(common_lock
        SHARED
        common_lock/plugin.cpp)

target_include_directories(common_lock PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(common_lock
        log)

# Log Service
add_library(log_service
        SHARED
        log_service/plugin.cpp)

target_include_directories(log_service PRIVATE
        ${BOOST_ROOT}
        ${EIGEN_LIB})

target_link_libraries(log_service
        log)

# Timewarp GL
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

# Pose Lookup
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

# OpenVINS
add_library(ov_core_lib
        SHARED
        open_vins/ov_core/src/dummy.cpp
        open_vins/ov_core/src/init/InertialInitializer.cpp
        open_vins/ov_core/src/sim/BsplineSE3.cpp
        open_vins/ov_core/src/track/TrackBase.cpp
        open_vins/ov_core/src/track/TrackDescriptor.cpp
        open_vins/ov_core/src/track/TrackKLT.cpp
        open_vins/ov_core/src/track/TrackSIM.cpp
        open_vins/ov_core/src/types/Landmark.cpp
        open_vins/ov_core/src/feat/Feature.cpp
        open_vins/ov_core/src/feat/FeatureInitializer.cpp)

add_library(ov_msckf_lib
        SHARED
        open_vins/ov_msckf/src/state/State.cpp
        open_vins/ov_msckf/src/state/StateHelper.cpp
        open_vins/ov_msckf/src/state/Propagator.cpp
        open_vins/ov_msckf/src/core/VioManager.cpp
        open_vins/ov_msckf/src/update/UpdaterHelper.cpp
        open_vins/ov_msckf/src/update/UpdaterMSCKF.cpp
        open_vins/ov_msckf/src/update/UpdaterSLAM.cpp)

add_library(slam
        SHARED
        open_vins/ov_msckf/src/slam2.cpp)


target_include_directories(ov_core_lib PRIVATE
        ${EIGEN_DIR}
        ${BOOST_ROOT}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(ov_msckf_lib PRIVATE
        ${EIGEN_DIR}
        ${jnigraphics-lib}
        ${BOOST_ROOT}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_include_directories(slam PRIVATE
        ${EIGEN_DIR}
        ${BOOST_ROOT}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

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
        boost_chrono-lib
        boost_thread-lib
        boost_filesystem-lib
        boost_atomic-lib
        android
        log)

target_link_libraries(ov_msckf_lib
        EGL
        GLESv3
        log
        ov_core_lib
        cv_core-lib
        cv_imgproc-lib
        cv_tracking-lib
        cv_calib3d-lib
        boost_chrono-lib
        boost_thread-lib
        boost_filesystem-lib
        boost_atomic-lib
        android)

target_link_libraries(slam
        EGL
        GLESv3
        log
        ov_core_lib
        ov_msckf_lib
        cv_core-lib
        cv_imgproc-lib
        cv_tracking-lib)

# Pose Prediction
add_library(pose_prediction
        SHARED
        pose_prediction/plugin.cpp)

target_include_directories(pose_prediction PRIVATE
        ${EIGEN_DIR}
        ${BOOST_ROOT}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(pose_prediction
        EGL
        GLESv3
        log)

# Rk4 Integrator
add_library(rk4_integrator
        SHARED
        rk4_integrator/plugin.cpp)

target_include_directories(rk4_integrator PRIVATE
        ${EIGEN_DIR}
        ${BOOST_ROOT}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(rk4_integrator
        EGL
        GLESv3
        log)

# Offline IMU Cam

add_library(offline_imu_cam
        SHARED
        offline_imu_cam/plugin.cpp)

target_include_directories(offline_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${BOOST_ROOT}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(offline_imu_cam
        EGL
        GLESv3
        log
        cv_core-lib
        cv_imgproc-lib
        cv_highgui-lib
        cv_imgcodecs-lib)

# Android IMU Cam

add_library(android_imu_cam
        SHARED
        android_imu_cam/plugin.cpp)

target_include_directories(android_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${EIGEN_DIR}
        ${BOOST_ROOT}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

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
        android)