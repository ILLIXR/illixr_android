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
find_package(Boost 1.72 REQUIRED CONFIG COMPONENTS chrono thread filesystem atomic)
include_directories(${OpenCV_INCLUDE_DIRS})
#set(CMAKE_BUILD_TYPE "Release")
find_package(Eigen3 REQUIRED)

include_directories(include)
set(${CMAKE_C_FLAGS} "${CMAKE_C_FLAGS}")
add_library(native_app_glue STATIC
        ${ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c
)

find_package(PkgConfig REQUIRED)
# now build app's shared lib
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Werror")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Export ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS}  -latomic -u ANativeActivity_onCreate -v -fvisibility=hidden")

add_subdirectory(services/extended_window)
add_subdirectory(services/common_lock)
add_subdirectory(services/log_service)
add_subdirectory(services/pose_prediction)
#add_subdirectory(services/pose_lookup)
#add_subdirectory(services/fauxpose)

add_subdirectory(utils)
add_library(native-activity SHARED
        main.cpp
        src/plugin.cpp
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

target_include_directories(native-activity PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)


if (ILLIXR_CAM)
    #add_subdirectory(plugins/offline_cam)
    #add_subdirectory(plugins/offline_imu)
    add_subdirectory(plugins/offline_imu_cam)
endif ()

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

# add lib dependencies


target_link_libraries(native-activity
        extended_window
        android
        native_app_glue
        EGL
        GLESv3
        log
        Eigen3::Eigen
)


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
#add_subdirectory(plugins/android_cam)
#add_subdirectory(plugins/android_imu)
add_subdirectory(plugins/android_imu_cam)
add_subdirectory(plugins/gldemo)
#add_subdirectory(plugins/gtsam_integrator)
add_subdirectory(plugins/rk4_integrator)
add_subdirectory(plugins/timewarp_gl)


add_subdirectory(src)
