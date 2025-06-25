add_library(android_imu_cam SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
)

target_include_directories(android_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

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
