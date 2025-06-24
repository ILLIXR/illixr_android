add_library(offline_imu_cam
        SHARED
        plugin.cpp
)

target_include_directories(offline_imu_cam PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

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
