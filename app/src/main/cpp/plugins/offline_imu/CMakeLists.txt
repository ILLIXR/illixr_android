add_library(offline_imu
        SHARED
        plugin.cpp
)

target_link_libraries(offline_imu
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

target_include_directories(offline_imu PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)
