add_library(android_imu
        SHARED
        plugin.cpp
        )

target_include_directories(android_imu PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(android_imu
        EGL
        GLESv3
        log
        android
        )
