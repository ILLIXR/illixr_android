add_library(android_cam
        SHARED
        plugin.cpp
        )


target_include_directories(android_cam PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(android_cam
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
        )
