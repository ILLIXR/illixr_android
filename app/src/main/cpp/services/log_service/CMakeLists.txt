add_library(log_service
        SHARED
        plugin.cpp
)

target_include_directories(log_service PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(log_service
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

