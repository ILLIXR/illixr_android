add_library(runtime
        SHARED
        plugin.cpp
)
target_include_directories(runtime PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(runtime
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

