add_library(gldemo
        SHARED
        plugin.cpp
)

target_include_directories(gldemo PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(gldemo
        EGL
        GLESv3
        log
        Eigen3::Eigen
)
