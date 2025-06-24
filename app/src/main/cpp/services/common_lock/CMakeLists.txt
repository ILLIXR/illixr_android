add_library(common_lock
        SHARED
        plugin.cpp
)

target_include_directories(common_lock PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(common_lock
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

