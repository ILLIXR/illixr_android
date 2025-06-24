add_library(pose_lookup
        SHARED
        plugin.cpp
)

target_include_directories(pose_lookup PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(pose_lookup
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

