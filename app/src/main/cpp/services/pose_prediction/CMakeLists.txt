add_library(pose_prediction
        SHARED
        plugin.cpp
)

target_include_directories(pose_prediction PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(pose_prediction
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

