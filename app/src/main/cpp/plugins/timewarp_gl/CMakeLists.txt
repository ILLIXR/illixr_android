add_library(timewarp_gl
        SHARED
        plugin.cpp
        ../../GLFW/utils/hmd.cpp
        $<TARGET_OBJECTS:illixr_hmd>
)

target_include_directories(timewarp_gl PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(timewarp_gl
        EGL
        GLESv3
        log
        android
        Eigen3::Eigen
)

