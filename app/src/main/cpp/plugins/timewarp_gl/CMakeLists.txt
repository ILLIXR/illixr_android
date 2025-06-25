find_package(glfw3 QUIET)
if (NOT glfw3_FOUND)
    pkg_search_module(glfw3 REQUIRED glfw3)
endif ()

        plugin.cpp
        ../../GLFW/utils/hmd.cpp
        $<TARGET_OBJECTS:illixr_hmd>
)

target_include_directories(timewarp_gl PRIVATE
        ${glfw3_INCLUDE_DIRS}
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

