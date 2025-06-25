find_package(glfw3 QUIET)
if (NOT glfw3_FOUND)
    pkg_search_module(glfw3 REQUIRED glfw3)
endif ()

add_library(timewarp_gl SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/extended_window.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/global_module_defs.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/math_util.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/shader_util.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/frame.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/common_lock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/hmd.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
        shaders/basic_shader.hpp
        shaders/timewarp_shader.hpp

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

