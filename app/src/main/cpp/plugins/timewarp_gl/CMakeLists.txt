find_package(glfw3 QUIET)
if (NOT glfw3_FOUND)
    pkg_search_module(glfw3 REQUIRED glfw3)
endif ()

set(PLUGIN_NAME "plugin.timewarp_gl${ILLIXR_BUILD_SUFFIX}")

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED
        plugin.cpp
        plugin.hpp
        shaders/timewarp_shader.hpp
        $<TARGET_OBJECTS:illixr_hmd>
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/frame.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/error_util.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/extended_window.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/global_module_defs.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/math_util.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/shader_util.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${glfw3_INCLUDE_DIRS}
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(${PLUGIN_NAME}
        plugin.extended_window${ILLIXR_BUILD_SUFFIX}
        vulkan
        EGL
        GLESv3
        android
        Eigen3::Eigen
        Boost::serialization
        spdlog::spdlog_android
)

#if (Vulkan_EXTERNAL)
#    add_dependencies(${PLUGIN_NAME} ${Vulkan_DEP_STR})
#endif ()
target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)
