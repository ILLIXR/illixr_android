# module to build and install the rk4_integrator ILLIXR plugin

set(PLUGIN_NAME plugin.rk4_integrator${ILLIXR_BUILD_SUFFIX})

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
        $<TARGET_OBJECTS:illixr_switchboard>
)


target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(${PLUGIN_NAME}
        EGL
        GLESv3
        Eigen3::Eigen
        spdlog::spdlog_android
)

target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)

if (BUILDING_SPDLOG)
    add_dependencies(${PLUGIN_NAME} spdlog_android)
endif()
if (BUILDING_BOOST)
    add_dependencies(${PLUGIN_NAME} Boost_Install)
endif()
