# module to build and install the pose_lookup ILLIXR plugin

set(PLUGIN_NAME plugin.pose_lookup${ILLIXR_BUILD_SUFFIX})

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED service.cpp
            service.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_loading.hpp
            utils.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/error_util.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/csv_iterator.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/global_module_defs.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/phonebook.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
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
