# module to build and install the offline_imu ILLIXR plugin

set(PLUGIN_NAME plugin.offline_imu${ILLIXR_BUILD_SUFFIX})

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_loading.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/managed_thread.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/relative_clock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/csv_iterator.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_libraries(${PLUGIN_NAME}
        Eigen3::Eigen
        spdlog::spdlog_android
)
target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)
