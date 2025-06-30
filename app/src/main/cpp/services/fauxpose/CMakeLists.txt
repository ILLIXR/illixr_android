# module to build and install the fauxpose ILLIXR service

set(PLUGIN_NAME plugin.fauxpose${ILLIXR_BUILD_SUFFIX})

set(FAUXPOSE_SOURCES service.cpp
    service.hpp
    ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
    ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
    ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose.hpp
)

add_library(${PLUGIN_NAME} SHARED ${FAUXPOSE_SOURCES})

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_libraries(${PLUGIN_NAME}
        EGL
        GLESv3
        log
        spdlog::spdlog_android
)
target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)

