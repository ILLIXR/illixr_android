set(PLUGIN_NAME plugin.unity.pose${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        service.cpp
        service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/phonebook.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(${PLUGIN_NAME} PUBLIC
        Eigen3::Eigen
        spdlog::spdlog_android
)

target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)
