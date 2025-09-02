set(PLUGIN_NAME plugin.common_lock${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        service.cpp
        service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/common_lock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        $<TARGET_OBJECTS:illixr_switchboard>
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(${PLUGIN_NAME}
        EGL
        GLESv3
        Eigen3::Eigen
        spdlog::spdlog_android
        Boost::serialization
)

