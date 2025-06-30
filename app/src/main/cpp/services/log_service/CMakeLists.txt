add_library(log_service SHARED
        service.cpp
        service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/log_service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
)

target_include_directories(log_service PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(log_service
        EGL
        GLESv3
        log
        Eigen3::Eigen
        spdlog::spdlog_android
)

