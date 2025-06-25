add_library(gldemo SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/frame.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/common_lock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/extended_window.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/gl_util/obj.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/log_service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
)

target_include_directories(gldemo PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(gldemo
        extended_window
        EGL
        GLESv3
        log
        Eigen3::Eigen
)
