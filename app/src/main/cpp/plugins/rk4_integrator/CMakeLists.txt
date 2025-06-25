add_library(rk4_integrator SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
)

target_include_directories(rk4_integrator PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(rk4_integrator
        EGL
        GLESv3
        log
        Eigen3::Eigen
)

