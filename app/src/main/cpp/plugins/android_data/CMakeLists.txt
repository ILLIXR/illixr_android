add_library(android_data SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
)

target_include_directories(android_data PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(android_data
        EGL
        GLESv3
        spdlog::spdlog_android
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_imgcodecs
        camera2ndk
        mediandk
        android
        Eigen3::Eigen
        spdlog::spdlog_android
)
