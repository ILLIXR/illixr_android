set(PLUGIN_NAME plugin.android_data${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(${PLUGIN_NAME}
        EGL
        GLESv3
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

if (BUILDING_SPDLOG)
    add_dependencies(${PLUGIN_NAME} spdlog_android)
endif()
if (BUILDING_OPENCV)
    add_dependencies(${PLUGIN_NAME} OpenCV_Android)
endif()
if (BUILDING_BOOST)
    add_dependencies(${PLUGIN_NAME} Boost_Install)
endif()
