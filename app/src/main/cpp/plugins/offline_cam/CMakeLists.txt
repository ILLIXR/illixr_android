# module to build and install the offline_cam ILLIXR plugin

set(PLUGIN_NAME plugin.offline_cam${ILLIXR_BUILD_SUFFIX})

# source files, listed individually so that any changes will trigger a rebuild
add_library(${PLUGIN_NAME} SHARED plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_loading.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/opencv_data_types.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/phonebook.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/relative_clock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/csv_iterator.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_libraries(${PLUGIN_NAME}
        EGL
        GLESv3
        opencv_core
        opencv_imgproc
        opencv_highgui
        opencv_imgcodecs
        Eigen3::Eigen
        spdlog::spdlog_android
)

target_compile_features(${PLUGIN_NAME} PRIVATE cxx_std_17)
