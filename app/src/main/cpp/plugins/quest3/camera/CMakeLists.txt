set(PLUGIN_NAME plugin.quest3.camera.${ILLIXR_BUILD_SUFFIX})

add_library(${PLUGIN_NAME} SHARED
        plugin.cpp
        plugin.hpp
)

target_include_directories(${PLUGIN_NAME} PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_directories(${PLUGIN_NAME}
        opencv_core
        camera2ndk
        android
        spdlog::spdlog_android
)