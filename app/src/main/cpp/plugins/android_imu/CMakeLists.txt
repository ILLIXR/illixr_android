add_library(android_imu SHARED
        plugin.cpp
        plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/imu.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/misc.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/phonebook.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/threadloop.hpp
)

target_include_directories(android_imu PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(android_imu
        EGL
        GLESv3
        log
        android
)
