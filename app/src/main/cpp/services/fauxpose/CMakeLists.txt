add_library(faux_pose SHARED
        service.cpp
        service.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/data_format/pose_prediction.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
)

target_include_directories(faux_pose PRIVATE
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(faux_pose
        EGL
        GLESv3
        log
)
