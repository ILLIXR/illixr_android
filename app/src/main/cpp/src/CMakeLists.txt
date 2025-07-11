add_library(native-activity SHARED
        illixr.hpp
        main.cpp
        plugin.cpp
        runtime_impl.cpp
        noop_record_logger.hpp
        sqlite_record_logger.hpp
        stdout_record_logger.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/dynamic_lib.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/error_util.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/extended_window.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/relative_clock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/runtime.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/stoplight.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/switchboard.hpp
)
target_include_directories(native-activity PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue)

target_link_libraries(native-activity
        plugin.extended_window${ILLIXR_BUILD_SUFFIX}
        OpenVINS_plugin
        plugin.pose_prediction${ILLIXR_BUILD_SUFFIX}
        plugin.pose_lookup${ILLIXR_BUILD_SUFFIX}
        plugin.fauxpose${ILLIXR_BUILD_SUFFIX}
        plugin.common_lock${ILLIXR_BUILD_SUFFIX}
        plugin.timewarp_gl${ILLIXR_BUILD_SUFFIX}
        plugin.rk4_integrator${ILLIXR_BUILD_SUFFIX}
        plugin.offline_imu${ILLIXR_BUILD_SUFFIX}
        plugin.offline_cam${ILLIXR_BUILD_SUFFIX}
        #plugin.gtsam_integrator${ILLIXR_BUILD_SUFFIX}
        plugin.gldemo${ILLIXR_BUILD_SUFFIX}
        plugin.android_data${ILLIXR_BUILD_SUFFIX}
        android
        native_app_glue
        EGL
        GLESv3
        Eigen3::Eigen
        Boost::serialization
        spdlog::spdlog_android
)

