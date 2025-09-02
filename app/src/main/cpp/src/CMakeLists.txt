if (ANDROID)
    set(VULKAN_FILES
            display/android_display.hpp
            display/android_display.cpp
            vulkan_display.hpp
            vulkan_display.cpp
            ${CMAKE_SOURCE_DIR}/include/illixr/vk/vk_extension_request.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/vk/vulkan_objects.hpp
    )
else()
    set(VULKAN_FILES
            display/x11_direct.hpp
            display/x11_direct.cpp
            display/glfw_extended.hpp
            display/glfw_extended.cpp
            display/headless.hpp
            display/headless.cpp
            display/android_display.hpp
            display/android_display.cpp
            ${CMAKE_SOURCE_DIR}/include/illixr/vk/vk_extension_request.hpp
            ${CMAKE_SOURCE_DIR}/include/illixr/vk/vulkan_objects.hpp
    )
endif()
add_library(native-activity SHARED
        illixr.hpp
        main.cpp
        plugin.cpp
        runtime_impl.cpp
        noop_record_logger.hpp
        sqlite_record_logger.hpp
        stdout_record_logger.hpp
        ${VULKAN_FILES}
        ${CMAKE_SOURCE_DIR}/include/illixr/dynamic_lib.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/error_util.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/extended_window.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/plugin.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/relative_clock.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/runtime.hpp
        ${CMAKE_SOURCE_DIR}/include/illixr/stoplight.hpp
        $<TARGET_OBJECTS:illixr_vulkan_utils>
        $<TARGET_OBJECTS:illixr_switchboard>
)
target_include_directories(native-activity PRIVATE
        ${ILLIXR_SOURCE_DIR}/include
        ${OpenCV_LIBS}
        ${jnigraphics-lib}
        ${ANDROID_NDK}/sources/android/native_app_glue
)

target_link_libraries(native-activity
        plugin.extended_window${ILLIXR_BUILD_SUFFIX}
        OpenVINS_plugin
        plugin.pose_prediction${ILLIXR_BUILD_SUFFIX}
        plugin.pose_lookup${ILLIXR_BUILD_SUFFIX}
        plugin.fauxpose${ILLIXR_BUILD_SUFFIX}
        plugin.common_lock${ILLIXR_BUILD_SUFFIX}
        plugin.timewarp_vk${ILLIXR_BUILD_SUFFIX}
        plugin.rk4_integrator${ILLIXR_BUILD_SUFFIX}
        plugin.offline_imu${ILLIXR_BUILD_SUFFIX}
        plugin.offline_cam${ILLIXR_BUILD_SUFFIX}
        #plugin.gtsam_integrator${ILLIXR_BUILD_SUFFIX}
        plugin.vkdemo${ILLIXR_BUILD_SUFFIX}
        plugin.native_renderer${ILLIXR_BUILD_SUFFIX}
        plugin.android_data${ILLIXR_BUILD_SUFFIX}
        android
        native_app_glue
        EGL
        GLESv3
        Eigen3::Eigen
        Boost::serialization
        spdlog::spdlog_android
        ${VULKAN_LIB}
)

if (BUILDING_SPDLOG)
    add_dependencies(native-activity spdlog_android)
endif()
if (BUILDING_OPENCV)
    add_dependencies(native-activity OpenCV_Android)
endif()
if (BUILDING_BOOST)
    add_dependencies(native-activity Boost_Install)
endif()
